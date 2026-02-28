// LandscapeHelper.cpp
#include "APIEnvir.h"
#include "ACAPinc.h"
#include "LandscapeHelper.hpp"
#include "BrowserRepl.hpp"
#include "APICommon.h"

#include <cmath>
#include <vector>
#include <algorithm>

namespace LandscapeHelper {

	constexpr double PI = 3.14159265358979323846;

	// ---------- Глобальные (множественные пути) ----------
	static std::vector<API_Guid> g_pathGuids;
	static API_Guid  g_protoGuid = APINULLGuid;
	static double    g_stepM = 0.0; // ВНУТРИ: метры (UI → мм → м)
	static int       g_count = 1;

	static inline void LogA(const char* s) {
		// if (BrowserRepl::HasInstance())
		//	BrowserRepl::GetInstance().LogToBrowser(GS::UniString(s));
		(void)s;
	}
	static inline void Log(const GS::UniString& s) {
		// if (BrowserRepl::HasInstance())
		//	BrowserRepl::GetInstance().LogToBrowser(s);
		(void)s;
	}
	static inline double UiStepToMeters(double stepMm) { return stepMm / 1000.0; }

	// ---------- Геометрия ----------
	struct Seg {
		enum Kind { Line, Arc } kind;
		API_Coord a{}, b{};   // Line
		API_Coord c{};        // Arc: center
		double    r = 0.0;
		double    a0 = 0.0;   // start angle
		double    a1 = 0.0;   // end angle (a1 - a0 = signed sweep)
		double    L = 0.0;   // length
	};

	static inline double Dist(const API_Coord& p, const API_Coord& q) {
		return std::hypot(q.x - p.x, q.y - p.y);
	}
	static inline API_Coord Lerp(const API_Coord& p, const API_Coord& q, double t) {
		return { p.x + (q.x - p.x) * t, p.y + (q.y - p.y) * t };
	}
	static inline void PushLine(std::vector<Seg>& segs, const API_Coord& a, const API_Coord& b) {
		Seg s; s.kind = Seg::Line; s.a = a; s.b = b; s.L = Dist(a, b);
		if (s.L > 1e-9) segs.push_back(s);
	}
	static inline double Norm2PI(double a) {
		const double two = 2.0 * PI;
		while (a < 0.0)   a += two;
		while (a >= two)  a -= two;
		return a;
	}
	static inline double CCWDelta(double a0, double a1) {
		a0 = Norm2PI(a0); a1 = Norm2PI(a1);
		double d = a1 - a0; if (d < 0.0) d += 2.0 * PI;
		return d; // [0,2pi)
	}

	// --------- Безье для сплайна ---------
	static inline API_Coord Add(const API_Coord& a, const API_Coord& b) { return { a.x + b.x, a.y + b.y }; }
	static inline API_Coord Sub(const API_Coord& a, const API_Coord& b) { return { a.x - b.x, a.y - b.y }; }
	static inline API_Coord Mul(const API_Coord& a, double s) { return { a.x * s,   a.y * s }; }
	static inline API_Coord FromAngLen(double ang, double len) { return { std::cos(ang) * len, std::sin(ang) * len }; }
	static inline API_Coord BezierPoint(const API_Coord& P0, const API_Coord& C1,
		const API_Coord& C2, const API_Coord& P3, double t)
	{
		const double u = 1.0 - t;
		const double b0 = u * u * u, b1 = 3 * u * u * t, b2 = 3 * u * t * t, b3 = t * t * t;
		return { b0 * P0.x + b1 * C1.x + b2 * C2.x + b3 * P3.x,
				 b0 * P0.y + b1 * C1.y + b2 * C2.y + b3 * P3.y };
	}

	// ============= Полилиния (coords + parcs + pends(Int32)) =============
	static void BuildFromPolyMemo(std::vector<Seg>& out, API_ElementMemo& memo)
	{
		if (memo.coords == nullptr) return;

		const Int32 nAll = (Int32)(BMGetHandleSize((GSHandle)memo.coords) / sizeof(API_Coord));
		const Int32 nPts = std::max<Int32>(0, nAll - 1);            // валидные 1..nPts
		if (nPts < 2) return;

		// «концы» цепочек (многоконтур/разрывы) — Int32
		std::vector<Int32> ends;
		if (memo.pends != nullptr) {
			const Int32 nEnds = (Int32)(BMGetHandleSize((GSHandle)memo.pends) / sizeof(Int32));
			for (Int32 k = 0; k < nEnds; ++k) {
				const Int32 ind = (*memo.pends)[k];
				if (ind >= 1 && ind <= nPts) ends.push_back(ind);
			}
		}
		if (ends.empty()) ends.push_back(nPts); // одна открытая цепочка 1..nPts

		auto isEnd = [&](Int32 i) -> bool {
			return std::find(ends.begin(), ends.end(), i) != ends.end();
			};

		// карта дуг по begIndex (разрешаем только рёбра 1..nPts-1)
		std::vector<double> arcByBeg(nPts + 1, 0.0);
		if (memo.parcs != nullptr) {
			const Int32 nArcs = (Int32)(BMGetHandleSize((GSHandle)memo.parcs) / sizeof(API_PolyArc));
			for (Int32 k = 0; k < nArcs; ++k) {
				const API_PolyArc& pa = (*memo.parcs)[k];
				if (pa.begIndex >= 1 && pa.begIndex <= nPts - 1)
					arcByBeg[pa.begIndex] = pa.arcAngle; // со знаком
			}
		}

		for (Int32 i = 1; i <= nPts - 1; ++i) {
			if (isEnd(i)) continue;               // не соединяем через конец цепочки

			const Int32 j = i + 1;
			const API_Coord& A = (*memo.coords)[i];
			const API_Coord& B = (*memo.coords)[j];

			const double angArc = arcByBeg[i];
			if (std::fabs(angArc) < 1e-9) { PushLine(out, A, B); continue; }

			// две возможные окружности — выбираем ту, у которой sweep по знаку/модулю ближе к arcAngle
			const double dx = B.x - A.x, dy = B.y - A.y;
			const double chord = std::hypot(dx, dy);
			if (chord < 1e-9) continue;

			const double r = std::fabs(chord / (2.0 * std::sin(std::fabs(angArc) * 0.5)));
			const double mx = (A.x + B.x) * 0.5, my = (A.y + B.y) * 0.5;
			const double nx = -dy / chord, ny = dx / chord;
			const double d = std::sqrt(std::max(r * r - 0.25 * chord * chord, 0.0));

			struct Cand { API_Coord c; double a0, a1, L; };
			auto makeCand = [&](double sx, double sy) -> Cand {
				const double cx = mx + sx * d, cy = my + sy * d;
				const double aA = std::atan2(A.y - cy, A.x - cx);
				const double aB = std::atan2(B.y - cy, B.x - cx);
				double sweep = (angArc > 0.0) ? CCWDelta(aA, aB) : -CCWDelta(aB, aA);
				Cand cnd; cnd.c = { cx, cy }; cnd.a0 = aA; cnd.a1 = aA + sweep; cnd.L = r * std::fabs(sweep);
				return cnd;
				};

			const Cand c1 = makeCand(nx, ny);
			const Cand c2 = makeCand(-nx, -ny);

			const double d1 = std::fabs((c1.a1 - c1.a0) - angArc);
			const double d2 = std::fabs((c2.a1 - c2.a0) - angArc);
			const Cand& best = (d1 <= d2 ? c1 : c2);

			Seg s; s.kind = Seg::Arc; s.c = best.c; s.r = r; s.a0 = best.a0; s.a1 = best.a1; s.L = best.L;
			if (s.L > 1e-9) out.push_back(s);
		}
	}

	// ============= Сборка пути по элементу =============
	static bool BuildPathSegments(const API_Guid& pathGuid, std::vector<Seg>& segs, double* totalLen)
	{
		segs.clear();
		if (totalLen) *totalLen = 0.0;

		API_Element e = {}; e.header.guid = pathGuid;
		if (ACAPI_Element_Get(&e) != NoError) return false;

		switch (e.header.type.typeID) {
		case API_LineID:
			PushLine(segs, e.line.begC, e.line.endC);
			break;

		case API_ArcID: {
			Seg s; s.kind = Seg::Arc; s.c = e.arc.origC; s.r = e.arc.r;
			double a0 = Norm2PI(e.arc.begAng);
			double sweep = e.arc.endAng - a0;
			while (sweep <= -2.0 * PI) sweep += 2.0 * PI;
			while (sweep > 2.0 * PI) sweep -= 2.0 * PI;
			s.a0 = a0; s.a1 = a0 + sweep; s.L = s.r * std::fabs(sweep);
			if (s.L > 1e-9) segs.push_back(s);
			break;
		}

		case API_CircleID: {
			Seg s; s.kind = Seg::Arc; s.c = e.circle.origC; s.r = e.circle.r;
			s.a0 = 0.0; s.a1 = 2.0 * PI; s.L = 2.0 * PI * s.r;
			segs.push_back(s);
			break;
		}

		case API_PolyLineID: {
			API_ElementMemo memo = {};
			if (ACAPI_Element_GetMemo(pathGuid, &memo) == NoError && memo.coords != nullptr)
				BuildFromPolyMemo(segs, memo);
			ACAPI_DisposeElemMemoHdls(&memo);
			break;
		}

		case API_SplineID: {
			// Кубические Безье по bezierDirs (качественно + предсказуемо)
			API_ElementMemo memo = {};
			if (ACAPI_Element_GetMemo(pathGuid, &memo, APIMemoMask_Polygon) == NoError &&
				memo.coords != nullptr && memo.bezierDirs != nullptr)
			{
				const Int32 n = (Int32)(BMGetHandleSize((GSHandle)memo.coords) / sizeof(API_Coord));
				if (n >= 2) {
					for (Int32 i = 0; i < n - 1; ++i) {
						const API_Coord P0 = (*memo.coords)[i];
						const API_Coord P3 = (*memo.coords)[i + 1];
						const API_SplineDir d0 = (*memo.bezierDirs)[i];
						const API_SplineDir d1 = (*memo.bezierDirs)[i + 1];
						const API_Coord C1 = Add(P0, FromAngLen(d0.dirAng, d0.lenNext));
						const API_Coord C2 = Sub(P3, FromAngLen(d1.dirAng, d1.lenPrev));

						const int N = 32; // сабсегментов на ребро
						API_Coord prev = P0;
						for (int k = 1; k <= N; ++k) {
							const double t = (double)k / (double)N;
							const API_Coord pt = BezierPoint(P0, C1, C2, P3, t);
							PushLine(segs, prev, pt);
							prev = pt;
						}
					}
				}
			}
			ACAPI_DisposeElemMemoHdls(&memo);
			break;
		}

		default: return false;
		}

		if (segs.empty()) return false;

		double sum = 0.0; for (const Seg& s : segs) sum += s.L;
		if (totalLen) *totalLen = sum;

		GS::UniString dbg; dbg.Printf("[Distrib] path len=%.3f, segs=%u", sum, (unsigned)segs.size());
		Log(dbg);
		return sum > 1e-9;
	}

	// ============= Параметризация по длине s =============
	static void EvalOnPath(const std::vector<Seg>& segs, double s, API_Coord* outP, double* outTanAngleRad)
	{
		double acc = 0.0;
		for (const Seg& seg : segs) {
			if (s > acc + seg.L) { acc += seg.L; continue; }
			const double f = (seg.L < 1e-9) ? 0.0 : (s - acc) / seg.L;

			if (seg.kind == Seg::Line) {
				if (outP)           *outP = Lerp(seg.a, seg.b, f);
				if (outTanAngleRad) *outTanAngleRad = std::atan2(seg.b.y - seg.a.y, seg.b.x - seg.a.x);
			}
			else {
				const double sweep = seg.a1 - seg.a0;                 // со знаком!
				const double ang = seg.a0 + f * sweep;
				if (outP)           *outP = { seg.c.x + seg.r * std::cos(ang), seg.c.y + seg.r * std::sin(ang) };
				if (outTanAngleRad) *outTanAngleRad = ang + ((sweep >= 0.0) ? +PI / 2.0 : -PI / 2.0);
			}
			return;
		}
		const Seg& seg = segs.back();
		if (seg.kind == Seg::Line) {
			if (outP)           *outP = seg.b;
			if (outTanAngleRad) *outTanAngleRad = std::atan2(seg.b.y - seg.a.y, seg.b.x - seg.a.x);
		}
		else {
			const double sweep = seg.a1 - seg.a0;
			if (outP)           *outP = { seg.c.x + seg.r * std::cos(seg.a1), seg.c.y + seg.r * std::sin(seg.a1) };
			if (outTanAngleRad) *outTanAngleRad = seg.a1 + ((sweep >= 0.0) ? +PI / 2.0 : -PI / 2.0);
		}
	}

	// ---------- Утилиты выбора ----------
	static inline bool IsPathType(API_ElemTypeID tid) {
		switch (tid) {
		case API_LineID:
		case API_ArcID:
		case API_CircleID:
		case API_PolyLineID:
		case API_SplineID:
			return true;
		default:
			return false;
		}
	}

	static bool AutoGrabPathsIfNeeded() {
		if (!g_pathGuids.empty()) return true;

		API_SelectionInfo si = {}; GS::Array<API_Neig> neigs;
		ACAPI_Selection_Get(&si, &neigs, false, false);
		BMKillHandle((GSHandle*)&si.marquee.coords);

		for (const API_Neig& n : neigs) {
			API_Element el = {}; el.header.guid = n.guid;
			if (ACAPI_Element_GetHeader(&el.header) != NoError) continue;
			if (IsPathType(el.header.type.typeID)) g_pathGuids.push_back(n.guid);
		}

		if (!g_pathGuids.empty()) {
			GS::UniString dbg; dbg.Printf("[Distrib] PATH AUTOGRAB, count=%u", (unsigned)g_pathGuids.size());
			Log(dbg);
			return true;
		}
		return false;
	}

	static bool AutoGrabProtoIfNeeded() {
		if (g_protoGuid != APINULLGuid) {
			LogA("[Distrib] PROTO already set, skip autograb");
			return true;
		}

		LogA("[Distrib] AutoGrabProtoIfNeeded ENTER");
		API_SelectionInfo si = {}; GS::Array<API_Neig> neigs;
		ACAPI_Selection_Get(&si, &neigs, false, false);
		BMKillHandle((GSHandle*)&si.marquee.coords);

		GS::UniString selDbg; selDbg.Printf("[Distrib] autograb selection count=%u", (unsigned)neigs.GetSize());
		Log(selDbg);

		for (const API_Neig& n : neigs) {
			API_Element el = {}; el.header.guid = n.guid;
			if (ACAPI_Element_GetHeader(&el.header) != NoError) {
				LogA("[Distrib] autograb GetHeader failed");
				continue;
			}
			const API_ElemTypeID tid = el.header.type.typeID;
			GS::UniString typeDbg; typeDbg.Printf("[Distrib] autograb checking type=%d", (int)tid);
			Log(typeDbg);
			if (tid == API_ObjectID || tid == API_LampID || tid == API_ColumnID || tid == API_BeamID) {
				g_protoGuid = n.guid; 
				GS::UniString protoDbg; protoDbg.Printf("[Distrib] PROTO AUTOGRAB: %s (type=%d)", 
					APIGuidToString(n.guid).ToCStr().Get(), (int)tid);
				Log(protoDbg);
				return true;
			}
		}
		LogA("[Distrib] autograb no suitable proto found");
		return false;
	}

	// ---------- Публичные API ----------
	bool SetDistributionLine()
	{
		g_pathGuids.clear();

		API_SelectionInfo si = {}; GS::Array<API_Neig> neigs;
		ACAPI_Selection_Get(&si, &neigs, false, false);
		BMKillHandle((GSHandle*)&si.marquee.coords);

		for (const API_Neig& n : neigs) {
			API_Element el = {}; el.header.guid = n.guid;
			if (ACAPI_Element_GetHeader(&el.header) != NoError) continue;
			if (IsPathType(el.header.type.typeID)) g_pathGuids.push_back(n.guid);
		}

		if (!g_pathGuids.empty()) {
			GS::UniString dbg; dbg.Printf("[Distrib] PATH SET, count=%u", (unsigned)g_pathGuids.size());
			Log(dbg);
			return true;
		}
		LogA("[Distrib] ERR no-path");
		return false;
	}

	bool SetDistributionObject()
	{
		LogA("[Distrib] SetDistributionObject ENTER");
		API_SelectionInfo si = {}; GS::Array<API_Neig> neigs;
		ACAPI_Selection_Get(&si, &neigs, false, false);
		BMKillHandle((GSHandle*)&si.marquee.coords);

		GS::UniString selDbg; selDbg.Printf("[Distrib] selection count=%u", (unsigned)neigs.GetSize());
		Log(selDbg);

		for (const API_Neig& n : neigs) {
			API_Element el = {}; el.header.guid = n.guid;
			if (ACAPI_Element_GetHeader(&el.header) != NoError) {
				LogA("[Distrib] ERR GetHeader failed");
				continue;
			}
			const API_ElemTypeID tid = el.header.type.typeID;
			GS::UniString typeDbg; typeDbg.Printf("[Distrib] checking type=%d (Object=%d, Lamp=%d, Column=%d, Beam=%d)",
				(int)tid, (int)API_ObjectID, (int)API_LampID, (int)API_ColumnID, (int)API_BeamID);
			Log(typeDbg);
			if (tid == API_ObjectID || tid == API_LampID || tid == API_ColumnID || tid == API_BeamID) {
				g_protoGuid = n.guid; 
				GS::UniString protoDbg; protoDbg.Printf("[Distrib] PROTO SET: %s (type=%d)", 
					APIGuidToString(n.guid).ToCStr().Get(), (int)tid);
				Log(protoDbg);
				return true;
			}
		}
		LogA("[Distrib] ERR no-proto");
		return false;
	}

	bool SetDistributionStep(double stepMM)
	{
		if (stepMM > 0.0) {
			g_stepM = UiStepToMeters(stepMM);
			GS::UniString m; m.Printf("[Distrib] step(mm)=%.3f  step(m)=%.6f", stepMM, g_stepM); Log(m);
			return true;
		}
		return false;
	}
	bool SetDistributionCount(int count)
	{
		if (count >= 1) {
			g_count = count;
			GS::UniString m; m.Printf("[Distrib] count=%d", count); Log(m);
			return true;
		}
		return false;
	}

	static bool DistributeOnSinglePath(const API_Element& proto, API_ElemTypeID tid,
		const std::vector<Seg>& segs, double totalLen,
		const double useStepM, const int useCount,
		API_ElementMemo* protoMemo, UInt32* outCreated)
	{
		// точки размещения
		std::vector<double> sVals;
		if (useStepM > 1e-9) {
			for (double s = 0.0; s <= totalLen + 1e-9; s += useStepM)
				sVals.push_back(std::min(s, totalLen));
		}
		else {
			if (useCount == 1) sVals.push_back(0.0);
			else {
				const double st = totalLen / (double)(useCount - 1);
				for (int i = 0; i < useCount; ++i)
					sVals.push_back(std::min(st * i, totalLen));
			}
		}

		UInt32 created = 0;
		for (double s : sVals) {
			API_Coord P; double ang = 0.0;
			EvalOnPath(segs, s, &P, &ang);

			API_Element e = proto; 
			e.header.guid = APINULLGuid;  // Важно: сбрасываем GUID для создания нового элемента

			if (tid == API_ObjectID) { 
				e.object.pos = P;  
				e.object.angle = ang; 
			}
			else if (tid == API_LampID) { 
				e.lamp.pos = P;  
				e.lamp.angle = ang; 
			}
			else if (tid == API_BeamID) {
				// Балка: размещаем по центру (середина балки совпадает с точкой P на пути)
				// Добавляем поворот на 90 градусов (PI/2) к углу пути
				const double beamAng = ang + PI / 2.0;
				const double beamLen = std::hypot(
					proto.beam.endC.x - proto.beam.begC.x,
					proto.beam.endC.y - proto.beam.begC.y);
				// Новая балка: середина в точке P, направление перпендикулярно пути (повернуто на 90°)
				const double halfLen = beamLen * 0.5;
				e.beam.begC.x = P.x - halfLen * std::cos(beamAng);
				e.beam.begC.y = P.y - halfLen * std::sin(beamAng);
				e.beam.endC.x = P.x + halfLen * std::cos(beamAng);
				e.beam.endC.y = P.y + halfLen * std::sin(beamAng);
			}
			else if (tid == API_ColumnID) { 
				// Колонна: устанавливаем позицию и угол поворота
				// Важно: сохраняем все параметры из прототипа (bottomOffset, topOffset, floorInd и т.д.)
				e.column.origoPos = P; 
				e.column.axisRotationAngle = ang;
				// Явно сохраняем этаж из прототипа (должен скопироваться, но для надежности)
				// e.header.floorInd уже скопирован из proto через e = proto
			}

			const GSErrCode ce = ACAPI_Element_Create(&e, protoMemo);
			if (ce == NoError) {
				++created;
				if (tid == API_ColumnID) {
					GS::UniString colDbg; colDbg.Printf("[Distrib] Column created at (%.3f, %.3f), floor=%d, ang=%.3fdeg", 
						P.x, P.y, (int)e.header.floorInd, ang * 180.0 / PI);
					Log(colDbg);
				}
			}
			else {
				GS::UniString msg; 
				msg.Printf("[Distrib] Create err=%d (type=%d, floor=%d)", 
					(int)ce, (int)tid, (int)e.header.floorInd); 
				Log(msg);
			}
		}

		if (outCreated) *outCreated += created;
		GS::UniString dbg; dbg.Printf("[Distrib] path created=%u", (unsigned)created); Log(dbg);
		return created > 0;
	}

	bool DistributeSelected(double stepMM, int count)
	{
		// режим
		double useStepM = 0.0;
		int    useCount = 0;
		if (stepMM > 1e-9) { g_stepM = UiStepToMeters(stepMM); useStepM = g_stepM; useCount = 0; }
		else if (count >= 1) { g_count = count; useStepM = 0.0; useCount = count; }
		else { useStepM = g_stepM; useCount = g_count; }

		{
			GS::UniString dbg; dbg.Printf("[Distrib] use: %s, step(m)=%.6f, count=%d",
				useStepM > 0.0 ? "STEP" : "COUNT", useStepM, useCount); Log(dbg);
		}

		// автоподхваты
		if (!AutoGrabPathsIfNeeded()) { LogA("[Distrib] ERR no-paths");  return false; }
		if (!AutoGrabProtoIfNeeded()) { LogA("[Distrib] ERR no-proto");  return false; }
		if (useStepM <= 0.0 && useCount < 1) { LogA("[Distrib] ERR invalid-params"); return false; }

		// прототип
		API_Element proto = {}; proto.header.guid = g_protoGuid;
		if (ACAPI_Element_Get(&proto) != NoError) { 
			GS::UniString errMsg; errMsg.Printf("[Distrib] ERR proto-get, guid=%s", APIGuidToString(g_protoGuid).ToCStr().Get());
			Log(errMsg);
			return false; 
		}
		const API_ElemTypeID tid = proto.header.type.typeID;
		GS::UniString protoDbg; protoDbg.Printf("[Distrib] Proto: type=%d, floor=%d, guid=%s", 
			(int)tid, (int)proto.header.floorInd, APIGuidToString(g_protoGuid).ToCStr().Get());
		Log(protoDbg);
		if (tid == API_ColumnID) {
			GS::UniString colDbg; colDbg.Printf("[Distrib] Proto column: origoPos=(%.3f, %.3f), bottomOffset=%.6f, topOffset=%.6f",
				proto.column.origoPos.x, proto.column.origoPos.y, proto.column.bottomOffset, proto.column.topOffset);
			Log(colDbg);
		}
		if (tid != API_ObjectID && tid != API_LampID && tid != API_ColumnID && tid != API_BeamID) { 
			LogA("[Distrib] ERR proto-type"); 
			return false; 
		}

		// Undo + общий мемо
		GSErrCode err = ACAPI_CallUndoableCommand("Distribute Along Multiple Paths", [&]() -> GSErrCode {
			API_ElementMemo memo = {}; bool hasMemo = false;
			// Загружаем memo для всех типов, которые могут его требовать
			if (tid == API_ObjectID || tid == API_LampID || tid == API_BeamID || tid == API_ColumnID) {
				GSErrCode memoErr = ACAPI_Element_GetMemo(proto.header.guid, &memo);
				if (memoErr == NoError) {
					hasMemo = true;
					GS::UniString memoDbg; memoDbg.Printf("[Distrib] Memo loaded OK for type=%d", (int)tid);
					Log(memoDbg);
				} else {
					GS::UniString memoDbg; memoDbg.Printf("[Distrib] Memo load failed err=%d for type=%d (continuing without memo)", (int)memoErr, (int)tid);
					Log(memoDbg);
				}
			}

			UInt32 totalCreated = 0;

			for (const API_Guid& pg : g_pathGuids) {
				std::vector<Seg> segs; double totalLen = 0.0;
				if (!BuildPathSegments(pg, segs, &totalLen) || totalLen < 1e-6) {
					LogA("[Distrib] skip: empty/invalid path");
					continue;
				}
				(void)DistributeOnSinglePath(proto, tid, segs, totalLen, useStepM, useCount,
					hasMemo ? &memo : nullptr, &totalCreated);
			}

			if (hasMemo) ACAPI_DisposeElemMemoHdls(&memo);

			GS::UniString fin; fin.Printf("[Distrib] DONE, total created=%u", (unsigned)totalCreated);
			Log(fin);
			return NoError;
			});

		return err == NoError;
	}

} // namespace LandscapeHelper
