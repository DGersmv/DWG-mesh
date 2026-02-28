#include "RoadHelper.hpp"

#include "BrowserRepl.hpp"
#include "GroundHelper.hpp"
#include "ShellHelper.hpp"

#include "APIEnvir.h"
#include "ACAPinc.h"
#include "APICommon.h"
#include "APIdefs_Elements.h"
#include "APIdefs_3D.h"
#include "BM.hpp"

#include <cmath>
#include <cstdarg>
#include <algorithm>
#include <cstring>
#include <cstdio>

namespace RoadHelper {

    constexpr double kPI = 3.1415926535897932384626433832795;
    constexpr double kEPS = 1e-9;

    // ----------------------------------------------------------------------------
    // глобальные guid и этаж
    // ----------------------------------------------------------------------------
    static API_Guid g_centerLineGuid = APINULLGuid;
    static API_Guid g_terrainMeshGuid = APINULLGuid;
    static short    g_refFloor = 0;

    // ----------------------------------------------------------------------------
    // лог
    // ----------------------------------------------------------------------------
    static inline void Log(const char* fmt, ...)
    {
        va_list vl;
        va_start(vl, fmt);
        char buf[4096];
        vsnprintf(buf, sizeof(buf), fmt, vl);
        va_end(vl);

        GS::UniString s(buf);

        if (BrowserRepl::HasInstance()) {
            // BrowserRepl::GetInstance().LogToBrowser(s);
        }

#ifdef DEBUG_UI_LOGS
        ACAPI_WriteReport("%s", false, s.ToCStr().Get());
#endif
    }

    // ----------------------------------------------------------------------------
    // выбрать осевую линию (пользователь сам выделил путь в Archicad)
    // ----------------------------------------------------------------------------
    bool SetCenterLine()
    {
        Log("[RoadHelper] Выбор осевой линии...");

        API_SelectionInfo   selInfo;
        GS::Array<API_Neig> selNeigs;
        if (ACAPI_Selection_Get(&selInfo, &selNeigs, false, false) != NoError || selNeigs.IsEmpty()) {
            Log("[RoadHelper] Нет выделения для осевой линии");
            g_centerLineGuid = APINULLGuid;
            return false;
        }

        g_centerLineGuid = selNeigs[0].guid;

        // забираем этаж
        API_Elem_Head head{};
        head.guid = g_centerLineGuid;
        if (ACAPI_Element_GetHeader(&head) == NoError) {
            g_refFloor = head.floorInd;
        }

        Log("[RoadHelper] Осевая линия зафиксирована: %s (floor=%d)",
            APIGuidToString(g_centerLineGuid).ToCStr().Get(),
            (int)g_refFloor);

        // подчистим выделение структуры выбора
        BMKillHandle((GSHandle*)&selInfo.marquee.coords);
        return true;
    }

    // ----------------------------------------------------------------------------
    // выбрать mesh рельефа (нужно просто чтобы не ругалось BuildRoad)
    // ----------------------------------------------------------------------------
    bool SetTerrainMesh()
    {
        Log("[RoadHelper] Выбор Mesh рельефа...");

        API_SelectionInfo   selInfo;
        GS::Array<API_Neig> selNeigs;
        if (ACAPI_Selection_Get(&selInfo, &selNeigs, false, false) != NoError || selNeigs.IsEmpty()) {
            Log("[RoadHelper] Нет выделения Mesh");
            g_terrainMeshGuid = APINULLGuid;
            return false;
        }

        // берём первый mesh из выделения
        for (const API_Neig& n : selNeigs) {
            API_Element hdr = {};
            hdr.header.guid = n.guid;
            if (ACAPI_Element_GetHeader(&hdr.header) == NoError &&
                hdr.header.type.typeID == API_MeshID)
            {
                g_terrainMeshGuid = n.guid;
                Log("[RoadHelper] Mesh рельефа: %s",
                    APIGuidToString(g_terrainMeshGuid).ToCStr().Get());
                BMKillHandle((GSHandle*)&selInfo.marquee.coords);
                return true;
            }
        }

        Log("[RoadHelper] В выделении нет Mesh");
        g_terrainMeshGuid = APINULLGuid;
        BMKillHandle((GSHandle*)&selInfo.marquee.coords);
        return false;
    }

    // ============================================================================
    // Вспомогательная геометрия 2D для одной осевой
    // ============================================================================

    // Структуры для работы с сегментами пути (скопировано из LandscapeHelper)
    struct Seg {
        enum Kind { Line, Arc } kind;
        API_Coord a{}, b{};   // Line
        API_Coord c{};        // Arc: center
        double    r = 0.0;
        double    a0 = 0.0;   // start angle
        double    a1 = 0.0;   // end angle (a1 - a0 = signed sweep)
        double    L = 0.0;   // length
    };

    // Вспомогательные функции для работы с сегментами
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
        const double two = 2.0 * kPI;
        while (a < 0.0)   a += two;
        while (a >= two)  a -= two;
        return a;
    }
    static inline double CCWDelta(double a0, double a1) {
        a0 = Norm2PI(a0); a1 = Norm2PI(a1);
        double d = a1 - a0; if (d < 0.0) d += 2.0 * kPI;
        return d; // [0,2pi)
    }

    // Безье для сплайна
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

    // Сборка сегментов пути из элемента
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
            while (sweep <= -2.0 * kPI) sweep += 2.0 * kPI;
            while (sweep > 2.0 * kPI) sweep -= 2.0 * kPI;
            s.a0 = a0; s.a1 = a0 + sweep; s.L = s.r * std::fabs(sweep);
            if (s.L > 1e-9) segs.push_back(s);
            break;
        }

        case API_CircleID: {
            Seg s; s.kind = Seg::Arc; s.c = e.circle.origC; s.r = e.circle.r;
            s.a0 = 0.0; s.a1 = 2.0 * kPI; s.L = 2.0 * kPI * s.r;
            segs.push_back(s);
            break;
        }

        case API_PolyLineID: {
            API_ElementMemo memo = {};
            if (ACAPI_Element_GetMemo(pathGuid, &memo) == NoError && memo.coords != nullptr) {
                const Int32 nAll = (Int32)(BMGetHandleSize((GSHandle)memo.coords) / sizeof(API_Coord));
                const Int32 nPts = std::max<Int32>(0, nAll - 1);
                if (nPts >= 2) {
                    for (Int32 i = 1; i <= nPts - 1; ++i) {
                        const API_Coord& A = (*memo.coords)[i];
                        const API_Coord& B = (*memo.coords)[i + 1];
                        PushLine(segs, A, B);
                    }
                }
            }
            ACAPI_DisposeElemMemoHdls(&memo);
            break;
        }

        case API_SplineID: {
            // Кубические Безье по bezierDirs
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

        Log("[RoadHelper] path len=%.3f, segs=%u", sum, (unsigned)segs.size());
        return sum > 1e-9;
    }

    // Параметризация по длине s
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
                if (outTanAngleRad) *outTanAngleRad = ang + ((sweep >= 0.0) ? +kPI / 2.0 : -kPI / 2.0);
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
            if (outTanAngleRad) *outTanAngleRad = seg.a1 + ((sweep >= 0.0) ? +kPI / 2.0 : -kPI / 2.0);
        }
    }

    // 1) Собираем список XY-точек оси как ломаную
    //    Для дуг генерируем точки по окружности
    static bool CollectAxisPoints2D(const API_Guid& guid, GS::Array<API_Coord>& outPts)
    {
        outPts.Clear();

        API_Element el = {};
        el.header.guid = guid;
        if (ACAPI_Element_Get(&el) != NoError) {
            Log("[RoadHelper] CollectAxisPoints2D: не смогли прочитать элемент оси");
            return false;
        }

        switch (el.header.type.typeID) {
        case API_LineID: {
            outPts.Push(el.line.begC);
            outPts.Push(el.line.endC);
            break;
        }

        case API_ArcID: {
            // Для дуги генерируем точки по окружности
            const API_Coord center = el.arc.origC;
            const double radius = el.arc.r;
            const double startAngle = el.arc.begAng;
            const double endAngle = el.arc.endAng;
            
            // Вычисляем количество точек для дуги (минимум 8, максимум 64)
            double sweep = endAngle - startAngle;
            while (sweep <= -2.0 * kPI) sweep += 2.0 * kPI;
            while (sweep > 2.0 * kPI) sweep -= 2.0 * kPI;
            
            const double absSweep = std::abs(sweep);
            const int numPoints = std::max(8, std::min(64, (int)(absSweep * 16.0 / kPI)));
            
            for (int i = 0; i <= numPoints; ++i) {
                const double t = (double)i / (double)numPoints;
                const double angle = startAngle + t * sweep;
                const API_Coord pt = {
                    center.x + radius * std::cos(angle),
                    center.y + radius * std::sin(angle)
                };
                outPts.Push(pt);
            }
            
            Log("[RoadHelper] CollectAxisPoints2D: сгенерировано %u точек для дуги", (unsigned)outPts.GetSize());
            break;
        }

        case API_CircleID: {
            // Для круга генерируем точки по окружности
            const API_Coord center = el.circle.origC;
            const double radius = el.circle.r;
            
            const int numPoints = 32; // фиксированное количество точек для круга
            for (int i = 0; i < numPoints; ++i) {
                const double angle = 2.0 * kPI * (double)i / (double)numPoints;
                const API_Coord pt = {
                    center.x + radius * std::cos(angle),
                    center.y + radius * std::sin(angle)
                };
                outPts.Push(pt);
            }
            
            Log("[RoadHelper] CollectAxisPoints2D: сгенерировано %u точек для круга", (unsigned)outPts.GetSize());
            break;
        }

        case API_PolyLineID:
        case API_SplineID: {
            API_ElementMemo memo = {};
            if (ACAPI_Element_GetMemo(guid, &memo, APIMemoMask_Polygon) == NoError &&
                memo.coords != nullptr)
            {
                const Int32 nAll = (Int32)(BMGetHandleSize((GSHandle)memo.coords) / sizeof(API_Coord));
                const Int32 nPts = std::max<Int32>(0, nAll - 1);
                for (Int32 i = 1; i <= nPts; ++i) { // coords[0] сторож
                    outPts.Push((*memo.coords)[i]);
                }
            }
            ACAPI_DisposeElemMemoHdls(&memo);
            break;
        }

        default:
            Log("[RoadHelper] CollectAxisPoints2D: неподдерживаемый тип");
            return false;
        }

        if (outPts.GetSize() < 2) {
            Log("[RoadHelper] CollectAxisPoints2D: мало точек");
            outPts.Clear();
            return false;
        }

        return true;
    }

    // Проверка замкнутости линии (сравнение начальной и конечной точек с точностью 1мм)
    static bool IsLineClosed(const GS::Array<API_Coord>& pts)
    {
        if (pts.GetSize() < 3) return false; // минимум 3 точки для замкнутой линии
        
        const API_Coord& first = pts[0];
        const API_Coord& last = pts[pts.GetSize() - 1];
        
        const double tolerance = 1.0; // 1мм
        const double dx = last.x - first.x;
        const double dy = last.y - first.y;
        const double dist = std::sqrt(dx * dx + dy * dy);
        
        bool isClosed = (dist <= tolerance);
        Log("[RoadHelper] Проверка замкнутости: dist=%.3fмм, closed=%s", dist, isClosed ? "ДА" : "НЕТ");
        
        return isClosed;
    }

    // 2) Вычисляем нормаль в точке i по соседним сегментам
    static void ComputePerpAtIndex(
        const GS::Array<API_Coord>& axis,
        UIndex i,
        double& nx,
        double& ny
    ) {
        // Берём направление касательной как разницу между соседними точками
        UIndex i0 = (i == 0) ? 0 : i - 1;
        UIndex i1 = (i + 1 < axis.GetSize()) ? i + 1 : i;

        const API_Coord& A = axis[i0];
        const API_Coord& B = axis[i1];

        double vx = B.x - A.x;
        double vy = B.y - A.y;
        double len = std::sqrt(vx * vx + vy * vy);
        if (len < kEPS) {
            nx = 0.0;
            ny = 0.0;
            return;
        }

        // нормализованная касательная
        vx /= len;
        vy /= len;

        // влево от направления (перпендикуляр)
        nx = -vy;
        ny = vx;
    }

    // 3) Строим левую и правую кромку просто сдвигом по этому перпендикуляру
    static void OffsetSides(
        const GS::Array<API_Coord>& axis,
        double halfWidthM,
        GS::Array<API_Coord>& leftSide,
        GS::Array<API_Coord>& rightSide
    ) {
        leftSide.Clear();
        rightSide.Clear();

        for (UIndex i = 0; i < axis.GetSize(); ++i) {
            double nx = 0.0, ny = 0.0;
            ComputePerpAtIndex(axis, i, nx, ny);

            API_Coord L{ axis[i].x + nx * halfWidthM,
                          axis[i].y + ny * halfWidthM };
            API_Coord R{ axis[i].x - nx * halfWidthM,
                          axis[i].y - ny * halfWidthM };

            leftSide.Push(L);
            rightSide.Push(R);
        }
    }

    // ============================================================================
    // создание линий в модели
    // ============================================================================

    // Сплайн по списку 2D-точек
    static API_Guid CreateSplineFromPts(const GS::Array<API_Coord>& pts)
    {
        if (pts.GetSize() < 2)
            return APINULLGuid;

        // мы используем ShellHelper::CreateSplineFromPoints
        return ShellHelper::CreateSplineFromPoints(pts);
    }

    // Прямая линия между двумя точками
    static bool CreateSimpleLine2D(const API_Coord& a, const API_Coord& b, const char* tag)
    {
        API_Element lineEl = {};
        lineEl.header.type = API_LineID;
        if (ACAPI_Element_GetDefaults(&lineEl, nullptr) != NoError)
            return false;

        lineEl.header.floorInd = g_refFloor;
        lineEl.line.begC = a;
        lineEl.line.endC = b;

        const GSErrCode e = ACAPI_CallUndoableCommand("Create Road Line", [&]() -> GSErrCode {
            return ACAPI_Element_Create(&lineEl, nullptr);
            });

        if (e == NoError) {
            Log("[RoadHelper] Line created (%s)", tag);
            return true;
        }

        Log("[RoadHelper] Line FAILED (%s): err=%d", tag, (int)e);
        return false;
    }

    // ============================================================================
    // главная команда
    // ============================================================================

    // Функция откладывания точек по spline с заданным расстоянием
    static bool SamplePointsAlongSpline(const API_Guid& splineGuid, double stepMM, GS::Array<API_Coord>& outPts)
    {
        outPts.Clear();
        
        std::vector<Seg> segs;
        double totalLen = 0.0;
        
        if (!BuildPathSegments(splineGuid, segs, &totalLen)) {
            Log("[RoadHelper] ERROR: не удалось построить сегменты пути");
            return false;
        }
        
        if (totalLen < 1e-6) {
            Log("[RoadHelper] ERROR: путь слишком короткий");
            return false;
        }
        
        const double stepM = stepMM / 1000.0; // мм -> м
        const double epsilon = 1e-6;
        
        // Откладываем точки с заданным шагом
        for (double s = 0.0; s <= totalLen + epsilon; s += stepM) {
            const double clampedS = std::min(s, totalLen);
            API_Coord pt;
            EvalOnPath(segs, clampedS, &pt, nullptr);
            outPts.Push(pt);
        }
        
        // Обязательно добавляем последнюю точку
        API_Coord lastPt;
        EvalOnPath(segs, totalLen, &lastPt, nullptr);
        outPts.Push(lastPt);
        
        Log("[RoadHelper] Отложено %u точек по spline (шаг=%.1fмм, длина=%.3fм)", 
            (unsigned)outPts.GetSize(), stepMM, totalLen);
        
        return outPts.GetSize() >= 2;
    }

    // Копирование элемента с перемещением по вектору используя ACAPI_Element_Edit
    static API_Guid CopyElementWithOffset(const API_Guid& sourceGuid, double offsetX, double offsetY)
    {
        // Получаем информацию об элементе для определения правильного neigID
        API_Element sourceEl = {};
        sourceEl.header.guid = sourceGuid;
        if (ACAPI_Element_Get(&sourceEl) != NoError) {
            Log("[RoadHelper] ERROR: не удалось прочитать исходный элемент");
            return APINULLGuid;
        }

        // Создаем массив с одним элементом для редактирования
        GS::Array<API_Neig> items;
        API_Neig neig;
        neig.guid = sourceGuid;
        
        // Определяем правильный neigID в зависимости от типа элемента
        switch (sourceEl.header.type.typeID) {
        case API_LineID:
            neig.neigID = APINeig_Line;
            break;
        case API_ArcID:
            neig.neigID = APINeig_Arc;
            break;
        case API_CircleID:
            neig.neigID = APINeig_Circ;
            break;
        case API_PolyLineID:
            neig.neigID = APINeig_PolyLine;
            break;
        case API_SplineID:
            neig.neigID = APINeig_Spline;
            break;
        default:
            Log("[RoadHelper] ERROR: неподдерживаемый тип элемента для копирования");
            return APINULLGuid;
        }
        
        items.Push(neig);

        // Параметры редактирования для копирования с перемещением
        API_EditPars editPars = {};
        editPars.typeID = APIEdit_Drag;
        editPars.withDelete = false; // не удаляем оригинал
        editPars.begC.x = 0.0;
        editPars.begC.y = 0.0;
        editPars.begC.z = 0.0;
        editPars.endC.x = offsetX;
        editPars.endC.y = offsetY;
        editPars.endC.z = 0.0;

        // Выполняем редактирование (копирование с перемещением)
        const GSErrCode err = ACAPI_Element_Edit(&items, editPars);
        if (err == NoError && !items.IsEmpty()) {
            API_Guid newGuid = items[0].guid;
            Log("[RoadHelper] Копия создана: %s", APIGuidToString(newGuid).ToCStr().Get());
            return newGuid;
        } else {
            Log("[RoadHelper] ERROR: не удалось создать копию через Edit, err=%d", (int)err);
            return APINULLGuid;
        }
    }

    // Универсальный алгоритм для всех типов линий кроме spline
    static bool BuildUniversalRoad(const API_Guid& sourceGuid, double halfWidthM, API_Guid& leftGuid, API_Guid& rightGuid)
    {
        // Получаем точки исходной линии для определения направления
        GS::Array<API_Coord> centerPts;
        if (!CollectAxisPoints2D(sourceGuid, centerPts)) {
            Log("[RoadHelper] ERROR: не удалось прочитать точки линии");
            return false;
        }

        if (centerPts.GetSize() < 2) {
            Log("[RoadHelper] ERROR: недостаточно точек для универсального алгоритма");
            return false;
        }

        // Проверяем замкнутость
        bool isClosed = IsLineClosed(centerPts);
        Log("[RoadHelper] Универсальный алгоритм: %s линия, точек=%u", 
            isClosed ? "замкнутая" : "открытая", (unsigned)centerPts.GetSize());

        // Для дуг и кругов используем специальный алгоритм с построением перпендикуляров
        API_Element el = {};
        el.header.guid = sourceGuid;
        if (ACAPI_Element_Get(&el) == NoError && 
            (el.header.type.typeID == API_ArcID || el.header.type.typeID == API_CircleID)) {
            
            // Используем алгоритм с перпендикулярами для дуг
            GS::Array<API_Coord> leftPts, rightPts;
            if (!BuildPerpendicularPoints(centerPts, halfWidthM, leftPts, rightPts)) {
                Log("[RoadHelper] ERROR: не удалось построить перпендикуляры для дуги");
                return false;
            }
            
            // Создаем сплайны из точек
            leftGuid = CreateSplineFromPts(leftPts);
            rightGuid = CreateSplineFromPts(rightPts);
            
            if (leftGuid == APINULLGuid || rightGuid == APINULLGuid) {
                Log("[RoadHelper] ERROR: не удалось создать сплайны для дуги");
                return false;
            }
            
            Log("[RoadHelper] Универсальный алгоритм: созданы сплайны для дуги L=%s R=%s (ширина=%.3fм)", 
                APIGuidToString(leftGuid).ToCStr().Get(),
                APIGuidToString(rightGuid).ToCStr().Get(),
                halfWidthM * 2.0);
            
            return true;
        }

        // Для остальных типов линий используем копирование с смещением
        // Определяем направление перпендикуляра
        double nx = 0.0, ny = 0.0;
        
        if (isClosed) {
            // Для замкнутых линий используем направление от первой точки
            ComputePerpAtIndex(centerPts, 0, nx, ny);
        } else {
            // Для открытых линий используем направление от начала к концу
            const API_Coord& start = centerPts[0];
            const API_Coord& end = centerPts[centerPts.GetSize() - 1];
            
            double dx = end.x - start.x;
            double dy = end.y - start.y;
            double len = std::sqrt(dx * dx + dy * dy);
            
            if (len < kEPS) {
                Log("[RoadHelper] ERROR: линия слишком короткая");
                return false;
            }
            
            // Перпендикуляр (поворот на 90 градусов)
            nx = -dy / len;
            ny = dx / len;
        }

        // Создаем две копии с смещением по перпендикуляру в undoable команде
        const double leftOffsetX = nx * halfWidthM;
        const double leftOffsetY = ny * halfWidthM;
        const double rightOffsetX = -nx * halfWidthM;
        const double rightOffsetY = -ny * halfWidthM;

        GSErrCode err = ACAPI_CallUndoableCommand("Copy Road Lines", [&]() -> GSErrCode {
            leftGuid = CopyElementWithOffset(sourceGuid, leftOffsetX, leftOffsetY);
            rightGuid = CopyElementWithOffset(sourceGuid, rightOffsetX, rightOffsetY);
            
            if (leftGuid == APINULLGuid || rightGuid == APINULLGuid) {
                Log("[RoadHelper] ERROR: не удалось создать копии линии");
                return APIERR_GENERAL;
            }
            
            return NoError;
        });

        if (err != NoError) {
            Log("[RoadHelper] ERROR: не удалось выполнить команду копирования, err=%d", (int)err);
            return false;
        }

        Log("[RoadHelper] Универсальный алгоритм: созданы копии L=%s R=%s (ширина=%.3fм)", 
            APIGuidToString(leftGuid).ToCStr().Get(),
            APIGuidToString(rightGuid).ToCStr().Get(),
            halfWidthM * 2.0);

        return true;
    }

    // Функция построения перпендикуляров для получения двух рядов точек
    bool BuildPerpendicularPoints(const GS::Array<API_Coord>& centerPts, double halfWidthM, 
                                 GS::Array<API_Coord>& leftPts, GS::Array<API_Coord>& rightPts)
    {
        leftPts.Clear();
        rightPts.Clear();
        
        if (centerPts.GetSize() < 2) {
            Log("[RoadHelper] ERROR: недостаточно точек для построения перпендикуляров");
            return false;
        }
        
        for (UIndex i = 0; i < centerPts.GetSize(); ++i) {
            double nx = 0.0, ny = 0.0;
            ComputePerpAtIndex(centerPts, i, nx, ny);
            
            const API_Coord& center = centerPts[i];
            API_Coord left{ center.x + nx * halfWidthM, center.y + ny * halfWidthM };
            API_Coord right{ center.x - nx * halfWidthM, center.y - ny * halfWidthM };
            
            leftPts.Push(left);
            rightPts.Push(right);
        }
        
        Log("[RoadHelper] Построено %u перпендикуляров (ширина=%.3fм)", 
            (unsigned)leftPts.GetSize(), halfWidthM * 2.0);
        
        return leftPts.GetSize() >= 2 && rightPts.GetSize() >= 2;
    }

    bool BuildRoad(const RoadParams& params)
    {
        Log("[RoadHelper] >>> BuildRoad: width=%.1fмм, step=%.1fмм",
            params.widthMM, params.sampleStepMM);

        if (g_centerLineGuid == APINULLGuid) {
            Log("[RoadHelper] ERROR: нет осевой линии (сначала SetCenterLine())");
            return false;
        }
        if (g_refFloor == 0) {
            // не критично, но на всякий случай чтоб линии не улетели в другой этаж
            Log("[RoadHelper] WARN: g_refFloor=0");
        }

        if (params.widthMM <= 0.0) {
            Log("[RoadHelper] ERROR: ширина <= 0");
            return false;
        }

        // Определяем тип линии
        API_Element el = {};
        el.header.guid = g_centerLineGuid;
        if (ACAPI_Element_Get(&el) != NoError) {
            Log("[RoadHelper] ERROR: не удалось прочитать элемент");
            return false;
        }

        const double halfWidthM = (params.widthMM / 1000.0) * 0.5; // мм -> м/2
        GS::Array<API_Coord> leftPts;
        GS::Array<API_Coord> rightPts;
        bool success = false;

        // Выбираем алгоритм в зависимости от типа линии
        API_Guid leftGuid = APINULLGuid;
        API_Guid rightGuid = APINULLGuid;
        
        if (el.header.type.typeID == API_SplineID) {
            // Для spline используем старый алгоритм с откладыванием точек
            if (params.sampleStepMM <= 0.0) {
                Log("[RoadHelper] ERROR: для spline нужен шаг > 0");
                return false;
            }

            GS::Array<API_Coord> centerPts;
            if (!SamplePointsAlongSpline(g_centerLineGuid, params.sampleStepMM, centerPts)) {
                Log("[RoadHelper] ERROR: не удалось отложить точки по spline");
                return false;
            }

            GS::Array<API_Coord> leftPts, rightPts;
            success = BuildPerpendicularPoints(centerPts, halfWidthM, leftPts, rightPts);
            if (success) {
                leftGuid = CreateSplineFromPts(leftPts);
                rightGuid = CreateSplineFromPts(rightPts);
            }
            Log("[RoadHelper] Использован алгоритм для spline");
        } else {
            // Для всех остальных типов линий используем универсальный алгоритм с копированием
            success = BuildUniversalRoad(g_centerLineGuid, halfWidthM, leftGuid, rightGuid);
            Log("[RoadHelper] Использован универсальный алгоритм с копированием");
        }

        if (!success || leftGuid == APINULLGuid || rightGuid == APINULLGuid) {
            Log("[RoadHelper] ERROR: не удалось создать боковые линии");
            return false;
        }

        Log("[RoadHelper] боковые линии ок: L=%s  R=%s",
            APIGuidToString(leftGuid).ToCStr().Get(),
            APIGuidToString(rightGuid).ToCStr().Get());

        // Замыкаем начало и концы прямыми линиями (только для открытых линий)
        // Получаем точки для проверки замкнутости
        GS::Array<API_Coord> centerPts;
        if (CollectAxisPoints2D(g_centerLineGuid, centerPts)) {
            bool isClosed = IsLineClosed(centerPts);
            if (!isClosed) {
                // Для открытых линий создаем замыкающие линии
                // Получаем точки боковых линий
                GS::Array<API_Coord> leftPts, rightPts;
                if (CollectAxisPoints2D(leftGuid, leftPts) && CollectAxisPoints2D(rightGuid, rightPts)) {
                    if (leftPts.GetSize() >= 2 && rightPts.GetSize() >= 2) {
                        const UIndex last = leftPts.GetSize() - 1;
                        const API_Coord capA0 = leftPts[0];
                        const API_Coord capB0 = rightPts[0];
                        const API_Coord capA1 = leftPts[last];
                        const API_Coord capB1 = rightPts[last];

                        bool ok1 = CreateSimpleLine2D(capA0, capB0, "start cap");
                        bool ok2 = CreateSimpleLine2D(capA1, capB1, "end cap");

                        if (!ok1 || !ok2) {
                            Log("[RoadHelper] WARNING: не смогли сделать капы");
                        }
                    }
                }
            } else {
                Log("[RoadHelper] Замкнутая линия - капы не нужны");
            }
        }

        Log("[RoadHelper] ✅ ГОТОВО: создали контур дороги");
        return true;
    }

    // ============================================================================
    // Перенесено из MeshHelper: функции создания Morph (НЕ создают mesh!)
    // ============================================================================
    
    // Внутренняя реализация создания Morph из точек (без Undo-обертки)
    // thicknessMM: толщина в мм (0 = плоский, только верхняя поверхность)
    // materialTop, materialBottom, materialSide: индексы материалов для граней
    static bool CreateMorphFromPointsInternal(const GS::Array<API_Coord3D>& points, double thicknessMM,
                                              API_AttributeIndex materialTop, API_AttributeIndex materialBottom, API_AttributeIndex materialSide)
    {
        const UIndex numPoints = points.GetSize();
        const double thickness = thicknessMM / 1000.0; // convert to meters
        Log("[RoadHelper] CreateMorphFromPoints: START with %d points, thickness=%.3f m", (int)numPoints, thickness);
        
        if (numPoints < 3) {
            Log("[RoadHelper] ERROR: Need at least 3 points to create Morph");
            return false;
        }
        
        // Use first point as reference (including Z coordinate)
        const double refX = points[0].x;
        const double refY = points[0].y;
        const double refZ = points[0].z;
        
        // Create Morph element
        API_Element element = {};
        element.header.type = API_MorphID;
        
        GSErrCode err = ACAPI_Element_GetDefaults(&element, nullptr);
        if (err != NoError) {
            Log("[RoadHelper] ERROR: ACAPI_Element_GetDefaults failed, err=%d", (int)err);
            return false;
        }
        
        Log("[RoadHelper] Morph defaults obtained, floorInd=%d", (int)element.header.floorInd);
        
        // Use all points to create Morph with real Z coordinates from mesh
        Log("[RoadHelper] Creating Morph from %d points with varying Z coordinates (refZ=%.3f m)", (int)numPoints, refZ);
        
        // Setup transformation matrix
        double* tmx = element.morph.tranmat.tmx;
        tmx[ 0] = 1.0;  tmx[ 4] = 0.0;  tmx[ 8] = 0.0;
        tmx[ 1] = 0.0;  tmx[ 5] = 1.0;  tmx[ 9] = 0.0;
        tmx[ 2] = 0.0;  tmx[ 6] = 0.0;  tmx[10] = 1.0;
        // Use first point as reference for positioning (including Z)
        tmx[ 3] = refX;  tmx[ 7] = refY;  tmx[11] = refZ;
        
        // Create body structure via ACAPI_Body_Create
        void* bodyData = nullptr;
        GSErrCode bodyErr = ACAPI_Body_Create(nullptr, nullptr, &bodyData);
        if (bodyErr != NoError || bodyData == nullptr) {
            Log("[RoadHelper] ERROR: ACAPI_Body_Create failed, err=%d", (int)bodyErr);
            return false;
        }
        
        Log("[RoadHelper] Body created, adding vertices...");
        
        // Determine number of vertices based on thickness
        const bool hasThickness = (thickness > 1e-9);
        const UIndex totalVertices = hasThickness ? (numPoints * 2) : numPoints;
        
        // Add vertices (relative to reference point, including Z)
        GS::Array<UInt32> vertexIndices;
        vertexIndices.SetSize(totalVertices);
        
        // Add top vertices (always)
        for (UIndex i = 0; i < numPoints; ++i) {
            // Coordinates relative to first point (including Z)
            API_Coord3D coord = {
                points[i].x - refX,
                points[i].y - refY,
                points[i].z - refZ  // Z relative to reference Z from mesh
            };
            
            UInt32 vertexIndex;
            GSErrCode vertErr = ACAPI_Body_AddVertex(bodyData, coord, vertexIndex);
            if (vertErr != NoError) {
                Log("[RoadHelper] ERROR: ACAPI_Body_AddVertex failed for top vertex %d, err=%d", (int)i, (int)vertErr);
                ACAPI_Body_Dispose(&bodyData);
                return false;
            }
            vertexIndices[i] = vertexIndex;
            // Не логируем каждый vertex для производительности - только ошибки
        }
        
        // Add bottom vertices if thickness > 0
        if (hasThickness) {
            for (UIndex i = 0; i < numPoints; ++i) {
                // Bottom vertices: same XY, but Z shifted down by thickness
                API_Coord3D coord = {
                    points[i].x - refX,
                    points[i].y - refY,
                    points[i].z - refZ - thickness  // Z shifted down
                };
                
                UInt32 vertexIndex;
                GSErrCode vertErr = ACAPI_Body_AddVertex(bodyData, coord, vertexIndex);
                if (vertErr != NoError) {
                    Log("[RoadHelper] ERROR: ACAPI_Body_AddVertex failed for bottom vertex %d, err=%d", (int)i, (int)vertErr);
                    ACAPI_Body_Dispose(&bodyData);
                    return false;
                }
                vertexIndices[numPoints + i] = vertexIndex;
                // Не логируем каждый vertex для производительности - только ошибки
            }
            Log("[RoadHelper] Added %d top vertices and %d bottom vertices (total: %d)", 
                (int)numPoints, (int)numPoints, (int)totalVertices);
        } else {
            Log("[RoadHelper] Added %d top vertices (flat surface)", (int)numPoints);
        }
        
        Log("[RoadHelper] Vertices added, creating edges and triangulating...");
        
        // The points are arranged as: left contour (0..n/2-1), then right contour in reverse (n/2..n-1)
        // Triangulation scheme: for each segment i:
        //   Triangle 1: L_i, R_i, L_{i+1}
        //   Triangle 2: L_{i+1}, R_i, R_{i+1}
        const UIndex numLeftPoints = numPoints / 2;
        const UIndex numSegments = numLeftPoints - 1; // segments between left points
        const UIndex numTriangles = numSegments * 2; // 2 triangles per segment
        
        Log("[RoadHelper] Triangulating %d points (%d left + %d right) into %d triangles (2 per segment)", 
            (int)numPoints, (int)numLeftPoints, (int)numLeftPoints, (int)numTriangles);
        
        // Add polygon normal - we'll compute it per triangle
        Int32 polyNormalIndex;
        API_Vector3D normal = {0.0, 0.0, 1.0}; // default normal
        GSErrCode normErr = ACAPI_Body_AddPolyNormal(bodyData, normal, polyNormalIndex);
        if (normErr != NoError) {
            Log("[RoadHelper] ERROR: ACAPI_Body_AddPolyNormal failed, err=%d", (int)normErr);
            ACAPI_Body_Dispose(&bodyData);
            return false;
        }
        
        // Materials for different faces (passed as parameters)
        API_OverriddenAttribute materialTopAttr;
        materialTopAttr = materialTop;
        
        API_OverriddenAttribute materialBottomAttr;
        materialBottomAttr = materialBottom;
        
        API_OverriddenAttribute materialSideAttr;
        materialSideAttr = materialSide;
        
        // Helper function to create a triangle (defined before use for both top and bottom surfaces)
        // materialParam: material to use for this triangle
        auto CreateTriangle = [&](UIndex v0, UIndex v1, UIndex v2, const API_Coord3D& p0_abs, const API_Coord3D& p1_abs, const API_Coord3D& p2_abs, int triNum, const API_OverriddenAttribute& materialParam) -> bool {
                // Create edges for this triangle
                Int32 edge01, edge12, edge20;
                GSErrCode edgeErr;
                
                edgeErr = ACAPI_Body_AddEdge(bodyData, v0, v1, edge01);
                if (edgeErr != NoError) {
                    Log("[RoadHelper] ERROR: ACAPI_Body_AddEdge failed for triangle %d, edge %d-%d, err=%d", 
                        triNum, (int)v0, (int)v1, (int)edgeErr);
                    return false;
                }
                
                edgeErr = ACAPI_Body_AddEdge(bodyData, v1, v2, edge12);
                if (edgeErr != NoError) {
                    Log("[RoadHelper] ERROR: ACAPI_Body_AddEdge failed for triangle %d, edge %d-%d, err=%d", 
                        triNum, (int)v1, (int)v2, (int)edgeErr);
                    return false;
                }
                
                edgeErr = ACAPI_Body_AddEdge(bodyData, v2, v0, edge20);
                if (edgeErr != NoError) {
                    Log("[RoadHelper] ERROR: ACAPI_Body_AddEdge failed for triangle %d, edge %d-%d, err=%d", 
                        triNum, (int)v2, (int)v0, (int)edgeErr);
                    return false;
                }
                
                // Compute normal for this triangle (from absolute coordinates)
                API_Coord3D p0 = {p0_abs.x - refX, p0_abs.y - refY, p0_abs.z - refZ};
                API_Coord3D p1 = {p1_abs.x - refX, p1_abs.y - refY, p1_abs.z - refZ};
                API_Coord3D p2 = {p2_abs.x - refX, p2_abs.y - refY, p2_abs.z - refZ};
                
                // Compute cross product for normal
                double v1x = p1.x - p0.x, v1y = p1.y - p0.y, v1z = p1.z - p0.z;
                double v2x = p2.x - p0.x, v2y = p2.y - p0.y, v2z = p2.z - p0.z;
                
                API_Vector3D triNormal;
                triNormal.x = v1y * v2z - v1z * v2y;
                triNormal.y = v1z * v2x - v1x * v2z;
                triNormal.z = v1x * v2y - v1y * v2x;
                
                // Normalize
                double len = std::sqrt(triNormal.x * triNormal.x + triNormal.y * triNormal.y + triNormal.z * triNormal.z);
                if (len > 1e-9) {
                    triNormal.x /= len;
                    triNormal.y /= len;
                    triNormal.z /= len;
                } else {
                    triNormal = normal; // fallback to default
                }
                
                // Add normal for this triangle
                Int32 triNormalIndex;
                GSErrCode normErr2 = ACAPI_Body_AddPolyNormal(bodyData, triNormal, triNormalIndex);
                if (normErr2 != NoError) {
                    triNormalIndex = polyNormalIndex; // fallback
                }
                
                // Create polygon (triangle) from edges
                GS::Array<Int32> triEdges;
                triEdges.Push(edge01);
                triEdges.Push(edge12);
                triEdges.Push(edge20);
                
                // Material will be passed as parameter (top, bottom, or side)
                // Default to top material for now, will be overridden in calls
                UInt32 triPolyIndex;
                GSErrCode polyErr = ACAPI_Body_AddPolygon(
                    bodyData,
                    triEdges,
                    triNormalIndex,
                    materialParam, // Use passed material
                    triPolyIndex
                );
                
                if (polyErr != NoError) {
                    Log("[RoadHelper] ERROR: ACAPI_Body_AddPolygon failed for triangle %d, err=%d", 
                        triNum, (int)polyErr);
                    return false;
                }
                
                return true;
        };
        
        // Create triangles between left and right contours (top surface)
        for (UIndex i = 0; i < numSegments; ++i) {
            // Indices:
            // L_i = i (left point i)
            // L_{i+1} = i+1 (left point i+1)
            // R_i = numLeftPoints + (numLeftPoints - 1 - i) = 2*numLeftPoints - 1 - i (right point i, in reverse order)
            // R_{i+1} = 2*numLeftPoints - 1 - (i+1) = 2*numLeftPoints - 2 - i (right point i+1, in reverse order)
            
            UIndex left_i = i;
            UIndex left_i1 = i + 1;
            UIndex right_i = 2 * numLeftPoints - 1 - i;
            UIndex right_i1 = 2 * numLeftPoints - 2 - i;
            
            // Triangle 1: L_i, R_i, L_{i+1}
            UIndex v0_tri1 = vertexIndices[left_i];
            UIndex v1_tri1 = vertexIndices[right_i];
            UIndex v2_tri1 = vertexIndices[left_i1];
            
            // Triangle 2: L_{i+1}, R_i, R_{i+1}
            UIndex v0_tri2 = vertexIndices[left_i1];
            UIndex v1_tri2 = vertexIndices[right_i];
            UIndex v2_tri2 = vertexIndices[right_i1];
            
            // Create Triangle 1: L_i, R_i, L_{i+1} (top surface)
            if (!CreateTriangle(v0_tri1, v1_tri1, v2_tri1, 
                                points[left_i], points[right_i], points[left_i1],
                                (int)(i * 2), materialTopAttr)) {
                ACAPI_Body_Dispose(&bodyData);
                return false;
            }
            
            // Create Triangle 2: L_{i+1}, R_i, R_{i+1} (top surface)
            if (!CreateTriangle(v0_tri2, v1_tri2, v2_tri2,
                                points[left_i1], points[right_i], points[right_i1],
                                (int)(i * 2 + 1), materialTopAttr)) {
                ACAPI_Body_Dispose(&bodyData);
                return false;
            }
        }
        
        Log("[RoadHelper] Top surface: %d triangles added", (int)numTriangles);
        
        // Create bottom surface if thickness > 0
        if (hasThickness) {
            Log("[RoadHelper] Creating bottom surface...");
            
            // Bottom surface: same triangles but with bottom vertices and reversed order for correct normals
            for (UIndex i = 0; i < numSegments; ++i) {
                UIndex left_i = i;
                UIndex left_i1 = i + 1;
                UIndex right_i = 2 * numLeftPoints - 1 - i;
                UIndex right_i1 = 2 * numLeftPoints - 2 - i;
                
                // Bottom vertices indices (offset by numPoints)
                UIndex bottom_left_i = numPoints + left_i;
                UIndex bottom_left_i1 = numPoints + left_i1;
                UIndex bottom_right_i = numPoints + right_i;
                UIndex bottom_right_i1 = numPoints + right_i1;
                
                // Bottom Triangle 1: L_{i+1}, R_i, L_i (reversed order for downward normal)
                API_Coord3D p0_bot = {points[left_i1].x - refX, points[left_i1].y - refY, points[left_i1].z - refZ - thickness};
                API_Coord3D p1_bot = {points[right_i].x - refX, points[right_i].y - refY, points[right_i].z - refZ - thickness};
                API_Coord3D p2_bot = {points[left_i].x - refX, points[left_i].y - refY, points[left_i].z - refZ - thickness};
                
                if (!CreateTriangle(vertexIndices[bottom_left_i1], vertexIndices[bottom_right_i], vertexIndices[bottom_left_i],
                                    p0_bot, p1_bot, p2_bot, (int)(numTriangles + i * 2), materialBottomAttr)) {
                    ACAPI_Body_Dispose(&bodyData);
                    return false;
                }
                
                // Bottom Triangle 2: R_{i+1}, R_i, L_{i+1} (reversed order)
                API_Coord3D p3_bot = {points[right_i1].x - refX, points[right_i1].y - refY, points[right_i1].z - refZ - thickness};
                
                if (!CreateTriangle(vertexIndices[bottom_right_i1], vertexIndices[bottom_right_i], vertexIndices[bottom_left_i1],
                                    p3_bot, p1_bot, p0_bot, (int)(numTriangles + i * 2 + 1), materialBottomAttr)) {
                    ACAPI_Body_Dispose(&bodyData);
                    return false;
                }
            }
            Log("[RoadHelper] Bottom surface: %d triangles added", (int)numTriangles);
            
            // Create side faces (left, right, front, back)
            Log("[RoadHelper] Creating side faces...");
            const UIndex numSideTriangles = numSegments * 4 + 4; // 2 per segment on each side + 2 for front + 2 for back
            
            // Left side faces (connecting left contour top to bottom)
            for (UIndex i = 0; i < numSegments; ++i) {
                UIndex left_i = i;
                UIndex left_i1 = i + 1;
                UIndex bottom_left_i = numPoints + left_i;
                UIndex bottom_left_i1 = numPoints + left_i1;
                
                // Side triangle 1: L_i_top, L_i_bottom, L_{i+1}_bottom
                API_Coord3D p_top = {points[left_i].x - refX, points[left_i].y - refY, points[left_i].z - refZ};
                API_Coord3D p_bot = {points[left_i].x - refX, points[left_i].y - refY, points[left_i].z - refZ - thickness};
                API_Coord3D p_bot_next = {points[left_i1].x - refX, points[left_i1].y - refY, points[left_i1].z - refZ - thickness};
                
                if (!CreateTriangle(vertexIndices[left_i], vertexIndices[bottom_left_i], vertexIndices[bottom_left_i1],
                                    p_top, p_bot, p_bot_next, (int)(numTriangles * 2 + i * 2), materialSideAttr)) {
                    ACAPI_Body_Dispose(&bodyData);
                    return false;
                }
                
                // Side triangle 2: L_i_top, L_{i+1}_bottom, L_{i+1}_top
                API_Coord3D p_top_next = {points[left_i1].x - refX, points[left_i1].y - refY, points[left_i1].z - refZ};
                
                if (!CreateTriangle(vertexIndices[left_i], vertexIndices[bottom_left_i1], vertexIndices[left_i1],
                                    p_top, p_bot_next, p_top_next, (int)(numTriangles * 2 + i * 2 + 1), materialSideAttr)) {
                    ACAPI_Body_Dispose(&bodyData);
                    return false;
                }
            }
            
            // Right side faces (connecting right contour top to bottom, in reverse order)
            for (UIndex i = 0; i < numSegments; ++i) {
                UIndex right_i = 2 * numLeftPoints - 1 - i;
                UIndex right_i1 = 2 * numLeftPoints - 2 - i;
                UIndex bottom_right_i = numPoints + right_i;
                UIndex bottom_right_i1 = numPoints + right_i1;
                
                // Side triangle 1: R_i_top, R_i_bottom, R_{i+1}_bottom - material 3
                API_Coord3D p_top = {points[right_i].x - refX, points[right_i].y - refY, points[right_i].z - refZ};
                API_Coord3D p_bot = {points[right_i].x - refX, points[right_i].y - refY, points[right_i].z - refZ - thickness};
                API_Coord3D p_bot_next = {points[right_i1].x - refX, points[right_i1].y - refY, points[right_i1].z - refZ - thickness};
                
                if (!CreateTriangle(vertexIndices[right_i], vertexIndices[bottom_right_i], vertexIndices[bottom_right_i1],
                                    p_top, p_bot, p_bot_next, (int)(numTriangles * 2 + numSegments * 2 + i * 2), materialSideAttr)) {
                    ACAPI_Body_Dispose(&bodyData);
                    return false;
                }
                
                // Side triangle 2: R_i_top, R_{i+1}_bottom, R_{i+1}_top
                API_Coord3D p_top_next = {points[right_i1].x - refX, points[right_i1].y - refY, points[right_i1].z - refZ};
                
                if (!CreateTriangle(vertexIndices[right_i], vertexIndices[bottom_right_i1], vertexIndices[right_i1],
                                    p_top, p_bot_next, p_top_next, (int)(numTriangles * 2 + numSegments * 2 + i * 2 + 1), materialSideAttr)) {
                    ACAPI_Body_Dispose(&bodyData);
                    return false;
                }
            }
            
            // Front side (connecting first left and first right points)
            UIndex left_first = 0;
            UIndex right_first = 2 * numLeftPoints - 1;
            UIndex bottom_left_first = numPoints + left_first;
            UIndex bottom_right_first = numPoints + right_first;
            
            // Front triangle 1: L_0_top, L_0_bottom, R_0_bottom - material 3
            API_Coord3D p_left_top = {points[left_first].x - refX, points[left_first].y - refY, points[left_first].z - refZ};
            API_Coord3D p_left_bot = {points[left_first].x - refX, points[left_first].y - refY, points[left_first].z - refZ - thickness};
            API_Coord3D p_right_bot = {points[right_first].x - refX, points[right_first].y - refY, points[right_first].z - refZ - thickness};
            
            if (!CreateTriangle(vertexIndices[left_first], vertexIndices[bottom_left_first], vertexIndices[bottom_right_first],
                                p_left_top, p_left_bot, p_right_bot, (int)(numTriangles * 2 + numSegments * 4), materialSideAttr)) {
                ACAPI_Body_Dispose(&bodyData);
                return false;
            }
            
            // Front triangle 2: L_0_top, R_0_bottom, R_0_top
            API_Coord3D p_right_top = {points[right_first].x - refX, points[right_first].y - refY, points[right_first].z - refZ};
            
            if (!CreateTriangle(vertexIndices[left_first], vertexIndices[bottom_right_first], vertexIndices[right_first],
                                p_left_top, p_right_bot, p_right_top, (int)(numTriangles * 2 + numSegments * 4 + 1), materialSideAttr)) {
                ACAPI_Body_Dispose(&bodyData);
                return false;
            }
            
            // Back side (connecting last left and last right points)
            UIndex left_last = numLeftPoints - 1;
            UIndex right_last = numLeftPoints; // first point of right contour in our array
            UIndex bottom_left_last = numPoints + left_last;
            UIndex bottom_right_last = numPoints + right_last;
            
            // Back triangle 1: L_last_top, L_last_bottom, R_last_bottom - material 3
            API_Coord3D p_left_top_back = {points[left_last].x - refX, points[left_last].y - refY, points[left_last].z - refZ};
            API_Coord3D p_left_bot_back = {points[left_last].x - refX, points[left_last].y - refY, points[left_last].z - refZ - thickness};
            API_Coord3D p_right_bot_back = {points[right_last].x - refX, points[right_last].y - refY, points[right_last].z - refZ - thickness};
            
            if (!CreateTriangle(vertexIndices[left_last], vertexIndices[bottom_left_last], vertexIndices[bottom_right_last],
                                p_left_top_back, p_left_bot_back, p_right_bot_back, (int)(numTriangles * 2 + numSegments * 4 + 2), materialSideAttr)) {
                ACAPI_Body_Dispose(&bodyData);
                return false;
            }
            
            // Back triangle 2: L_last_top, R_last_bottom, R_last_top
            API_Coord3D p_right_top_back = {points[right_last].x - refX, points[right_last].y - refY, points[right_last].z - refZ};
            
            if (!CreateTriangle(vertexIndices[left_last], vertexIndices[bottom_right_last], vertexIndices[right_last],
                                p_left_top_back, p_right_bot_back, p_right_top_back, (int)(numTriangles * 2 + numSegments * 4 + 3), materialSideAttr)) {
                ACAPI_Body_Dispose(&bodyData);
                return false;
            }
            
            Log("[RoadHelper] Side faces: %d triangles added (left: %d, right: %d, front: 2, back: 2)", 
                (int)numSideTriangles, (int)(numSegments * 2), (int)(numSegments * 2));
        }
        
        const UIndex totalTriangles = hasThickness ? (numTriangles * 2 + numSegments * 4 + 4) : numTriangles;
        Log("[RoadHelper] Total %d triangles added, finishing body...", (int)totalTriangles);
        
        // Finish body and copy to memo
        API_ElementMemo memo = {};
        BNZeroMemory(&memo, sizeof(API_ElementMemo));
        
        GSErrCode finishErr = ACAPI_Body_Finish(bodyData, &memo.morphBody, &memo.morphMaterialMapTable);
        ACAPI_Body_Dispose(&bodyData);
        
        if (finishErr != NoError) {
            Log("[RoadHelper] ERROR: ACAPI_Body_Finish failed, err=%d", (int)finishErr);
            ACAPI_DisposeElemMemoHdls(&memo);
            return false;
        }
        
        Log("[RoadHelper] Body finished, creating Morph element...");
        
        // Create Morph element
        GSErrCode createErr = ACAPI_Element_Create(&element, &memo);
        
        Log("[RoadHelper] ACAPI_Element_Create returned err=%d", (int)createErr);
        
        if (createErr != NoError) {
            Log("[RoadHelper] ERROR: Failed to create Morph, err=%d", (int)createErr);
            ACAPI_DisposeElemMemoHdls(&memo);
            return false;
        }
            
        ACAPI_DisposeElemMemoHdls(&memo);
        
        Log("[RoadHelper] SUCCESS: Morph created from %d points with varying Z coordinates (refZ=%.3f m)", 
            (int)numPoints, refZ);
        return true;
    }
    
    // Вычисление площади 3D треугольника через векторное произведение
    static double CalculateTriangleArea3D(const API_Coord3D& a, const API_Coord3D& b, const API_Coord3D& c)
    {
        // Векторы от a к b и от a к c
        double v1x = b.x - a.x, v1y = b.y - a.y, v1z = b.z - a.z;
        double v2x = c.x - a.x, v2y = c.y - a.y, v2z = c.z - a.z;
        
        // Векторное произведение v1 × v2
        double crossX = v1y * v2z - v1z * v2y;
        double crossY = v1z * v2x - v1x * v2z;
        double crossZ = v1x * v2y - v1y * v2x;
        
        // Длина вектора = площадь параллелограмма, делим на 2 для площади треугольника
        double area = 0.5 * std::sqrt(crossX * crossX + crossY * crossY + crossZ * crossZ);
        return area;
    }
    
    // Вычисление площади верхней поверхности Morph
    double CalculateMorphSurfaceArea(const GS::Array<API_Coord3D>& points)
    {
        const UIndex numPoints = points.GetSize();
        if (numPoints < 3) {
            Log("[RoadHelper] ERROR: Need at least 3 points to calculate area");
            return 0.0;
        }
        
        // Схема триангуляции та же, что при создании Morph
        const UIndex numLeftPoints = numPoints / 2;
        const UIndex numSegments = numLeftPoints - 1;
        
        double totalArea = 0.0;
        
        for (UIndex i = 0; i < numSegments; ++i) {
            UIndex left_i = i;
            UIndex left_i1 = i + 1;
            UIndex right_i = 2 * numLeftPoints - 1 - i;
            UIndex right_i1 = 2 * numLeftPoints - 2 - i;
            
            // Triangle 1: L_i, R_i, L_{i+1}
            double area1 = CalculateTriangleArea3D(
                points[left_i],
                points[right_i],
                points[left_i1]
            );
            
            // Triangle 2: L_{i+1}, R_i, R_{i+1}
            double area2 = CalculateTriangleArea3D(
                points[left_i1],
                points[right_i],
                points[right_i1]
            );
            
            totalArea += area1 + area2;
        }
        
        Log("[RoadHelper] Calculated surface area: %.3f m² (%d triangles)", totalArea, (int)(numSegments * 2));
        return totalArea;
    }
    
    // Создание текстовой выноски с площадью
    bool CreateAreaLabel(const API_Coord& position, double areaM2)
    {
        Log("[RoadHelper] CreateAreaLabel: creating text label at (%.3f, %.3f) with area %.3f m2", 
            position.x, position.y, areaM2);
        
        API_Element element = {};
        element.header.type = API_TextID;
        
        GSErrCode err = ACAPI_Element_GetDefaults(&element, nullptr);
        if (err != NoError) {
            Log("[RoadHelper] ERROR: ACAPI_Element_GetDefaults failed for text, err=%d", (int)err);
            return false;
        }
        
        // Устанавливаем позицию и параметры текста
        element.text.loc = position;
        element.text.anchor = APIAnc_LB; // Left Bottom - горизонтальная ориентация
        element.text.width = 100.0; // Фиксированная ширина для горизонтального текста
        element.text.nonBreaking = true; // Без переноса строк
        
        // Формируем текст с площадью
        char textBuf[256];
        snprintf(textBuf, sizeof(textBuf), "S = %.2f m2", areaM2);
        
        API_ElementMemo memo = {};
        BNZeroMemory(&memo, sizeof(API_ElementMemo));
        
        // В AC27 textContent это char** - нужно выделить память через BMAllocateHandle
        GS::UniString textStr(textBuf);
        Int32 textLen = textStr.GetLength() + 1;
        memo.textContent = reinterpret_cast<char**>(BMAllocateHandle(textLen * sizeof(char), ALLOCATE_CLEAR, 0));
        if (memo.textContent != nullptr) {
            strcpy_s(*memo.textContent, textLen, textStr.ToCStr().Get());
        }
        
        // Создаем текстовый элемент
        err = ACAPI_CallUndoableCommand("Create Area Label", [&]() -> GSErrCode {
            return ACAPI_Element_Create(&element, &memo);
        });
        
        ACAPI_DisposeElemMemoHdls(&memo);
        
        if (err != NoError) {
            Log("[RoadHelper] ERROR: Failed to create text element, err=%d", (int)err);
            return false;
        }
        
        Log("[RoadHelper] SUCCESS: Area label created: %s", textBuf);
        return true;
    }
    
    // Создание Morph из произвольного количества точек (публичная функция)
    bool CreateMorphFromPoints(const GS::Array<API_Coord3D>& points, double thicknessMM,
                                API_AttributeIndex materialTop, API_AttributeIndex materialBottom, API_AttributeIndex materialSide)
    {
        return ACAPI_CallUndoableCommand("Create Morph from Points", [&]() -> GSErrCode {
            return CreateMorphFromPointsInternal(points, thicknessMM, materialTop, materialBottom, materialSide) ? NoError : APIERR_GENERAL;
        }) == NoError;
    }
    
    // Кэш для списка покрытий (чтобы не загружать каждый раз)
    static GS::Array<SurfaceFinishInfo> g_cachedFinishes;
    static bool g_finishesCacheValid = false;
    
    // Получить список всех доступных покрытий (материалы - они используются как покрытия/текстуры)
    GS::Array<SurfaceFinishInfo> GetSurfaceFinishesList()
    {
        // Возвращаем кэш, если он валиден
        if (g_finishesCacheValid && g_cachedFinishes.GetSize() > 0) {
            return g_cachedFinishes;
        }
        
        // Очищаем кэш и загружаем заново
        g_cachedFinishes.Clear();
        
        GS::Array<API_Attribute> attributes;
        GSErrCode err = ACAPI_Attribute_GetAttributesByType(API_MaterialID, attributes);
        
        if (err == NoError) {
            for (UIndex i = 0; i < attributes.GetSize(); ++i) {
                SurfaceFinishInfo info;
                // Получаем числовой индекс из API_AttributeIndex
                // Используем порядковый номер + 1, так как индексы начинаются с 1
                info.index = (Int32)(i + 1);
                info.name = attributes[i].header.name;
                g_cachedFinishes.Push(info);
            }
            g_finishesCacheValid = true;
        }
        
        return g_cachedFinishes;
    }
    
    // Сбросить кэш покрытий (вызывать при изменении материалов в проекте)
    void InvalidateSurfaceFinishesCache()
    {
        g_finishesCacheValid = false;
        g_cachedFinishes.Clear();
    }

} // namespace RoadHelper
