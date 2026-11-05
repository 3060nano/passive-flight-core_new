#pragma once

#include "passive_flight_c_api/PerturbedFlightTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Одна точка совместной траектории:
 *
 * 1) nominal* — невозмущённое состояние x*(t);
 * 2) delta*   — малое возмущение Delta x(t);
 * 3) total*   — сумма x*(t) + Delta x(t).
 *
 * Угол атаки вычисляется как:
 *
 * alpha = theta - Theta.
 */
typedef struct PFPerturbedTrajectoryPoint {
    /* Время от момента сброса, общее для трёх состояний, с. */
    double timeS;

    /* Номинальная горизонтальная координата `x*(t)`, м. */
    double nominalDownrangeM;
    /* Номинальная геометрическая высота `H*(t)`, м. */
    double nominalAltitudeM;
    /* Номинальная скорость `V*(t)`, м/с. */
    double nominalSpeedMps;
    /* Номинальный угол траектории `Theta*(t)`, рад. */
    double nominalFlightPathAngleRad;
    /* Номинальная угловая скорость тангажа `omega_z*(t)`, рад/с. */
    double nominalPitchRateRadps;
    /* Номинальный угол тангажа `theta*(t)`, рад. */
    double nominalPitchAngleRad;
    /* Номинальный угол атаки `theta* - Theta*`, рад. */
    double nominalAngleOfAttackRad;

    /* Малое отклонение горизонтальной координаты `Delta x`, м. */
    double deltaDownrangeM;
    /* Малое отклонение геометрической высоты `Delta H`, м. */
    double deltaAltitudeM;
    /* Малое отклонение скорости `Delta V`, м/с. */
    double deltaSpeedMps;
    /* Малое отклонение угла траектории `Delta Theta`, рад. */
    double deltaFlightPathAngleRad;
    /* Малое отклонение угловой скорости `Delta omega_z`, рад/с. */
    double deltaPitchRateRadps;
    /* Малое отклонение угла тангажа `Delta theta`, рад. */
    double deltaPitchAngleRad;
    /* Малое отклонение угла атаки `Delta theta - Delta Theta`, рад. */
    double deltaAngleOfAttackRad;

    /* Линейная оценка полной горизонтальной координаты, м. */
    double totalDownrangeM;
    /* Линейная оценка полной геометрической высоты, м. */
    double totalAltitudeM;
    /* Линейная оценка полной скорости, м/с. */
    double totalSpeedMps;
    /* Линейная оценка полного угла траектории, рад. */
    double totalFlightPathAngleRad;
    /* Линейная оценка полной угловой скорости тангажа, рад/с. */
    double totalPitchRateRadps;
    /* Линейная оценка полного угла тангажа, рад. */
    double totalPitchAngleRad;
    /* Линейная оценка полного угла атаки, рад. */
    double totalAngleOfAttackRad;
} PFPerturbedTrajectoryPoint;

/*
 * Возвращает совместную историю невозмущённого
 * и линейного возмущённого движения.
 *
 * Рекомендуемый двухпроходный вызов:
 *
 * 1)
 * points = NULL;
 * pointCapacity = 0.
 *
 * Функция заполнит requiredPointCount
 * и вернёт PF_RESULT_BUFFER_TOO_SMALL.
 *
 * 2)
 * Выделить points[requiredPointCount]
 * и вызвать функцию повторно.
 *
 * Внутренний шаг интегрирования остаётся 0.001 с.
 * Во внешний массив сохраняется точка каждые 0.01 с
 * плюс обязательная конечная точка.
 *
 * input содержит условия и начальные возмущения в СИ;
 * points — выходной массив либо NULL при запросе размера;
 * pointCapacity — ёмкость массива в точках;
 * requiredPointCount и writtenPointCount — обязательные выходные счётчики.
 * Возвращает PFResultCode.
 */
PF_API int32_t PF_CALL
pfCalculatePerturbedTrajectory(
    const PFPerturbedSimulationInput* input,

    PFPerturbedTrajectoryPoint* points,
    uint64_t pointCapacity,

    uint64_t* requiredPointCount,
    uint64_t* writtenPointCount
);

#ifdef __cplusplus
}
#endif
