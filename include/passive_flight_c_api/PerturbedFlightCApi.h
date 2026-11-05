#pragma once

#include "passive_flight_c_api/PerturbedFlightTypes.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Результат первого линейного приближения
 * изменения параметров события падения H = 0.
 *
 * Поля delta* относятся именно к изменению
 * параметров события падения, а не просто
 * к Delta x(t_f*) в момент падения
 * невозмущённой траектории.
 */
typedef struct PFPerturbedImpactOutput {
    /* Поправка к времени достижения поверхности `Delta t_f`, с. */
    double deltaFallTimeS;
    /* Поправка к горизонтальной дальности падения, м. */
    double deltaDownrangeM;

    /* Поправка к скорости в событии падения, м/с. */
    double deltaImpactSpeedMps;
    /* Поправка к углу наклона траектории при падении, рад. */
    double deltaImpactFlightPathAngleRad;
    /* Поправка к угловой скорости тангажа при падении, рад/с. */
    double deltaImpactPitchRateRadps;
    /* Поправка к углу тангажа при падении, рад. */
    double deltaImpactPitchAngleRad;
    /* Поправка к углу атаки при падении, рад. */
    double deltaImpactAngleOfAttackRad;

    /*
     * Полная оценка параметров возмущённого объекта
     * в его собственный момент достижения H = 0.
     */
    /* Оценённое время падения возмущённого объекта, с. */
    double perturbedImpactTimeS;
    /* Оценённая горизонтальная дальность возмущённого объекта, м. */
    double perturbedImpactDownrangeM;

    /* Оценённая скорость возмущённого объекта при падении, м/с. */
    double perturbedImpactSpeedMps;
    /* Оценённый угол траектории возмущённого объекта при падении, рад. */
    double perturbedImpactFlightPathAngleRad;
    /* Оценённая угловая скорость тангажа при падении, рад/с. */
    double perturbedImpactPitchRateRadps;
    /* Оценённый угол тангажа при падении, рад. */
    double perturbedImpactPitchAngleRad;
    /* Оценённый угол атаки при падении, рад. */
    double perturbedImpactAngleOfAttackRad;

    /* Значение PFTerminationReason для опорного расчёта. */
    int32_t terminationReason;
} PFPerturbedImpactOutput;

/*
 * Возвращает версию расширения C API для возмущённого движения.
 * Результат — указатель на статическую ASCII-строку; освобождение не требуется.
 */
PF_API const char* PF_CALL
pfGetPerturbedApiVersion(void);

/*
 * Выполняет расчёт линейного продольного
 * возмущённого движения вдоль номинальной траектории
 * и терминальную поправку первого порядка
 * к событию падения H = 0.
 *
 * Реализуются соотношения `d(Delta x)/dt = A(t)*Delta x`,
 * `Delta t_f = -Delta H_f/H_dot_f*` и
 * `Delta y_impact = Delta y_f + y_dot_f*Delta t_f`.
 * input содержит номинальные условия и начальные возмущения в СИ;
 * output получает поправки и оценённое состояние падения в СИ.
 * Возвращает PFResultCode.
 */
PF_API int32_t PF_CALL
pfCalculatePerturbedImpact(
    const PFPerturbedSimulationInput* input,
    PFPerturbedImpactOutput* output
);

#ifdef __cplusplus
}
#endif
