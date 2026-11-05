#pragma once

#include "passive_flight_c_api/PassiveFlightCApi.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Общие входные параметры для расчётов
 * линейного продольного возмущённого движения.
 *
 * Номинальный режим:
 *
 * - objectId;
 * - releaseAltitudeM;
 * - releaseSpeedMps.
 *
 * Начальный вектор малых возмущений:
 *
 * Delta x0 =
 * [Delta V,
 *  Delta Theta,
 *  Delta omega_z,
 *  Delta theta,
 *  Delta x,
 *  Delta H]^T.
 *
 * Угол атаки не является независимым состоянием:
 *
 * Delta alpha =
 *     Delta theta - Delta Theta.
 */
typedef struct PFPerturbedSimulationInput {
    /* Стабильный идентификатор объекта: ненулевая UTF-8-строка с '\0'. */
    const char* objectId;

    /* Геометрическая высота горизонтального сброса, м. */
    double releaseAltitudeM;
    /* Модуль скорости в момент сброса, м/с. */
    double releaseSpeedMps;

    /* Начальное отклонение скорости `Delta V`, м/с. */
    double deltaSpeedMps;
    /* Начальное отклонение угла траектории `Delta Theta`, рад. */
    double deltaFlightPathAngleRad;
    /* Начальное отклонение угловой скорости `Delta omega_z`, рад/с. */
    double deltaPitchRateRadps;
    /* Начальное отклонение угла тангажа `Delta theta`, рад. */
    double deltaPitchAngleRad;
    /* Начальное отклонение горизонтальной координаты `Delta x`, м. */
    double deltaDownrangeM;
    /* Начальное отклонение геометрической высоты `Delta H`, м. */
    double deltaAltitudeM;
} PFPerturbedSimulationInput;

#ifdef __cplusplus
}
#endif
