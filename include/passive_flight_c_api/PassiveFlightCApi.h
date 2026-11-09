#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
    #if defined(PF_C_API_EXPORTS)
        #define PF_API __declspec(dllexport)
    #else
        #define PF_API __declspec(dllimport)
    #endif

    #define PF_CALL __cdecl
#else
    #define PF_API __attribute__((visibility("default")))
    #define PF_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Коды завершения функций C API.
 */
typedef enum PFResultCode {
    /* Операция успешно завершена. */
    PF_RESULT_OK = 0,

    /* Обязательный указатель равен NULL. */
    PF_RESULT_NULL_ARGUMENT = 1,
    /* Входное числовое значение или строка некорректны. */
    PF_RESULT_INVALID_INPUT = 2,
    /* Объект с указанным идентификатором не зарегистрирован. */
    PF_RESULT_OBJECT_NOT_FOUND = 3,
    /* Переданный выходной буфер имеет недостаточную ёмкость. */
    PF_RESULT_BUFFER_TOO_SMALL = 4,
    /* Индекс объекта или элемента находится вне допустимого диапазона. */
    PF_RESULT_INDEX_OUT_OF_RANGE = 5,
    /* Численное моделирование завершилось без требуемого физического результата. */
    PF_RESULT_SIMULATION_FAILED = 6,
    /* Во внутренней реализации возникло непредвиденное исключение. */
    PF_RESULT_INTERNAL_ERROR = 7
} PFResultCode;

/*
 * Причины завершения моделирования.
 *
 * Значения не зависят от внутреннего C++ enum.
 */
typedef enum PFTerminationReason {
    /* Объект достиг поверхности. */
    PF_TERMINATION_GROUND_REACHED = 0,
    /* Достигнуто предельное время интегрирования. */
    PF_TERMINATION_MAXIMUM_TIME_REACHED = 1,
    /* Достигнуто предельное число шагов интегрирования. */
    PF_TERMINATION_MAXIMUM_STEPS_REACHED = 2,
    /* Исходные данные моделирования некорректны. */
    PF_TERMINATION_INVALID_INPUT = 3,
    /* Во время интегрирования получено недопустимое состояние. */
    PF_TERMINATION_INVALID_STATE = 4
} PFTerminationReason;

/*
 * Входные параметры расчёта.
 *
 * objectId должен указывать на строку UTF-8,
 * завершённую нулевым символом.
 */
typedef struct PFSimulationInput {
    /* Стабильный идентификатор объекта: ненулевая UTF-8-строка с '\0'. */
    const char* objectId;

    /* Геометрическая высота горизонтального сброса, м. */
    double releaseAltitudeM;
    /* Модуль скорости объекта в момент сброса, м/с. */
    double releaseSpeedMps;
} PFSimulationInput;

/*
 * Краткий результат расчёта.
 */
typedef struct PFSimulationOutput {
    /* Горизонтальная дальность от точки сброса до конечного состояния, м. */
    double downrangeM;
    /* Время от сброса до конечного состояния, с. */
    double fallTimeS;

    /* Модуль скорости в конечном состоянии, м/с. */
    double impactSpeedMps;
    /* Угол наклона траектории при падении, рад. */
    double impactFlightPathAngleRad;
    /* Угол тангажа при падении, рад. */
    double impactPitchAngleRad;
    /* Угол атаки при падении `alpha = theta - Theta`, рад. */
    double impactAngleOfAttackRad;

    /* Значение PFTerminationReason, представленное 32-битным целым числом. */
    int32_t terminationReason;
} PFSimulationOutput;

/*
 * Одна точка траектории.
 */
typedef struct PFTrajectoryPoint {
    /* Время от момента сброса, с. */
    double timeS;

    /* Горизонтальная координата относительно точки сброса, м. */
    double downrangeM;
    /* Геометрическая высота над принятой поверхностью, м. */
    double altitudeM;
    /* Модуль воздушной скорости, м/с. */
    double speedMps;

    /* Угол наклона траектории `Theta`, рад. */
    double flightPathAngleRad;
    /* Угол тангажа `theta`, рад. */
    double pitchAngleRad;
    /* Угловая скорость тангажа `omega_z`, рад/с. */
    double pitchRateRadps;
    /* Угол атаки `alpha = theta - Theta`, рад. */
    double angleOfAttackRad;

    /* Число Маха `M = V/a`, безразмерное. */
    double mach;
    /* Скоростной напор `q = rho*V²/2`, Па. */
    double dynamicPressurePa;

    /* Коэффициент лобового сопротивления `Cx`, безразмерный. */
    double cx;
    /* Коэффициент нормальной силы `Cy`, безразмерный. */
    double cy;
    /* Коэффициент продольного момента `mz`, безразмерный. */
    double mz;
} PFTrajectoryPoint;

/*
 * Возвращает версию C API.
 *
 * Указатель существует на протяжении всей работы DLL.
 * Освобождать память не нужно.
 *
 * Возвращает указатель на статическую ASCII-строку версии C API.
 */
PF_API const char* PF_CALL
pfGetApiVersion(void);

/*
 * Возвращает текстовое имя кода результата.
 *
 * Освобождать память не нужно.
 *
 * resultCode — значение PFResultCode.
 * Возвращает указатель на статическую ASCII-строку.
 */
PF_API const char* PF_CALL
pfGetResultCodeName(
    int32_t resultCode
);

/*
 * Возвращает количество зарегистрированных объектов, безразмерный счётчик.
 */
PF_API uint64_t PF_CALL
pfGetObjectCount(void);

/*
 * Получает идентификатор объекта по индексу.
 *
 * requiredSize получает необходимый размер буфера
 * с учётом завершающего нулевого символа.
 *
 * Для определения размера можно вызвать функцию
 * с buffer = NULL и bufferSize = 0.
 *
 * objectIndex — индекс объекта от 0 до pfGetObjectCount()-1;
 * buffer — выходной буфер UTF-8 либо NULL при запросе размера;
 * bufferSize — ёмкость buffer в байтах;
 * requiredSize — обязательный выход требуемого числа байтов.
 * Возвращает PFResultCode.
 */
PF_API int32_t PF_CALL
pfGetObjectId(
    uint64_t objectIndex,
    char* buffer,
    uint64_t bufferSize,
    uint64_t* requiredSize
);

/*
 * Получает отображаемое название объекта в UTF-8.
 * Параметры и возвращаемый код имеют тот же смысл, что у pfGetObjectId.
 */
PF_API int32_t PF_CALL
pfGetObjectDisplayName(
    uint64_t objectIndex,
    char* buffer,
    uint64_t bufferSize,
    uint64_t* requiredSize
);

/*
 * Выполняет расчёт без сохранения полной истории.
 * input содержит условия сброса в СИ; output получает краткий результат в СИ.
 * Возвращает PFResultCode.
 */
PF_API int32_t PF_CALL
pfCalculate(
    const PFSimulationInput* input,
    PFSimulationOutput* output
);

/*
 * Выполняет расчёт с сохранением траектории.
 *
 * Рекомендуется вызывать функцию два раза.
 *
 * Первый вызов:
 *
 * points = NULL;
 * pointCapacity = 0.
 *
 * Функция заполнит requiredPointCount.
 *
 * Второй вызов:
 *
 * передаётся массив необходимого размера.
 *
 * input — условия сброса в СИ; output — краткий результат;
 * points — выходной массив либо NULL при запросе размера;
 * pointCapacity — число элементов в points;
 * requiredPointCount — обязательное требуемое число элементов;
 * writtenPointCount — обязательное фактически записанное число элементов.
 * Возвращает PFResultCode.
 */
PF_API int32_t PF_CALL
pfCalculateTrajectory(
    const PFSimulationInput* input,
    PFSimulationOutput* output,

    PFTrajectoryPoint* points,
    uint64_t pointCapacity,

    uint64_t* requiredPointCount,
    uint64_t* writtenPointCount
);

/*
 * Упрощённый скалярный интерфейс для SimInTech.
 *
 * Входы блока:
 *
 * - objectId;
 * - releaseAltitudeM;
 * - releaseSpeedMps.
 *
 * Выходы блока:
 *
 * - относ;
 * - время падения;
 * - скорость встречи;
 * - угол наклона траектории при встрече;
 * - угол тангажа при встрече;
 * - угол атаки при встрече;
 * - причина завершения.
 *
 * Все выходные указатели обязательны.
 * Углы передаются в радианах, расстояния — в метрах, время — в секундах,
 * скорости — в м/с; функция возвращает PFResultCode.
 */
PF_API int32_t PF_CALL
pfSimInTechCalculate(
    const char* objectId,
    double releaseAltitudeM,
    double releaseSpeedMps,

    double* downrangeM,
    double* fallTimeS,

    double* impactSpeedMps,
    double* impactFlightPathAngleRad,
    double* impactPitchAngleRad,
    double* impactAngleOfAttackRad,

    int32_t* terminationReason
);

/*
 * Упрощённое получение траектории для SimInTech.
 *
 * Функция возвращает три массива:
 *
 * - timeS — время;
 * - downrangeM — горизонтальная координата;
 * - altitudeM — высота.
 *
 * Первый вызов:
 *
 * timeS = NULL;
 * downrangeM = NULL;
 * altitudeM = NULL;
 * pointCapacity = 0.
 *
 * requiredPointCount получит необходимый размер.
 *
 * Второй вызов выполняется с тремя массивами
 * длиной не менее requiredPointCount.
 *
 * Функция повторно выполняет моделирование, чтобы DLL не хранила
 * изменяемое состояние между последовательными вызовами.
 *
 * Входные расстояния заданы в метрах, скорость — в м/с; выходные массивы
 * содержат секунды и метры. pointCapacity и оба счётчика измеряются в точках.
 * Возвращает PFResultCode.
 */
PF_API int32_t PF_CALL
pfSimInTechCalculateTrajectory(
    const char* objectId,
    double releaseAltitudeM,
    double releaseSpeedMps,

    double* timeS,
    double* downrangeM,
    double* altitudeM,

    uint64_t pointCapacity,
    uint64_t* requiredPointCount,
    uint64_t* writtenPointCount
);

#ifdef __cplusplus
}
#endif
