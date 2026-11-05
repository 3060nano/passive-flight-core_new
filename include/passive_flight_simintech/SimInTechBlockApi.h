#pragma once

/*
 * Эти стандартные заголовки должны подключаться
 * раньше служебного c_types.h из SimInTech.
 *
 * c_types.h использует:
 *
 * - uint64_t;
 * - uintptr_t;
 * - intptr_t;
 * - size_t.
 *
 * Но самостоятельно необходимые заголовки
 * официальный файл не подключает.
 */
#include <stddef.h>
#include <stdint.h>

#include "c_types.h"

#if defined(_WIN32)
    #if defined(PF_SIT_BLOCK_EXPORTS)
        #define PF_SIT_API __declspec(dllexport)
    #else
        #define PF_SIT_API __declspec(dllimport)
    #endif

    #define PF_SIT_CALL __stdcall
#else
    #define PF_SIT_API
    #define PF_SIT_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Нативный интерфейс расчётного модуля SimInTech.
 *
 * Сигнатуры функций соответствуют шаблону
 * MinGW_DLL из поставки SimInTech. Угловые выходы блоков преобразуются
 * в градусы на границе SimInTech; ядро и C API продолжают использовать радианы.
 */

/*
 * Передаёт SimInTech метаданные и размеры памяти блока.
 * nExtVars, nDinVars, nAlgVars, nStateVars и nConsts получают числа внешних,
 * дифференциальных, алгебраических переменных, переменных состояния и констант.
 * sizeOfStateVars, sizeOfConsts и sizeOfLocalVars получают размеры областей, байт.
 * dinVarsDimension и algVarsDimension получают суммарные размерности массивов.
 * extVarsInfo, dinVarsInfo, algVarsInfo, stateVarsInfo и constInfo получают
 * указатели на таблицы описаний; неиспользуемые таблицы устанавливаются в NULL.
 * solverData получает настройки решателя; schemeHash32 — хеш схемы блока;
 * algorithmName — буфер имени алгоритма; algorithmObjectId — непрозрачный объект.
 * Возвращает r_Success при корректных указателях либо r_Fail.
 */
PF_SIT_API int PF_SIT_CALL
INFO_FUNC(
    int* nExtVars,
    int* nDinVars,
    int* nAlgVars,
    int* nStateVars,
    int* nConsts,

    int* sizeOfStateVars,
    int* sizeOfConsts,
    int* sizeOfLocalVars,

    int* dinVarsDimension,
    int* algVarsDimension,

    void** extVarsInfo,
    void** dinVarsInfo,
    void** algVarsInfo,
    void** stateVarsInfo,
    void** constInfo,

    solver_struct* solverData,
    unsigned int* schemeHash32,
    char* algorithmName,
    void** algorithmObjectId
);

/*
 * Инициализирует один экземпляр блока и его локальную память.
 * step и modelTime заданы в секундах; extVarsAddress содержит адреса портов;
 * dinVars, derivatives, algVars и algFunctions относятся к массивам решателя;
 * stateVars, consts и locals указывают на выделенные SimInTech области памяти;
 * solverData и algorithmObjectId передают контекст исполнительной среды.
 * Возвращает r_Success либо r_Fail при отсутствии обязательной локальной памяти.
 */
PF_SIT_API int PF_SIT_CALL
INIT_FUNC(
    double step,
    double modelTime,

    void** extVarsAddress,

    double* dinVars,
    double* derivatives,
    double* algVars,
    double* algFunctions,

    void* stateVars,
    void* consts,
    void* locals,

    solver_struct* solverData,
    void* algorithmObjectId
);

/*
 * Выполняет действие SimInTech для текущего момента модельного времени.
 * action задаёт фазу вызова; step и modelTime измеряются в секундах;
 * остальные указатели имеют тот же смысл, что в INIT_FUNC. Функция считывает
 * входные порты, при необходимости запускает C API и записывает выходные порты.
 * Возвращает r_Success либо r_Fail при недоступности обязательных данных.
 */
PF_SIT_API int PF_SIT_CALL
RUN_FUNC(
    int action,
    double step,
    double modelTime,

    void** extVarsAddress,

    double* dinVars,
    double* derivatives,
    double* algVars,
    double* algFunctions,

    void* stateVars,
    void* consts,
    void* locals,

    solver_struct* solverData,
    void* algorithmObjectId
);

/*
 * Обслуживает запрос SimInTech к дискретному состоянию блока.
 * Принимает стандартный набор параметров RUN_FUNC; текущие блоки не содержат
 * отдельного состояния и возвращают r_Success без изменения физических величин.
 */
PF_SIT_API int PF_SIT_CALL
STATE_FUNC(
    int action,
    double step,
    double modelTime,

    void** extVarsAddress,

    double* dinVars,
    double* derivatives,
    double* algVars,
    double* algFunctions,

    void* stateVars,
    void* consts,
    void* locals,

    solver_struct* solverData,
    void* algorithmObjectId
);

/*
 * Обслуживает контурный вызов SimInTech.
 * Принимает стандартный набор параметров RUN_FUNC; текущие блоки не реализуют
 * отдельные контурные уравнения и возвращают r_Success.
 */
PF_SIT_API int PF_SIT_CALL
CONTUR_FUNC(
    int action,
    double step,
    double modelTime,

    void** extVarsAddress,

    double* dinVars,
    double* derivatives,
    double* algVars,
    double* algFunctions,

    void* stateVars,
    void* consts,
    void* locals,

    solver_struct* solverData,
    void* algorithmObjectId
);

#ifdef __cplusplus
}
#endif
