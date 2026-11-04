#pragma once

/*
 * Преобразует угловую величину на границе SimInTech
 * из радиан в градусы, не изменяя единицы C API и ядра.
 */
static inline double pfSimInTechRadiansToDegrees(
    double valueRad
) {
    return
        valueRad *
        57.2957795130823208768;
}
