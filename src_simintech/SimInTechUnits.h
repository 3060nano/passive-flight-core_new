#pragma once

/*
 * Преобразует угловую величину на границе SimInTech
 * из радиан в градусы, не изменяя единицы C API и ядра.
 * valueRad — угол или угловая скорость в рад либо рад/с.
 * Возвращаемое значение выражено соответственно в градусах либо градусах/с.
 */
static inline double pfSimInTechRadiansToDegrees(
    double valueRad
) {
    return
        valueRad *
        57.2957795130823208768;
}
