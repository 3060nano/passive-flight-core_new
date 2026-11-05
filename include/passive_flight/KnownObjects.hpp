#pragma once

#include "passive_flight/ObjectModel.hpp"

namespace passive_flight {
    /**
     * Создаёт контрольную числовую модель авиабомбы ФАБ-1500Т
     * с готовыми массовыми и аэродинамическими характеристиками.
     *
     * Источник:
     * А.Г. Постников, В.С. Чуйко,
     * "Методы решения прикладных задач внешней баллистики",
     * 1979, таблица 1.1.
     *
     * Модель предназначена прежде всего для валидации
     * уравнений движения passive-flight-core.
     *
     * На этом этапе объект ещё не добавляется в ObjectRegistry:
     * сначала проверяется непосредственно связка
     *
     *     табличная аэродинамика -> динамика -> траектория.
     *
     * @return Полностью заполненная числовая модель в единицах СИ.
     */
    [[nodiscard]]
    ObjectModel makeFab1500TPostnikovModel();

    /**
     * Возвращает табличные аэродинамические характеристики
     * ФАБ-1500Т из того же источника.
     *
     * @return Пять таблиц коэффициентов и производных в функции числа Маха.
     */
    [[nodiscard]]
    TabulatedAerodynamicData
    makeFab1500TPostnikovAerodynamicData();

    /**
     * Новая табличная модель с Cy/mz из предоставленной таблицы
     * (интерполировано) и Cx0 * 1.05.
     */
    [[nodiscard]]
    ObjectModel makeFab1500TModifiedModel();

    [[nodiscard]]
    TabulatedAerodynamicData
    makeFab1500TModifiedAerodynamicData();

    /**
 * Планирующая модель ФАБ-1500Т с триммированием на 1°.
 * Производные масштабированы к площади миделя,
 * добавлены Cy0 и mz0.
 */
    [[nodiscard]]
    ObjectModel makeFab1500TGlideModel();

    [[nodiscard]]
    TabulatedAerodynamicData
    makeFab1500TGlideAerodynamicData();
} // namespace passive_flight

