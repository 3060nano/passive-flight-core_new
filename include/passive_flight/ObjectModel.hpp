#pragma once

#include "passive_flight/TabulatedAerodynamicData.hpp"

#include <string>

namespace passive_flight {

/// Способ задания аэродинамических характеристик расчётного объекта.
enum class AerodynamicModelType {
    /// Коэффициенты заданы таблицами как функции числа Маха.
    Tabulated
};

/// Общие сведения о расчётном объекте.
struct ObjectMetadata {
    /// Человекочитаемое наименование объекта в кодировке UTF-8.
    std::string displayName;

    /// Версия числовой модели объекта в текстовом формате.
    std::string modelVersion;

    /// Краткое описание назначения и источников модели.
    std::string description;
};

/// Массовые и инерционные характеристики объекта в СИ.
struct MassProperties {
    /// Масса объекта `m`, кг.
    double massKg{};

    /// Экваториальный момент инерции относительно оси тангажа `I_z`, кг·м².
    double pitchMomentOfInertiaKgM2{};

    /// Продольная координата центра масс от носовой точки корпуса, м.
    double centerOfMassXM{};
};

/// Опорная площадь и длина аэродинамических коэффициентов.
struct ReferenceGeometry {
    /// Опорная площадь `S`; для ФАБ-1500Т это площадь миделя, м².
    double areaM2{};

    /// Опорная длина `L_ref` коэффициента продольного момента, м.
    double referenceLengthM{};

    /// Возвращает опорную длину для размеризации момента и угловой скорости.
    /// @return Значение `L_ref`, м.
    [[nodiscard]]
    double effectiveReferenceLengthM() const noexcept {
        return referenceLengthM;
    }
};

/// Габариты корпуса, необходимые для паспорта ФАБ-1500Т.
struct BodyGeometry {
    /// Полная длина корпуса от носа до хвостовой точки, м.
    double lengthM{};

    /// Максимальный диаметр корпуса, м.
    double diameterM{};
};

/// Числовая модель объекта с табличными аэродинамическими характеристиками.
struct ObjectModel {
    /// Стабильный уникальный идентификатор модели в ObjectRegistry.
    std::string id;

    /// Текстовые сведения, предназначенные для пользователя и отчётности.
    ObjectMetadata metadata;

    /// Массовые и инерционные характеристики в единицах СИ.
    MassProperties mass;

    /// Опорные геометрические величины аэродинамических коэффициентов.
    ReferenceGeometry reference;

    /// Основные габариты корпуса в метрах.
    BodyGeometry body;

    /// Способ получения аэродинамических коэффициентов данной модели.
    AerodynamicModelType aerodynamicModelType{
        AerodynamicModelType::Tabulated
    };

    /// Табличные зависимости коэффициентов от числа Маха.
    TabulatedAerodynamicData tabulatedAerodynamics;
};

} // namespace passive_flight
