#pragma once

#include "passive_flight/TabulatedAerodynamicData.hpp"

#include <string>

namespace passive_flight {

/// Способ задания аэродинамических характеристик расчётного объекта.
enum class AerodynamicModelType {
    Tabulated
};

/// Общие сведения о расчётном объекте.
struct ObjectMetadata {
    std::string displayName;
    std::string modelVersion;
    std::string description;
};

/// Массовые и инерционные характеристики объекта в СИ.
struct MassProperties {
    double massKg{};
    double pitchMomentOfInertiaKgM2{};
    double centerOfMassXM{};
};

/// Опорная площадь и длина аэродинамических коэффициентов.
struct ReferenceGeometry {
    double areaM2{};
    double referenceLengthM{};

    [[nodiscard]]
    double effectiveReferenceLengthM() const noexcept {
        return referenceLengthM;
    }
};

/// Габариты корпуса, необходимые для паспорта ФАБ-1500Т.
struct BodyGeometry {
    double lengthM{};
    double diameterM{};
};

/// Числовая модель объекта с табличными аэродинамическими характеристиками.
struct ObjectModel {
    std::string id;
    ObjectMetadata metadata;

    MassProperties mass;
    ReferenceGeometry reference;
    BodyGeometry body;

    AerodynamicModelType aerodynamicModelType{
        AerodynamicModelType::Tabulated
    };

    TabulatedAerodynamicData tabulatedAerodynamics;
};

} // namespace passive_flight
