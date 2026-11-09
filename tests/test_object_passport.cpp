#include "passive_flight/ObjectPassport.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <stdexcept>
#include <string>

namespace {

/// Требует выполнения логического условия теста паспорта.
void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

/// Проверяет абсолютное отклонение чисел в единицах паспортного параметра.
void requireNear(
    double actual,
    double expected,
    double tolerance,
    const char* message
) {
    if (std::abs(actual - expected) > tolerance) {
        throw std::runtime_error(message);
    }
}

/// Требует отсутствия нарушений валидации и формирует подробную диагностику.
void requireValid(
    const passive_flight::ObjectPassport& passport,
    const char* message
) {
    const auto issues = passive_flight::validate(passport);
    if (issues.empty()) {
        return;
    }

    std::string details = message;
    for (const auto& issue : issues) {
        details += "\n" + issue.field + ": " + issue.message;
    }
    throw std::runtime_error(details);
}

/// Проверяет значения и происхождение параметров контрольного паспорта.
void testFab1500TPassport() {
    const auto passport =
        passive_flight::makeFab1500TPostnikovPassport();
    const auto& object = passport.object;

    require(
        object.id == "FAB_1500T_POSTNIKOV_1979",
        "FAB-1500T identifier"
    );
    require(
        object.aerodynamicModelType ==
            passive_flight::AerodynamicModelType::Tabulated,
        "FAB-1500T aerodynamic model type"
    );
    requireNear(object.mass.massKg, 1519.0, 1.0e-12, "FAB-1500T mass");
    requireNear(
        object.mass.pitchMomentOfInertiaKgM2,
        1122.3,
        1.0e-12,
        "FAB-1500T pitch inertia"
    );
    requireNear(
        object.mass.centerOfMassXM,
        1.16,
        1.0e-12,
        "FAB-1500T center of mass"
    );
    requireNear(object.body.lengthM, 3.46, 1.0e-12, "FAB-1500T length");
    requireNear(object.body.diameterM, 0.58, 1.0e-12, "FAB-1500T diameter");
    requireNear(
        object.reference.areaM2,
        std::numbers::pi_v<double> * 0.58 * 0.58 / 4.0,
        1.0e-12,
        "FAB-1500T reference area"
    );
    requireNear(
        object.reference.referenceLengthM,
        3.46,
        1.0e-12,
        "FAB-1500T reference length"
    );
    require(object.tabulatedAerodynamics.cx0.size() == 19, "Cx0 node count");
    require(object.tabulatedAerodynamics.cyAlpha.size() == 7, "CyAlpha node count");

    const auto* source = passive_flight::findParameter(
        passport,
        "aerodynamics.source"
    );
    require(source != nullptr, "Aerodynamic source record is missing");
    require(
        source->status == passive_flight::ParameterStatus::SourceDocument,
        "Aerodynamic source status"
    );
    requireValid(passport, "FAB-1500T passport must be valid");
}

/// Проверяет обнаружение пустой обязательной аэродинамической таблицы.
void testEmptyTabulatedTableDetected() {
    auto passport = passive_flight::makeFab1500TPostnikovPassport();
    passport.object.tabulatedAerodynamics.cyAlpha.clear();
    require(
        !passive_flight::validate(passport.object).empty(),
        "Empty aerodynamic table must be detected"
    );
}

/// Проверяет обнаружение неположительной массы объекта.
void testInvalidMassDetected() {
    auto passport = passive_flight::makeFab1500TPostnikovPassport();
    passport.object.mass.massKg = -1519.0;
    require(
        !passive_flight::validate(passport).empty(),
        "Negative mass must be detected"
    );
}

/// Проверяет обнаружение повторяющегося пути паспортного параметра.
void testDuplicateParameterDetected() {
    auto passport = passive_flight::makeFab1500TPostnikovPassport();
    passport.parameters.push_back(passport.parameters.front());

    const auto issues = passive_flight::validate(passport);
    bool duplicateFound = false;
    for (const auto& issue : issues) {
        duplicateFound |=
            issue.message == "Duplicate parameter provenance record";
    }
    require(duplicateFound, "Duplicate provenance record must be detected");
}

/// Проверяет стабильные текстовые имена перечислений паспорта.
void testNames() {
    require(
        std::string(passive_flight::parameterStatusName(
            passive_flight::ParameterStatus::SourceDocument
        )) == "source_document",
        "Parameter status name"
    );
    require(
        std::string(passive_flight::aerodynamicModelTypeName(
            passive_flight::AerodynamicModelType::Tabulated
        )) == "tabulated",
        "Aerodynamic model type name"
    );
}

} // namespace

/// Запускает проверки структуры, значений и валидации паспортов.
int main() {
    try {
        testFab1500TPassport();
        testEmptyTabulatedTableDetected();
        testInvalidMassDetected();
        testDuplicateParameterDetected();
        testNames();
        std::cout << "All object passport tests passed.\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
