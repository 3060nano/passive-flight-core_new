#include "passive_flight/ObjectAerodynamicsAdapter.hpp"
#include "passive_flight/ObjectPassport.hpp"
#include "passive_flight/TabulatedAerodynamicModel.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

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

void testFab1500TAdapter() {
    const auto passport =
        passive_flight::makeFab1500TPostnikovPassport();

    const auto adapted =
        passive_flight::makeAerodynamicModel(passport.object);
    const passive_flight::TabulatedAerodynamicModel direct(
        passport.object.reference.referenceLengthM,
        passport.object.tabulatedAerodynamics
    );

    passive_flight::AerodynamicInput input;
    input.mach = 0.75;
    input.angleOfAttackRad = 0.02;
    input.pitchRateRadS = 0.1;
    input.speedMps = 250.0;

    const auto adaptedResult = adapted->evaluate(input);
    const auto directResult = direct.evaluate(input);

    requireNear(adaptedResult.cx, directResult.cx, 1.0e-14, "Adapter Cx");
    requireNear(adaptedResult.cy, directResult.cy, 1.0e-14, "Adapter Cy");
    requireNear(adaptedResult.mz, directResult.mz, 1.0e-14, "Adapter mz");
}

void testInvalidTableIsRejected() {
    auto passport = passive_flight::makeFab1500TPostnikovPassport();
    passport.object.tabulatedAerodynamics.cx0.clear();

    bool thrown = false;
    try {
        static_cast<void>(
            passive_flight::makeAerodynamicModel(passport.object)
        );
    } catch (const std::invalid_argument&) {
        thrown = true;
    }

    if (!thrown) {
        throw std::runtime_error("Invalid table must be rejected");
    }
}

} // namespace

int main() {
    try {
        testFab1500TAdapter();
        testInvalidTableIsRejected();
        std::cout << "All object aerodynamics adapter tests passed.\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
