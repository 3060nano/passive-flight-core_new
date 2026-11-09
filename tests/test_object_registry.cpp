#include "passive_flight/ObjectRegistry.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
    /// Требует выполнения логического условия теста реестра.
    void require(bool condition, const char *message) {
        if (!condition) {
            throw std::runtime_error(message);
        }
    }

    /// Проверяет состав и поиск объектов стандартного реестра.
    void testDefaultRegistry() {
        const auto registry = passive_flight::makeDefaultObjectRegistry();
        require(registry.size() == 3, "Default registry size");
        require(
            registry.contains("FAB_1500T_POSTNIKOV_1979"),
            "Default registry must contain FAB-1500T"
        );
        require(
            registry.findObject("FAB_1500T_POSTNIKOV_1979") != nullptr,
            "FAB-1500T model lookup"
        );

        const auto descriptors = registry.descriptors();
        require(descriptors.size() == 3, "Descriptor count");
        require(
            descriptors.front().id == "FAB_1500T_POSTNIKOV_1979",
            "FAB-1500T must have SimInTech index zero"
        );
    }

    /// Проверяет добавление дополнительного корректного табличного объекта.
    void testAdditionalTabulatedObjectCanBeAdded() {
        auto registry = passive_flight::makeDefaultObjectRegistry();
        auto passport = passive_flight::makeFab1500TPostnikovPassport();
        passport.object.id = "FAB_1500T_RESEARCH_COPY";
        passport.object.metadata.displayName = "FAB-1500T research copy";

        registry.add(std::move(passport));
        require(registry.size() == 4, "Additional object count");
        require(
            registry.contains("FAB_1500T_RESEARCH_COPY"),
            "Additional object lookup"
        );
    }

    /// Проверяет запрет повторной регистрации одного идентификатора.
    void testDuplicateIsRejected() {
        passive_flight::ObjectRegistry registry;
        registry.add(passive_flight::makeFab1500TPostnikovPassport());

        bool thrown = false;
        try {
            registry.add(passive_flight::makeFab1500TPostnikovPassport());
        } catch (const std::invalid_argument &) {
            thrown = true;
        }
        require(thrown, "Duplicate identifier must be rejected");
    }

    /// Проверяет отклонение паспорта с некорректными физическими параметрами.
    void testInvalidPassportIsRejected() {
        passive_flight::ObjectRegistry registry;
        auto passport = passive_flight::makeFab1500TPostnikovPassport();
        passport.object.mass.massKg = -1.0;

        bool thrown = false;
        try {
            registry.add(std::move(passport));
        } catch (const std::invalid_argument &) {
            thrown = true;
        }
        require(thrown, "Invalid passport must be rejected");
    }

    /// Проверяет запуск решателя по идентификатору из реестра.
    void testSimulation() {
        const auto registry = passive_flight::makeDefaultObjectRegistry();

        passive_flight::SimulationRequest request;
        request.objectId = "FAB_1500T_POSTNIKOV_1979";
        request.release.altitudeM = 100.0;
        request.release.speedMps = 200.0;

        passive_flight::SimulationOptions options;
        options.timeStepS = 0.001;
        options.maximumTimeS = 30.0;
        options.maximumSteps = 100'000;
        options.saveHistory = false;

        const auto result = registry.simulate(request, options);
        require(
            result.terminationReason ==
            passive_flight::TerminationReason::GroundReached,
            "FAB-1500T simulation must reach ground"
        );

        request.objectId = "UNKNOWN_OBJECT";
        const auto unknown = registry.simulate(request, options);
        require(
            unknown.terminationReason ==
            passive_flight::TerminationReason::InvalidInput,
            "Unknown object must be rejected"
        );
    }
} // namespace

/// Запускает модульные проверки ObjectRegistry.
int main() {
    try {
        testDefaultRegistry();
        testAdditionalTabulatedObjectCanBeAdded();
        testDuplicateIsRejected();
        testInvalidPassportIsRejected();
        testSimulation();
        std::cout << "All object registry tests passed.\n";
        return EXIT_SUCCESS;
    } catch (const std::exception &error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
