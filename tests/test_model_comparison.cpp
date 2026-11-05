#include "passive_flight/ObjectRegistry.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

passive_flight::SimulationResult simulate(
    const passive_flight::ObjectRegistry& registry,
    const std::string& objectId,
    double altitudeM,
    double speedMps
) {
    passive_flight::SimulationRequest request;
    request.objectId = objectId;
    request.release.altitudeM = altitudeM;
    request.release.speedMps = speedMps;

    passive_flight::SimulationOptions options;
    options.timeStepS = 0.001;
    options.maximumTimeS = 300.0;
    options.maximumSteps = 2'000'000;
    options.groundAltitudeM = 0.0;
    options.saveHistory = false;

    return registry.simulate(request, options);
}

} // namespace

int main() {
    try {
        const auto registry = passive_flight::makeDefaultObjectRegistry();

        constexpr double altitudeM = 10000.0;
        constexpr double speedMps = 300.0;

        const auto original = simulate(
            registry,
            "FAB_1500T_POSTNIKOV_1979",
            altitudeM,
            speedMps
        );

        const auto modified = simulate(
            registry,
            "FAB_1500T_MODIFIED_CY_MZ",
            altitudeM,
            speedMps
        );

        require(
            original.terminationReason ==
                passive_flight::TerminationReason::GroundReached,
            "Original model must reach ground"
        );
        require(
            modified.terminationReason ==
                passive_flight::TerminationReason::GroundReached,
            "Modified model must reach ground"
        );

        const auto originalSummary =
            passive_flight::summarize(original);
        const auto modifiedSummary =
            passive_flight::summarize(modified);

        const double deltaDownrange =
            modifiedSummary.downrangeM -
            originalSummary.downrangeM;

        const double relativeChangePercent =
            (deltaDownrange / originalSummary.downrangeM) * 100.0;

        std::cout << "Comparison of FAB-1500T models\n";
        std::cout << "Release: H = " << altitudeM
                  << " m, V = " << speedMps << " m/s\n\n";

        std::cout << "Original (Postnikov/Chuyko):\n";
        std::cout << "  downrange = " << originalSummary.downrangeM << " m\n";
        std::cout << "  fall time = " << originalSummary.fallTimeS << " s\n";
        std::cout << "  impact speed = " << originalSummary.impactSpeedMps
                  << " m/s\n\n";

        std::cout << "Modified (Cy/mz + Cx0*1.05):\n";
        std::cout << "  downrange = " << modifiedSummary.downrangeM << " m\n";
        std::cout << "  fall time = " << modifiedSummary.fallTimeS << " s\n";
        std::cout << "  impact speed = " << modifiedSummary.impactSpeedMps
                  << " m/s\n\n";

        std::cout << "Delta downrange = " << deltaDownrange << " m\n";
        std::cout << "Relative change = " << relativeChangePercent << " %\n";

        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
