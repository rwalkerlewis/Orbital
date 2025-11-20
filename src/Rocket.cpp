#include "Rocket.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
constexpr double g0 = 9.80665;

double clamp(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}
} // namespace

RocketEngine::RocketEngine(std::string name,
                           double thrustSeaLevel,
                           double thrustVacuum,
                           double ispSeaLevel,
                           double ispVacuum)
    : name_(std::move(name)),
      thrustSeaLevel_(thrustSeaLevel),
      thrustVacuum_(thrustVacuum),
      ispSeaLevel_(ispSeaLevel),
      ispVacuum_(ispVacuum) {}

double RocketEngine::thrust(double ambientPressureRatio) const {
    const double ratio = clamp(ambientPressureRatio, 0.0, 1.0);
    return thrustVacuum_ - (thrustVacuum_ - thrustSeaLevel_) * ratio;
}

double RocketEngine::isp(double ambientPressureRatio) const {
    const double ratio = clamp(ambientPressureRatio, 0.0, 1.0);
    return ispVacuum_ - (ispVacuum_ - ispSeaLevel_) * ratio;
}

double RocketEngine::massFlow(double thrustValue, double ispValue) const {
    if (ispValue <= 0.0) {
        return 0.0;
    }
    return thrustValue / (ispValue * g0);
}

RocketVehicle::RocketVehicle(std::string name, std::vector<StageSpec> stages, double payloadMass)
    : name_(std::move(name)),
      payloadMass_(payloadMass) {
    stages_.reserve(stages.size());
    for (auto& spec : stages) {
        stages_.push_back(StageState{
            .spec = spec,
            .remainingPropellant = spec.propellantMass,
            .separated = false,
        });
    }
}

double RocketVehicle::totalMass() const {
    double mass = payloadMass_;
    for (const auto& stage : stages_) {
        if (stage.separated) {
            continue;
        }
        mass += stage.spec.dryMass + stage.remainingPropellant;
    }
    return mass;
}

double RocketVehicle::referenceArea() const {
    const StageState* stage = activeStage();
    if (stage) {
        return stage->spec.referenceArea;
    }
    // fallback to last known area
    for (auto it = stages_.rbegin(); it != stages_.rend(); ++it) {
        if (it->spec.referenceArea > 0.0) {
            return it->spec.referenceArea;
        }
    }
    return 10.0;
}

double RocketVehicle::dragCoefficient(double mach) const {
    const StageState* stage = activeStage();
    if (!stage) {
        return 0.3;
    }

    if (mach < 0.8) {
        return stage->spec.cdSubsonic;
    }
    if (mach > 1.2) {
        return stage->spec.cdSupersonic;
    }
    const double t = (mach - 0.8) / 0.4;
    return stage->spec.cdSubsonic + t * (stage->spec.cdSupersonic - stage->spec.cdSubsonic + 0.08);
}

StageState* RocketVehicle::activeStage() {
    while (activeStageIndex_ < stages_.size() && stages_[activeStageIndex_].separated) {
        ++activeStageIndex_;
    }
    if (activeStageIndex_ >= stages_.size()) {
        return nullptr;
    }
    return &stages_[activeStageIndex_];
}

const StageState* RocketVehicle::activeStage() const {
    return const_cast<RocketVehicle*>(this)->activeStage();
}

std::optional<std::string> RocketVehicle::activeStageName() const {
    const StageState* stage = activeStage();
    if (!stage) {
        return std::nullopt;
    }
    return stage->spec.name;
}

ForceReport RocketVehicle::step(double dt,
                                double ambientPressure,
                                double dynamicPressure,
                                double throttleCommand) {
    ForceReport report{};
    StageState* stage = activeStage();
    if (!stage) {
        return report;
    }

    const double pressureRatio = ambientPressure / 101325.0;
    double thrust = stage->spec.engine.thrust(pressureRatio);
    double isp = stage->spec.engine.isp(pressureRatio);

    double throttle = clamp(throttleCommand, 0.0, 1.0);
    if (stage->spec.maxDynamicPressure > 0.0 && dynamicPressure > stage->spec.maxDynamicPressure) {
        throttle = std::min(throttle, clamp(stage->spec.maxDynamicPressure / dynamicPressure, 0.2, 1.0));
    }

    thrust *= throttle;
    isp = std::max(isp, 1.0);

    double massFlow = stage->spec.engine.massFlow(thrust, isp);
    double requestedPropellant = massFlow * dt;
    const double availablePropellant = stage->remainingPropellant;

    if (requestedPropellant > availablePropellant) {
        const double scale = availablePropellant / std::max(requestedPropellant, 1e-6);
        thrust *= scale;
        massFlow *= scale;
        requestedPropellant = availablePropellant;
    }

    stage->remainingPropellant -= requestedPropellant;

    report.thrust = thrust;
    report.propellantConsumed = requestedPropellant;
    report.massFlow = massFlow;
    report.activeStageName = stage->spec.name;
    report.engineFiring = thrust > 1.0;

    if (stage->remainingPropellant <= 1e-3) {
        stage->remainingPropellant = 0.0;
        stage->separated = true;
        report.stageJustSeparated = true;
    }

    return report;
}

namespace {
StageSpec makeFalcon9Stage1() {
    return StageSpec{
        .name = "Falcon 9 Booster",
        .engine = RocketEngine("Merlin 1D Octaweb", 7600000.0, 8227000.0, 282.0, 311.0),
        .dryMass = 25600.0,
        .propellantMass = 395700.0,
        .referenceArea = 10.5,
        .cdSubsonic = 0.25,
        .cdSupersonic = 0.45,
        .maxDynamicPressure = 50000.0,
    };
}

StageSpec makeFalcon9Stage2() {
    return StageSpec{
        .name = "Falcon 9 Upper",
        .engine = RocketEngine("Merlin Vacuum", 801000.0, 934000.0, 311.0, 348.0),
        .dryMass = 4000.0,
        .propellantMass = 92670.0,
        .referenceArea = 10.5,
        .cdSubsonic = 0.15,
        .cdSupersonic = 0.35,
        .maxDynamicPressure = 20000.0,
    };
}

StageSpec makeSoyuzCore() {
    return StageSpec{
        .name = "Soyuz Core",
        .engine = RocketEngine("RD-108A", 792000.0, 990000.0, 256.0, 315.0),
        .dryMass = 6500.0,
        .propellantMass = 65000.0,
        .referenceArea = 9.2,
        .cdSubsonic = 0.3,
        .cdSupersonic = 0.55,
        .maxDynamicPressure = 42000.0,
    };
}

StageSpec makeSoyuzBoosters() {
    return StageSpec{
        .name = "Soyuz Boosters",
        .engine = RocketEngine("RD-117 Cluster", 4000000.0, 4350000.0, 252.0, 320.0),
        .dryMass = 15800.0,
        .propellantMass = 178000.0,
        .referenceArea = 9.8,
        .cdSubsonic = 0.32,
        .cdSupersonic = 0.6,
        .maxDynamicPressure = 45000.0,
    };
}

StageSpec makeSoyuzUpper() {
    return StageSpec{
        .name = "Fregat Upper",
        .engine = RocketEngine("S5.92", 0.0, 20000.0, 0.0, 333.0),
        .dryMass = 1000.0,
        .propellantMass = 5350.0,
        .referenceArea = 4.0,
        .cdSubsonic = 0.2,
        .cdSupersonic = 0.4,
        .maxDynamicPressure = 15000.0,
    };
}
} // namespace

RocketVehicle makeRocket(const std::string& id) {
    if (id == "falcon9") {
        return RocketVehicle("Falcon 9-class",
                             {makeFalcon9Stage1(), makeFalcon9Stage2()},
                             13150.0);
    }

    if (id == "soyuz") {
        return RocketVehicle("Soyuz-ST",
                             {makeSoyuzBoosters(), makeSoyuzCore(), makeSoyuzUpper()},
                             7500.0);
    }

    throw std::runtime_error("Unknown rocket id: " + id);
}

std::vector<std::string> availableRocketIds() {
    return {"falcon9", "soyuz"};
}
