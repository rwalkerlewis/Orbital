#pragma once

#include <optional>
#include <string>
#include <vector>

class RocketEngine {
public:
    RocketEngine() = default;
    RocketEngine(std::string name,
                 double thrustSeaLevel,
                 double thrustVacuum,
                 double ispSeaLevel,
                 double ispVacuum);

    [[nodiscard]] double thrust(double ambientPressureRatio) const;
    [[nodiscard]] double isp(double ambientPressureRatio) const;
    [[nodiscard]] double massFlow(double thrustValue, double ispValue) const;
    [[nodiscard]] const std::string& name() const { return name_; }

private:
    std::string name_;
    double thrustSeaLevel_ = 0.0;
    double thrustVacuum_ = 0.0;
    double ispSeaLevel_ = 0.0;
    double ispVacuum_ = 0.0;
};

struct StageSpec {
    std::string name;
    RocketEngine engine;
    double dryMass = 0.0;
    double propellantMass = 0.0;
    double referenceArea = 10.0;
    double cdSubsonic = 0.25;
    double cdSupersonic = 0.45;
    double maxDynamicPressure = 50000.0;
};

struct StageState {
    StageSpec spec;
    double remainingPropellant = 0.0;
    bool separated = false;
};

struct ForceReport {
    double thrust = 0.0;
    double propellantConsumed = 0.0;
    double massFlow = 0.0;
    std::string activeStageName;
    bool stageJustSeparated = false;
    bool engineFiring = false;
};

class RocketVehicle {
public:
    RocketVehicle() = default;
    RocketVehicle(std::string name, std::vector<StageSpec> stages, double payloadMass);

    [[nodiscard]] double totalMass() const;
    [[nodiscard]] double referenceArea() const;
    [[nodiscard]] double dragCoefficient(double mach) const;

    ForceReport step(double dt,
                     double ambientPressure,
                     double dynamicPressure,
                     double throttleCommand = 1.0);

    [[nodiscard]] const std::string& name() const { return name_; }
    [[nodiscard]] std::optional<std::string> activeStageName() const;

private:
    StageState* activeStage();
    [[nodiscard]] const StageState* activeStage() const;

    std::string name_;
    std::vector<StageState> stages_;
    size_t activeStageIndex_ = 0;
    double payloadMass_ = 0.0;
};

RocketVehicle makeRocket(const std::string& id);
std::vector<std::string> availableRocketIds();
