#pragma once

#include <memory>
#include <vector>

#include "Atmosphere.hpp"
#include "Rocket.hpp"
#include "SimulationTypes.hpp"

class TrajectoryRenderer;

struct SimulationResult {
    std::vector<TelemetryPoint> telemetry;
    double maxAltitude = 0.0;
    double maxDownrange = 0.0;
};

class GuidanceProgram {
public:
    explicit GuidanceProgram(GuidanceSettings settings);

    double update(double timeSeconds, double altitude, double downrange);

private:
    double basePitch(double timeSeconds, double altitude) const;

    GuidanceSettings settings_;
    double lastPitch_ = 90.0;
    double lastTime_ = 0.0;
    bool initialized_ = false;
};

class TrajectorySimulation {
public:
    TrajectorySimulation(LaunchSite site,
                         RocketVehicle rocket,
                         SimulationSettings settings);

    SimulationResult run();

private:
    struct State {
        double time = 0.0;
        Vector2 position;
        Vector2 velocity;
    };

    void recordTelemetry(const TelemetryPoint& point);
    TelemetryPoint buildTelemetry(const ForceReport& forces,
                                  const AtmosphereSample& atmosphere,
                                  double pitchCommand,
                                  double flightPathAngle,
                                  double dragMagnitude,
                                  double vehicleMass) const;

    LaunchSite site_;
    RocketVehicle rocket_;
    SimulationSettings settings_;
    AtmosphereModel atmosphere_;
    State state_;
    std::vector<Vector2> path_;
    std::vector<TelemetryPoint> telemetry_;
    double maxAltitude_ = 0.0;
    double maxDownrange_ = 0.0;
};
