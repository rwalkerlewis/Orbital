#include "Simulation.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "Visualization.hpp"

namespace {
constexpr double pi = 3.14159265358979323846;

constexpr double degToRad(double deg) {
    return deg * pi / 180.0;
}

constexpr double radToDeg(double rad) {
    return rad * 180.0 / pi;
}

constexpr double earthRadius = 6371000.0;
constexpr double earthAngularVelocity = 7.2921159e-5;
constexpr double earthMu = 3.986004418e14;
} // namespace

std::vector<LaunchSite> builtinLaunchSites() {
    return {
        {"ksc", "LC-39A", "Kennedy Space Center, USA", 28.608389, -80.604333, 3.0, 90.0},
        {"vafb", "SLC-4E", "Vandenberg SFB, USA", 34.632093, -120.610829, 105.0, 180.0},
        {"csG", "ELA-3", "Centre Spatial Guyanais, France", 5.239, -52.768, 10.0, 90.0},
        {"tanegashima", "Yoshinobu Pad", "Tanegashima Space Center, Japan", 30.401, 130.973, 20.0, 110.0},
    };
}

const LaunchSite& findLaunchSite(const std::string& id) {
    static const std::vector<LaunchSite> sites = builtinLaunchSites();
    const auto it = std::find_if(sites.begin(), sites.end(), [&](const LaunchSite& site) {
        return site.id == id;
    });
    if (it == sites.end()) {
        throw std::runtime_error("Unknown launch site id: " + id);
    }
    return *it;
}

GuidanceProgram::GuidanceProgram(GuidanceSettings settings)
    : settings_(settings) {}

double GuidanceProgram::basePitch(double timeSeconds, double altitude) const {
    if (timeSeconds < settings_.verticalHoldSeconds) {
        return 90.0;
    }

    const double timeProgress = std::clamp(
        (timeSeconds - settings_.verticalHoldSeconds) / std::max(settings_.pitchProgramDuration, 1.0),
        0.0,
        1.0);

    double commanded = 90.0 - timeProgress * (90.0 - settings_.finalPitchDegrees);

    if (altitude > settings_.gravityTurnStartAltitude) {
        const double altitudeProgress = std::clamp(
            (altitude - settings_.gravityTurnStartAltitude) / 50000.0,
            0.0,
            1.0);
        commanded = (1.0 - altitudeProgress) * commanded + altitudeProgress * settings_.finalPitchDegrees;
    }

    return std::clamp(commanded, settings_.finalPitchDegrees, 90.0);
}

double GuidanceProgram::update(double timeSeconds, double altitude, double /*downrange*/) {
    const double base = basePitch(timeSeconds, altitude);

    if (!initialized_) {
        initialized_ = true;
        lastPitch_ = base;
        lastTime_ = timeSeconds;
        return base;
    }

    const double deltaTime = timeSeconds - lastTime_;
    const double maxStep = settings_.maxPitchRateDegPerSec * std::max(deltaTime, 0.0);
    const double delta = std::clamp(base - lastPitch_, -maxStep, maxStep);

    lastPitch_ += delta;
    lastTime_ = timeSeconds;
    return lastPitch_;
}

TrajectorySimulation::TrajectorySimulation(LaunchSite site,
                                           RocketVehicle rocket,
                                           SimulationSettings settings)
    : site_(std::move(site)),
      rocket_(std::move(rocket)),
      settings_(settings) {
    state_.position = {0.0, site_.altitudeMeters};
    state_.time = 0.0;

    const double latRad = degToRad(site_.latitudeDeg);
    const double azimuthRad = degToRad(site_.launchAzimuthDeg);
    const double initialHorizontal = earthAngularVelocity * std::cos(latRad) * (earthRadius + site_.altitudeMeters);

    state_.velocity = {initialHorizontal * std::cos(azimuthRad),
                       initialHorizontal * std::sin(azimuthRad)};

    path_.push_back(state_.position);
    maxAltitude_ = state_.position.y;
    maxDownrange_ = state_.position.x;
}

SimulationResult TrajectorySimulation::run() {
    GuidanceProgram guidance(settings_.guidance);
    std::unique_ptr<TrajectoryRenderer> renderer;

    if (settings_.enableVisualization) {
        renderer = std::make_unique<TrajectoryRenderer>(settings_.visualizationWidth,
                                                        settings_.visualizationHeight);
    }

    double renderAccumulator = 0.0;
    const double dt = settings_.timeStep;

    while (state_.time < settings_.maxDuration) {
        const double altitude = state_.position.y;
        if (altitude < 0.0 && state_.time > 5.0) {
            break;
        }

        const AtmosphereSample atmosphere = atmosphere_.sample(altitude);
        const double pitchCommand = guidance.update(state_.time, altitude, state_.position.x);
        const double speed = state_.velocity.norm();
        const double mach = atmosphere.speedOfSound > 1e-6 ? speed / atmosphere.speedOfSound : 0.0;
        const double dynamicPressure = atmosphere.dynamicPressure(speed);
        const double flightPathAngle = radToDeg(std::atan2(state_.velocity.y, state_.velocity.x));

        const double massBefore = rocket_.totalMass();
        const ForceReport forces = rocket_.step(dt, atmosphere.pressure, dynamicPressure);
        const double massAfter = rocket_.totalMass();
        const double effectiveMass = std::max(0.5 * (massBefore + massAfter), 1.0);

        const double pitchRad = degToRad(pitchCommand);
        Vector2 thrustDirection{std::cos(pitchRad), std::sin(pitchRad)};
        Vector2 thrustForce = thrustDirection * forces.thrust;

        const double referenceArea = rocket_.referenceArea();
        const double cd = rocket_.dragCoefficient(mach);
        const double dragMagnitude = 0.5 * atmosphere.density * speed * speed * cd * referenceArea;
        Vector2 dragForce = speed > 1e-3 ? (-dragMagnitude) * state_.velocity.normalized() : Vector2{};

        const double radius = earthRadius + std::max(altitude, 0.0);
        const double gravity = earthMu / (radius * radius);
        Vector2 gravityForce{0.0, -gravity * effectiveMass};

        const Vector2 netForce = thrustForce + dragForce + gravityForce;
        const Vector2 acceleration = (1.0 / effectiveMass) * netForce;

        state_.velocity += acceleration * dt;
        state_.position += state_.velocity * dt;
        state_.time += dt;

        path_.push_back(state_.position);
        maxAltitude_ = std::max(maxAltitude_, state_.position.y);
        maxDownrange_ = std::max(maxDownrange_, state_.position.x);

        TelemetryPoint telemetry = buildTelemetry(forces,
                                                  atmosphere,
                                                  pitchCommand,
                                                  flightPathAngle,
                                                  dragMagnitude,
                                                  massAfter);
        telemetry.mach = mach;
        telemetry.dynamicPressure = dynamicPressure;
        recordTelemetry(telemetry);

        renderAccumulator += dt;
        if (renderer && renderAccumulator >= settings_.renderInterval) {
            renderer->render(path_, telemetry, site_, maxAltitude_, maxDownrange_, false);
            renderAccumulator = 0.0;
        }

        if (!forces.engineFiring && !rocket_.activeStageName().has_value()) {
            break;
        }
    }

    if (renderer && !telemetry_.empty()) {
        renderer->render(path_, telemetry_.back(), site_, maxAltitude_, maxDownrange_, true);
    }

    SimulationResult result;
    result.telemetry = telemetry_;
    result.maxAltitude = maxAltitude_;
    result.maxDownrange = maxDownrange_;
    return result;
}

void TrajectorySimulation::recordTelemetry(const TelemetryPoint& point) {
    telemetry_.push_back(point);
}

TelemetryPoint TrajectorySimulation::buildTelemetry(const ForceReport& forces,
                                                    const AtmosphereSample& atmosphere,
                                                    double pitchCommand,
                                                    double flightPathAngle,
                                                    double dragMagnitude,
                                                    double vehicleMass) const {
    TelemetryPoint telemetry;
    telemetry.time = state_.time;
    telemetry.altitude = state_.position.y;
    telemetry.downrange = state_.position.x;
    telemetry.speed = state_.velocity.norm();
    telemetry.pitchCommand = pitchCommand;
    telemetry.flightPathAngle = flightPathAngle;
    telemetry.dynamicPressure = atmosphere.dynamicPressure(telemetry.speed);
    telemetry.thrust = forces.thrust;
    telemetry.drag = dragMagnitude;
    telemetry.mass = vehicleMass;
    telemetry.stageName = forces.activeStageName;
    return telemetry;
}
