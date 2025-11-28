#pragma once

#include <cmath>
#include <string>
#include <vector>

struct Vector2 {
    double x = 0.0;
    double y = 0.0;

    [[nodiscard]] double norm() const {
        return std::sqrt(x * x + y * y);
    }

    [[nodiscard]] Vector2 normalized() const {
        const double magnitude = norm();
        if (magnitude < 1e-9) {
            return {0.0, 0.0};
        }
        return {x / magnitude, y / magnitude};
    }

    Vector2& operator+=(const Vector2& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2& operator-=(const Vector2& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vector2& operator*=(double scalar) {
        x *= scalar;
        y *= scalar;
        return *this;
    }
};

inline Vector2 operator+(Vector2 lhs, const Vector2& rhs) {
    lhs += rhs;
    return lhs;
}

inline Vector2 operator-(Vector2 lhs, const Vector2& rhs) {
    lhs -= rhs;
    return lhs;
}

inline Vector2 operator*(Vector2 lhs, double scalar) {
    lhs *= scalar;
    return lhs;
}

inline Vector2 operator*(double scalar, Vector2 rhs) {
    rhs *= scalar;
    return rhs;
}

struct LaunchSite {
    std::string id;
    std::string name;
    std::string location;
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double altitudeMeters = 0.0;
    double launchAzimuthDeg = 90.0;
};

struct GuidanceSettings {
    double verticalHoldSeconds = 10.0;
    double pitchProgramDuration = 160.0;
    double finalPitchDegrees = 5.0;
    double gravityTurnStartAltitude = 15000.0;
    double maxPitchRateDegPerSec = 1.5;
};

struct SimulationSettings {
    double timeStep = 0.25;
    double maxDuration = 600.0;
    bool enableVisualization = true;
    double renderInterval = 0.5;
    int visualizationWidth = 100;
    int visualizationHeight = 30;
    GuidanceSettings guidance;
};

struct LatLon {
    double latDeg = 0.0;
    double lonDeg = 0.0;
};

struct TelemetryPoint {
    double time = 0.0;
    double altitude = 0.0;
    double downrange = 0.0;
    double latitude = 0.0;
    double longitude = 0.0;
    double speed = 0.0;
    double pitchCommand = 90.0;
    double flightPathAngle = 90.0;
    double mach = 0.0;
    double dynamicPressure = 0.0;
    double thrust = 0.0;
    double drag = 0.0;
    double mass = 0.0;
    std::string stageName;
};

std::vector<LaunchSite> builtinLaunchSites();
const LaunchSite& findLaunchSite(const std::string& id);
