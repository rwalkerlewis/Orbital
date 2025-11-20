#pragma once

#include <vector>

#include "SimulationTypes.hpp"

class TrajectoryRenderer {
public:
    TrajectoryRenderer(int width, int height);

    void render(const std::vector<Vector2>& path,
                const std::vector<LatLon>& groundTrack,
                const TelemetryPoint& telemetry,
                const LaunchSite& site,
                double maxAltitude,
                double maxDownrange,
                bool finalFrame) const;

private:
    void renderGroundTrack(const std::vector<LatLon>& groundTrack,
                           const TelemetryPoint& telemetry) const;

    int width_;
    int height_;
};
