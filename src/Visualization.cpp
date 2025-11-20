#include "Visualization.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>

TrajectoryRenderer::TrajectoryRenderer(int width, int height)
    : width_(width), height_(height) {}

namespace {
double safeMax(double value, double fallback) {
    if (value < fallback) {
        return fallback;
    }
    return value;
}
} // namespace

void TrajectoryRenderer::render(const std::vector<Vector2>& path,
                                const TelemetryPoint& telemetry,
                                const LaunchSite& site,
                                double maxAltitude,
                                double maxDownrange,
                                bool /*finalFrame*/) const {
    if (width_ <= 0 || height_ <= 0) {
        return;
    }

    const double altScale = safeMax(maxAltitude, telemetry.altitude + 1.0);
    const double rangeScale = safeMax(maxDownrange, telemetry.downrange + 1.0);

    std::vector<std::string> buffer(height_, std::string(width_, ' '));

    auto project = [&](const Vector2& point) {
        const double xRatio = rangeScale <= 0.0 ? 0.0 : point.x / rangeScale;
        const double yRatio = altScale <= 0.0 ? 0.0 : point.y / altScale;
        int x = static_cast<int>(std::clamp(xRatio, 0.0, 1.0) * (width_ - 1));
        int y = static_cast<int>(std::clamp(yRatio, 0.0, 1.0) * (height_ - 1));
        y = height_ - 1 - y;
        x = std::clamp(x, 0, width_ - 1);
        y = std::clamp(y, 0, height_ - 1);
        return std::pair<int, int>{x, y};
    };

    for (const auto& point : path) {
        const auto [x, y] = project(point);
        buffer[y][x] = '.';
    }

    if (!path.empty()) {
        const auto [x, y] = project(path.back());
        buffer[y][x] = 'R';
    }

    for (int y = 0; y < height_; ++y) {
        buffer[y][0] = '|';
    }
    for (int x = 0; x < width_; ++x) {
        buffer[height_ - 1][x] = '-';
    }
    buffer[height_ - 1][0] = '+';

    std::cout << "\033[2J\033[H";
    std::cout << "Launch Site: " << site.name << " (" << site.location << ")\n";
    std::cout << "Time: " << std::fixed << std::setprecision(1) << telemetry.time << " s"
              << " | Altitude: " << telemetry.altitude / 1000.0 << " km"
              << " | Downrange: " << telemetry.downrange / 1000.0 << " km\n";
    std::cout << "Speed: " << telemetry.speed / 1000.0 << " km/s"
              << " | Mach: " << telemetry.mach
              << " | Pitch Cmd: " << telemetry.pitchCommand << " deg"
              << " | Stage: " << (telemetry.stageName.empty() ? "None" : telemetry.stageName) << "\n";
    std::cout << "Dynamic Pressure: " << telemetry.dynamicPressure / 1000.0 << " kPa"
              << " | Thrust: " << telemetry.thrust / 1000.0 << " kN"
              << " | Drag: " << telemetry.drag / 1000.0 << " kN"
              << " | Mass: " << telemetry.mass / 1000.0 << " t\n\n";

    for (const auto& line : buffer) {
        std::cout << line << '\n';
    }
    std::cout.flush();
}
