#include "Visualization.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

TrajectoryRenderer::TrajectoryRenderer(int width, int height)
    : width_(width), height_(height) {}

namespace {
double safeMax(double value, double fallback) {
    if (value < fallback) {
        return fallback;
    }
    return value;
}

constexpr double earthSpanLat = 180.0;
constexpr double earthSpanLon = 360.0;

struct LandPatch {
    double latCenter;
    double lonCenter;
    double latRadius;
    double lonRadius;
    double elevation;
};

double wrapLonDeg(double lon) {
    double wrapped = std::fmod(lon + 180.0, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    return wrapped - 180.0;
}

const std::vector<LandPatch>& predefinedPatches() {
    static const std::vector<LandPatch> patches = {
        {47.0, -100.0, 28.0, 40.0, 1.0},   // North America
        {10.0, -80.0, 10.0, 20.0, 0.8},    // Central America
        {-15.0, -60.0, 25.0, 20.0, 0.85},  // South America
        {55.0, 60.0, 30.0, 85.0, 1.0},     // Eurasia
        {10.0, 20.0, 30.0, 18.0, 0.95},    // Africa
        {20.0, 80.0, 10.0, 12.0, 0.7},     // India
        {24.0, 50.0, 10.0, 15.0, 0.65},    // Arabian Peninsula
        {-25.0, 135.0, 15.0, 22.0, 0.75},  // Australia
        {72.0, -40.0, 8.0, 12.0, 0.6},     // Greenland
        {0.0, 150.0, 12.0, 14.0, 0.55},    // Southeast Asia archipelago
        {35.0, 140.0, 6.0, 8.0, 0.4},      // Japan
        {-5.0, 170.0, 6.0, 10.0, 0.4},     // Pacific islands
        {0.0, -20.0, 8.0, 15.0, 0.4},      // West Africa bulge
        {65.0, 100.0, 8.0, 12.0, 0.5},     // Siberia extension
        {-33.0, 20.0, 6.0, 8.0, 0.45},     // South Africa
    };
    return patches;
}

std::vector<std::string> buildEarthAsciiMap() {
    constexpr int mapWidth = 120;
    constexpr int mapHeight = 40;
    std::vector<std::string> rows(mapHeight, std::string(mapWidth, '~'));
    const auto& patches = predefinedPatches();

    for (int row = 0; row < mapHeight; ++row) {
        const double lat = 90.0 - (static_cast<double>(row) + 0.5) * (earthSpanLat / mapHeight);
        for (int col = 0; col < mapWidth; ++col) {
            const double lon = -180.0 + (static_cast<double>(col) + 0.5) * (earthSpanLon / mapWidth);
            double score = -1.0;
            for (const auto& patch : patches) {
                double lonDelta = wrapLonDeg(lon - patch.lonCenter);
                const double latNorm = (lat - patch.latCenter) / patch.latRadius;
                const double lonNorm = lonDelta / patch.lonRadius;
                const double ellipse = latNorm * latNorm + lonNorm * lonNorm;
                const double patchScore = patch.elevation * (1.0 - ellipse);
                if (patchScore > score) {
                    score = patchScore;
                }
            }

            char glyph = '~';
            if (lat < -70.0) {
                glyph = '*'; // Antarctic ice
            } else if (score > 0.35) {
                glyph = '#';
            } else if (score > 0.15) {
                glyph = '+';
            } else if (score > -0.1) {
                glyph = '.';
            }

            // draw equator and tropics on ocean cells
            const double latBandWidth = earthSpanLat / mapHeight;
            if (std::abs(lat) < 0.5 * latBandWidth && glyph == '~') {
                glyph = '-';
            } else if ((std::abs(lat - 23.5) < 0.5 * latBandWidth ||
                        std::abs(lat + 23.5) < 0.5 * latBandWidth) &&
                       glyph == '~') {
                glyph = '=';
            }

            rows[row][col] = glyph;
        }
    }

    return rows;
}

const std::vector<std::string>& earthAsciiMap() {
    static const std::vector<std::string> map = buildEarthAsciiMap();
    return map;
}
} // namespace

void TrajectoryRenderer::render(const std::vector<Vector2>& path,
                                const std::vector<LatLon>& groundTrack,
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

    renderGroundTrack(groundTrack, telemetry);
    std::cout.flush();
}

void TrajectoryRenderer::renderGroundTrack(const std::vector<LatLon>& groundTrack,
                                           const TelemetryPoint& telemetry) const {
    const auto& base = earthAsciiMap();
    if (base.empty()) {
        return;
    }

    auto buffer = base;
    const int mapHeight = static_cast<int>(buffer.size());
    const int mapWidth = static_cast<int>(buffer.front().size());

    auto project = [&](const LatLon& coord) {
        const double xRatio = std::clamp((coord.lonDeg + 180.0) / earthSpanLon, 0.0, 1.0);
        const double yRatio = std::clamp((90.0 - coord.latDeg) / earthSpanLat, 0.0, 1.0);
        int x = static_cast<int>(std::round(xRatio * (mapWidth - 1)));
        int y = static_cast<int>(std::round(yRatio * (mapHeight - 1)));
        x = std::clamp(x, 0, mapWidth - 1);
        y = std::clamp(y, 0, mapHeight - 1);
        return std::pair<int, int>{x, y};
    };

    if (!groundTrack.empty()) {
        const std::size_t stride = std::max<std::size_t>(1, groundTrack.size() / (mapWidth * 2));
        for (std::size_t i = 0; i < groundTrack.size(); i += stride) {
            const auto [x, y] = project(groundTrack[i]);
            buffer[y][x] = '.';
        }

        const auto [rx, ry] = project(groundTrack.back());
        buffer[ry][rx] = 'R';
    }

    std::cout << "\nEarth Ground Track  (Lat " << std::fixed << std::setprecision(2) << telemetry.latitude
              << "°, Lon " << telemetry.longitude << "°)\n";
    std::cout << "Legend: # land  + coastline  ~ ocean  R vehicle\n";
    for (const auto& line : buffer) {
        std::cout << line << '\n';
    }
}
