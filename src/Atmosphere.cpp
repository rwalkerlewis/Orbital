#include "Atmosphere.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace {
constexpr double g0 = 9.80665;
constexpr double R = 287.05287;
constexpr double gammaAir = 1.4;
constexpr double seaLevelTemp = 288.15;
constexpr double seaLevelPressure = 101325.0;
} // namespace

AtmosphereModel::AtmosphereModel() {
    struct RawLayer {
        double baseAltitude;
        double lapseRate;
    };

    const std::array<RawLayer, 7> rawLayers{{
        {0.0, -0.0065},
        {11000.0, 0.0},
        {20000.0, 0.001},
        {32000.0, 0.0028},
        {47000.0, 0.0},
        {51000.0, -0.0028},
        {71000.0, -0.002},
    }};

    double temperature = seaLevelTemp;
    double pressure = seaLevelPressure;

    for (size_t i = 0; i < rawLayers.size(); ++i) {
        const auto& raw = rawLayers[i];
        layers_.push_back({raw.baseAltitude, raw.lapseRate, temperature, pressure});

        if (i + 1 >= rawLayers.size()) {
            continue;
        }

        const double nextBase = rawLayers[i + 1].baseAltitude;
        const double deltaH = nextBase - raw.baseAltitude;

        if (std::abs(raw.lapseRate) > 1e-9) {
            const double topTemp = temperature + raw.lapseRate * deltaH;
            const double exponent = -g0 / (raw.lapseRate * R);
            pressure = pressure * std::pow(topTemp / temperature, exponent);
            temperature = topTemp;
        } else {
            const double exponent = (-g0 * deltaH) / (R * temperature);
            pressure = pressure * std::exp(exponent);
            // temperature unchanged
        }
    }
}

AtmosphereSample AtmosphereModel::sample(double altitudeMeters) const {
    if (altitudeMeters < 0.0) {
        altitudeMeters = 0.0;
    }

    const Layer* layer = &layers_.front();

    for (const auto& candidate : layers_) {
        if (altitudeMeters >= candidate.baseAltitude) {
            layer = &candidate;
        } else {
            break;
        }
    }

    const double deltaH = altitudeMeters - layer->baseAltitude;
    double temperature = layer->baseTemperature + layer->lapseRate * deltaH;
    double pressure = layer->basePressure;

    if (std::abs(layer->lapseRate) > 1e-9) {
        const double term = 1.0 + (layer->lapseRate * deltaH) / layer->baseTemperature;
        const double exponent = -g0 / (layer->lapseRate * R);
        pressure = layer->basePressure * std::pow(term, exponent);
    } else {
        const double exponent = (-g0 * deltaH) / (R * layer->baseTemperature);
        pressure = layer->basePressure * std::exp(exponent);
    }

    const double density = pressure / (R * temperature);
    const double speedOfSound = std::sqrt(gammaAir * R * temperature);

    return AtmosphereSample{
        .temperature = temperature,
        .pressure = pressure,
        .density = density,
        .speedOfSound = speedOfSound,
    };
}
