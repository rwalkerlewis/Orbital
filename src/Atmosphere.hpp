#pragma once

#include <vector>

struct AtmosphereSample {
    double temperature;     // Kelvin
    double pressure;        // Pascals
    double density;         // kg/m^3
    double speedOfSound;    // m/s

    [[nodiscard]] double dynamicPressure(double velocity) const {
        return 0.5 * density * velocity * velocity;
    }
};

class AtmosphereModel {
public:
    AtmosphereModel();

    [[nodiscard]] AtmosphereSample sample(double altitudeMeters) const;

private:
    struct Layer {
        double baseAltitude;
        double lapseRate;
        double baseTemperature;
        double basePressure;
    };

    std::vector<Layer> layers_;
};
