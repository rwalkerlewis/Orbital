#include <exception>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "Rocket.hpp"
#include "Simulation.hpp"
#include "SimulationTypes.hpp"

namespace {
void printUsage(const char* executable) {
    std::cout << "Usage: " << executable << " [options]\n"
              << "Options:\n"
              << "  --rocket <id>           Select rocket configuration (default: falcon9)\n"
              << "  --site <id>             Select launch site (default: ksc)\n"
              << "  --no-visual             Disable live ASCII visualization\n"
              << "  --dt <seconds>          Integrator time step (default: 0.25)\n"
              << "  --duration <seconds>    Maximum simulation time (default: 600)\n"
              << "  --width <cols>          Visualization width (default: 100)\n"
              << "  --height <rows>         Visualization height (default: 30)\n"
              << "  --vertical-hold <sec>   Vertical ascent duration (default: 10)\n"
              << "  --pitch-duration <sec>  Pitch program duration (default: 160)\n"
              << "  --final-pitch <deg>     Final pitch angle relative to horizon (default: 5)\n"
              << "  --gravity-turn-alt <m>  Altitude to begin gravity turn (default: 15000)\n"
              << "  --list-rockets          Print available rocket ids\n"
              << "  --list-sites            Print available launch site ids\n"
              << "  -h, --help              Show this help message\n";
}

void listRockets() {
    std::cout << "Available rockets:\n";
    for (const auto& id : availableRocketIds()) {
        std::cout << "  - " << id << '\n';
    }
}

void listSites() {
    std::cout << "Available launch sites:\n";
    for (const auto& site : builtinLaunchSites()) {
        std::cout << "  - " << site.id << " (" << site.name << ", " << site.location << ")\n";
    }
}

} // namespace

int main(int argc, char** argv) {
    std::string rocketId = "falcon9";
    std::string siteId = "ksc";
    SimulationSettings settings;

    bool listRocketFlag = false;
    bool listSiteFlag = false;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--rocket" && i + 1 < argc) {
                rocketId = argv[++i];
            } else if (arg == "--site" && i + 1 < argc) {
                siteId = argv[++i];
            } else if (arg == "--dt" && i + 1 < argc) {
                settings.timeStep = std::stod(argv[++i]);
            } else if (arg == "--duration" && i + 1 < argc) {
                settings.maxDuration = std::stod(argv[++i]);
            } else if (arg == "--width" && i + 1 < argc) {
                settings.visualizationWidth = std::stoi(argv[++i]);
            } else if (arg == "--height" && i + 1 < argc) {
                settings.visualizationHeight = std::stoi(argv[++i]);
            } else if (arg == "--vertical-hold" && i + 1 < argc) {
                settings.guidance.verticalHoldSeconds = std::stod(argv[++i]);
            } else if (arg == "--pitch-duration" && i + 1 < argc) {
                settings.guidance.pitchProgramDuration = std::stod(argv[++i]);
            } else if (arg == "--final-pitch" && i + 1 < argc) {
                settings.guidance.finalPitchDegrees = std::stod(argv[++i]);
            } else if (arg == "--gravity-turn-alt" && i + 1 < argc) {
                settings.guidance.gravityTurnStartAltitude = std::stod(argv[++i]);
            } else if (arg == "--max-pitch-rate" && i + 1 < argc) {
                settings.guidance.maxPitchRateDegPerSec = std::stod(argv[++i]);
            } else if (arg == "--no-visual") {
                settings.enableVisualization = false;
            } else if (arg == "--list-rockets") {
                listRocketFlag = true;
            } else if (arg == "--list-sites") {
                listSiteFlag = true;
            } else if (arg == "-h" || arg == "--help") {
                printUsage(argv[0]);
                return 0;
            } else {
                std::cerr << "Unknown argument: " << arg << "\n";
                printUsage(argv[0]);
                return 1;
            }
        }

        if (listRocketFlag) {
            listRockets();
            return 0;
        }

        if (listSiteFlag) {
            listSites();
            return 0;
        }

        if (settings.timeStep <= 0.0) {
            throw std::runtime_error("Time step must be positive");
        }

        RocketVehicle rocket = makeRocket(rocketId);
        LaunchSite site = findLaunchSite(siteId);

        TrajectorySimulation simulation(site, std::move(rocket), settings);
        const auto result = simulation.run();

        if (!settings.enableVisualization && !result.telemetry.empty()) {
            const auto& finalPoint = result.telemetry.back();
            std::cout << std::fixed << std::setprecision(2);
            std::cout << "Final time: " << finalPoint.time << " s\n"
                      << "Altitude: " << finalPoint.altitude / 1000.0 << " km\n"
                      << "Downrange: " << finalPoint.downrange / 1000.0 << " km\n"
                      << "Speed: " << finalPoint.speed / 1000.0 << " km/s\n";
        }

        std::cout << std::fixed << std::setprecision(2)
                  << "Peak Altitude: " << result.maxAltitude / 1000.0 << " km | "
                  << "Peak Downrange: " << result.maxDownrange / 1000.0 << " km\n";

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "Simulation failed: " << ex.what() << "\n";
        return 1;
    }
}
