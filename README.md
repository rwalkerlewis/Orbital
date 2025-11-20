# Orbital Launch Simulator

`orbital` is a self-contained C++ simulator that models realistic multi-stage launch vehicles departing from real-world launch sites. The physics core includes:

- Standard-atmosphere pressure, density, and speed-of-sound sampling up to 86 km
- Multi-stage liquid engines with altitude-dependent thrust/ISP and propellant depletion
- 2D translational dynamics with gravity losses, aerodynamic drag, and dynamic-pressure-aware throttling
- An adaptive pitch program (vertical rise, gravity turn, rate-limited slew)
- ASCII-based live visualization that renders the trajectory, max-Q statistics, and stage telemetry in real time

## Building

```bash
cmake -S . -B build
cmake --build build
```

> **Heads-up:** On some minimal environments you may need to install the standard C++ runtime (`libstdc++-dev`). If `cmake` fails with `cannot find -lstdc++`, install the missing toolchain packages before retrying.

## Running

From the `build` directory:

```bash
./orbital --rocket falcon9 --site ksc
```

Command-line options:

| Option | Description |
| --- | --- |
| `--rocket <id>` | Choose `falcon9` (default) or `soyuz` preset |
| `--site <id>` | Choose `ksc`, `vafb`, `csG`, or `tanegashima` |
| `--no-visual` | Disable the live ASCII renderer (useful for CI) |
| `--dt <s>` | Integrator time step (default `0.25`) |
| `--duration <s>` | Maximum simulation duration (default `600`) |
| `--width <cols>`, `--height <rows>` | Visualization resolution |
| `--vertical-hold <s>` | Vertical ascent hold before pitching |
| `--pitch-duration <s>` | Time to complete the pitch program |
| `--final-pitch <deg>` | Target pitch angle relative to horizon |
| `--gravity-turn-alt <m>` | Altitude that triggers gravity turn blending |
| `--max-pitch-rate <deg/s>` | Rate limit for guidance commands |
| `--list-rockets`, `--list-sites` | Inspect available presets |

Example: simulate a polar launch from Vandenberg with a Soyuz-class stack and a coarser time step:

```bash
./orbital --rocket soyuz --site vafb --dt 0.5 --duration 900
```

## Project structure

- `src/Atmosphere.*` – ICAO-standard atmosphere sampler
- `src/Rocket.*` – Stage/engine definitions with throttle and drag models
- `src/SimulationTypes.hpp` – Shared math types and configuration structs
- `src/Simulation.*` – Integrator, guidance computer, launch-site catalog
- `src/Visualization.*` – Terminal renderer for live trajectory plots
- `src/main.cpp` – CLI entry point and preset wiring

All files remain ASCII-only for portability. The code targets C++20 and avoids external dependencies beyond the standard library.