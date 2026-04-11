# 3D Mountains

This project currently focuses on the C application in [`C/main/main.c`](C/main/main.c): a procedural mountain terrain viewer with a terrain-aware atmosphere prototype, slice rendering, wind overlays, and an ImGui control panel.

## C App Overview

The C build combines three main parts:

- terrain generation with layered noise, hydraulic erosion, and a shaded mesh
- an atmosphere solver with staggered face velocities, scalar advection, warm-rain microphysics, pressure projection, and terrain masking
- a raylib + ImGui visualization layer for horizontal slices, vertical slices, solver stats, and camera controls

## Screenshots

### Main View

![Main View](docs/screenshots/main.png)


### Slice View

![Slice View](docs/screenshots/wind.png)

  
## Build

The current build script expects raylib headers and libraries to be available from Homebrew paths on macOS.

```bash
brew install raylib
bash C/build_weather.sh
```

This produces:

```text
C/weather
```

## Run

```bash
./C/weather
```


## Main C Files

- [`C/main/main.c`](C/main/main.c)
  App entry point. It creates the window, sets up raylib + ImGui, handles camera orbit/input, runs the frame loop, and wires terrain and weather together.

- [`C/main/terrain.c`](C/main/terrain.c)
  Terrain system. It generates the mountain heightmap, applies erosion/smoothing, computes terrain derivatives, and builds the renderable terrain mesh.

- [`C/main/weather.c`](C/main/weather.c)
  Atmosphere and UI system. It owns the simulation state, timestep update path, diagnostics, slice textures, wind overlays, HUD, and control panel.

## Shared Headers

- [`C/main/sim.h`](C/main/sim.h)
  Shared simulation types, constants, indexing helpers, and sampling helpers used by both terrain and weather.

- [`C/main/terrain.h`](C/main/terrain.h)
  Terrain-facing declarations exported to the rest of the app.

- [`C/main/weather.h`](C/main/weather.h)
  Weather/app-facing declarations exported to `main.c`.

## Archived Reference

- [`C/save/main.c`](C/save/main.c)
  Preserved monolithic version kept as a reference copy. The current build does not use it.



## Controls

- `Left mouse drag`: orbit camera
- `Mouse wheel`: zoom
- `Space`: pause or resume the solver
- `R`: reset the atmosphere state
- `Tab`: cycle displayed field
- `H`: toggle horizontal slice
- `J`: toggle vertical slice
- `T`: toggle terrain
- `G`: hide or show the control panel

## Displayed Fields

The viewer can inspect:

- temperature
- pressure
- relative humidity
- dew point
- water vapor
- cloud water
- rain water
- wind speed
- vertical wind
- buoyancy
