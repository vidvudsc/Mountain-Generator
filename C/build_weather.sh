#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$ROOT_DIR/C/.build/weather_imgui"
OUTPUT_BIN="$ROOT_DIR/C/weather"

mkdir -p "$BUILD_DIR"

CFLAGS=(
  -std=c99
  -O2
  -Wno-typedef-redefinition
  -I"$ROOT_DIR/third_party/cimgui"
  -I"$ROOT_DIR/third_party/rlImGui"
  -I"$ROOT_DIR/third_party/rlImGui/extras"
  -I/opt/homebrew/include
)

CPPFLAGS=(
  -std=c++17
  -O2
  -I"$ROOT_DIR/third_party/cimgui"
  -I"$ROOT_DIR/third_party/cimgui/imgui"
  -I"$ROOT_DIR/third_party/rlImGui"
  -I"$ROOT_DIR/third_party/rlImGui/extras"
  -I/opt/homebrew/include
)

clang "${CFLAGS[@]}" -c "$ROOT_DIR/C/weather.c" -o "$BUILD_DIR/weather.o"

clang++ "${CPPFLAGS[@]}" -c "$ROOT_DIR/third_party/cimgui/cimgui.cpp" -o "$BUILD_DIR/cimgui.o"
clang++ "${CPPFLAGS[@]}" -c "$ROOT_DIR/third_party/cimgui/imgui/imgui.cpp" -o "$BUILD_DIR/imgui.o"
clang++ "${CPPFLAGS[@]}" -c "$ROOT_DIR/third_party/cimgui/imgui/imgui_demo.cpp" -o "$BUILD_DIR/imgui_demo.o"
clang++ "${CPPFLAGS[@]}" -c "$ROOT_DIR/third_party/cimgui/imgui/imgui_draw.cpp" -o "$BUILD_DIR/imgui_draw.o"
clang++ "${CPPFLAGS[@]}" -c "$ROOT_DIR/third_party/cimgui/imgui/imgui_tables.cpp" -o "$BUILD_DIR/imgui_tables.o"
clang++ "${CPPFLAGS[@]}" -c "$ROOT_DIR/third_party/cimgui/imgui/imgui_widgets.cpp" -o "$BUILD_DIR/imgui_widgets.o"
clang++ "${CPPFLAGS[@]}" -c "$ROOT_DIR/third_party/rlImGui/rlImGui.cpp" -o "$BUILD_DIR/rlImGui.o"

clang++ \
  "$BUILD_DIR/weather.o" \
  "$BUILD_DIR/cimgui.o" \
  "$BUILD_DIR/imgui.o" \
  "$BUILD_DIR/imgui_demo.o" \
  "$BUILD_DIR/imgui_draw.o" \
  "$BUILD_DIR/imgui_tables.o" \
  "$BUILD_DIR/imgui_widgets.o" \
  "$BUILD_DIR/rlImGui.o" \
  -L/opt/homebrew/lib \
  -lraylib \
  -lm \
  -framework OpenGL \
  -framework Cocoa \
  -framework IOKit \
  -framework CoreVideo \
  -o "$OUTPUT_BIN"

echo "Built $OUTPUT_BIN"
