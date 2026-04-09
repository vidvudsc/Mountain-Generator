#include "raylib.h"
#include "raymath.h"
#include "terrain.h"
#include "weather.h"
#define CIMGUI_DEFINE_ENUMS_AND_STRUCTS
#include "cimgui.h"
#include "rlImGui.h"
#include <float.h>
#include <stdbool.h>
#include <stdio.h>

static ImVec2_c ImVec2Make(float x, float y) {
    ImVec2_c value = { x, y };
    return value;
}

static ImVec4_c ImVec4Make(float x, float y, float z, float w) {
    ImVec4_c value = { x, y, z, w };
    return value;
}

static Rectangle GetControlPanelRect(void) {
    const float width = 420.0f;
    const float margin = 12.0f;
    return (Rectangle) { GetScreenWidth() - width - margin, margin, width, GetScreenHeight() - margin * 2.0f };
}

static Rectangle GetUiToggleRect(void) {
    const float margin = 12.0f;
    return (Rectangle) { GetScreenWidth() - 132.0f - margin, margin, 132.0f, 36.0f };
}

static void ApplyImGuiTheme(void) {
    igStyleColorsDark(NULL);
    ImGuiStyle *style = igGetStyle();
    style->WindowPadding = ImVec2Make(14.0f, 12.0f);
    style->WindowRounding = 10.0f;
    style->FramePadding = ImVec2Make(9.0f, 6.0f);
    style->FrameRounding = 7.0f;
    style->ItemSpacing = ImVec2Make(10.0f, 8.0f);
    style->ScrollbarSize = 13.0f;
    style->ScrollbarRounding = 10.0f;
    style->GrabRounding = 6.0f;
    style->TabRounding = 6.0f;

    style->Colors[ImGuiCol_Text] = ImVec4Make(0.90f, 0.93f, 0.97f, 1.00f);
    style->Colors[ImGuiCol_WindowBg] = ImVec4Make(0.06f, 0.08f, 0.10f, 0.98f);
    style->Colors[ImGuiCol_ChildBg] = ImVec4Make(0.07f, 0.10f, 0.13f, 0.94f);
    style->Colors[ImGuiCol_Border] = ImVec4Make(0.20f, 0.28f, 0.36f, 1.00f);
    style->Colors[ImGuiCol_FrameBg] = ImVec4Make(0.10f, 0.14f, 0.18f, 1.00f);
    style->Colors[ImGuiCol_FrameBgHovered] = ImVec4Make(0.14f, 0.22f, 0.29f, 1.00f);
    style->Colors[ImGuiCol_FrameBgActive] = ImVec4Make(0.17f, 0.28f, 0.37f, 1.00f);
    style->Colors[ImGuiCol_TitleBg] = ImVec4Make(0.05f, 0.11f, 0.18f, 1.00f);
    style->Colors[ImGuiCol_TitleBgActive] = ImVec4Make(0.08f, 0.18f, 0.28f, 1.00f);
    style->Colors[ImGuiCol_Button] = ImVec4Make(0.12f, 0.25f, 0.39f, 1.00f);
    style->Colors[ImGuiCol_ButtonHovered] = ImVec4Make(0.17f, 0.35f, 0.53f, 1.00f);
    style->Colors[ImGuiCol_ButtonActive] = ImVec4Make(0.22f, 0.44f, 0.66f, 1.00f);
    style->Colors[ImGuiCol_Header] = ImVec4Make(0.11f, 0.23f, 0.36f, 1.00f);
    style->Colors[ImGuiCol_HeaderHovered] = ImVec4Make(0.16f, 0.34f, 0.52f, 1.00f);
    style->Colors[ImGuiCol_HeaderActive] = ImVec4Make(0.21f, 0.42f, 0.63f, 1.00f);
    style->Colors[ImGuiCol_Tab] = ImVec4Make(0.08f, 0.13f, 0.19f, 1.00f);
    style->Colors[ImGuiCol_TabSelected] = ImVec4Make(0.13f, 0.26f, 0.41f, 1.00f);
}

static bool DrawLabeledSliderFloat(const char *label, const char *id, float *value, float minValue, float maxValue, const char *format) {
    igTextDisabled("%s", label);
    return igSliderFloat(id, value, minValue, maxValue, format, ImGuiSliderFlags_None);
}

static bool DrawLabeledSliderInt(const char *label, const char *id, int *value, int minValue, int maxValue, const char *format) {
    igTextDisabled("%s", label);
    return igSliderInt(id, value, minValue, maxValue, format, ImGuiSliderFlags_None);
}

static bool DrawCombo(const char *label, const char *id, int *currentItem, const char *const items[], int itemCount) {
    igTextDisabled("%s", label);
    return igCombo_Str_arr(id, currentItem, items, itemCount, itemCount);
}

static Color SkyColor(const WeatherSystem *weather) {
    float dayProgress = fmodf(weather->simTimeSeconds, 86400.0f) / 86400.0f;
    float sun = fmaxf(0.0f, sinf(6.28318530718f * (dayProgress - 0.25f)));
    Color night = (Color){ 10, 18, 32, 255 };
    Color dawn = (Color){ 227, 153, 108, 255 };
    Color noon = (Color){ 120, 176, 228, 255 };
    Color overcast = (Color){ 149, 158, 170, 255 };
    Color base = (sun > 0.08f) ? ColorLerp(dawn, noon, sun) : ColorLerp(night, dawn, sun * 6.0f);
    float cloudMix = Clamp(weather->stats.maxCloud * 180.0f, 0.0f, 0.55f);
    return ColorLerp(base, overcast, cloudMix);
}

static void DrawHud(const TerrainSurface *terrain, const WeatherSystem *weather) {
    igSetNextWindowPos(ImVec2Make(14.0f, 14.0f), ImGuiCond_Always, ImVec2Make(0.0f, 0.0f));
    igSetNextWindowBgAlpha(0.78f);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
                             ImGuiWindowFlags_AlwaysAutoResize |
                             ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoSavedSettings |
                             ImGuiWindowFlags_NoNav |
                             ImGuiWindowFlags_NoFocusOnAppearing;

    if (igBegin("##StatusHud", NULL, flags)) {
        int hour = ((int)(weather->simTimeSeconds / 3600.0f)) % 24;
        int minute = ((int)(weather->simTimeSeconds / 60.0f)) % 60;
        igText("Clean Restart Prototype");
        igSeparator();
        igText("Time %02d:%02d", hour, minute);
        igText("Terrain %.0f to %.0f m", terrain->minHeight, terrain->maxHeight);
        igText("Surface %.1f C", weather->stats.meanSurfaceTemp);
        igText("Humidity %.0f %%", weather->stats.meanHumidity);
        igText("Wind %.1f m/s", weather->stats.maxWind);
        igText("Cloud %.2f g/kg", weather->stats.maxCloud * 1000.0f);
        igText("Rain %.1f mm/h", weather->stats.maxRain);
        igText("Steps %d", weather->stats.stepsLastFrame);
        igText("Backlog %.1f s", weather->stats.backlogSeconds);
        igText("Solver %.2f ms", weather->stats.solverCpuMilliseconds);
    }
    igEnd();
}

static bool DrawControls(TerrainSurface *terrain, TerrainConfig *terrainConfig, WeatherSystem *weather, bool *showUI,
                         float *terrainAlpha, bool *showWireframe) {
    bool terrainChanged = false;

    if (!*showUI) {
        Rectangle toggleRect = GetUiToggleRect();
        igSetNextWindowPos(ImVec2Make(toggleRect.x, toggleRect.y), ImGuiCond_Always, ImVec2Make(0.0f, 0.0f));
        igSetNextWindowBgAlpha(0.92f);
        if (igBegin("##ShowControls", NULL,
                    ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove |
                        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings)) {
            if (igButton("Show Controls", ImVec2Make(toggleRect.width - 18.0f, 0.0f))) *showUI = true;
        }
        igEnd();
        return false;
    }

    static const char *kSliceAxisLabels[] = { "X Slice", "Z Slice" };
    const char *fieldLabels[WEATHER_FIELD_COUNT];
    for (int i = 0; i < WEATHER_FIELD_COUNT; i++) fieldLabels[i] = Weather_FieldLabel((WeatherField)i);

    Rectangle panel = GetControlPanelRect();
    igSetNextWindowPos(ImVec2Make(panel.x, panel.y), ImGuiCond_Always, ImVec2Make(0.0f, 0.0f));
    igSetNextWindowSize(ImVec2Make(panel.width, panel.height), ImGuiCond_Always);
    igSetNextWindowBgAlpha(0.98f);

    if (igBegin("Terrain + Weather", NULL,
                ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings |
                    ImGuiWindowFlags_NoCollapse)) {
        if (igButton(weather->view.paused ? "Resume" : "Pause", ImVec2Make(110.0f, 0.0f))) weather->view.paused = !weather->view.paused;
        igSameLine(0.0f, 8.0f);
        if (igButton("Reset Weather", ImVec2Make(128.0f, 0.0f))) {
            Weather_Reset(weather, terrain);
        }
        igSameLine(0.0f, 8.0f);
        if (igButton("Hide Panel", ImVec2Make(106.0f, 0.0f))) *showUI = false;

        igSeparator();
        if (igBeginChild_Str("##MainScroll", ImVec2Make(0.0f, 0.0f), ImGuiChildFlags_Borders, ImGuiWindowFlags_None)) {
            igPushItemWidth(-FLT_MIN);

            if (igBeginTabBar("##Tabs", ImGuiTabBarFlags_None)) {
                if (igBeginTabItem("Weather", NULL, ImGuiTabItemFlags_None)) {
                    igSeparatorText("View");
                    int field = (int)weather->view.field;
                    if (DrawCombo("Field", "##Field", &field, fieldLabels, WEATHER_FIELD_COUNT)) {
                        weather->view.field = (WeatherField)field;
                        Weather_MarkViewDirty(weather);
                    }
                    igCheckbox("Horizontal slice", &weather->view.showHorizontalSlice);
                    DrawLabeledSliderInt("Horizontal layer", "##Layer", &weather->view.horizontalLayer, 0, weather->config.ny - 1, "%d");
                    igCheckbox("Vertical slice", &weather->view.showVerticalSlice);
                    int axis = (int)weather->view.verticalAxis;
                    if (DrawCombo("Vertical axis", "##Axis", &axis, kSliceAxisLabels, 2)) {
                        weather->view.verticalAxis = (WeatherSliceAxis)axis;
                        Weather_MarkViewDirty(weather);
                    }
                    {
                        int sliceMax = (weather->view.verticalAxis == WEATHER_SLICE_X) ? weather->config.nx - 1 : weather->config.nz - 1;
                        weather->view.verticalSlice = Clamp(weather->view.verticalSlice, 0, sliceMax);
                        DrawLabeledSliderInt("Slice index", "##Slice", &weather->view.verticalSlice, 0, sliceMax, "%d");
                    }
                    igCheckbox("Wind vectors", &weather->view.showWindVectors);

                    igSeparatorText("Simulation");
                    DrawLabeledSliderFloat("Time scale", "##TimeScale", &weather->config.timeScale, 20.0f, 600.0f, "%.0fx");
                    igTextWrapped("This is a clean restart architecture. Terrain and weather are split into separate modules so the next iterations can improve realism without dragging a monolithic file around.");
                    igText("Field units: %s", Weather_FieldUnits(weather->view.field));
                    igText("Steps/frame: %d", weather->stats.stepsLastFrame);
                    igText("Backlog: %.1f s", weather->stats.backlogSeconds);
                    igText("Solver CPU: %.2f ms", weather->stats.solverCpuMilliseconds);
                    igEndTabItem();
                }

                if (igBeginTabItem("Terrain", NULL, ImGuiTabItemFlags_None)) {
                    igSeparatorText("Generation");
                    DrawLabeledSliderInt("Seed", "##Seed", &terrainConfig->seed, 1, 999999, "%d");
                    if (igButton("Randomize Seed", ImVec2Make(150.0f, 0.0f))) {
                        terrainConfig->seed = GetRandomValue(1, 999999);
                    }
                    igSameLine(0.0f, 8.0f);
                    if (igButton("Rebuild Terrain", ImVec2Make(150.0f, 0.0f))) {
                        terrainChanged = Terrain_Rebuild(terrain, terrainConfig);
                    }

                    igSeparatorText("Shape");
                    DrawLabeledSliderFloat("Base frequency", "##BaseFreq", &terrainConfig->baseFrequency, 2.0f, 10.0f, "%.2f");
                    DrawLabeledSliderFloat("Warp frequency", "##WarpFreq", &terrainConfig->warpFrequency, 1.0f, 6.0f, "%.2f");
                    DrawLabeledSliderFloat("Height scale", "##Height", &terrainConfig->heightScale, 50.0f, 180.0f, "%.0f");
                    DrawLabeledSliderFloat("Ridge sharpness", "##Ridge", &terrainConfig->ridgeSharpness, 1.1f, 3.0f, "%.2f");
                    DrawLabeledSliderFloat("Peak bias", "##PeakBias", &terrainConfig->peakBias, 1.0f, 2.2f, "%.2f");
                    DrawLabeledSliderInt("Octaves", "##Octaves", &terrainConfig->octaves, 3, 7, "%d");
                    DrawLabeledSliderInt("Smooth passes", "##Smooth", &terrainConfig->smoothingPasses, 0, 8, "%d");
                    DrawLabeledSliderInt("Thermal passes", "##Thermal", &terrainConfig->thermalPasses, 0, 32, "%d");

                    igSeparatorText("Render");
                    igCheckbox("Wireframe", showWireframe);
                    DrawLabeledSliderFloat("Terrain alpha", "##TerrainAlpha", terrainAlpha, 0.05f, 1.0f, "%.2f");
                    igText("Range: %.0f to %.0f m", terrain->minHeight, terrain->maxHeight);
                    igEndTabItem();
                }

                igEndTabBar();
            }

            igPopItemWidth();
        }
        igEndChild();
    }
    igEnd();
    return terrainChanged;
}

int main(void) {
    TerrainConfig terrainConfig;
    TerrainConfig_Default(&terrainConfig, GetRandomValue(1, 999999));
    TerrainSurface terrain = { 0 };
    WeatherConfig weatherConfig = { 0 };
    WeatherSystem weather = { 0 };

    InitWindow(1360, 820, "3D Mountains Clean Restart");
    SetTargetFPS(60);

    if (!Terrain_Init(&terrain, &terrainConfig)) {
        fprintf(stderr, "Failed to initialize terrain.\n");
        CloseWindow();
        return 1;
    }

    WeatherConfig_Default(&weatherConfig, &terrain);
    if (!Weather_Init(&weather, &weatherConfig, &terrain)) {
        fprintf(stderr, "Failed to initialize weather.\n");
        Terrain_Shutdown(&terrain);
        CloseWindow();
        return 1;
    }

    rlImGuiSetup(true);
    ApplyImGuiTheme();

    bool showUI = true;
    bool showWireframe = false;
    float terrainAlpha = 1.0f;

    Camera3D camera = { 0 };
    Vector3 target = {
        terrain.worldWidth * 0.5f,
        Lerp(terrain.minHeight, terrain.maxHeight, 0.38f),
        terrain.worldDepth * 0.5f
    };
    float orbitYaw = -0.78f;
    float orbitPitch = 0.52f;
    float orbitDistance = fmaxf(terrain.worldWidth, terrain.worldDepth) * 1.08f;
    bool imguiWantsMouse = false;
    bool imguiWantsKeyboard = false;

    while (!WindowShouldClose()) {
        float frameDt = GetFrameTime();
        Rectangle panel = GetControlPanelRect();
        Rectangle uiButton = GetUiToggleRect();
        Vector2 mousePosition = GetMousePosition();
        bool mouseOverFixedUI = (showUI && CheckCollisionPointRec(mousePosition, panel)) ||
                                (!showUI && CheckCollisionPointRec(mousePosition, uiButton));
        bool blockCameraMouse = mouseOverFixedUI || imguiWantsMouse;
        bool blockShortcuts = imguiWantsKeyboard;
        float wheelMove = blockCameraMouse ? 0.0f : GetMouseWheelMove();

        if (!blockCameraMouse && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 delta = GetMouseDelta();
            orbitYaw -= delta.x * 0.0040f;
            orbitPitch += delta.y * 0.0030f;
        }
        orbitPitch = Clamp(orbitPitch, 0.08f, 1.42f);

        if (!blockShortcuts && IsKeyDown(KEY_LEFT)) orbitYaw += frameDt * 1.4f;
        if (!blockShortcuts && IsKeyDown(KEY_RIGHT)) orbitYaw -= frameDt * 1.4f;
        if (!blockShortcuts && IsKeyDown(KEY_UP)) orbitPitch -= frameDt * 0.9f;
        if (!blockShortcuts && IsKeyDown(KEY_DOWN)) orbitPitch += frameDt * 0.9f;
        orbitDistance -= wheelMove * 10.0f;
        orbitDistance = Clamp(orbitDistance, 90.0f, 780.0f);

        if (!blockShortcuts && IsKeyPressed(KEY_SPACE)) weather.view.paused = !weather.view.paused;
        if (!blockShortcuts && IsKeyPressed(KEY_R)) Weather_Reset(&weather, &terrain);
        if (!blockShortcuts && IsKeyPressed(KEY_G)) showUI = !showUI;
        if (!blockShortcuts && IsKeyPressed(KEY_TAB)) {
            weather.view.field = (WeatherField)((weather.view.field + 1) % WEATHER_FIELD_COUNT);
            Weather_MarkViewDirty(&weather);
        }
        if (!blockShortcuts && IsKeyPressed(KEY_H)) {
            weather.view.showHorizontalSlice = !weather.view.showHorizontalSlice;
            Weather_MarkViewDirty(&weather);
        }
        if (!blockShortcuts && IsKeyPressed(KEY_J)) {
            weather.view.showVerticalSlice = !weather.view.showVerticalSlice;
            Weather_MarkViewDirty(&weather);
        }

        if (!weather.view.paused) {
            Weather_Update(&weather, &terrain, frameDt);
        }
        Weather_UpdateSliceTextures(&weather, false);

        float camX = sinf(orbitYaw) * cosf(orbitPitch) * orbitDistance;
        float camZ = cosf(orbitYaw) * cosf(orbitPitch) * orbitDistance;
        float camY = sinf(orbitPitch) * orbitDistance;
        camera.position = Vector3Add(target, (Vector3){ camX, camY, camZ });
        camera.target = target;
        camera.up = (Vector3){ 0.0f, 1.0f, 0.0f };
        camera.fovy = 45.0f;
        camera.projection = CAMERA_PERSPECTIVE;

        BeginDrawing();
        ClearBackground(SkyColor(&weather));

        BeginMode3D(camera);
        Terrain_Draw(&terrain, terrainAlpha, showWireframe);
        Weather_DrawSlices3D(&weather);
        Weather_DrawWindVectors(&weather);
        EndMode3D();

        rlImGuiBeginDelta(frameDt);
        if (showUI) DrawHud(&terrain, &weather);

        WeatherField prevField = weather.view.field;
        WeatherSliceAxis prevAxis = weather.view.verticalAxis;
        int prevLayer = weather.view.horizontalLayer;
        int prevSlice = weather.view.verticalSlice;
        bool prevH = weather.view.showHorizontalSlice;
        bool prevV = weather.view.showVerticalSlice;

        bool terrainChanged = DrawControls(&terrain, &terrainConfig, &weather, &showUI, &terrainAlpha, &showWireframe);
        ImGuiIO *io = igGetIO_Nil();
        imguiWantsMouse = (io != NULL) ? io->WantCaptureMouse : false;
        imguiWantsKeyboard = (io != NULL) ? io->WantCaptureKeyboard : false;
        rlImGuiEnd();

        if (prevField != weather.view.field || prevAxis != weather.view.verticalAxis || prevLayer != weather.view.horizontalLayer ||
            prevSlice != weather.view.verticalSlice || prevH != weather.view.showHorizontalSlice || prevV != weather.view.showVerticalSlice) {
            Weather_MarkViewDirty(&weather);
        }

        if (terrainChanged) {
            weather.config.topHeight = terrain.maxHeight + 180.0f;
            Weather_Shutdown(&weather);
            WeatherConfig_Default(&weatherConfig, &terrain);
            if (!Weather_Init(&weather, &weatherConfig, &terrain)) {
                fprintf(stderr, "Failed to rebuild weather after terrain update.\n");
                rlImGuiShutdown();
                Terrain_Shutdown(&terrain);
                CloseWindow();
                return 1;
            }
            target = (Vector3){
                terrain.worldWidth * 0.5f,
                Lerp(terrain.minHeight, terrain.maxHeight, 0.38f),
                terrain.worldDepth * 0.5f
            };
            Weather_UpdateSliceTextures(&weather, true);
        }

        EndDrawing();
    }

    rlImGuiShutdown();
    Weather_Shutdown(&weather);
    Terrain_Shutdown(&terrain);
    CloseWindow();
    return 0;
}
