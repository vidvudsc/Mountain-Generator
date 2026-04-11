#include "weather.h"
#define CIMGUI_DEFINE_ENUMS_AND_STRUCTS
#include "cimgui.h"
#include "rlImGui.h"
#include <stdio.h>

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    AtmosphereSim sim = { 0 };

    InitWindow(1360, 820, "Mountain Atmosphere Solver");
    SetTargetFPS(60);

    if (!CreateSimFromGeneratedTerrain(&sim)) {
        fprintf(stderr, "Failed to initialize generated terrain.\n");
        CloseWindow();
        return 1;
    }

    rlImGuiSetup(true);
    ApplyImGuiTheme();

    Camera3D camera = { 0 };
    Vector3 target = {
        sim.worldWidth * 0.5f,
        Lerp(sim.terrain.minTerrainY, sim.terrain.maxTerrainY, 0.45f),
        sim.worldDepth * 0.5f
    };
    float orbitYaw = -0.78f;
    float orbitPitch = 0.52f;
    float orbitDistance = fmaxf(sim.worldWidth, sim.worldDepth) * 1.15f;
    bool imguiWantsMouse = false;
    bool imguiWantsKeyboard = false;

    while (!WindowShouldClose()) {
        float frameDt = GetFrameTime();
        Rectangle panel = GetControlPanelRect();
        Rectangle uiButton = GetUiToggleRect();
        Vector2 mousePosition = GetMousePosition();
        bool mouseOverFixedUI = (sim.showUI && CheckCollisionPointRec(mousePosition, panel)) ||
                                (!sim.showUI && CheckCollisionPointRec(mousePosition, uiButton));
        bool blockCameraMouse = mouseOverFixedUI || imguiWantsMouse;
        bool blockShortcuts = imguiWantsKeyboard;
        float wheelMove = blockCameraMouse ? 0.0f : GetMouseWheelMove();

        if (!blockCameraMouse && IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            Vector2 delta = GetMouseDelta();
            orbitYaw -= delta.x * 0.0040f;
            orbitPitch += delta.y * 0.0030f;
        }
        orbitPitch = Clamp(orbitPitch, 0.06f, 1.42f);

        if (!blockShortcuts && IsKeyDown(KEY_LEFT)) orbitYaw += frameDt * 1.4f;
        if (!blockShortcuts && IsKeyDown(KEY_RIGHT)) orbitYaw -= frameDt * 1.4f;
        if (!blockShortcuts && IsKeyDown(KEY_UP)) orbitPitch -= frameDt * 0.9f;
        if (!blockShortcuts && IsKeyDown(KEY_DOWN)) orbitPitch += frameDt * 0.9f;
        orbitDistance -= wheelMove * 10.0f;
        orbitDistance = Clamp(orbitDistance, 90.0f, 780.0f);

        if (IsKeyPressed(KEY_G)) sim.showUI = !sim.showUI;
        if (!blockShortcuts && IsKeyPressed(KEY_SPACE)) sim.paused = !sim.paused;
        if (!blockShortcuts && IsKeyPressed(KEY_R)) ResetAtmosphere(&sim);
        if (!blockShortcuts && IsKeyPressed(KEY_TAB)) {
            sim.fieldMode = (FieldMode)((sim.fieldMode + 1) % FIELD_COUNT);
            sim.sliceDirty = true;
        }
        if (!blockShortcuts && IsKeyPressed(KEY_H)) {
            sim.showHorizontalSlice = !sim.showHorizontalSlice;
            sim.sliceDirty = true;
        }
        if (!blockShortcuts && IsKeyPressed(KEY_J)) {
            sim.showVerticalSlice = !sim.showVerticalSlice;
            sim.sliceDirty = true;
        }
        if (!blockShortcuts && IsKeyPressed(KEY_T)) sim.showTerrain = !sim.showTerrain;

        if (!sim.paused) UpdateAtmosphere(&sim, frameDt);
        UpdateSliceTexturesIfNeeded(&sim, false);

        float camX = sinf(orbitYaw) * cosf(orbitPitch) * orbitDistance;
        float camZ = cosf(orbitYaw) * cosf(orbitPitch) * orbitDistance;
        float camY = sinf(orbitPitch) * orbitDistance;
        camera.position = Vector3Add(target, (Vector3) { camX, camY, camZ });
        camera.target = target;
        camera.up = (Vector3) { 0.0f, 1.0f, 0.0f };
        camera.fovy = 45.0f;
        camera.projection = CAMERA_PERSPECTIVE;

        BeginDrawing();
        ClearBackground(SkyColor(&sim));

        BeginMode3D(camera);
        if (sim.showTerrain) DrawModel(sim.terrainModel, (Vector3) { 0.0f, 0.0f, 0.0f }, 1.0f, Fade(WHITE, sim.terrainAlpha));
        if (sim.showWireframe) DrawModelWires(sim.terrainModel, (Vector3) { 0.0f, 0.0f, 0.0f }, 1.0f, Fade(BLACK, 0.25f));
        DrawSlices3D(&sim, camera);
        EndMode3D();

        if (sim.paused) {
            DrawRectangle(GetScreenWidth() / 2 - 62, 18, 124, 36, Fade(BLACK, 0.52f));
            DrawRectangleLines(GetScreenWidth() / 2 - 62, 18, 124, 36, Fade(RAYWHITE, 0.18f));
            DrawText("PAUSED", GetScreenWidth() / 2 - 30, 27, 20, ORANGE);
        }

        FieldMode prevFieldMode = sim.fieldMode;
        SliceAxis prevVerticalAxis = sim.verticalAxis;
        int prevHorizontalLayer = sim.horizontalLayer;
        int prevVerticalSlice = sim.verticalSlice;
        bool prevShowHorizontalSlice = sim.showHorizontalSlice;
        bool prevShowVerticalSlice = sim.showVerticalSlice;

        rlImGuiBeginDelta(frameDt);
        if (sim.showUI) DrawImGuiHud(&sim);
        bool terrainChanged = DrawImGuiControls(&sim);
        ImGuiIO *io = igGetIO_Nil();
        imguiWantsMouse = (io != NULL) ? io->WantCaptureMouse : false;
        imguiWantsKeyboard = (io != NULL) ? io->WantCaptureKeyboard : false;
        rlImGuiEnd();

        if (prevFieldMode != sim.fieldMode ||
            prevVerticalAxis != sim.verticalAxis ||
            prevHorizontalLayer != sim.horizontalLayer ||
            prevVerticalSlice != sim.verticalSlice ||
            prevShowHorizontalSlice != sim.showHorizontalSlice ||
            prevShowVerticalSlice != sim.showVerticalSlice) {
            sim.sliceDirty = true;
        }

        if (terrainChanged) {
            target = (Vector3) {
                sim.worldWidth * 0.5f,
                Lerp(sim.terrain.minTerrainY, sim.terrain.maxTerrainY, 0.45f),
                sim.worldDepth * 0.5f
            };
            sim.sliceDirty = true;
            UpdateSliceTexturesIfNeeded(&sim, true);
        }

        EndDrawing();
    }

    rlImGuiShutdown();
    DestroySim(&sim);
    CloseWindow();
    return 0;
}
