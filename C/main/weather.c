#include "weather.h"
#include "rlgl.h"
#define CIMGUI_DEFINE_ENUMS_AND_STRUCTS
#include "cimgui.h"
#include "rlImGui.h"
#include <stdio.h>

static const char *const kFieldLabels[FIELD_COUNT] = {
    "Temperature",
    "Pressure",
    "Relative Humidity",
    "Dew Point",
    "Water Vapor",
    "Cloud Water",
    "Rain Water",
    "Wind Speed",
    "Vertical Wind",
    "Buoyancy"
};

static const char *const kSliceAxisLabels[2] = {
    "X Slice",
    "Z Slice"
};

typedef struct SimViewSettings {
    FieldMode fieldMode;
    SliceAxis verticalAxis;
    int horizontalLayer;
    int verticalSlice;
    bool showHorizontalSlice;
    bool showVerticalSlice;
    bool showTerrain;
    bool showWireframe;
    bool showWindArrows;
    bool paused;
    bool showUI;
    float terrainAlpha;
    float timeScale;
} SimViewSettings;

typedef struct AtmosForcing {
    float solar;
    float humidityTarget;
    float synopticU;
    float synopticV;
    Vector3 sunDir;
} AtmosForcing;

static bool RebuildGeneratedTerrain(AtmosphereSim *sim);
static void FieldRange(FieldMode mode, float *outMin, float *outMax);
static float FieldDisplayValue(const AtmosphereSim *sim, int x, int y, int z, FieldMode mode);
static void UpdateStats(AtmosphereSim *sim);

static const Color kSpectralStops[] = {
    { 44, 18, 84, 255 },
    { 59, 82, 184, 255 },
    { 47, 180, 222, 255 },
    { 92, 214, 112, 255 },
    { 248, 231, 80, 255 },
    { 242, 135, 39, 255 },
    { 197, 48, 44, 255 }
};

static const Color kHumidityStops[] = {
    { 110, 72, 34, 255 },
    { 161, 136, 70, 255 },
    { 79, 179, 153, 255 },
    { 96, 198, 228, 255 },
    { 232, 246, 255, 255 }
};

static const Color kCloudStops[] = {
    { 18, 26, 48, 255 },
    { 69, 70, 138, 255 },
    { 141, 112, 194, 255 },
    { 226, 205, 242, 255 },
    { 248, 250, 255, 255 }
};

static const Color kDivergingStops[] = {
    { 190, 82, 40, 255 },
    { 239, 206, 118, 255 },
    { 237, 241, 244, 255 },
    { 96, 195, 227, 255 },
    { 35, 79, 196, 255 }
};

static float BaseSeaLevelTemp(float simSeconds) {
    float dayProgress = fmodf(simSeconds, DAY_SECONDS) / DAY_SECONDS;
    float solar = fmaxf(0.0f, sinf(TAU * (dayProgress - 0.25f)));
    return 6.0f + 15.0f * solar;
}

static float BasePressureAtAltitude(float altitudeMeters) {
    return 1013.25f * expf(-altitudeMeters / 8300.0f);
}

static float PotentialTemperatureFromTemp(float tempC, float pressureHpa) {
    float tempK = tempC + 273.15f;
    float pressureRatio = powf(REF_PRESSURE_HPA / fmaxf(pressureHpa, 1.0f), DRY_AIR_KAPPA);
    return tempK * pressureRatio;
}

static float TemperatureFromPotentialTemperature(float thetaK, float pressureHpa) {
    float pressureRatio = powf(fmaxf(pressureHpa, 1.0f) / REF_PRESSURE_HPA, DRY_AIR_KAPPA);
    return thetaK * pressureRatio - 273.15f;
}

static float AirDensityKgM3(float pressureHpa, float tempC) {
    float tempK = fmaxf(tempC + 273.15f, 180.0f);
    return (pressureHpa * 100.0f) / (DRY_AIR_GAS_CONSTANT * tempK);
}

static float SaturationMixingRatio(float tempC, float pressureHpa) {
    float es = 6.112f * expf(17.67f * tempC / (tempC + 243.5f));
    es = Clamp(es, 0.01f, pressureHpa * 0.95f);
    return 0.622f * es / fmaxf(pressureHpa - es, 1.0f);
}

static float RelativeHumidity(float tempC, float pressureHpa, float mixingRatio) {
    float qsat = SaturationMixingRatio(tempC, pressureHpa);
    if (qsat <= 0.000001f) return 0.0f;
    return Clamp((mixingRatio / qsat) * 100.0f, 0.0f, 130.0f);
}

static float DewPointC(float pressureHpa, float mixingRatio) {
    float vaporPressure = (mixingRatio * pressureHpa) / (0.622f + mixingRatio);
    vaporPressure = Clamp(vaporPressure, 0.05f, pressureHpa * 0.95f);
    float lnRatio = logf(vaporPressure / 6.112f);
    return (243.5f * lnRatio) / (17.67f - lnRatio);
}

static ImVec2_c ImVec2Make(float x, float y) {
    ImVec2_c value = { x, y };
    return value;
}

static ImVec4_c ImVec4Make(float x, float y, float z, float w) {
    ImVec4_c value = { x, y, z, w };
    return value;
}

Rectangle GetControlPanelRect(void) {
    const float width = 434.0f;
    const float margin = 12.0f;
    return (Rectangle) { GetScreenWidth() - width - margin, margin, width, GetScreenHeight() - margin * 2.0f };
}

Rectangle GetUiToggleRect(void) {
    const float margin = 12.0f;
    return (Rectangle) { GetScreenWidth() - 124.0f - margin, margin, 124.0f, 36.0f };
}

void ApplyImGuiTheme(void) {
    igStyleColorsDark(NULL);

    ImGuiStyle *style = igGetStyle();
    style->WindowPadding = ImVec2Make(14.0f, 12.0f);
    style->WindowRounding = 10.0f;
    style->WindowBorderSize = 1.0f;
    style->ChildRounding = 8.0f;
    style->ChildBorderSize = 1.0f;
    style->PopupRounding = 8.0f;
    style->PopupBorderSize = 1.0f;
    style->FramePadding = ImVec2Make(10.0f, 6.0f);
    style->FrameRounding = 6.0f;
    style->FrameBorderSize = 1.0f;
    style->ItemSpacing = ImVec2Make(10.0f, 8.0f);
    style->ItemInnerSpacing = ImVec2Make(8.0f, 6.0f);
    style->CellPadding = ImVec2Make(8.0f, 6.0f);
    style->IndentSpacing = 14.0f;
    style->ScrollbarSize = 14.0f;
    style->ScrollbarRounding = 10.0f;
    style->GrabMinSize = 10.0f;
    style->GrabRounding = 6.0f;
    style->TabRounding = 6.0f;
    style->TabBorderSize = 1.0f;
    style->SeparatorTextBorderSize = 0.0f;
    style->WindowTitleAlign = ImVec2Make(0.04f, 0.50f);
    style->ButtonTextAlign = ImVec2Make(0.50f, 0.50f);

    style->Colors[ImGuiCol_Text] = ImVec4Make(0.88f, 0.92f, 0.98f, 1.00f);
    style->Colors[ImGuiCol_TextDisabled] = ImVec4Make(0.45f, 0.53f, 0.62f, 1.00f);
    style->Colors[ImGuiCol_WindowBg] = ImVec4Make(0.05f, 0.08f, 0.11f, 0.98f);
    style->Colors[ImGuiCol_ChildBg] = ImVec4Make(0.07f, 0.10f, 0.14f, 0.90f);
    style->Colors[ImGuiCol_PopupBg] = ImVec4Make(0.06f, 0.09f, 0.13f, 0.98f);
    style->Colors[ImGuiCol_Border] = ImVec4Make(0.19f, 0.28f, 0.39f, 0.95f);
    style->Colors[ImGuiCol_BorderShadow] = ImVec4Make(0.00f, 0.00f, 0.00f, 0.00f);
    style->Colors[ImGuiCol_FrameBg] = ImVec4Make(0.09f, 0.13f, 0.18f, 1.00f);
    style->Colors[ImGuiCol_FrameBgHovered] = ImVec4Make(0.13f, 0.21f, 0.31f, 1.00f);
    style->Colors[ImGuiCol_FrameBgActive] = ImVec4Make(0.14f, 0.25f, 0.37f, 1.00f);
    style->Colors[ImGuiCol_TitleBg] = ImVec4Make(0.04f, 0.09f, 0.16f, 1.00f);
    style->Colors[ImGuiCol_TitleBgActive] = ImVec4Make(0.07f, 0.16f, 0.29f, 1.00f);
    style->Colors[ImGuiCol_TitleBgCollapsed] = ImVec4Make(0.04f, 0.09f, 0.16f, 0.84f);
    style->Colors[ImGuiCol_MenuBarBg] = ImVec4Make(0.07f, 0.10f, 0.14f, 1.00f);
    style->Colors[ImGuiCol_ScrollbarBg] = ImVec4Make(0.04f, 0.06f, 0.09f, 0.88f);
    style->Colors[ImGuiCol_ScrollbarGrab] = ImVec4Make(0.18f, 0.26f, 0.36f, 1.00f);
    style->Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4Make(0.24f, 0.37f, 0.53f, 1.00f);
    style->Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4Make(0.29f, 0.47f, 0.67f, 1.00f);
    style->Colors[ImGuiCol_CheckMark] = ImVec4Make(0.52f, 0.76f, 1.00f, 1.00f);
    style->Colors[ImGuiCol_SliderGrab] = ImVec4Make(0.31f, 0.56f, 0.91f, 1.00f);
    style->Colors[ImGuiCol_SliderGrabActive] = ImVec4Make(0.45f, 0.72f, 1.00f, 1.00f);
    style->Colors[ImGuiCol_Button] = ImVec4Make(0.11f, 0.23f, 0.39f, 1.00f);
    style->Colors[ImGuiCol_ButtonHovered] = ImVec4Make(0.16f, 0.34f, 0.57f, 1.00f);
    style->Colors[ImGuiCol_ButtonActive] = ImVec4Make(0.22f, 0.45f, 0.73f, 1.00f);
    style->Colors[ImGuiCol_Header] = ImVec4Make(0.10f, 0.22f, 0.38f, 1.00f);
    style->Colors[ImGuiCol_HeaderHovered] = ImVec4Make(0.16f, 0.34f, 0.57f, 1.00f);
    style->Colors[ImGuiCol_HeaderActive] = ImVec4Make(0.21f, 0.43f, 0.70f, 1.00f);
    style->Colors[ImGuiCol_Separator] = ImVec4Make(0.18f, 0.28f, 0.39f, 0.95f);
    style->Colors[ImGuiCol_SeparatorHovered] = ImVec4Make(0.28f, 0.47f, 0.73f, 1.00f);
    style->Colors[ImGuiCol_SeparatorActive] = ImVec4Make(0.34f, 0.57f, 0.88f, 1.00f);
    style->Colors[ImGuiCol_ResizeGrip] = ImVec4Make(0.18f, 0.31f, 0.49f, 0.28f);
    style->Colors[ImGuiCol_ResizeGripHovered] = ImVec4Make(0.25f, 0.48f, 0.76f, 0.80f);
    style->Colors[ImGuiCol_ResizeGripActive] = ImVec4Make(0.33f, 0.59f, 0.92f, 0.95f);
    style->Colors[ImGuiCol_TabHovered] = ImVec4Make(0.17f, 0.33f, 0.54f, 1.00f);
    style->Colors[ImGuiCol_Tab] = ImVec4Make(0.08f, 0.14f, 0.22f, 1.00f);
    style->Colors[ImGuiCol_TabSelected] = ImVec4Make(0.13f, 0.28f, 0.47f, 1.00f);
    style->Colors[ImGuiCol_TabSelectedOverline] = ImVec4Make(0.46f, 0.74f, 1.00f, 1.00f);
    style->Colors[ImGuiCol_TabDimmed] = ImVec4Make(0.07f, 0.11f, 0.17f, 1.00f);
    style->Colors[ImGuiCol_TabDimmedSelected] = ImVec4Make(0.10f, 0.20f, 0.33f, 1.00f);
    style->Colors[ImGuiCol_TextSelectedBg] = ImVec4Make(0.25f, 0.48f, 0.78f, 0.35f);
}

static bool DrawLabeledCombo(const char *label, const char *id, int *currentItem, const char *const items[], int itemCount) {
    igTextDisabled("%s", label);
    return igCombo_Str_arr(id, currentItem, items, itemCount, itemCount);
}

static bool DrawLabeledSliderFloat(const char *label, const char *id, float *value, float minValue, float maxValue, const char *format) {
    igTextDisabled("%s", label);
    return igSliderFloat(id, value, minValue, maxValue, format, ImGuiSliderFlags_None);
}

static bool DrawLabeledSliderInt(const char *label, const char *id, int *value, int minValue, int maxValue, const char *format) {
    igTextDisabled("%s", label);
    return igSliderInt(id, value, minValue, maxValue, format, ImGuiSliderFlags_None);
}

static float PressureAtCell(const AtmosphereSim *sim, int y, int index) {
    return Clamp(sim->basePressure[y] + sim->pressurePert[index] * 0.01f, 360.0f, 1060.0f);
}

static float ColumnValleyFactor(const AtmosphereSim *sim, int column) {
    float terrainNorm = NormalizeRange(sim->columnTerrainMeters[column], sim->terrain.minTerrainMeters, sim->terrain.maxTerrainMeters);
    float slopeFactor = Clamp(sim->columnSlope[column] * 4.0f, 0.0f, 1.0f);
    return Clamp((1.0f - terrainNorm) * (1.0f - 0.75f * slopeFactor), 0.0f, 1.0f);
}

static float ColumnRidgeFactor(const AtmosphereSim *sim, int column) {
    float terrainNorm = NormalizeRange(sim->columnTerrainMeters[column], sim->terrain.minTerrainMeters, sim->terrain.maxTerrainMeters);
    float slopeFactor = Clamp(sim->columnSlope[column] * 3.2f, 0.0f, 1.0f);
    return Clamp(0.58f * terrainNorm + 0.72f * slopeFactor, 0.0f, 1.0f);
}

static bool FluidCellValid(const AtmosphereSim *sim, int x, int y, int z) {
    if (x < 0 || x >= sim->nx || y < 0 || y >= sim->ny || z < 0 || z >= sim->nz) return false;
    return IsFluidCell(sim, x, y, z);
}

static float VirtualPotentialTemperature(float thetaK, float qv, float qc, float qr) {
    return thetaK * (1.0f + 0.61f * qv - qc - qr);
}

static float BaseRelativeHumidityProfile(const AtmosphereSim *sim, const AtmosForcing *forcing, int y) {
    float altNorm = Clamp(((float)y + 0.5f) / (float)sim->ny, 0.0f, 1.0f);
    return Clamp(forcing->humidityTarget + 0.16f * (1.0f - altNorm) - 0.10f * altNorm, 0.32f, 0.98f);
}

static float CellBuoyancy(const AtmosphereSim *sim, int y, float thetaK, float qv, float qc, float qr) {
    float thetaV = VirtualPotentialTemperature(thetaK, qv, qc, qr);
    float thetaVRef = VirtualPotentialTemperature(sim->baseTheta[y], sim->baseQv[y], 0.0f, 0.0f);
    return GRAVITY_ACCEL * (thetaV - thetaVRef) / fmaxf(thetaVRef, 1.0f);
}

static float VerticalFaceDensity(const AtmosphereSim *sim, int y) {
    if (y <= 0) return sim->baseDensity[0];
    if (y >= sim->ny) return sim->baseDensity[sim->ny - 1];
    return 0.5f * (sim->baseDensity[y - 1] + sim->baseDensity[y]);
}

static float CellDivergence(const AtmosphereSim *sim, int x, int y, int z) {
    float uLeft = sim->uFace[UIndex(sim, x, y, z)];
    float uRight = sim->uFace[UIndex(sim, x + 1, y, z)];
    float vSouth = sim->vFace[VIndex(sim, x, y, z)];
    float vNorth = sim->vFace[VIndex(sim, x, y, z + 1)];
    float wBottom = sim->wFace[WIndex(sim, x, y, z)];
    float wTop = sim->wFace[WIndex(sim, x, y + 1, z)];
    return (uRight - uLeft) / sim->dxMeters +
           (vNorth - vSouth) / sim->dzMeters +
           (wTop - wBottom) / sim->dyMeters;
}

static float MassFluxDivergence(const AtmosphereSim *sim, int x, int y, int z) {
    float rho = sim->baseDensity[y];
    float rhoWBottom = VerticalFaceDensity(sim, y);
    float rhoWTop = VerticalFaceDensity(sim, y + 1);
    float uLeft = sim->uFace[UIndex(sim, x, y, z)];
    float uRight = sim->uFace[UIndex(sim, x + 1, y, z)];
    float vSouth = sim->vFace[VIndex(sim, x, y, z)];
    float vNorth = sim->vFace[VIndex(sim, x, y, z + 1)];
    float wBottom = sim->wFace[WIndex(sim, x, y, z)];
    float wTop = sim->wFace[WIndex(sim, x, y + 1, z)];
    return (rho * uRight - rho * uLeft) / sim->dxMeters +
           (rho * vNorth - rho * vSouth) / sim->dzMeters +
           (rhoWTop * wTop - rhoWBottom * wBottom) / sim->dyMeters;
}

static double CellAirMassKg(const AtmosphereSim *sim, int y) {
    return (double)sim->baseDensity[y] * (double)sim->dxMeters * (double)sim->dyMeters * (double)sim->dzMeters;
}

static void SanitizeState(AtmosphereSim *sim) {
    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int index = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) {
                    sim->theta[index] = 0.0f;
                    sim->qv[index] = 0.0f;
                    sim->qc[index] = 0.0f;
                    sim->qr[index] = 0.0f;
                    sim->pressurePert[index] = 0.0f;
                    continue;
                }

                if (!isfinite(sim->theta[index])) {
                    sim->theta[index] = sim->baseTheta[y];
                    sim->nonFiniteCorrections++;
                }
                if (!isfinite(sim->pressurePert[index])) {
                    sim->pressurePert[index] = 0.0f;
                    sim->nonFiniteCorrections++;
                }
                if (!isfinite(sim->qv[index])) {
                    sim->qv[index] = sim->baseQv[y];
                    sim->nonFiniteCorrections++;
                }
                if (!isfinite(sim->qc[index])) {
                    sim->qc[index] = 0.0f;
                    sim->nonFiniteCorrections++;
                }
                if (!isfinite(sim->qr[index])) {
                    sim->qr[index] = 0.0f;
                    sim->nonFiniteCorrections++;
                }

                if (sim->qv[index] < 0.0f) {
                    sim->qv[index] = 0.0f;
                    sim->negativeWaterCorrections++;
                }
                if (sim->qc[index] < 0.0f) {
                    sim->qc[index] = 0.0f;
                    sim->negativeWaterCorrections++;
                }
                if (sim->qr[index] < 0.0f) {
                    sim->qr[index] = 0.0f;
                    sim->negativeWaterCorrections++;
                }
            }
        }
    }

    for (int i = 0; i < sim->cellsU; i++) {
        if (!isfinite(sim->uFace[i])) {
            sim->uFace[i] = 0.0f;
            sim->nonFiniteCorrections++;
        }
    }
    for (int i = 0; i < sim->cellsV; i++) {
        if (!isfinite(sim->vFace[i])) {
            sim->vFace[i] = 0.0f;
            sim->nonFiniteCorrections++;
        }
    }
    for (int i = 0; i < sim->cellsW; i++) {
        if (!isfinite(sim->wFace[i])) {
            sim->wFace[i] = 0.0f;
            sim->nonFiniteCorrections++;
        }
    }
}

void DestroySim(AtmosphereSim *sim) {
    if (sim->terrainModel.meshCount > 0) UnloadModel(sim->terrainModel);
    if (sim->horizontalSliceModel.meshCount > 0) UnloadModel(sim->horizontalSliceModel);
    if (sim->verticalSliceModelX.meshCount > 0) UnloadModel(sim->verticalSliceModelX);
    if (sim->verticalSliceModelZ.meshCount > 0) UnloadModel(sim->verticalSliceModelZ);
    if (sim->horizontalTex.id != 0) UnloadTexture(sim->horizontalTex);
    if (sim->verticalTex.id != 0) UnloadTexture(sim->verticalTex);

    free(sim->columnTerrainMeters);
    free(sim->columnTerrainWorldY);
    free(sim->columnDx);
    free(sim->columnDz);
    free(sim->columnSlope);
    free(sim->surfaceTemp);
    free(sim->surfaceWetness);
    free(sim->surfaceRain);
    free(sim->surfaceRainScratch);
    free(sim->groundLayer);

    free(sim->basePressure);
    free(sim->baseTemp);
    free(sim->baseTheta);
    free(sim->baseDensity);
    free(sim->baseQv);

    free(sim->theta);
    free(sim->qv);
    free(sim->qc);
    free(sim->qr);
    free(sim->pressurePert);
    free(sim->uFace);
    free(sim->vFace);
    free(sim->wFace);

    free(sim->temp);
    free(sim->pressure);
    free(sim->buoyancy);
    free(sim->divergence);
    free(sim->pressureScratch);
    free(sim->u);
    free(sim->v);
    free(sim->w);

    free(sim->thetaNext);
    free(sim->qvNext);
    free(sim->qcNext);
    free(sim->qrNext);
    free(sim->uFaceNext);
    free(sim->vFaceNext);
    free(sim->wFaceNext);

    free(sim->horizontalPixels);
    free(sim->verticalPixels);
    FreeTerrain(&sim->terrain);
    memset(sim, 0, sizeof(*sim));
}

static bool AllocateSim(AtmosphereSim *sim) {
    sim->cells2D = sim->nx * sim->nz;
    sim->cells3D = sim->cells2D * sim->ny;
    sim->cellsU = (sim->nx + 1) * sim->ny * sim->nz;
    sim->cellsV = sim->nx * sim->ny * (sim->nz + 1);
    sim->cellsW = sim->nx * (sim->ny + 1) * sim->nz;

    sim->columnTerrainMeters = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->columnTerrainWorldY = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->columnDx = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->columnDz = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->columnSlope = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->surfaceTemp = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->surfaceWetness = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->surfaceRain = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->surfaceRainScratch = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->groundLayer = (int *)calloc((size_t)sim->cells2D, sizeof(int));

    sim->basePressure = (float *)calloc((size_t)sim->ny, sizeof(float));
    sim->baseTemp = (float *)calloc((size_t)sim->ny, sizeof(float));
    sim->baseTheta = (float *)calloc((size_t)sim->ny, sizeof(float));
    sim->baseDensity = (float *)calloc((size_t)sim->ny, sizeof(float));
    sim->baseQv = (float *)calloc((size_t)sim->ny, sizeof(float));

    sim->theta = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->qv = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->qc = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->qr = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->pressurePert = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->uFace = (float *)calloc((size_t)sim->cellsU, sizeof(float));
    sim->vFace = (float *)calloc((size_t)sim->cellsV, sizeof(float));
    sim->wFace = (float *)calloc((size_t)sim->cellsW, sizeof(float));

    sim->temp = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->pressure = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->buoyancy = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->divergence = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->pressureScratch = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->u = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->v = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->w = (float *)calloc((size_t)sim->cells3D, sizeof(float));

    sim->thetaNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->qvNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->qcNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->qrNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->uFaceNext = (float *)calloc((size_t)sim->cellsU, sizeof(float));
    sim->vFaceNext = (float *)calloc((size_t)sim->cellsV, sizeof(float));
    sim->wFaceNext = (float *)calloc((size_t)sim->cellsW, sizeof(float));

    int horizontalTexWidth = sim->nx * SLICE_TEX_SCALE;
    int horizontalTexHeight = sim->nz * SLICE_TEX_SCALE;
    int verticalTexWidth = ((sim->nx > sim->nz) ? sim->nx : sim->nz) * SLICE_TEX_SCALE;
    int verticalTexHeight = sim->ny * SLICE_TEX_SCALE;

    sim->horizontalPixels = (Color *)calloc((size_t)horizontalTexWidth * (size_t)horizontalTexHeight, sizeof(Color));
    sim->verticalPixels = (Color *)calloc((size_t)verticalTexWidth * (size_t)verticalTexHeight, sizeof(Color));

    if (!sim->columnTerrainMeters || !sim->columnTerrainWorldY || !sim->columnDx || !sim->columnDz ||
        !sim->columnSlope || !sim->surfaceTemp || !sim->surfaceWetness || !sim->surfaceRain ||
        !sim->surfaceRainScratch || !sim->groundLayer || !sim->basePressure || !sim->baseTemp ||
        !sim->baseTheta || !sim->baseDensity || !sim->baseQv || !sim->theta || !sim->qv ||
        !sim->qc || !sim->qr || !sim->pressurePert || !sim->uFace || !sim->vFace || !sim->wFace ||
        !sim->temp || !sim->pressure || !sim->buoyancy || !sim->divergence || !sim->pressureScratch ||
        !sim->u || !sim->v || !sim->w || !sim->thetaNext || !sim->qvNext || !sim->qcNext ||
        !sim->qrNext || !sim->uFaceNext || !sim->vFaceNext || !sim->wFaceNext ||
        !sim->horizontalPixels || !sim->verticalPixels) {
        return false;
    }

    Image hImage = GenImageColor(horizontalTexWidth, horizontalTexHeight, BLANK);
    Image vImage = GenImageColor(verticalTexWidth, verticalTexHeight, BLANK);
    sim->horizontalTex = LoadTextureFromImage(hImage);
    sim->verticalTex = LoadTextureFromImage(vImage);
    UnloadImage(hImage);
    UnloadImage(vImage);

    if (sim->horizontalTex.id == 0 || sim->verticalTex.id == 0) return false;

    SetTextureFilter(sim->horizontalTex, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(sim->verticalTex, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(sim->horizontalTex, TEXTURE_WRAP_CLAMP);
    SetTextureWrap(sim->verticalTex, TEXTURE_WRAP_CLAMP);

    sim->horizontalSliceModel = LoadModelFromMesh(GenMeshPlane(sim->worldWidth, sim->worldDepth, 1, 1));
    sim->verticalSliceModelX = LoadModelFromMesh(GenMeshPlane(sim->topWorldY, sim->worldDepth, 1, 1));
    sim->verticalSliceModelZ = LoadModelFromMesh(GenMeshPlane(sim->worldWidth, sim->topWorldY, 1, 1));

    if (sim->horizontalSliceModel.meshCount == 0 || sim->verticalSliceModelX.meshCount == 0 || sim->verticalSliceModelZ.meshCount == 0) {
        return false;
    }

    sim->horizontalSliceModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = sim->horizontalTex;
    sim->verticalSliceModelX.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = sim->verticalTex;
    sim->verticalSliceModelZ.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = sim->verticalTex;
    return true;
}

static void BuildAtmosColumns(AtmosphereSim *sim) {
    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            float terrainX = (float)x * sim->dxWorld;
            float terrainZ = (float)z * sim->dzWorld;
            int column = ColumnIndex(sim, x, z);
            float terrainMeters = BilinearSample(sim->terrain.terrainMeters, sim->terrain.width, sim->terrain.height, terrainX, terrainZ);
            float dx = BilinearSample(sim->terrain.terrainDx, sim->terrain.width, sim->terrain.height, terrainX, terrainZ);
            float dz = BilinearSample(sim->terrain.terrainDz, sim->terrain.width, sim->terrain.height, terrainX, terrainZ);

            sim->columnTerrainMeters[column] = terrainMeters;
            sim->columnTerrainWorldY[column] = WorldYFromMeters(terrainMeters);
            sim->columnDx[column] = dx;
            sim->columnDz[column] = dz;
            sim->columnSlope[column] = sqrtf(dx * dx + dz * dz);
            sim->groundLayer[column] = GroundLayerForAltitude(sim, terrainMeters);
        }
    }
}

static SimViewSettings CaptureSimViewSettings(const AtmosphereSim *sim) {
    SimViewSettings settings = {
        .fieldMode = sim->fieldMode,
        .verticalAxis = sim->verticalAxis,
        .horizontalLayer = sim->horizontalLayer,
        .verticalSlice = sim->verticalSlice,
        .showHorizontalSlice = sim->showHorizontalSlice,
        .showVerticalSlice = sim->showVerticalSlice,
        .showTerrain = sim->showTerrain,
        .showWireframe = sim->showWireframe,
        .showWindArrows = sim->showWindArrows,
        .paused = sim->paused,
        .showUI = sim->showUI,
        .terrainAlpha = sim->terrainAlpha,
        .timeScale = sim->timeScale
    };
    return settings;
}

static void RestoreSimViewSettings(AtmosphereSim *sim, const SimViewSettings *settings) {
    sim->fieldMode = settings->fieldMode;
    sim->verticalAxis = settings->verticalAxis;
    sim->horizontalLayer = Clamp(settings->horizontalLayer, 0, sim->ny - 1);
    sim->verticalSlice = Clamp(settings->verticalSlice, 0, (settings->verticalAxis == SLICE_X) ? sim->nx - 1 : sim->nz - 1);
    sim->showHorizontalSlice = settings->showHorizontalSlice;
    sim->showVerticalSlice = settings->showVerticalSlice;
    sim->showTerrain = settings->showTerrain;
    sim->showWireframe = settings->showWireframe;
    sim->showWindArrows = settings->showWindArrows;
    sim->paused = settings->paused;
    sim->showUI = settings->showUI;
    sim->terrainAlpha = settings->terrainAlpha;
    sim->timeScale = settings->timeScale;
}

static AtmosForcing UpdateForcingInputs(AtmosphereSim *sim) {
    float dayProgress = fmodf(sim->simSeconds, DAY_SECONDS) / DAY_SECONDS;
    float solar = fmaxf(0.0f, sinf(TAU * (dayProgress - 0.25f)));
    Vector3 sunDir = Vector3Normalize((Vector3) {
        cosf(TAU * (dayProgress - 0.25f)),
        0.25f + 0.95f * solar,
        0.35f + 0.55f * sinf(TAU * (dayProgress - 0.12f))
    });
    float synopticPhase = sim->simSeconds / (DAY_SECONDS * 1.6f);
    float synopticDir = 0.40f + 0.55f * sinf(synopticPhase * 0.90f);
    float synopticSpeed = 6.0f + 2.4f * sinf(synopticPhase + 0.9f);

    AtmosForcing forcing = {
        .solar = solar,
        .humidityTarget = 0.46f + 0.18f * (1.0f - solar),
        .synopticU = cosf(synopticDir) * synopticSpeed,
        .synopticV = sinf(synopticDir) * synopticSpeed * 0.75f,
        .sunDir = sunDir
    };

    sim->sunDir = sunDir;
    sim->solarStrength = solar;
    sim->prevailingU = forcing.synopticU;
    sim->prevailingV = forcing.synopticV;
    return forcing;
}

static void InitBackgroundAtmosphere(AtmosphereSim *sim, const AtmosForcing *forcing) {
    float seaLevelTemp = BaseSeaLevelTemp(sim->simSeconds);

    for (int y = 0; y < sim->ny; y++) {
        float altitude = CellAltitudeMeters(sim, y);
        float baseTemp = seaLevelTemp - 0.0064f * altitude;
        float basePressure = BasePressureAtAltitude(altitude);
        float baseRh = BaseRelativeHumidityProfile(sim, forcing, y);

        sim->baseTemp[y] = baseTemp;
        sim->basePressure[y] = basePressure;
        sim->baseTheta[y] = PotentialTemperatureFromTemp(baseTemp, basePressure);
        sim->baseDensity[y] = AirDensityKgM3(basePressure, baseTemp);
        sim->baseQv[y] = SaturationMixingRatio(baseTemp, basePressure) * baseRh;
    }
}

static void UpdateVelocityDiagnostics(AtmosphereSim *sim) {
    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int i = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) {
                    sim->u[i] = 0.0f;
                    sim->v[i] = 0.0f;
                    sim->w[i] = 0.0f;
                    continue;
                }

                sim->u[i] = 0.5f * (sim->uFace[UIndex(sim, x, y, z)] + sim->uFace[UIndex(sim, x + 1, y, z)]);
                sim->v[i] = 0.5f * (sim->vFace[VIndex(sim, x, y, z)] + sim->vFace[VIndex(sim, x, y, z + 1)]);
                sim->w[i] = 0.5f * (sim->wFace[WIndex(sim, x, y, z)] + sim->wFace[WIndex(sim, x, y + 1, z)]);
            }
        }
    }
}

static void UpdateLandSurfaceState(AtmosphereSim *sim, const AtmosForcing *forcing, float dt) {
    float seaLevelTemp = BaseSeaLevelTemp(sim->simSeconds);

    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            int column = ColumnIndex(sim, x, z);
            float terrainAlt = sim->columnTerrainMeters[column];
            float terrainNorm = NormalizeRange(terrainAlt, sim->terrain.minTerrainMeters, sim->terrain.maxTerrainMeters);
            float valley = ColumnValleyFactor(sim, column);
            float ridge = ColumnRidgeFactor(sim, column);
            Vector3 normal = Vector3Normalize((Vector3) { -sim->columnDx[column], 1.0f, -sim->columnDz[column] });
            float exposure = fmaxf(0.0f, Vector3DotProduct(normal, forcing->sunDir));
            float rainInput = sim->surfaceRain[column] * dt * 0.00022f;
            float evapLoss = dt * (0.00003f + 0.00008f * forcing->solar) * (0.35f + 0.65f * exposure);
            float retention = Clamp(0.18f + 0.52f * valley + 0.16f * (1.0f - terrainNorm), 0.08f, 1.0f);
            float eqTemp = seaLevelTemp - 0.0061f * terrainAlt +
                           forcing->solar * exposure * 9.5f -
                           ridge * 1.5f -
                           valley * (1.0f - forcing->solar) * 1.1f -
                           sim->surfaceWetness[column] * 2.0f;

            sim->surfaceWetness[column] = Clamp(sim->surfaceWetness[column] +
                                                (retention - sim->surfaceWetness[column]) * dt * 0.00004f +
                                                rainInput - evapLoss,
                                                0.04f,
                                                1.0f);
            sim->surfaceTemp[column] = Lerp(sim->surfaceTemp[column], eqTemp, Clamp(dt / 900.0f, 0.0f, 0.25f));
        }
    }
}

static void ApplySurfaceFluxes(AtmosphereSim *sim, const AtmosForcing *forcing, float dt) {
    UpdateLandSurfaceState(sim, forcing, dt);

    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            int column = ColumnIndex(sim, x, z);
            int y0 = sim->groundLayer[column] + 1;
            if (y0 < 0 || y0 >= sim->ny) continue;

            float slopeMag = sim->columnSlope[column];
            float gradX = sim->columnDx[column];
            float gradZ = sim->columnDz[column];
            float gradLen = sqrtf(gradX * gradX + gradZ * gradZ);
            float slopeDirX = (gradLen > 0.0001f) ? gradX / gradLen : 0.0f;
            float slopeDirZ = (gradLen > 0.0001f) ? gradZ / gradLen : 0.0f;

            for (int layer = 0; layer < 3; layer++) {
                int y = y0 + layer;
                if (y >= sim->ny || !IsFluidCell(sim, x, y, z)) break;

                int i = AtmosIndex(sim, x, y, z);
                float pressure = PressureAtCell(sim, y, i);
                float thetaScale = powf(REF_PRESSURE_HPA / fmaxf(pressure, 1.0f), DRY_AIR_KAPPA);
                float weight = expf(-(float)layer * 0.8f);
                float airRh = RelativeHumidity(sim->temp[i], sim->pressure[i], sim->qv[i]) * 0.01f;
                float sensible = 0.0011f * (sim->surfaceTemp[column] - sim->temp[i]) * weight;
                float evapFlux = 0.0000045f * sim->surfaceWetness[column] * fmaxf(0.0f, 1.0f - airRh) * weight;

                sim->theta[i] += sensible * thetaScale * dt;
                sim->qv[i] = fmaxf(0.0f, sim->qv[i] + evapFlux * dt);
            }

            float thermalExcess = sim->surfaceTemp[column] - sim->temp[AtmosIndex(sim, x, y0, z)];
            float upslopeAccel = 0.0009f * thermalExcess * slopeMag;
            float drag = 1.0f / (1.0f + dt * (0.035f + slopeMag * 0.14f));

            if (x >= 0 && x < sim->nx + 1) {
                sim->uFace[UIndex(sim, x, y0, z)] *= drag;
                if (x + 1 <= sim->nx) sim->uFace[UIndex(sim, x + 1, y0, z)] *= drag;
            }
            sim->vFace[VIndex(sim, x, y0, z)] *= drag;
            if (z + 1 <= sim->nz) sim->vFace[VIndex(sim, x, y0, z + 1)] *= drag;

            if (x + 1 < sim->nx) {
                sim->uFace[UIndex(sim, x + 1, y0, z)] += slopeDirX * upslopeAccel * dt;
            }
            if (z + 1 < sim->nz) {
                sim->vFace[VIndex(sim, x, y0, z + 1)] += slopeDirZ * upslopeAccel * dt;
            }
        }
    }
}

static void ApplyBodyForces(AtmosphereSim *sim, float dt) {
    for (int z = 0; z < sim->nz; z++) {
        for (int y = 1; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                bool lowerFluid = FluidCellValid(sim, x, y - 1, z);
                bool upperFluid = FluidCellValid(sim, x, y, z);
                int wIndex = WIndex(sim, x, y, z);

                if (!(lowerFluid && upperFluid)) {
                    sim->wFace[wIndex] = 0.0f;
                    continue;
                }

                int lowerIndex = AtmosIndex(sim, x, y - 1, z);
                int upperIndex = AtmosIndex(sim, x, y, z);
                float buoyancy = 0.5f * (
                    CellBuoyancy(sim, y - 1, sim->theta[lowerIndex], sim->qv[lowerIndex], sim->qc[lowerIndex], sim->qr[lowerIndex]) +
                    CellBuoyancy(sim, y, sim->theta[upperIndex], sim->qv[upperIndex], sim->qc[upperIndex], sim->qr[upperIndex]));

                sim->wFace[wIndex] += buoyancy * dt;
            }
        }
    }
}

static void AdvectMomentum(AtmosphereSim *sim, float dt) {
    UpdateVelocityDiagnostics(sim);

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x <= sim->nx; x++) {
                int index = UIndex(sim, x, y, z);
                bool leftFluid = FluidCellValid(sim, x - 1, y, z);
                bool rightFluid = FluidCellValid(sim, x, y, z);
                if (!(leftFluid && rightFluid)) {
                    sim->uFaceNext[index] = sim->uFace[index];
                    continue;
                }

                float px = (float)x - 0.5f;
                float py = (float)y;
                float pz = (float)z;
                float advU = 0.0f;
                float advV = 0.0f;
                float advW = 0.0f;
                SampleVelocityField(sim, px, py, pz, &advU, &advV, &advW);

                float backX = px - advU * dt / sim->dxMeters;
                float backY = py - advW * dt / sim->dyMeters;
                float backZ = pz - advV * dt / sim->dzMeters;
                float advected = SampleUFaceField(sim, sim->uFace, backX, backY, backZ);
                sim->uFaceNext[index] = Lerp(advected, sim->uFace[index], VELOCITY_BLEND);
            }
        }
    }

    for (int z = 0; z <= sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int index = VIndex(sim, x, y, z);
                bool southFluid = FluidCellValid(sim, x, y, z - 1);
                bool northFluid = FluidCellValid(sim, x, y, z);
                if (!(southFluid && northFluid)) {
                    sim->vFaceNext[index] = sim->vFace[index];
                    continue;
                }

                float px = (float)x;
                float py = (float)y;
                float pz = (float)z - 0.5f;
                float advU = 0.0f;
                float advV = 0.0f;
                float advW = 0.0f;
                SampleVelocityField(sim, px, py, pz, &advU, &advV, &advW);

                float backX = px - advU * dt / sim->dxMeters;
                float backY = py - advW * dt / sim->dyMeters;
                float backZ = pz - advV * dt / sim->dzMeters;
                float advected = SampleVFaceField(sim, sim->vFace, backX, backY, backZ);
                sim->vFaceNext[index] = Lerp(advected, sim->vFace[index], VELOCITY_BLEND);
            }
        }
    }

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y <= sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int index = WIndex(sim, x, y, z);
                bool lowerFluid = FluidCellValid(sim, x, y - 1, z);
                bool upperFluid = FluidCellValid(sim, x, y, z);
                if (!(lowerFluid && upperFluid)) {
                    sim->wFaceNext[index] = 0.0f;
                    continue;
                }

                float px = (float)x;
                float py = (float)y - 0.5f;
                float pz = (float)z;
                float advU = 0.0f;
                float advV = 0.0f;
                float advW = 0.0f;
                SampleVelocityField(sim, px, py, pz, &advU, &advV, &advW);

                float backX = px - advU * dt / sim->dxMeters;
                float backY = py - advW * dt / sim->dyMeters;
                float backZ = pz - advV * dt / sim->dzMeters;
                float advected = SampleWFaceField(sim, sim->wFace, backX, backY, backZ);
                sim->wFaceNext[index] = Lerp(advected, sim->wFace[index], VELOCITY_BLEND);
            }
        }
    }

    SwapFields(&sim->uFace, &sim->uFaceNext);
    SwapFields(&sim->vFace, &sim->vFaceNext);
    SwapFields(&sim->wFace, &sim->wFaceNext);
}

static void AdvectScalars(AtmosphereSim *sim, float dt) {
    UpdateVelocityDiagnostics(sim);

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int index = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) {
                    sim->thetaNext[index] = 0.0f;
                    sim->qvNext[index] = 0.0f;
                    sim->qcNext[index] = 0.0f;
                    sim->qrNext[index] = 0.0f;
                    continue;
                }

                float advU = 0.0f;
                float advV = 0.0f;
                float advW = 0.0f;
                SampleVelocityField(sim, (float)x, (float)y, (float)z, &advU, &advV, &advW);

                float backX = (float)x - advU * dt / sim->dxMeters;
                float backY = (float)y - advW * dt / sim->dyMeters;
                float backZ = (float)z - advV * dt / sim->dzMeters;

                sim->thetaNext[index] = SampleAtmosField(sim, sim->theta, backX, backY, backZ);
                sim->qvNext[index] = fmaxf(0.0f, SampleAtmosField(sim, sim->qv, backX, backY, backZ));
                sim->qcNext[index] = fmaxf(0.0f, SampleAtmosField(sim, sim->qc, backX, backY, backZ));
                sim->qrNext[index] = fmaxf(0.0f, SampleAtmosField(sim, sim->qr, backX, backY, backZ));
            }
        }
    }

    SwapFields(&sim->theta, &sim->thetaNext);
    SwapFields(&sim->qv, &sim->qvNext);
    SwapFields(&sim->qc, &sim->qcNext);
    SwapFields(&sim->qr, &sim->qrNext);
}

static void ApplyMicrophysics(AtmosphereSim *sim, float dt) {
    memset(sim->surfaceRainScratch, 0, (size_t)sim->cells2D * sizeof(float));

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                if (!IsFluidCell(sim, x, y, z)) continue;

                int index = AtmosIndex(sim, x, y, z);
                float pressure = PressureAtCell(sim, y, index);
                float temp = TemperatureFromPotentialTemperature(sim->theta[index], pressure);
                for (int adjust = 0; adjust < 2; adjust++) {
                    float qsat = SaturationMixingRatio(temp, pressure);
                    if (sim->qv[index] > qsat) {
                        float condense = sim->qv[index] - qsat;
                        sim->qv[index] -= condense;
                        sim->qc[index] += condense;
                        temp += (LATENT_HEAT_VAPORIZATION / SPECIFIC_HEAT_AIR) * condense;
                        sim->theta[index] = PotentialTemperatureFromTemp(temp, pressure);
                    } else if (sim->qv[index] < qsat && sim->qc[index] > 0.0f) {
                        float evaporate = fminf(sim->qc[index], qsat - sim->qv[index]);
                        sim->qc[index] -= evaporate;
                        sim->qv[index] += evaporate;
                        temp -= (LATENT_HEAT_VAPORIZATION / SPECIFIC_HEAT_AIR) * evaporate;
                        sim->theta[index] = PotentialTemperatureFromTemp(temp, pressure);
                    } else {
                        break;
                    }
                }

                float qsat = SaturationMixingRatio(temp, pressure);

                if (sim->qc[index] > AUTOCONVERSION_THRESHOLD) {
                    float autoRate = AUTOCONVERSION_RATE * (sim->qc[index] - AUTOCONVERSION_THRESHOLD) * dt;
                    float converted = fminf(sim->qc[index], autoRate);
                    sim->qc[index] -= converted;
                    sim->qr[index] += converted;
                }

                if (sim->qr[index] > 0.0f && sim->qc[index] > 0.0f) {
                    float accreted = fminf(sim->qc[index], ACCRETION_RATE * sim->qc[index] * sim->qr[index] * dt);
                    sim->qc[index] -= accreted;
                    sim->qr[index] += accreted;
                }

                if (sim->qr[index] > 0.0f && sim->qv[index] < qsat) {
                    float rainEvap = fminf(sim->qr[index], RAIN_EVAP_RATE * (qsat - sim->qv[index]) * dt);
                    sim->qr[index] -= rainEvap;
                    sim->qv[index] += rainEvap;
                    temp -= (LATENT_HEAT_VAPORIZATION / SPECIFIC_HEAT_AIR) * rainEvap;
                    sim->theta[index] = PotentialTemperatureFromTemp(temp, pressure);
                }
            }
        }
    }

    // Sedimentation uses a scratch field so rain cannot fall through multiple layers in one timestep.
    memset(sim->qrNext, 0, (size_t)sim->cells3D * sizeof(float));

    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            int column = ColumnIndex(sim, x, z);
            for (int y = sim->ny - 1; y >= 0; y--) {
                if (!IsFluidCell(sim, x, y, z)) continue;

                int index = AtmosIndex(sim, x, y, z);
                float rain = sim->qr[index];
                if (rain <= 0.0f) continue;

                float terminalVelocity = Clamp(1.2f + 65.0f * sqrtf(rain), 0.5f, 8.5f);
                float fallFraction = Clamp(terminalVelocity * dt / sim->dyMeters, 0.0f, 0.85f);
                float stay = rain * (1.0f - fallFraction);
                float fall = rain * fallFraction;

                sim->qrNext[index] += stay;

                if (fall <= 0.0f) continue;

                if (FluidCellValid(sim, x, y - 1, z)) {
                    sim->qrNext[AtmosIndex(sim, x, y - 1, z)] += fall;
                } else {
                    double columnRainMass = (double)fall * (double)sim->baseDensity[y] * (double)sim->dyMeters *
                                            (double)sim->dxMeters * (double)sim->dzMeters;
                    sim->precipitatedWaterMass += columnRainMass;
                    sim->surfaceRainScratch[column] += fall * sim->baseDensity[y] * sim->dyMeters * 3600.0f / fmaxf(dt, 0.001f);
                }
            }
        }
    }

    SwapFields(&sim->qr, &sim->qrNext);

    for (int column = 0; column < sim->cells2D; column++) {
        float rainRate = Clamp(sim->surfaceRainScratch[column], 0.0f, 180.0f);
        sim->surfaceRain[column] = Lerp(sim->surfaceRain[column], rainRate, Clamp(dt / 120.0f, 0.0f, 0.35f));
    }
}

static void ProjectPressure(AtmosphereSim *sim, float dt) {
    // Track projection quality so divergence regressions are visible in the HUD.
    sim->maxDivBeforeProjection = 0.0f;
    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int index = AtmosIndex(sim, x, y, z);
                sim->divergence[index] = IsFluidCell(sim, x, y, z) ? MassFluxDivergence(sim, x, y, z) : 0.0f;
                float absDiv = fabsf(sim->divergence[index]);
                if (absDiv > sim->maxDivBeforeProjection) sim->maxDivBeforeProjection = absDiv;
            }
        }
    }

    memset(sim->pressureScratch, 0, (size_t)sim->cells3D * sizeof(float));

    for (int iter = 0; iter < PROJECTION_ITERATIONS; iter++) {
        for (int z = 0; z < sim->nz; z++) {
            for (int y = 0; y < sim->ny; y++) {
                for (int x = 0; x < sim->nx; x++) {
                    int index = AtmosIndex(sim, x, y, z);
                    if (!IsFluidCell(sim, x, y, z)) {
                        sim->pressureScratch[index] = 0.0f;
                        continue;
                    }

                    float rhs = sim->divergence[index] / fmaxf(dt, 0.001f);
                    float diag = 0.0f;
                    float sum = 0.0f;
                    float horizCoeff = (1.0f / fmaxf(sim->baseDensity[y], 0.2f));

                    if (FluidCellValid(sim, x - 1, y, z)) {
                        float coeff = horizCoeff / (sim->dxMeters * sim->dxMeters);
                        diag += coeff;
                        sum += coeff * sim->pressurePert[AtmosIndex(sim, x - 1, y, z)];
                    }
                    if (FluidCellValid(sim, x + 1, y, z)) {
                        float coeff = horizCoeff / (sim->dxMeters * sim->dxMeters);
                        diag += coeff;
                        sum += coeff * sim->pressurePert[AtmosIndex(sim, x + 1, y, z)];
                    }
                    if (FluidCellValid(sim, x, y, z - 1)) {
                        float coeff = horizCoeff / (sim->dzMeters * sim->dzMeters);
                        diag += coeff;
                        sum += coeff * sim->pressurePert[AtmosIndex(sim, x, y, z - 1)];
                    }
                    if (FluidCellValid(sim, x, y, z + 1)) {
                        float coeff = horizCoeff / (sim->dzMeters * sim->dzMeters);
                        diag += coeff;
                        sum += coeff * sim->pressurePert[AtmosIndex(sim, x, y, z + 1)];
                    }
                    if (FluidCellValid(sim, x, y - 1, z)) {
                        float coeff = (1.0f / fmaxf(VerticalFaceDensity(sim, y), 0.2f)) / (sim->dyMeters * sim->dyMeters);
                        diag += coeff;
                        sum += coeff * sim->pressurePert[AtmosIndex(sim, x, y - 1, z)];
                    }
                    if (FluidCellValid(sim, x, y + 1, z)) {
                        float coeff = (1.0f / fmaxf(VerticalFaceDensity(sim, y + 1), 0.2f)) / (sim->dyMeters * sim->dyMeters);
                        diag += coeff;
                        sum += coeff * sim->pressurePert[AtmosIndex(sim, x, y + 1, z)];
                    }

                    sim->pressureScratch[index] = (diag > 0.0f) ? (sum - rhs) / diag : 0.0f;
                }
            }
        }

        SwapFields(&sim->pressurePert, &sim->pressureScratch);
    }

    double meanPressurePert = 0.0;
    int meanCount = 0;
    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                if (!IsFluidCell(sim, x, y, z)) continue;
                meanPressurePert += sim->pressurePert[AtmosIndex(sim, x, y, z)];
                meanCount++;
            }
        }
    }
    if (meanCount > 0) {
        float mean = (float)(meanPressurePert / (double)meanCount);
        for (int z = 0; z < sim->nz; z++) {
            for (int y = 0; y < sim->ny; y++) {
                for (int x = 0; x < sim->nx; x++) {
                    if (!IsFluidCell(sim, x, y, z)) continue;
                    sim->pressurePert[AtmosIndex(sim, x, y, z)] -= mean;
                }
            }
        }
    }

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 1; x < sim->nx; x++) {
                if (!(FluidCellValid(sim, x - 1, y, z) && FluidCellValid(sim, x, y, z))) continue;
                float beta = 1.0f / fmaxf(sim->baseDensity[y], 0.2f);
                float grad = (sim->pressurePert[AtmosIndex(sim, x, y, z)] -
                              sim->pressurePert[AtmosIndex(sim, x - 1, y, z)]) / sim->dxMeters;
                sim->uFace[UIndex(sim, x, y, z)] -= dt * beta * grad;
            }
        }
    }

    for (int z = 1; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                if (!(FluidCellValid(sim, x, y, z - 1) && FluidCellValid(sim, x, y, z))) continue;
                float beta = 1.0f / fmaxf(sim->baseDensity[y], 0.2f);
                float grad = (sim->pressurePert[AtmosIndex(sim, x, y, z)] -
                              sim->pressurePert[AtmosIndex(sim, x, y, z - 1)]) / sim->dzMeters;
                sim->vFace[VIndex(sim, x, y, z)] -= dt * beta * grad;
            }
        }
    }

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 1; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                if (!(FluidCellValid(sim, x, y - 1, z) && FluidCellValid(sim, x, y, z))) continue;
                float beta = 1.0f / fmaxf(VerticalFaceDensity(sim, y), 0.2f);
                float grad = (sim->pressurePert[AtmosIndex(sim, x, y, z)] -
                              sim->pressurePert[AtmosIndex(sim, x, y - 1, z)]) / sim->dyMeters;
                sim->wFace[WIndex(sim, x, y, z)] -= dt * beta * grad;
            }
        }
    }

    sim->maxDivAfterProjection = 0.0f;
    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                if (!IsFluidCell(sim, x, y, z)) continue;
                float absDiv = fabsf(MassFluxDivergence(sim, x, y, z));
                if (absDiv > sim->maxDivAfterProjection) sim->maxDivAfterProjection = absDiv;
            }
        }
    }
}

static void ApplyBoundaryConditions(AtmosphereSim *sim, const AtmosForcing *forcing, float dt) {
    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x <= sim->nx; x++) {
                int index = UIndex(sim, x, y, z);
                bool leftFluid = FluidCellValid(sim, x - 1, y, z);
                bool rightFluid = FluidCellValid(sim, x, y, z);
                float altNorm = Clamp(FaceAltitudeMeters(sim, y) / sim->topMeters, 0.0f, 1.0f);
                float target = forcing->synopticU * (0.60f + 0.50f * altNorm);

                if (!leftFluid && !rightFluid) {
                    sim->uFace[index] = 0.0f;
                    continue;
                }
                if (x == 0) {
                    sim->uFace[index] = (forcing->synopticU >= 0.0f) ? target :
                                        ((sim->nx > 0) ? sim->uFace[UIndex(sim, 1, y, z)] : 0.0f);
                    continue;
                }
                if (x == sim->nx) {
                    sim->uFace[index] = (forcing->synopticU <= 0.0f) ? target :
                                        sim->uFace[UIndex(sim, sim->nx - 1, y, z)];
                    continue;
                }
                if (!(leftFluid && rightFluid)) {
                    sim->uFace[index] = 0.0f;
                    continue;
                }

                float topWeight = Clamp((altNorm - (1.0f - TOP_SPONGE_FRACTION)) / TOP_SPONGE_FRACTION, 0.0f, 1.0f);
                if (topWeight > 0.0f) {
                    float blend = Clamp(topWeight * (0.12f + dt * 0.08f), 0.0f, 0.35f);
                    sim->uFace[index] = Lerp(sim->uFace[index], target, blend);
                }
            }
        }
    }

    for (int z = 0; z <= sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int index = VIndex(sim, x, y, z);
                bool southFluid = FluidCellValid(sim, x, y, z - 1);
                bool northFluid = FluidCellValid(sim, x, y, z);
                float altNorm = Clamp(FaceAltitudeMeters(sim, y) / sim->topMeters, 0.0f, 1.0f);
                float target = forcing->synopticV * (0.60f + 0.50f * altNorm);

                if (!southFluid && !northFluid) {
                    sim->vFace[index] = 0.0f;
                    continue;
                }
                if (z == 0) {
                    sim->vFace[index] = (forcing->synopticV >= 0.0f) ? target :
                                        ((sim->nz > 0) ? sim->vFace[VIndex(sim, x, y, 1)] : 0.0f);
                    continue;
                }
                if (z == sim->nz) {
                    sim->vFace[index] = (forcing->synopticV <= 0.0f) ? target :
                                        sim->vFace[VIndex(sim, x, y, sim->nz - 1)];
                    continue;
                }
                if (!(southFluid && northFluid)) {
                    sim->vFace[index] = 0.0f;
                    continue;
                }

                float topWeight = Clamp((altNorm - (1.0f - TOP_SPONGE_FRACTION)) / TOP_SPONGE_FRACTION, 0.0f, 1.0f);
                if (topWeight > 0.0f) {
                    float blend = Clamp(topWeight * (0.12f + dt * 0.08f), 0.0f, 0.35f);
                    sim->vFace[index] = Lerp(sim->vFace[index], target, blend);
                }
            }
        }
    }

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y <= sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int index = WIndex(sim, x, y, z);
                bool lowerFluid = FluidCellValid(sim, x, y - 1, z);
                bool upperFluid = FluidCellValid(sim, x, y, z);
                if (y == 0 || y == sim->ny || !(lowerFluid && upperFluid)) {
                    sim->wFace[index] = 0.0f;
                    continue;
                }

                float altNorm = Clamp(((float)y - 0.5f) / (float)sim->ny, 0.0f, 1.0f);
                float topWeight = Clamp((altNorm - (1.0f - TOP_SPONGE_FRACTION)) / TOP_SPONGE_FRACTION, 0.0f, 1.0f);
                if (topWeight > 0.0f) {
                    float damp = Clamp(topWeight * (0.18f + dt * 0.12f), 0.0f, 0.65f);
                    sim->wFace[index] *= (1.0f - damp);
                }
            }
        }
    }

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int index = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) {
                    sim->theta[index] = 0.0f;
                    sim->qv[index] = 0.0f;
                    sim->qc[index] = 0.0f;
                    sim->qr[index] = 0.0f;
                    sim->pressurePert[index] = 0.0f;
                    continue;
                }

                int edgeDist = x;
                if (sim->nx - 1 - x < edgeDist) edgeDist = sim->nx - 1 - x;
                if (z < edgeDist) edgeDist = z;
                if (sim->nz - 1 - z < edgeDist) edgeDist = sim->nz - 1 - z;
                float edgeWeight = Clamp(((float)EDGE_RELAX_BAND - (float)edgeDist) / (float)EDGE_RELAX_BAND, 0.0f, 1.0f);
                float topNorm = Clamp(((float)y + 0.5f) / (float)sim->ny, 0.0f, 1.0f);
                float topWeight = Clamp((topNorm - (1.0f - TOP_SPONGE_FRACTION)) / TOP_SPONGE_FRACTION, 0.0f, 1.0f);
                float relax = fmaxf(edgeWeight * 0.10f, topWeight * 0.22f);

                if (relax > 0.0f) {
                    float blend = Clamp(relax * (0.5f + 0.5f * dt), 0.0f, 0.75f);
                    sim->theta[index] = Lerp(sim->theta[index], sim->baseTheta[y], blend);
                    sim->qv[index] = Lerp(sim->qv[index], sim->baseQv[y], blend);
                    sim->qc[index] *= (1.0f - 0.7f * blend);
                    sim->qr[index] *= (1.0f - 0.8f * blend);
                }

                sim->qv[index] = fmaxf(0.0f, sim->qv[index]);
                sim->qc[index] = fmaxf(0.0f, sim->qc[index]);
                sim->qr[index] = fmaxf(0.0f, sim->qr[index]);
            }
        }
    }
}

static void UpdateDiagnostics(AtmosphereSim *sim) {
    UpdateVelocityDiagnostics(sim);

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int index = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) {
                    sim->temp[index] = 0.0f;
                    sim->pressure[index] = 0.0f;
                    sim->buoyancy[index] = 0.0f;
                    sim->divergence[index] = 0.0f;
                    continue;
                }

                float pressure = PressureAtCell(sim, y, index);
                sim->pressure[index] = pressure;
                sim->temp[index] = TemperatureFromPotentialTemperature(sim->theta[index], pressure);
                sim->buoyancy[index] = Clamp(CellBuoyancy(sim, y, sim->theta[index], sim->qv[index], sim->qc[index], sim->qr[index]),
                                             -4.0f,
                                             4.0f);
                sim->divergence[index] = CellDivergence(sim, x, y, z);
            }
        }
    }
}

static void UpdateStats(AtmosphereSim *sim) {
    double tempSum = 0.0;
    double pressureSum = 0.0;
    double atmosphericWaterMass = 0.0;
    int pressureCount = 0;
    float maxCloud = 0.0f;
    float maxRain = 0.0f;
    float maxWind = 0.0f;

    for (int mode = 0; mode < FIELD_COUNT; mode++) {
        sim->fieldDisplayMin[mode] = FLT_MAX;
        sim->fieldDisplayMax[mode] = -FLT_MAX;
    }

    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            int column = ColumnIndex(sim, x, z);
            tempSum += sim->surfaceTemp[column];
            if (sim->surfaceRain[column] > maxRain) maxRain = sim->surfaceRain[column];

            int y0 = sim->groundLayer[column] + 1;
            int y1 = y0 + 1;
            if (y0 >= 0 && y0 < sim->ny) {
                int i = AtmosIndex(sim, x, y0, z);
                pressureSum += sim->pressure[i];
                pressureCount++;
            }
            if (y1 >= 0 && y1 < sim->ny) {
                int i = AtmosIndex(sim, x, y1, z);
                float speed = sqrtf(sim->u[i] * sim->u[i] + sim->v[i] * sim->v[i] + sim->w[i] * sim->w[i]);
                if (speed > maxWind) maxWind = speed;
            }
        }
    }

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                if (!IsFluidCell(sim, x, y, z)) continue;

                for (int mode = 0; mode < FIELD_COUNT; mode++) {
                    float value = FieldDisplayValue(sim, x, y, z, (FieldMode)mode);
                    if (value < sim->fieldDisplayMin[mode]) sim->fieldDisplayMin[mode] = value;
                    if (value > sim->fieldDisplayMax[mode]) sim->fieldDisplayMax[mode] = value;
                }

                int i = AtmosIndex(sim, x, y, z);
                if (sim->qc[i] > maxCloud) maxCloud = sim->qc[i];
                atmosphericWaterMass += (double)(sim->qv[i] + sim->qc[i] + sim->qr[i]) * CellAirMassKg(sim, y);
            }
        }
    }

    for (int mode = 0; mode < FIELD_COUNT; mode++) {
        float minValue = sim->fieldDisplayMin[mode];
        float maxValue = sim->fieldDisplayMax[mode];

        if (minValue == FLT_MAX || maxValue == -FLT_MAX || !isfinite(minValue) || !isfinite(maxValue)) {
            FieldRange((FieldMode)mode, &sim->fieldDisplayMin[mode], &sim->fieldDisplayMax[mode]);
            continue;
        }

        if (mode == FIELD_VERTICAL_WIND || mode == FIELD_BUOYANCY) {
            float maxAbs = fmaxf(fabsf(minValue), fabsf(maxValue));
            float pad = fmaxf(maxAbs * 0.08f, 0.02f);
            sim->fieldDisplayMin[mode] = -(maxAbs + pad);
            sim->fieldDisplayMax[mode] = maxAbs + pad;
            continue;
        }

        float span = maxValue - minValue;
        if (span < 0.0001f) {
            float center = 0.5f * (minValue + maxValue);
            span = fmaxf(fabsf(center) * 0.15f, 0.1f);
            minValue = center - span;
            maxValue = center + span;
        } else {
            float pad = span * 0.08f;
            minValue -= pad;
            maxValue += pad;
        }

        if (mode == FIELD_WIND || mode == FIELD_CLOUD || mode == FIELD_RAIN || mode == FIELD_VAPOR) {
            minValue = fmaxf(0.0f, minValue);
        } else if (mode == FIELD_REL_HUMIDITY) {
            minValue = fmaxf(0.0f, minValue);
            maxValue = fminf(130.0f, maxValue);
        }

        sim->fieldDisplayMin[mode] = minValue;
        sim->fieldDisplayMax[mode] = maxValue;
    }

    sim->meanSurfaceTemp = (float)(tempSum / (double)sim->cells2D);
    sim->meanPressure = (pressureCount > 0) ? (float)(pressureSum / (double)pressureCount) : 0.0f;
    sim->maxCloud = maxCloud;
    sim->maxRain = maxRain;
    sim->maxWind = maxWind;
    sim->atmosphericWaterMass = atmosphericWaterMass;
    sim->totalWaterMass = atmosphericWaterMass + sim->precipitatedWaterMass;
    if (sim->initialWaterMass <= 0.0) sim->initialWaterMass = sim->totalWaterMass;
    sim->waterMassDrift = sim->totalWaterMass - sim->initialWaterMass;
}

static void InitializeAtmosphereState(AtmosphereSim *sim) {
    memset(sim->theta, 0, (size_t)sim->cells3D * sizeof(float));
    memset(sim->qv, 0, (size_t)sim->cells3D * sizeof(float));
    memset(sim->qc, 0, (size_t)sim->cells3D * sizeof(float));
    memset(sim->qr, 0, (size_t)sim->cells3D * sizeof(float));
    memset(sim->pressurePert, 0, (size_t)sim->cells3D * sizeof(float));
    memset(sim->uFace, 0, (size_t)sim->cellsU * sizeof(float));
    memset(sim->vFace, 0, (size_t)sim->cellsV * sizeof(float));
    memset(sim->wFace, 0, (size_t)sim->cellsW * sizeof(float));
    memset(sim->surfaceRain, 0, (size_t)sim->cells2D * sizeof(float));
    sim->precipitatedWaterMass = 0.0;
    sim->initialWaterMass = 0.0;
    sim->waterMassDrift = 0.0;
    sim->maxDivBeforeProjection = 0.0f;
    sim->maxDivAfterProjection = 0.0f;
    sim->nonFiniteCorrections = 0;
    sim->negativeWaterCorrections = 0;
    sim->sliceDirty = true;

    AtmosForcing forcing = UpdateForcingInputs(sim);
    InitBackgroundAtmosphere(sim, &forcing);

    float seaLevelTemp = BaseSeaLevelTemp(sim->simSeconds);
    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            int column = ColumnIndex(sim, x, z);
            float terrainAlt = sim->columnTerrainMeters[column];
            float terrainNorm = NormalizeRange(terrainAlt, sim->terrain.minTerrainMeters, sim->terrain.maxTerrainMeters);
            float valley = ColumnValleyFactor(sim, column);
            float ridge = ColumnRidgeFactor(sim, column);
            Vector3 normal = Vector3Normalize((Vector3) { -sim->columnDx[column], 1.0f, -sim->columnDz[column] });
            float exposure = fmaxf(0.0f, Vector3DotProduct(normal, forcing.sunDir));

            sim->surfaceTemp[column] = Clamp(seaLevelTemp - 0.0062f * terrainAlt +
                                             forcing.solar * exposure * 8.5f -
                                             ridge * 1.4f -
                                             valley * (1.0f - forcing.solar) * 1.0f,
                                             -28.0f,
                                             32.0f);
            sim->surfaceWetness[column] = Clamp(0.20f + 0.52f * valley + 0.16f * (1.0f - terrainNorm), 0.06f, 1.0f);
        }
    }

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int index = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) continue;

                int column = ColumnIndex(sim, x, z);
                float layerAboveGround = (float)(y - sim->groundLayer[column]);
                float boundary = expf(-0.45f * fmaxf(layerAboveGround - 1.0f, 0.0f));
                float altNorm = Clamp(CellAltitudeMeters(sim, y) / sim->topMeters, 0.0f, 1.0f);
                float perturb = (NoiseCoord((float)x * 0.23f, (float)y * 0.19f, (float)z * 0.17f + (float)sim->terrainGen.seed * 0.001f) - 0.5f) * 0.9f;
                float pressure = sim->basePressure[y];
                float temp = sim->baseTemp[y] +
                             boundary * (sim->surfaceTemp[column] - sim->baseTemp[y]) * 0.18f +
                             perturb * (0.7f - 0.3f * altNorm);
                float qsat = SaturationMixingRatio(temp, pressure);
                float rh = Clamp(0.68f + 0.18f * sim->surfaceWetness[column] * boundary - 0.12f * altNorm + perturb * 0.04f,
                                 0.32f,
                                 0.96f);

                sim->theta[index] = PotentialTemperatureFromTemp(temp, pressure);
                sim->qv[index] = qsat * rh;
            }
        }
    }

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            float altNorm = Clamp(CellAltitudeMeters(sim, y) / sim->topMeters, 0.0f, 1.0f);
            float profile = 0.58f + 0.52f * altNorm;
            for (int x = 0; x <= sim->nx; x++) {
                bool leftFluid = FluidCellValid(sim, x - 1, y, z);
                bool rightFluid = FluidCellValid(sim, x, y, z);
                if (leftFluid || rightFluid) sim->uFace[UIndex(sim, x, y, z)] = forcing.synopticU * profile;
            }
        }
    }

    for (int z = 0; z <= sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            float altNorm = Clamp(CellAltitudeMeters(sim, y) / sim->topMeters, 0.0f, 1.0f);
            float profile = 0.58f + 0.52f * altNorm;
            for (int x = 0; x < sim->nx; x++) {
                bool southFluid = FluidCellValid(sim, x, y, z - 1);
                bool northFluid = FluidCellValid(sim, x, y, z);
                if (southFluid || northFluid) sim->vFace[VIndex(sim, x, y, z)] = forcing.synopticV * profile;
            }
        }
    }

    ApplyBoundaryConditions(sim, &forcing, 0.0f);
    UpdateDiagnostics(sim);
    UpdateStats(sim);
}

static void StepAtmosphere(AtmosphereSim *sim, float dt) {
    AtmosForcing forcing = UpdateForcingInputs(sim);
    sim->nonFiniteCorrections = 0;
    sim->negativeWaterCorrections = 0;
    InitBackgroundAtmosphere(sim, &forcing);
    ApplySurfaceFluxes(sim, &forcing, dt);
    ApplyBodyForces(sim, dt);
    AdvectMomentum(sim, dt);
    ApplyBoundaryConditions(sim, &forcing, dt);
    SanitizeState(sim);
    UpdateVelocityDiagnostics(sim);
    AdvectScalars(sim, dt);
    ApplyMicrophysics(sim, dt);
    SanitizeState(sim);
    ProjectPressure(sim, dt);
    ApplyBoundaryConditions(sim, &forcing, dt);
    SanitizeState(sim);
    UpdateDiagnostics(sim);
}

void ResetAtmosphere(AtmosphereSim *sim) {
    sim->simSeconds = 11.0f * 3600.0f;
    sim->timeScale = DEFAULT_TIME_SCALE;
    sim->prevailingU = 7.5f;
    sim->prevailingV = 2.5f;
    sim->terrainAlpha = 1.0f;
    sim->fieldMode = FIELD_TEMPERATURE;
    sim->verticalAxis = SLICE_Z;
    sim->horizontalLayer = sim->ny / 2;
    sim->verticalSlice = sim->nz / 2;
    sim->showHorizontalSlice = true;
    sim->showVerticalSlice = true;
    sim->showTerrain = true;
    sim->showWireframe = false;
    sim->showWindArrows = true;
    sim->paused = false;
    sim->showUI = true;
    sim->simAccumulatorSeconds = 0.0f;
    sim->solverStepsLastFrame = 0;
    sim->solverCpuSecondsLastFrame = 0.0;
    sim->sliceDirty = true;
    sim->lastSliceUpdateTime = 0.0;
    sim->sunDir = Vector3Normalize((Vector3) { 0.2f, 1.0f, 0.3f });
    sim->solarStrength = 0.0f;

    InitializeAtmosphereState(sim);
}

void UpdateAtmosphere(AtmosphereSim *sim, float frameDt) {
    float scaledDt = Clamp(frameDt, 0.0f, 0.05f) * sim->timeScale;
    sim->simAccumulatorSeconds += scaledDt;
    if (sim->simAccumulatorSeconds > SIM_MAX_CATCHUP) sim->simAccumulatorSeconds = SIM_MAX_CATCHUP;

    double start = GetTime();
    int steps = 0;

    while (sim->simAccumulatorSeconds >= SIM_FIXED_DT) {
        if ((GetTime() - start) >= SIM_FRAME_BUDGET_SEC) break;

        sim->simSeconds = fmodf(sim->simSeconds + SIM_FIXED_DT, DAY_SECONDS);
        if (sim->simSeconds < 0.0f) sim->simSeconds += DAY_SECONDS;

        StepAtmosphere(sim, SIM_FIXED_DT);
        sim->simAccumulatorSeconds -= SIM_FIXED_DT;
        steps++;
    }

    sim->solverStepsLastFrame = steps;
    sim->solverCpuSecondsLastFrame = GetTime() - start;
    if (steps > 0) sim->sliceDirty = true;
    UpdateStats(sim);
}

static const char *FieldName(FieldMode mode) {
    switch (mode) {
        case FIELD_TEMPERATURE: return "Temperature";
        case FIELD_PRESSURE: return "Pressure";
        case FIELD_REL_HUMIDITY: return "Relative Humidity";
        case FIELD_DEWPOINT: return "Dew Point";
        case FIELD_VAPOR: return "Water Vapor";
        case FIELD_CLOUD: return "Cloud Water";
        case FIELD_RAIN: return "Rain Water";
        case FIELD_WIND: return "Wind Speed";
        case FIELD_VERTICAL_WIND: return "Vertical Wind";
        case FIELD_BUOYANCY: return "Buoyancy";
        default: return "Field";
    }
}

static const char *FieldUnits(FieldMode mode) {
    switch (mode) {
        case FIELD_TEMPERATURE: return "deg C";
        case FIELD_PRESSURE: return "hPa";
        case FIELD_REL_HUMIDITY: return "%";
        case FIELD_DEWPOINT: return "deg C";
        case FIELD_VAPOR: return "g/kg";
        case FIELD_CLOUD: return "g/kg";
        case FIELD_RAIN: return "g/kg";
        case FIELD_WIND: return "m/s";
        case FIELD_VERTICAL_WIND: return "m/s";
        case FIELD_BUOYANCY: return "m/s^2";
        default: return "";
    }
}

static void FieldRange(FieldMode mode, float *outMin, float *outMax) {
    switch (mode) {
        case FIELD_TEMPERATURE: *outMin = -35.0f; *outMax = 25.0f; break;
        case FIELD_PRESSURE: *outMin = 560.0f; *outMax = 1015.0f; break;
        case FIELD_REL_HUMIDITY: *outMin = 0.0f; *outMax = 100.0f; break;
        case FIELD_DEWPOINT: *outMin = -35.0f; *outMax = 18.0f; break;
        case FIELD_VAPOR: *outMin = 0.0f; *outMax = 14.0f; break;
        case FIELD_CLOUD: *outMin = 0.0f; *outMax = 3.0f; break;
        case FIELD_RAIN: *outMin = 0.0f; *outMax = 5.0f; break;
        case FIELD_WIND: *outMin = 0.0f; *outMax = 25.0f; break;
        case FIELD_VERTICAL_WIND: *outMin = -8.0f; *outMax = 8.0f; break;
        case FIELD_BUOYANCY: *outMin = -0.9f; *outMax = 0.9f; break;
        default: *outMin = 0.0f; *outMax = 1.0f; break;
    }
}

static void FieldDisplayRange(const AtmosphereSim *sim, FieldMode mode, float *outMin, float *outMax) {
    float minValue = sim->fieldDisplayMin[mode];
    float maxValue = sim->fieldDisplayMax[mode];

    if (!isfinite(minValue) || !isfinite(maxValue) || minValue >= maxValue) {
        FieldRange(mode, outMin, outMax);
        return;
    }

    *outMin = minValue;
    *outMax = maxValue;
}

static float FieldDisplayValue(const AtmosphereSim *sim, int x, int y, int z, FieldMode mode) {
    int i = AtmosIndex(sim, x, y, z);
    switch (mode) {
        case FIELD_TEMPERATURE: return sim->temp[i];
        case FIELD_PRESSURE: return sim->pressure[i];
        case FIELD_REL_HUMIDITY: return RelativeHumidity(sim->temp[i], sim->pressure[i], sim->qv[i]);
        case FIELD_DEWPOINT: return DewPointC(sim->pressure[i], sim->qv[i]);
        case FIELD_VAPOR: return sim->qv[i] * 1000.0f;
        case FIELD_CLOUD: return sim->qc[i] * 1000.0f;
        case FIELD_RAIN: return sim->qr[i] * 1000.0f;
        case FIELD_WIND: return sqrtf(sim->u[i] * sim->u[i] + sim->v[i] * sim->v[i] + sim->w[i] * sim->w[i]);
        case FIELD_VERTICAL_WIND: return sim->w[i];
        case FIELD_BUOYANCY: return sim->buoyancy[i];
        default: return 0.0f;
    }
}

static float FieldDisplaySample(const AtmosphereSim *sim, float x, float y, float z, FieldMode mode) {
    float temp = SampleAtmosField(sim, sim->temp, x, y, z);
    float pressure = SampleAtmosField(sim, sim->pressure, x, y, z);
    float vapor = SampleAtmosField(sim, sim->qv, x, y, z);
    float cloud = SampleAtmosField(sim, sim->qc, x, y, z);
    float rain = SampleAtmosField(sim, sim->qr, x, y, z);
    float u = SampleAtmosField(sim, sim->u, x, y, z);
    float v = SampleAtmosField(sim, sim->v, x, y, z);
    float w = SampleAtmosField(sim, sim->w, x, y, z);
    float buoyancy = SampleAtmosField(sim, sim->buoyancy, x, y, z);

    switch (mode) {
        case FIELD_TEMPERATURE: return temp;
        case FIELD_PRESSURE: return pressure;
        case FIELD_REL_HUMIDITY: return RelativeHumidity(temp, pressure, vapor);
        case FIELD_DEWPOINT: return DewPointC(pressure, vapor);
        case FIELD_VAPOR: return vapor * 1000.0f;
        case FIELD_CLOUD: return cloud * 1000.0f;
        case FIELD_RAIN: return rain * 1000.0f;
        case FIELD_WIND: return sqrtf(u * u + v * v + w * w);
        case FIELD_VERTICAL_WIND: return w;
        case FIELD_BUOYANCY: return buoyancy;
        default: return 0.0f;
    }
}

static Color FieldColorFromValue(const AtmosphereSim *sim, FieldMode mode, float displayValue, float directionU, float directionV) {
    (void)directionU;
    (void)directionV;
    float minValue = 0.0f;
    float maxValue = 1.0f;
    FieldDisplayRange(sim, mode, &minValue, &maxValue);
    float t = NormalizeRange(displayValue, minValue, maxValue);

    switch (mode) {
        case FIELD_TEMPERATURE:
            return RampColor(t, kSpectralStops, (int)(sizeof(kSpectralStops) / sizeof(kSpectralStops[0])));
        case FIELD_PRESSURE:
            return RampColor(t, kSpectralStops, (int)(sizeof(kSpectralStops) / sizeof(kSpectralStops[0])));
        case FIELD_REL_HUMIDITY:
            return RampColor(t, kHumidityStops, (int)(sizeof(kHumidityStops) / sizeof(kHumidityStops[0])));
        case FIELD_DEWPOINT:
            return RampColor(t, kSpectralStops, (int)(sizeof(kSpectralStops) / sizeof(kSpectralStops[0])));
        case FIELD_VAPOR:
            return RampColor(t, kSpectralStops, (int)(sizeof(kSpectralStops) / sizeof(kSpectralStops[0])));
        case FIELD_CLOUD:
            return RampColor(t, kCloudStops, (int)(sizeof(kCloudStops) / sizeof(kCloudStops[0])));
        case FIELD_RAIN:
            return RampColor(t, kSpectralStops, (int)(sizeof(kSpectralStops) / sizeof(kSpectralStops[0])));
        case FIELD_WIND:
            return RampColor(t, kSpectralStops, (int)(sizeof(kSpectralStops) / sizeof(kSpectralStops[0])));
        case FIELD_VERTICAL_WIND:
            return RampColor(t, kDivergingStops, (int)(sizeof(kDivergingStops) / sizeof(kDivergingStops[0])));
        case FIELD_BUOYANCY:
            return RampColor(t, kDivergingStops, (int)(sizeof(kDivergingStops) / sizeof(kDivergingStops[0])));
        default:
            return WHITE;
    }
}

static Color TerrainSliceColor(float terrainAlt, const AtmosphereSim *sim) {
    float hNorm = NormalizeRange(terrainAlt, sim->terrain.minTerrainMeters, sim->terrain.maxTerrainMeters);
    if (hNorm > 0.80f) return (Color) { 205, 210, 216, 255 };
    if (hNorm > 0.52f) return (Color) { 142, 126, 106, 255 };
    return (Color) { 89, 104, 74, 255 };
}

static void UpdateHorizontalSliceTexture(AtmosphereSim *sim) {
    int layer = Clamp(sim->horizontalLayer, 0, sim->ny - 1);
    int texWidth = sim->nx * SLICE_TEX_SCALE;
    int texHeight = sim->nz * SLICE_TEX_SCALE;

    for (int z = 0; z < texHeight; z++) {
        float sampleZ = (texHeight > 1) ? ((float)z / (float)(texHeight - 1)) * (float)(sim->nz - 1) : 0.0f;
        int cellZ = Clamp((int)lroundf(sampleZ), 0, sim->nz - 1);

        for (int x = 0; x < texWidth; x++) {
            float sampleX = (texWidth > 1) ? ((float)x / (float)(texWidth - 1)) * (float)(sim->nx - 1) : 0.0f;
            int cellX = Clamp((int)lroundf(sampleX), 0, sim->nx - 1);
            int dst = z * texWidth + x;

            if (!IsFluidCell(sim, cellX, layer, cellZ)) {
                sim->horizontalPixels[dst] = BLANK;
                continue;
            }

            float value = FieldDisplaySample(sim, sampleX, (float)layer, sampleZ, sim->fieldMode);
            float dirU = SampleAtmosField(sim, sim->u, sampleX, (float)layer, sampleZ);
            float dirV = SampleAtmosField(sim, sim->v, sampleX, (float)layer, sampleZ);
            Color color = FieldColorFromValue(sim, sim->fieldMode, value, dirU, dirV);
            unsigned char alpha = (sim->fieldMode == FIELD_CLOUD || sim->fieldMode == FIELD_RAIN) ? 216 : 236;
            sim->horizontalPixels[dst] = WithAlpha(color, alpha);
        }
    }
    UpdateTexture(sim->horizontalTex, sim->horizontalPixels);
}

static void UpdateVerticalSliceTexture(AtmosphereSim *sim) {
    int slice = Clamp(sim->verticalSlice, 0, (sim->verticalAxis == SLICE_X) ? sim->nx - 1 : sim->nz - 1);
    int logicalWidth = (sim->verticalAxis == SLICE_X) ? sim->nz : sim->nx;
    int texWidth = ((sim->nx > sim->nz) ? sim->nx : sim->nz) * SLICE_TEX_SCALE;
    int logicalTexWidth = logicalWidth * SLICE_TEX_SCALE;
    int texHeight = sim->ny * SLICE_TEX_SCALE;

    for (int y = 0; y < texHeight; y++) {
        float sampleY = (texHeight > 1) ? ((float)(texHeight - 1 - y) / (float)(texHeight - 1)) * (float)(sim->ny - 1) : 0.0f;
        int cellY = Clamp((int)lroundf(sampleY), 0, sim->ny - 1);

        for (int j = 0; j < texWidth; j++) {
            int dst = y * texWidth + j;
            if (j >= logicalTexWidth) {
                sim->verticalPixels[dst] = BLANK;
                continue;
            }

            float sampleJ = (logicalTexWidth > 1) ? ((float)j / (float)(logicalTexWidth - 1)) * (float)(logicalWidth - 1) : 0.0f;
            float sampleX = (sim->verticalAxis == SLICE_X) ? (float)slice : sampleJ;
            float sampleZ = (sim->verticalAxis == SLICE_X) ? sampleJ : (float)slice;
            int cellX = Clamp((int)lroundf(sampleX), 0, sim->nx - 1);
            int cellZ = Clamp((int)lroundf(sampleZ), 0, sim->nz - 1);
            int column = ColumnIndex(sim, cellX, cellZ);

            if (!IsFluidCell(sim, cellX, cellY, cellZ)) {
                sim->verticalPixels[dst] = TerrainSliceColor(sim->columnTerrainMeters[column], sim);
                continue;
            }

            float value = FieldDisplaySample(sim, sampleX, sampleY, sampleZ, sim->fieldMode);
            float dirU = SampleAtmosField(sim, sim->u, sampleX, sampleY, sampleZ);
            float dirV = SampleAtmosField(sim, sim->v, sampleX, sampleY, sampleZ);
            Color color = FieldColorFromValue(sim, sim->fieldMode, value, dirU, dirV);
            float topFade = Clamp(((float)(sim->ny - 1) - sampleY) / 0.9f, 0.0f, 1.0f);
            unsigned char alpha = (sim->fieldMode == FIELD_CLOUD || sim->fieldMode == FIELD_RAIN) ? 220 : 240;
            alpha = (unsigned char)((float)alpha * (0.18f + 0.82f * topFade));
            sim->verticalPixels[dst] = WithAlpha(color, alpha);
        }
    }
    UpdateTexture(sim->verticalTex, sim->verticalPixels);
}

static void UpdateSliceTextures(AtmosphereSim *sim) {
    if (sim->showHorizontalSlice) UpdateHorizontalSliceTexture(sim);
    if (sim->showVerticalSlice) UpdateVerticalSliceTexture(sim);
}

void UpdateSliceTexturesIfNeeded(AtmosphereSim *sim, bool force) {
    double now = GetTime();
    double minInterval = 1.0 / SLICE_UPDATE_RATE_HZ;
    bool ready = force || sim->lastSliceUpdateTime <= 0.0 || (now - sim->lastSliceUpdateTime) >= minInterval;
    if (!sim->sliceDirty || !ready) return;

    UpdateSliceTextures(sim);
    sim->sliceDirty = false;
    sim->lastSliceUpdateTime = now;
}

Color SkyColor(const AtmosphereSim *sim) {
    float dayProgress = fmodf(sim->simSeconds, DAY_SECONDS) / DAY_SECONDS;
    float sun = fmaxf(0.0f, sinf(TAU * (dayProgress - 0.25f)));
    Color night = (Color) { 10, 18, 33, 255 };
    Color dawn = (Color) { 226, 156, 112, 255 };
    Color noon = (Color) { 112, 170, 226, 255 };
    Color overcast = (Color) { 149, 160, 173, 255 };
    Color base = (sun > 0.07f) ? LerpColor(dawn, noon, sun) : LerpColor(night, dawn, sun * 7.5f);
    float overcastMix = Clamp(sim->maxCloud * 210.0f, 0.0f, 0.55f);
    return LerpColor(base, overcast, overcastMix);
}

static ImU32 ImGuiColorU32(Color color, float alphaScale) {
    ImVec4_c col = ImVec4Make((float)color.r / 255.0f,
                              (float)color.g / 255.0f,
                              (float)color.b / 255.0f,
                              ((float)color.a / 255.0f) * alphaScale);
    return igColorConvertFloat4ToU32(col);
}

static void DrawFieldLegendHud(const AtmosphereSim *sim, FieldMode mode) {
    float minValue = 0.0f;
    float maxValue = 1.0f;
    FieldDisplayRange(sim, mode, &minValue, &maxValue);

    ImVec2_c avail = igGetContentRegionAvail();
    float legendWidth = fminf(fmaxf(avail.x - 8.0f, 160.0f), 320.0f);
    float legendHeight = 16.0f;
    float labelHeight = 18.0f;
    const int segments = 56;

    igTextDisabled("Color scale");
    ImVec2_c pos = igGetCursorScreenPos();
    ImDrawList *drawList = igGetWindowDrawList();

    for (int segment = 0; segment < segments; segment++) {
        float t0 = (float)segment / (float)segments;
        float t1 = (float)(segment + 1) / (float)segments;
        float value0 = Lerp(minValue, maxValue, t0);
        float value1 = Lerp(minValue, maxValue, t1);
        Color color0 = FieldColorFromValue(sim, mode, value0, 1.0f, 0.0f);
        Color color1 = FieldColorFromValue(sim, mode, value1, 1.0f, 0.0f);
        ImVec2_c p0 = ImVec2Make(pos.x + legendWidth * t0, pos.y);
        ImVec2_c p1 = ImVec2Make(pos.x + legendWidth * t1, pos.y + legendHeight);
        ImU32 c0 = ImGuiColorU32(color0, 1.0f);
        ImU32 c1 = ImGuiColorU32(color1, 1.0f);
        ImDrawList_AddRectFilledMultiColor(drawList, p0, p1, c0, c1, c1, c0);
    }

    ImDrawList_AddRect(drawList,
                       pos,
                       ImVec2Make(pos.x + legendWidth, pos.y + legendHeight),
                       ImGuiColorU32((Color) { 18, 24, 36, 255 }, 0.95f),
                       3.0f,
                       0,
                       1.0f);

    char minLabel[32];
    char maxLabel[64];
    snprintf(minLabel, sizeof(minLabel), "%.1f", minValue);
    snprintf(maxLabel, sizeof(maxLabel), "%.1f %s", maxValue, FieldUnits(mode));

    ImDrawList_AddText_Vec2(drawList,
                            ImVec2Make(pos.x, pos.y + legendHeight + 4.0f),
                            ImGuiColorU32(RAYWHITE, 0.92f),
                            minLabel,
                            NULL);
    ImVec2_c maxSize = igCalcTextSize(maxLabel, NULL, false, -1.0f);
    ImDrawList_AddText_Vec2(drawList,
                            ImVec2Make(pos.x + legendWidth - maxSize.x, pos.y + legendHeight + 4.0f),
                            ImGuiColorU32(RAYWHITE, 0.92f),
                            maxLabel,
                            NULL);

    igDummy(ImVec2Make(legendWidth, legendHeight + labelHeight + 8.0f));
}

static void DrawWindArrowsOnHorizontalSlice(const AtmosphereSim *sim) {
    int layer = Clamp(sim->horizontalLayer, 0, sim->ny - 1);
    float y = WorldYFromMeters(CellAltitudeMeters(sim, layer));

    for (int z = 0; z < sim->nz; z += HSLICE_ARROW_STEP) {
        for (int x = 0; x < sim->nx; x += HSLICE_ARROW_STEP) {
            if (!IsFluidCell(sim, x, layer, z)) continue;
            int i = AtmosIndex(sim, x, layer, z);
            float speed = sqrtf(sim->u[i] * sim->u[i] + sim->v[i] * sim->v[i] + sim->w[i] * sim->w[i]);
            if (speed < 1.2f) continue;

            Vector3 start = { (float)x * sim->dxWorld, y, (float)z * sim->dzWorld };
            Vector3 tip = {
                start.x + sim->u[i] * 0.24f,
                start.y + sim->w[i] * 0.16f,
                start.z + sim->v[i] * 0.24f
            };
            Color color = WithAlpha(FieldColorFromValue(sim, FIELD_WIND, speed, sim->u[i], sim->v[i]), 220);
            DrawLine3D(start, tip, color);
        }
    }
}

static void DrawWindArrowsOnVerticalSlice(const AtmosphereSim *sim) {
    int slice = Clamp(sim->verticalSlice, 0, (sim->verticalAxis == SLICE_X) ? sim->nx - 1 : sim->nz - 1);

    if (sim->verticalAxis == SLICE_X) {
        float xWorld = (float)slice * sim->dxWorld;
        for (int z = 0; z < sim->nz; z += VSLICE_ARROW_STEP) {
            for (int y = 0; y < sim->ny; y += VSLICE_ARROW_STEP) {
                if (!IsFluidCell(sim, slice, y, z)) continue;
                int i = AtmosIndex(sim, slice, y, z);
                float speed = sqrtf(sim->v[i] * sim->v[i] + sim->w[i] * sim->w[i]);
                if (speed < 0.8f) continue;
                Vector3 start = { xWorld, WorldYFromMeters(CellAltitudeMeters(sim, y)), (float)z * sim->dzWorld };
                Vector3 tip = { start.x, start.y + sim->w[i] * 0.20f, start.z + sim->v[i] * 0.22f };
                Color color = WithAlpha(FieldColorFromValue(sim, FIELD_WIND, speed, 0.0f, sim->v[i]), 220);
                DrawLine3D(start, tip, color);
            }
        }
    } else {
        float zWorld = (float)slice * sim->dzWorld;
        for (int x = 0; x < sim->nx; x += VSLICE_ARROW_STEP) {
            for (int y = 0; y < sim->ny; y += VSLICE_ARROW_STEP) {
                if (!IsFluidCell(sim, x, y, slice)) continue;
                int i = AtmosIndex(sim, x, y, slice);
                float speed = sqrtf(sim->u[i] * sim->u[i] + sim->w[i] * sim->w[i]);
                if (speed < 0.8f) continue;
                Vector3 start = { (float)x * sim->dxWorld, WorldYFromMeters(CellAltitudeMeters(sim, y)), zWorld };
                Vector3 tip = { start.x + sim->u[i] * 0.22f, start.y + sim->w[i] * 0.20f, start.z };
                Color color = WithAlpha(FieldColorFromValue(sim, FIELD_WIND, speed, sim->u[i], 0.0f), 220);
                DrawLine3D(start, tip, color);
            }
        }
    }
}

void DrawSlices3D(const AtmosphereSim *sim, Camera camera) {
    (void)camera;
    rlDisableBackfaceCulling();

    if (sim->showHorizontalSlice) {
        float y = WorldYFromMeters(CellAltitudeMeters(sim, sim->horizontalLayer));
        DrawModel(sim->horizontalSliceModel, (Vector3) { sim->worldWidth * 0.5f, y, sim->worldDepth * 0.5f }, 1.0f, WHITE);
    }

    if (sim->showVerticalSlice) {
        if (sim->verticalAxis == SLICE_X) {
            float x = (float)Clamp(sim->verticalSlice, 0, sim->nx - 1) * sim->dxWorld;
            DrawModelEx(sim->verticalSliceModelX,
                        (Vector3) { x, sim->topWorldY * 0.5f, sim->worldDepth * 0.5f },
                        (Vector3) { 0.0f, 0.0f, 1.0f }, 90.0f,
                        (Vector3) { 1.0f, 1.0f, 1.0f },
                        WHITE);
        } else {
            float z = (float)Clamp(sim->verticalSlice, 0, sim->nz - 1) * sim->dzWorld;
            DrawModelEx(sim->verticalSliceModelZ,
                        (Vector3) { sim->worldWidth * 0.5f, sim->topWorldY * 0.5f, z },
                        (Vector3) { 1.0f, 0.0f, 0.0f }, 90.0f,
                        (Vector3) { 1.0f, 1.0f, 1.0f },
                        WHITE);
        }
    }

    rlEnableBackfaceCulling();

    if (sim->showWindArrows) {
        if (sim->showHorizontalSlice) DrawWindArrowsOnHorizontalSlice(sim);
        if (sim->showVerticalSlice) DrawWindArrowsOnVerticalSlice(sim);
    }
}

void DrawImGuiHud(const AtmosphereSim *sim) {
    int hour = ((int)(sim->simSeconds / 3600.0f)) % 24;
    int minute = ((int)(sim->simSeconds / 60.0f)) % 60;
    float fieldMin = 0.0f;
    float fieldMax = 1.0f;
    FieldDisplayRange(sim, sim->fieldMode, &fieldMin, &fieldMax);

    igSetNextWindowPos(ImVec2Make(14.0f, 14.0f), ImGuiCond_Always, ImVec2Make(0.0f, 0.0f));
    igSetNextWindowBgAlpha(0.78f);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
                             ImGuiWindowFlags_AlwaysAutoResize |
                             ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoSavedSettings |
                             ImGuiWindowFlags_NoNav |
                             ImGuiWindowFlags_NoFocusOnAppearing;

    if (igBegin("##AtmosphereHud", NULL, flags)) {
        igText("Mountain Atmosphere Solver");
        igSeparator();
        igTextDisabled("Low-Mach terrain-aware flow with moisture, warm rain, and projection");
        igText("Field  %s", FieldName(sim->fieldMode));
        igText("Scale  %.1f to %.1f %s", fieldMin, fieldMax, FieldUnits(sim->fieldMode));
        igText("Time   %02d:%02d", hour, minute);
        if (sim->paused) igTextColored(ImVec4Make(1.00f, 0.71f, 0.20f, 1.00f), "Paused");
        igText("Surface %.1f C", sim->meanSurfaceTemp);
        igText("Pressure %.1f hPa", sim->meanPressure);
        igText("Wind %.1f m/s", sim->maxWind);
        igText("Rain %.1f mm/h", sim->maxRain);
        igText("Cloud %.2f g/kg", sim->maxCloud * 1000.0f);
        igText("Steps %d  Backlog %.1f s", sim->solverStepsLastFrame, sim->simAccumulatorSeconds);
        igText("Solver %.2f ms", sim->solverCpuSecondsLastFrame * 1000.0);
        igText("Div %.3g -> %.3g", sim->maxDivBeforeProjection, sim->maxDivAfterProjection);
        igText("Water drift %.3e kg", sim->waterMassDrift);
        igText("State fixes %d nf / %d neg", sim->nonFiniteCorrections, sim->negativeWaterCorrections);
        ImGuiIO *io = igGetIO_Nil();
        if (io != NULL) igText("FPS %.0f", io->Framerate);
        igSeparator();
        DrawFieldLegendHud(sim, sim->fieldMode);
    }
    igEnd();
}

bool DrawImGuiControls(AtmosphereSim *sim) {
    bool terrainChanged = false;

    if (!sim->showUI) {
        Rectangle toggleRect = GetUiToggleRect();
        igSetNextWindowPos(ImVec2Make(toggleRect.x, toggleRect.y), ImGuiCond_Always, ImVec2Make(0.0f, 0.0f));
        igSetNextWindowBgAlpha(0.92f);

        ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration |
                                 ImGuiWindowFlags_AlwaysAutoResize |
                                 ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoResize |
                                 ImGuiWindowFlags_NoSavedSettings;

        if (igBegin("##ShowControls", NULL, flags)) {
            if (igButton("Show Controls", ImVec2Make(toggleRect.width - 18.0f, 0.0f))) sim->showUI = true;
        }
        igEnd();
        return false;
    }

    Rectangle panel = GetControlPanelRect();
    igSetNextWindowPos(ImVec2Make(panel.x, panel.y), ImGuiCond_Always, ImVec2Make(0.0f, 0.0f));
    igSetNextWindowSize(ImVec2Make(panel.width, panel.height), ImGuiCond_Always);
    igSetNextWindowBgAlpha(0.98f);

    ImGuiWindowFlags flags = ImGuiWindowFlags_NoMove |
                             ImGuiWindowFlags_NoResize |
                             ImGuiWindowFlags_NoSavedSettings |
                             ImGuiWindowFlags_NoCollapse;

    if (igBegin("Solver Controls", NULL, flags)) {
        if (igButton(sim->paused ? "Resume" : "Pause", ImVec2Make(110.0f, 0.0f))) sim->paused = !sim->paused;
        igSameLine(0.0f, 8.0f);
        if (igButton("Reset Atmosphere", ImVec2Make(150.0f, 0.0f))) ResetAtmosphere(sim);
        igSameLine(0.0f, 8.0f);
        if (igButton("Hide Panel", ImVec2Make(106.0f, 0.0f))) sim->showUI = false;

        igSeparator();
        if (igBeginChild_Str("##AtmosphereScroll", ImVec2Make(0.0f, 0.0f), ImGuiChildFlags_Borders, ImGuiWindowFlags_None)) {
            igPushItemWidth(-FLT_MIN);

            if (igBeginTabBar("##ControlTabs", ImGuiTabBarFlags_None)) {
                if (igBeginTabItem("Atmosphere", NULL, ImGuiTabItemFlags_None)) {
                    igSeparatorText("Display");
                    int fieldMode = (int)sim->fieldMode;
                    if (DrawLabeledCombo("Field", "##FieldMode", &fieldMode, kFieldLabels, FIELD_COUNT)) {
                        sim->fieldMode = (FieldMode)fieldMode;
                    }
                    igCheckbox("Show terrain", &sim->showTerrain);
                    igCheckbox("Wireframe terrain", &sim->showWireframe);
                    igCheckbox("Show wind vectors", &sim->showWindArrows);
                    igBeginDisabled(!sim->showTerrain);
                    DrawLabeledSliderFloat("Terrain opacity", "##TerrainOpacity", &sim->terrainAlpha, 0.05f, 1.0f, "%.2f");
                    igEndDisabled();

                    igSeparatorText("Slices");
                    igCheckbox("Horizontal slice", &sim->showHorizontalSlice);
                    igBeginDisabled(!sim->showHorizontalSlice);
                    DrawLabeledSliderInt("Horizontal layer", "##HorizontalLayer", &sim->horizontalLayer, 0, sim->ny - 1, "%d");
                    igEndDisabled();

                    igCheckbox("Vertical slice", &sim->showVerticalSlice);
                    igBeginDisabled(!sim->showVerticalSlice);
                    int axis = (int)sim->verticalAxis;
                    if (DrawLabeledCombo("Vertical axis", "##VerticalAxis", &axis, kSliceAxisLabels, 2)) {
                        sim->verticalAxis = (SliceAxis)axis;
                    }
                    int sliceMax = (sim->verticalAxis == SLICE_X) ? sim->nx - 1 : sim->nz - 1;
                    sim->verticalSlice = Clamp(sim->verticalSlice, 0, sliceMax);
                    DrawLabeledSliderInt("Slice index", "##SliceIndex", &sim->verticalSlice, 0, sliceMax, "%d");
                    igEndDisabled();

                    igSeparatorText("Solver Update");
                    DrawLabeledSliderFloat("Time scale", "##TimeScale", &sim->timeScale, 30.0f, 1200.0f, "%.0fx");
                    igTextWrapped("The atmosphere now advances by timestep integration: surface fluxes, momentum and scalar advection, warm-rain microphysics, pressure projection, and terrain boundary handling.");
                    igTextDisabled("Current units");
                    igText("%s", FieldUnits(sim->fieldMode));
                    igTextDisabled("Statistics");
                    igText("Surface temp  %.1f C", sim->meanSurfaceTemp);
                    igText("Pressure      %.1f hPa", sim->meanPressure);
                    igText("Peak wind     %.1f m/s", sim->maxWind);
                    igText("Peak rain     %.1f mm/h", sim->maxRain);
                    igText("Peak cloud    %.2f g/kg", sim->maxCloud * 1000.0f);
                    igText("Steps/frame   %d", sim->solverStepsLastFrame);
                    igText("Backlog       %.1f s", sim->simAccumulatorSeconds);
                    igText("Solver CPU    %.2f ms", sim->solverCpuSecondsLastFrame * 1000.0);
                    igText("Div pre/post  %.3g / %.3g", sim->maxDivBeforeProjection, sim->maxDivAfterProjection);
                    igText("Water drift   %.3e kg", sim->waterMassDrift);
                    igText("Rain sink     %.3e kg", sim->precipitatedWaterMass);
                    igText("State fixes   %d nf / %d neg", sim->nonFiniteCorrections, sim->negativeWaterCorrections);

                    igSeparatorText("Controls");
                    igPushTextWrapPos(0.0f);
                    igTextWrapped("The panel captures the mouse while you hover or drag inside it, so camera orbit and zoom stay locked out during UI interaction.");
                    igPopTextWrapPos();
                    igBulletText("Left mouse drag: orbit");
                    igBulletText("Mouse wheel: zoom");
                    igBulletText("Space: pause");
                    igBulletText("R: reset atmosphere");
                    igBulletText("Tab: cycle field");
                    igBulletText("H / J: toggle slices");
                    igBulletText("T: toggle terrain");
                    igBulletText("G: hide or show panel");
                    igEndTabItem();
                }

                if (igBeginTabItem("Terrain", NULL, ImGuiTabItemFlags_None)) {
                    igTextDisabled("Generation");
                    DrawLabeledSliderInt("Seed", "##TerrainSeed", &sim->terrainGen.seed, 1, 999999, "%d");
                    if (igButton("Randomize Seed", ImVec2Make(140.0f, 0.0f))) {
                        sim->terrainGen.seed = GetRandomValue(1, 999999);
                    }
                    igSameLine(0.0f, 8.0f);
                    if (igButton("Generate Terrain", ImVec2Make(150.0f, 0.0f))) {
                        terrainChanged = RebuildGeneratedTerrain(sim);
                    }

                    igSeparatorText("Noise");
                    DrawLabeledSliderFloat("Base frequency", "##TerrainFreq", &sim->terrainGen.frequency, 0.0010f, 0.0200f, "%.4f");
                    DrawLabeledSliderFloat("Amplitude", "##TerrainAmp", &sim->terrainGen.amplitude, 0.4f, 2.2f, "%.2f");
                    DrawLabeledSliderFloat("Height bias", "##TerrainPower", &sim->terrainGen.heightPower, 0.8f, 2.8f, "%.2f");
                    DrawLabeledSliderInt("Octaves", "##TerrainOctaves", &sim->terrainGen.octaves, 1, 6, "%d");
                    DrawLabeledSliderFloat("Lacunarity", "##TerrainLacunarity", &sim->terrainGen.lacunarity, 1.4f, 2.4f, "%.2f");
                    DrawLabeledSliderFloat("Persistence", "##TerrainPersistence", &sim->terrainGen.persistence, 0.35f, 0.72f, "%.2f");

                    igSeparatorText("Erosion");
                    DrawLabeledSliderInt("Droplets", "##TerrainDroplets", &sim->terrainGen.droplets, 10000, 1200000, "%d");
                    DrawLabeledSliderInt("Lifetime", "##TerrainLifetime", &sim->terrainGen.lifetime, 4, 48, "%d");
                    DrawLabeledSliderFloat("Inertia", "##TerrainInertia", &sim->terrainGen.inertia, 0.0f, 0.30f, "%.3f");
                    DrawLabeledSliderFloat("Capacity", "##TerrainCapacity", &sim->terrainGen.capacity, 0.5f, 6.0f, "%.2f");
                    DrawLabeledSliderFloat("Min slope", "##TerrainMinSlope", &sim->terrainGen.minSlope, 0.0001f, 0.0200f, "%.4f");
                    DrawLabeledSliderFloat("Erode rate", "##TerrainErode", &sim->terrainGen.erodeRate, 0.005f, 0.120f, "%.3f");
                    DrawLabeledSliderFloat("Deposit rate", "##TerrainDeposit", &sim->terrainGen.depositRate, 0.005f, 0.120f, "%.3f");
                    DrawLabeledSliderFloat("Evaporation", "##TerrainEvaporate", &sim->terrainGen.evaporate, 0.02f, 0.25f, "%.2f");
                    DrawLabeledSliderFloat("Max step", "##TerrainMaxStep", &sim->terrainGen.maxStep, 0.10f, 1.50f, "%.2f");
                    DrawLabeledSliderInt("Smooth passes", "##TerrainSmooth", &sim->terrainGen.smoothPasses, 0, 12, "%d");

                    igSeparatorText("Look");
                    DrawLabeledSliderFloat("Rock threshold", "##TerrainRockThreshold", &sim->terrainGen.rockThreshold, 0.20f, 0.75f, "%.2f");

                    igPushTextWrapPos(0.0f);
                    igTextWrapped("Generate Terrain rebuilds the mountain mesh and re-initializes the atmospheric state over the new terrain columns.");
                    igPopTextWrapPos();
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

static void InitializeSimDimensions(AtmosphereSim *sim) {
    sim->nx = ATM_W;
    sim->nz = ATM_Z;
    sim->ny = ATM_Y;
    sim->worldWidth = (float)(TERRAIN_GEN_SIZE - 1);
    sim->worldDepth = (float)(TERRAIN_GEN_SIZE - 1);
    sim->topMeters = TERRAIN_GEN_HEIGHT_WORLD * OBJ_VERTICAL_METERS + 2400.0f;
    sim->topWorldY = WorldYFromMeters(sim->topMeters);
    sim->dxWorld = sim->worldWidth / (float)(sim->nx - 1);
    sim->dzWorld = sim->worldDepth / (float)(sim->nz - 1);
    sim->dxMeters = sim->dxWorld * OBJ_HORIZONTAL_METERS;
    sim->dzMeters = sim->dzWorld * OBJ_HORIZONTAL_METERS;
    sim->dyMeters = sim->topMeters / (float)sim->ny;
}

static bool RebuildGeneratedTerrain(AtmosphereSim *sim) {
    bool preserveViewSettings = (sim->terrain.width > 0 && sim->terrain.height > 0);
    SimViewSettings viewSettings = { 0 };
    if (preserveViewSettings) {
        viewSettings = CaptureSimViewSettings(sim);
    }

    if (sim->terrainModel.meshCount > 0) {
        UnloadModel(sim->terrainModel);
        sim->terrainModel = (Model) { 0 };
    }
    FreeTerrain(&sim->terrain);

    if (!GenerateProceduralTerrain(&sim->terrain, &sim->terrainGen)) {
        return false;
    }

    sim->terrainModel = BuildTerrainModel(&sim->terrain, &sim->terrainGen);
    if (sim->terrainModel.meshCount == 0) {
        FreeTerrain(&sim->terrain);
        return false;
    }

    BuildAtmosColumns(sim);
    ResetAtmosphere(sim);
    if (preserveViewSettings) {
        RestoreSimViewSettings(sim, &viewSettings);
    }
    UpdateStats(sim);
    return true;
}

bool CreateSimFromGeneratedTerrain(AtmosphereSim *sim) {
    InitializeSimDimensions(sim);
    DefaultTerrainGenSettings(&sim->terrainGen, GetRandomValue(1, 999999));

    if (!AllocateSim(sim)) {
        DestroySim(sim);
        return false;
    }

    return RebuildGeneratedTerrain(sim);
}
