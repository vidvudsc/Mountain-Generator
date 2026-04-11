#ifndef SIM_H
#define SIM_H

#include "raylib.h"
#include "raymath.h"
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#define TAU 6.28318530717958647692f
#define OBJ_HORIZONTAL_METERS 120.0f
#define OBJ_VERTICAL_METERS 35.0f
#define DAY_SECONDS 86400.0f

#define ATM_W 64
#define ATM_Z 64
#define ATM_Y 48

#define DEFAULT_TIME_SCALE 240.0f
#define SIM_FIXED_DT 0.5f
#define SIM_MAX_CATCHUP 8.0f
#define SIM_FRAME_BUDGET_SEC 0.006
#define SLICE_UPDATE_RATE_HZ 20.0
#define SLICE_TEX_SCALE 4
#define PROJECTION_ITERATIONS 28

#define REF_PRESSURE_HPA 1000.0f
#define DRY_AIR_KAPPA 0.286f
#define DRY_AIR_GAS_CONSTANT 287.05f
#define SPECIFIC_HEAT_AIR 1004.0f
#define LATENT_HEAT_VAPORIZATION 2500000.0f
#define GRAVITY_ACCEL 9.81f
#define TOP_SPONGE_FRACTION 0.18f
#define EDGE_RELAX_BAND 4
#define AUTOCONVERSION_THRESHOLD 0.00045f
#define AUTOCONVERSION_RATE 0.0012f
#define ACCRETION_RATE 2.2f
#define RAIN_EVAP_RATE 0.35f
#define CLOUD_DIFFUSION 0.10f
#define VELOCITY_BLEND 0.10f

#define HSLICE_ARROW_STEP 8
#define VSLICE_ARROW_STEP 4

#define TERRAIN_GEN_SIZE 256
#define TERRAIN_GEN_HEIGHT_WORLD 80.0f

typedef enum FieldMode {
    FIELD_TEMPERATURE = 0,
    FIELD_PRESSURE,
    FIELD_REL_HUMIDITY,
    FIELD_DEWPOINT,
    FIELD_VAPOR,
    FIELD_CLOUD,
    FIELD_RAIN,
    FIELD_WIND,
    FIELD_VERTICAL_WIND,
    FIELD_BUOYANCY,
    FIELD_COUNT
} FieldMode;

typedef enum SliceAxis {
    SLICE_X = 0,
    SLICE_Z
} SliceAxis;

typedef struct TerrainGrid {
    int width;
    int height;
    float minTerrainY;
    float maxTerrainY;
    float minTerrainMeters;
    float maxTerrainMeters;
    float *terrainY;
    float *terrainMeters;
    float *terrainDx;
    float *terrainDz;
    float *terrainSlope;
} TerrainGrid;

typedef struct TerrainGenSettings {
    int seed;
    float frequency;
    float amplitude;
    float heightPower;
    int octaves;
    float lacunarity;
    float persistence;
    int droplets;
    int lifetime;
    float inertia;
    float capacity;
    float minSlope;
    float erodeRate;
    float depositRate;
    float evaporate;
    float maxStep;
    int smoothPasses;
    float rockThreshold;
} TerrainGenSettings;

typedef struct AtmosphereSim {
    TerrainGrid terrain;
    Model terrainModel;

    int nx;
    int nz;
    int ny;
    int cells2D;
    int cells3D;
    int cellsU;
    int cellsV;
    int cellsW;

    float worldWidth;
    float worldDepth;
    float topMeters;
    float topWorldY;
    float dxWorld;
    float dzWorld;
    float dxMeters;
    float dzMeters;
    float dyMeters;

    float *columnTerrainMeters;
    float *columnTerrainWorldY;
    float *columnDx;
    float *columnDz;
    float *columnSlope;
    float *surfaceTemp;
    float *surfaceWetness;
    float *surfaceRain;
    int *groundLayer;

    float *basePressure;
    float *baseTemp;
    float *baseTheta;
    float *baseDensity;
    float *baseQv;

    float *theta;
    float *qv;
    float *qc;
    float *qr;
    float *pressurePert;

    float *uFace;
    float *vFace;
    float *wFace;

    float *temp;
    float *pressure;
    float *buoyancy;
    float *divergence;
    float *pressureScratch;
    float *surfaceRainScratch;

    float *u;
    float *v;
    float *w;

    float *thetaNext;
    float *qvNext;
    float *qcNext;
    float *qrNext;
    float *uFaceNext;
    float *vFaceNext;
    float *wFaceNext;

    Texture2D horizontalTex;
    Texture2D verticalTex;
    Color *horizontalPixels;
    Color *verticalPixels;
    Model horizontalSliceModel;
    Model verticalSliceModelX;
    Model verticalSliceModelZ;

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
    float simAccumulatorSeconds;
    int solverStepsLastFrame;
    double solverCpuSecondsLastFrame;
    bool sliceDirty;
    double lastSliceUpdateTime;

    float simSeconds;
    float prevailingU;
    float prevailingV;
    Vector3 sunDir;
    float solarStrength;

    float meanSurfaceTemp;
    float meanPressure;
    float maxCloud;
    float maxRain;
    float maxWind;
    float maxDivBeforeProjection;
    float maxDivAfterProjection;
    int nonFiniteCorrections;
    int negativeWaterCorrections;
    double atmosphericWaterMass;
    double precipitatedWaterMass;
    double totalWaterMass;
    double initialWaterMass;
    double waterMassDrift;
    float fieldDisplayMin[FIELD_COUNT];
    float fieldDisplayMax[FIELD_COUNT];

    TerrainGenSettings terrainGen;
} AtmosphereSim;

static inline float Fract(float value) {
    return value - floorf(value);
}

static inline float NoiseCoord(float x, float y, float z) {
    return Fract(sinf(x * 12.9898f + y * 78.233f + z * 37.719f) * 43758.5453f);
}

static inline float NormalizeRange(float value, float minValue, float maxValue) {
    if (fabsf(maxValue - minValue) <= 0.00001f) return 0.0f;
    return Clamp((value - minValue) / (maxValue - minValue), 0.0f, 1.0f);
}

static inline Color LerpColor(Color a, Color b, float t) {
    t = Clamp(t, 0.0f, 1.0f);
    return (Color) {
        (unsigned char)Lerp((float)a.r, (float)b.r, t),
        (unsigned char)Lerp((float)a.g, (float)b.g, t),
        (unsigned char)Lerp((float)a.b, (float)b.b, t),
        (unsigned char)Lerp((float)a.a, (float)b.a, t)
    };
}

static inline Color WithAlpha(Color color, unsigned char alpha) {
    color.a = alpha;
    return color;
}

static inline Color RampColor(float t, const Color *stops, int stopCount) {
    t = Clamp(t, 0.0f, 1.0f);
    if (stopCount <= 1) return stops[0];

    float scaled = t * (float)(stopCount - 1);
    int index = (int)floorf(scaled);
    if (index >= stopCount - 1) return stops[stopCount - 1];

    float localT = scaled - (float)index;
    return LerpColor(stops[index], stops[index + 1], localT);
}

static inline int TerrainIndex(const TerrainGrid *terrain, int x, int z) {
    return z * terrain->width + x;
}

static inline int ColumnIndex(const AtmosphereSim *sim, int x, int z) {
    return z * sim->nx + x;
}

static inline int AtmosIndex(const AtmosphereSim *sim, int x, int y, int z) {
    return ((y * sim->nz) + z) * sim->nx + x;
}

static inline int UIndex(const AtmosphereSim *sim, int x, int y, int z) {
    return ((y * sim->nz) + z) * (sim->nx + 1) + x;
}

static inline int VIndex(const AtmosphereSim *sim, int x, int y, int z) {
    return ((y * (sim->nz + 1)) + z) * sim->nx + x;
}

static inline int WIndex(const AtmosphereSim *sim, int x, int y, int z) {
    return ((y * sim->nz) + z) * sim->nx + x;
}

static inline void SwapFields(float **a, float **b) {
    float *tmp = *a;
    *a = *b;
    *b = tmp;
}

static inline float BilinearSample(const float *field, int width, int height, float x, float z) {
    x = Clamp(x, 0.0f, (float)(width - 1));
    z = Clamp(z, 0.0f, (float)(height - 1));

    int x0 = (int)floorf(x);
    int z0 = (int)floorf(z);
    int x1 = (x0 + 1 < width) ? x0 + 1 : x0;
    int z1 = (z0 + 1 < height) ? z0 + 1 : z0;
    float tx = x - (float)x0;
    float tz = z - (float)z0;

    float v00 = field[z0 * width + x0];
    float v10 = field[z0 * width + x1];
    float v01 = field[z1 * width + x0];
    float v11 = field[z1 * width + x1];

    return Lerp(Lerp(v00, v10, tx), Lerp(v01, v11, tx), tz);
}

static inline bool IsFluidCell(const AtmosphereSim *sim, int x, int y, int z) {
    int column = ColumnIndex(sim, x, z);
    return y > sim->groundLayer[column];
}

static inline int GroundLayerForAltitude(const AtmosphereSim *sim, float terrainMeters) {
    int layer = (int)floorf((terrainMeters - 0.5f * sim->dyMeters) / sim->dyMeters);
    if (layer < -1) layer = -1;
    if (layer >= sim->ny) layer = sim->ny - 1;
    return layer;
}

static inline float CellAltitudeMeters(const AtmosphereSim *sim, int y) {
    return ((float)y + 0.5f) * sim->dyMeters;
}

static inline float FaceAltitudeMeters(const AtmosphereSim *sim, int y) {
    return (float)y * sim->dyMeters;
}

static inline float WorldYFromMeters(float meters) {
    return meters / OBJ_VERTICAL_METERS;
}

static inline float TrilinearSample3D(const float *field, int width, int height, int depth, float x, float y, float z) {
    x = Clamp(x, 0.0f, (float)(width - 1));
    y = Clamp(y, 0.0f, (float)(height - 1));
    z = Clamp(z, 0.0f, (float)(depth - 1));

    int x0 = (int)floorf(x);
    int y0 = (int)floorf(y);
    int z0 = (int)floorf(z);
    int x1 = (x0 + 1 < width) ? x0 + 1 : x0;
    int y1 = (y0 + 1 < height) ? y0 + 1 : y0;
    int z1 = (z0 + 1 < depth) ? z0 + 1 : z0;

    float tx = x - (float)x0;
    float ty = y - (float)y0;
    float tz = z - (float)z0;

    int yz00 = y0 * depth + z0;
    int yz01 = y0 * depth + z1;
    int yz10 = y1 * depth + z0;
    int yz11 = y1 * depth + z1;

    float c000 = field[yz00 * width + x0];
    float c100 = field[yz00 * width + x1];
    float c010 = field[yz10 * width + x0];
    float c110 = field[yz10 * width + x1];
    float c001 = field[yz01 * width + x0];
    float c101 = field[yz01 * width + x1];
    float c011 = field[yz11 * width + x0];
    float c111 = field[yz11 * width + x1];

    float c00 = Lerp(c000, c100, tx);
    float c10 = Lerp(c010, c110, tx);
    float c01 = Lerp(c001, c101, tx);
    float c11 = Lerp(c011, c111, tx);
    float c0 = Lerp(c00, c10, ty);
    float c1 = Lerp(c01, c11, ty);
    return Lerp(c0, c1, tz);
}

static inline float SampleAtmosField(const AtmosphereSim *sim, const float *field, float x, float y, float z) {
    x = Clamp(x, 0.0f, (float)(sim->nx - 1));
    y = Clamp(y, 0.0f, (float)(sim->ny - 1));
    z = Clamp(z, 0.0f, (float)(sim->nz - 1));

    int x0 = (int)floorf(x);
    int y0 = (int)floorf(y);
    int z0 = (int)floorf(z);
    int x1 = (x0 + 1 < sim->nx) ? x0 + 1 : x0;
    int y1 = (y0 + 1 < sim->ny) ? y0 + 1 : y0;
    int z1 = (z0 + 1 < sim->nz) ? z0 + 1 : z0;

    float tx = x - (float)x0;
    float ty = y - (float)y0;
    float tz = z - (float)z0;

    float sum = 0.0f;
    float totalWeight = 0.0f;

    int cornerX[8] = { x0, x1, x0, x1, x0, x1, x0, x1 };
    int cornerY[8] = { y0, y0, y1, y1, y0, y0, y1, y1 };
    int cornerZ[8] = { z0, z0, z0, z0, z1, z1, z1, z1 };
    float cornerWeight[8] = {
        (1.0f - tx) * (1.0f - ty) * (1.0f - tz),
        tx * (1.0f - ty) * (1.0f - tz),
        (1.0f - tx) * ty * (1.0f - tz),
        tx * ty * (1.0f - tz),
        (1.0f - tx) * (1.0f - ty) * tz,
        tx * (1.0f - ty) * tz,
        (1.0f - tx) * ty * tz,
        tx * ty * tz
    };

    for (int corner = 0; corner < 8; corner++) {
        int cx = cornerX[corner];
        int cy = cornerY[corner];
        int cz = cornerZ[corner];
        if (!IsFluidCell(sim, cx, cy, cz)) continue;

        float weight = cornerWeight[corner];
        sum += field[AtmosIndex(sim, cx, cy, cz)] * weight;
        totalWeight += weight;
    }

    if (totalWeight > 0.000001f) {
        return sum / totalWeight;
    }

    int nearestX = Clamp((int)lroundf(x), 0, sim->nx - 1);
    int nearestY = Clamp((int)lroundf(y), 0, sim->ny - 1);
    int nearestZ = Clamp((int)lroundf(z), 0, sim->nz - 1);
    float bestDist2 = FLT_MAX;
    int bestIndex = -1;

    for (int oz = -1; oz <= 1; oz++) {
        for (int oy = -1; oy <= 1; oy++) {
            for (int ox = -1; ox <= 1; ox++) {
                int sx = Clamp(nearestX + ox, 0, sim->nx - 1);
                int sy = Clamp(nearestY + oy, 0, sim->ny - 1);
                int sz = Clamp(nearestZ + oz, 0, sim->nz - 1);
                if (!IsFluidCell(sim, sx, sy, sz)) continue;

                float dx = (float)sx - x;
                float dy = (float)sy - y;
                float dz = (float)sz - z;
                float dist2 = dx * dx + dy * dy + dz * dz;
                if (dist2 < bestDist2) {
                    bestDist2 = dist2;
                    bestIndex = AtmosIndex(sim, sx, sy, sz);
                }
            }
        }
    }

    return (bestIndex >= 0) ? field[bestIndex] : 0.0f;
}

static inline float SampleUFaceField(const AtmosphereSim *sim, const float *field, float x, float y, float z) {
    return TrilinearSample3D(field, sim->nx + 1, sim->ny, sim->nz, x + 0.5f, y, z);
}

static inline float SampleVFaceField(const AtmosphereSim *sim, const float *field, float x, float y, float z) {
    return TrilinearSample3D(field, sim->nx, sim->ny, sim->nz + 1, x, y, z + 0.5f);
}

static inline float SampleWFaceField(const AtmosphereSim *sim, const float *field, float x, float y, float z) {
    return TrilinearSample3D(field, sim->nx, sim->ny + 1, sim->nz, x, y + 0.5f, z);
}

static inline void SampleVelocityField(const AtmosphereSim *sim, float x, float y, float z, float *outU, float *outV, float *outW) {
    if (outU != NULL) *outU = SampleAtmosField(sim, sim->u, x, y, z);
    if (outV != NULL) *outV = SampleAtmosField(sim, sim->v, x, y, z);
    if (outW != NULL) *outW = SampleAtmosField(sim, sim->w, x, y, z);
}

#endif
