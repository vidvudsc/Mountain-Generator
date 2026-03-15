#include "raylib.h"
#include "raymath.h"
#include "rlgl.h"
#define CIMGUI_DEFINE_ENUMS_AND_STRUCTS
#include "cimgui.h"
#include "rlImGui.h"
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define TAU 6.28318530717958647692f
#define OBJ_HORIZONTAL_METERS 120.0f
#define OBJ_VERTICAL_METERS 35.0f
#define DAY_SECONDS 86400.0f

#define ATM_W 64
#define ATM_Z 64
#define ATM_Y 16

#define DEFAULT_TIME_SCALE 100.0f
#define MAX_STEP_SECONDS 6.0f
#define SLICE_TEX_SCALE 4

#define HSLICE_ARROW_STEP 8
#define VSLICE_ARROW_STEP 4
#define WIND_ARROW_SCALE 0.42f
#define WIND_ARROW_SHAFT_RADIUS 0.10f
#define WIND_ARROW_HEAD_RADIUS 0.24f
#define WIND_ARROW_HEAD_LENGTH 0.95f

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
    float *surfaceRain;
    int *groundLayer;

    float *temp;
    float *pressure;
    float *vapor;
    float *cloud;
    float *rain;
    float *u;
    float *v;
    float *w;
    float *buoyancy;
    float *sunlight;
    float *divergence;
    float *pressureSolve;
    float *pressureScratch;

    float *tempNext;
    float *pressureNext;
    float *vaporNext;
    float *cloudNext;
    float *rainNext;
    float *uNext;
    float *vNext;
    float *wNext;

    Texture2D horizontalTex;
    Texture2D verticalTex;
    Texture2D volumeFieldTex;
    Texture2D cloudTerrainTex;
    Texture2D terrainTexDirt;
    Texture2D terrainTexCoast;
    Texture2D terrainTexRock;
    Texture2D terrainTexSnow;
    Color *horizontalPixels;
    Color *verticalPixels;
    Vector4 *volumeFieldPixels;
    Model horizontalSliceModel;
    Model verticalSliceModelX;
    Model verticalSliceModelZ;
    Model cloudVolumeModel;
    Shader cloudVolumeShader;
    Shader skyShader;
    Shader terrainShader;
    int cloudVolumeLocMin;
    int cloudVolumeLocMax;
    int cloudVolumeLocCamera;
    int cloudVolumeLocRes;
    int cloudVolumeLocAtlas;
    int cloudVolumeLocParams;
    int cloudVolumeLocSun;
    int skyLocSunDir;
    int skyLocCamRight;
    int skyLocCamUp;
    int skyLocCamForward;
    int skyLocAspect;
    int skyLocTanHalfFov;
    int skyLocSunStrength;
    int terrainLocTexDirt;
    int terrainLocTexCoast;
    int terrainLocTexRock;
    int terrainLocMinY;
    int terrainLocMaxY;
    int terrainLocRockThreshold;
    int terrainLocTexScale;
    int terrainLocMacroScale;
    int terrainLocSunDir;

    FieldMode fieldMode;
    SliceAxis verticalAxis;
    int horizontalLayer;
    int verticalSlice;
    int volumeAtlasCols;
    int volumeAtlasRows;
    bool showHorizontalSlice;
    bool showVerticalSlice;
    bool showTerrain;
    bool showWireframe;
    bool showWindArrows;
    bool showClouds;
    bool showVolume;
    bool paused;
    bool showUI;
    float terrainAlpha;
    float cloudRenderAlpha;
    float cloudRenderThreshold;
    float cloudDensityScale;
    float timeScale;
    float volumeThreshold;
    float volumeAlpha;
    int volumeStride;

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
    float fieldDisplayMin[FIELD_COUNT];
    float fieldDisplayMax[FIELD_COUNT];

    TerrainGenSettings terrainGen;
    int activeControlTab;
    unsigned int frameCounter;
} AtmosphereSim;

static bool RebuildGeneratedTerrain(AtmosphereSim *sim);
static void UpdateVolumeFieldTexture(AtmosphereSim *sim);

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
    bool showClouds;
    bool showVolume;
    bool paused;
    bool showUI;
    float terrainAlpha;
    float cloudRenderAlpha;
    float cloudRenderThreshold;
    float cloudDensityScale;
    float timeScale;
    float volumeThreshold;
    float volumeAlpha;
    int volumeStride;
    int activeControlTab;
} SimViewSettings;

static void FieldRange(FieldMode mode, float *outMin, float *outMax);
static float FieldDisplayValue(const AtmosphereSim *sim, int x, int y, int z, FieldMode mode);

static float Fract(float value) {
    return value - floorf(value);
}

static float NoiseCoord(float x, float y, float z) {
    return Fract(sinf(x * 12.9898f + y * 78.233f + z * 37.719f) * 43758.5453f);
}

static float NormalizeRange(float value, float minValue, float maxValue) {
    if (fabsf(maxValue - minValue) <= 0.00001f) return 0.0f;
    return Clamp((value - minValue) / (maxValue - minValue), 0.0f, 1.0f);
}

static Color LerpColor(Color a, Color b, float t) {
    t = Clamp(t, 0.0f, 1.0f);
    return (Color) {
        (unsigned char)Lerp((float)a.r, (float)b.r, t),
        (unsigned char)Lerp((float)a.g, (float)b.g, t),
        (unsigned char)Lerp((float)a.b, (float)b.b, t),
        (unsigned char)Lerp((float)a.a, (float)b.a, t)
    };
}

static Color WithAlpha(Color color, unsigned char alpha) {
    color.a = alpha;
    return color;
}

static Color HSVToColor(float hue, float saturation, float value) {
    float r = 0.0f;
    float g = 0.0f;
    float b = 0.0f;

    hue = fmodf(hue, 360.0f);
    if (hue < 0.0f) hue += 360.0f;
    saturation = Clamp(saturation, 0.0f, 1.0f);
    value = Clamp(value, 0.0f, 1.0f);

    if (saturation <= 0.0001f) {
        unsigned char v = (unsigned char)(value * 255.0f);
        return (Color) { v, v, v, 255 };
    }

    float sector = hue / 60.0f;
    int sectorIndex = (int)floorf(sector);
    float fraction = sector - (float)sectorIndex;
    float p = value * (1.0f - saturation);
    float q = value * (1.0f - saturation * fraction);
    float t = value * (1.0f - saturation * (1.0f - fraction));

    switch (sectorIndex) {
        case 0: r = value; g = t; b = p; break;
        case 1: r = q; g = value; b = p; break;
        case 2: r = p; g = value; b = t; break;
        case 3: r = p; g = q; b = value; break;
        case 4: r = t; g = p; b = value; break;
        default: r = value; g = p; b = q; break;
    }

    return (Color) {
        (unsigned char)(255.0f * r),
        (unsigned char)(255.0f * g),
        (unsigned char)(255.0f * b),
        255
    };
}

static Color RampColor(float t, const Color *stops, int stopCount) {
    t = Clamp(t, 0.0f, 1.0f);
    if (stopCount <= 1) return stops[0];

    float scaled = t * (float)(stopCount - 1);
    int index = (int)floorf(scaled);
    if (index >= stopCount - 1) return stops[stopCount - 1];

    float localT = scaled - (float)index;
    return LerpColor(stops[index], stops[index + 1], localT);
}

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

static const Color kPressureStops[] = {
    { 18, 33, 66, 255 },
    { 52, 92, 153, 255 },
    { 125, 167, 193, 255 },
    { 219, 225, 213, 255 },
    { 239, 199, 129, 255 },
    { 168, 92, 40, 255 }
};

static const Color kDewPointStops[] = {
    { 81, 44, 18, 255 },
    { 135, 92, 42, 255 },
    { 188, 154, 75, 255 },
    { 116, 180, 110, 255 },
    { 62, 146, 144, 255 },
    { 193, 239, 226, 255 }
};

static const Color kVaporStops[] = {
    { 18, 45, 56, 255 },
    { 24, 96, 128, 255 },
    { 49, 149, 175, 255 },
    { 98, 202, 204, 255 },
    { 196, 243, 230, 255 }
};

static const Color kRainStops[] = {
    { 12, 22, 52, 255 },
    { 24, 61, 140, 255 },
    { 57, 118, 214, 255 },
    { 110, 180, 240, 255 },
    { 202, 238, 255, 255 }
};

static const Color kWindStops[] = {
    { 17, 44, 73, 255 },
    { 26, 122, 129, 255 },
    { 85, 192, 124, 255 },
    { 230, 215, 91, 255 },
    { 233, 127, 44, 255 },
    { 192, 49, 43, 255 }
};

static const Color kDivergingStops[] = {
    { 190, 82, 40, 255 },
    { 239, 206, 118, 255 },
    { 237, 241, 244, 255 },
    { 96, 195, 227, 255 },
    { 35, 79, 196, 255 }
};

static Texture2D CreateCloudBillboardTexture(void) {
    const int size = 128;
    Color *pixels = (Color *)MemAlloc((size_t)size * (size_t)size * sizeof(Color));
    if (pixels == NULL) return (Texture2D) { 0 };

    for (int y = 0; y < size; y++) {
        for (int x = 0; x < size; x++) {
            float u = ((float)x + 0.5f) / (float)size * 2.0f - 1.0f;
            float v = ((float)y + 0.5f) / (float)size * 2.0f - 1.0f;
            float radius = sqrtf(u * u + v * v);
            float falloff = Clamp(1.0f - radius, 0.0f, 1.0f);
            float plume = powf(falloff, 1.75f);
            float noise = 0.0f;
            noise += (NoiseCoord((float)x * 0.095f, (float)y * 0.095f, 2.0f) - 0.5f) * 0.34f;
            noise += (NoiseCoord((float)x * 0.19f, (float)y * 0.19f, 5.0f) - 0.5f) * 0.18f;
            float alpha = Clamp(plume + noise, 0.0f, 1.0f);
            float mask = Clamp(falloff / 0.95f, 0.0f, 1.0f);
            mask = mask * mask * (3.0f - 2.0f * mask);
            alpha *= mask;
            unsigned char a = (unsigned char)(255.0f * Clamp(alpha * 1.15f, 0.0f, 1.0f));

            Color c = {
                (unsigned char)(230 + 18 * plume),
                (unsigned char)(236 + 12 * plume),
                (unsigned char)(244 + 6 * plume),
                a
            };
            pixels[y * size + x] = c;
        }
    }

    Image image = {
        .data = pixels,
        .width = size,
        .height = size,
        .mipmaps = 1,
        .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
    };

    Texture2D texture = LoadTextureFromImage(image);
    UnloadImage(image);

    if (texture.id != 0) {
        SetTextureFilter(texture, TEXTURE_FILTER_BILINEAR);
        SetTextureWrap(texture, TEXTURE_WRAP_CLAMP);
    }

    return texture;
}

static const char *kCloudVolumeVs =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "uniform mat4 mvp;\n"
    "uniform mat4 matModel;\n"
    "out vec3 fragWorldPos;\n"
    "void main() {\n"
    "    vec4 worldPos = matModel * vec4(vertexPosition, 1.0);\n"
    "    fragWorldPos = worldPos.xyz;\n"
    "    gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
    "}\n";

static const char *kCloudVolumeFs =
    "#version 330\n"
    "in vec3 fragWorldPos;\n"
    "out vec4 finalColor;\n"
    "uniform sampler2D texture0;\n"
    "uniform vec3 uVolumeMin;\n"
    "uniform vec3 uVolumeMax;\n"
    "uniform vec3 uCameraPos;\n"
    "uniform vec3 uVolumeRes;\n"
    "uniform vec2 uAtlasGrid;\n"
    "uniform vec4 uCloudParams;\n"
    "uniform vec3 uSunDir;\n"
    "uniform sampler2D texture1;\n"
    "\n"
    "float hash13(vec3 p) {\n"
    "    p = fract(p * 0.1031);\n"
    "    p += dot(p, p.zyx + 31.32);\n"
    "    return fract((p.x + p.y) * p.z);\n"
    "}\n"
    "\n"
    "float noise3dCustom(vec3 p) {\n"
    "    vec3 i = floor(p);\n"
    "    vec3 f = fract(p);\n"
    "    f = f * f * (3.0 - 2.0 * f);\n"
    "\n"
    "    float n000 = hash13(i + vec3(0.0, 0.0, 0.0));\n"
    "    float n100 = hash13(i + vec3(1.0, 0.0, 0.0));\n"
    "    float n010 = hash13(i + vec3(0.0, 1.0, 0.0));\n"
    "    float n110 = hash13(i + vec3(1.0, 1.0, 0.0));\n"
    "    float n001 = hash13(i + vec3(0.0, 0.0, 1.0));\n"
    "    float n101 = hash13(i + vec3(1.0, 0.0, 1.0));\n"
    "    float n011 = hash13(i + vec3(0.0, 1.0, 1.0));\n"
    "    float n111 = hash13(i + vec3(1.0, 1.0, 1.0));\n"
    "\n"
    "    float nx00 = mix(n000, n100, f.x);\n"
    "    float nx10 = mix(n010, n110, f.x);\n"
    "    float nx01 = mix(n001, n101, f.x);\n"
    "    float nx11 = mix(n011, n111, f.x);\n"
    "    float nxy0 = mix(nx00, nx10, f.y);\n"
    "    float nxy1 = mix(nx01, nx11, f.y);\n"
    "    return mix(nxy0, nxy1, f.z);\n"
    "}\n"
    "\n"
    "float fbm(vec3 p) {\n"
    "    float value = 0.0;\n"
    "    float amp = 0.55;\n"
    "    for (int i = 0; i < 3; i++) {\n"
    "        value += noise3dCustom(p) * amp;\n"
    "        p = p * 2.03 + vec3(13.1, 7.7, 5.3);\n"
    "        amp *= 0.5;\n"
    "    }\n"
    "    return value;\n"
    "}\n"
    "\n"
    "vec4 sampleLayer(float layer, vec2 uvXZ) {\n"
    "    float cols = uAtlasGrid.x;\n"
    "    float rows = uAtlasGrid.y;\n"
    "    float tileX = mod(layer, cols);\n"
    "    float tileY = floor(layer / cols);\n"
    "    vec2 atlasUV = (vec2(tileX, tileY) + clamp(uvXZ, vec2(0.001), vec2(0.999))) / vec2(cols, rows);\n"
    "    return texture(texture0, atlasUV);\n"
    "}\n"
    "\n"
    "vec4 sampleVolume(vec3 worldPos) {\n"
    "    vec3 size = uVolumeMax - uVolumeMin;\n"
    "    vec3 uvw = (worldPos - uVolumeMin) / size;\n"
    "    if (any(lessThan(uvw, vec3(0.0))) || any(greaterThan(uvw, vec3(1.0)))) return vec4(0.0);\n"
    "    float y = uvw.y * max(uVolumeRes.z - 1.0, 1.0);\n"
    "    float y0 = floor(y);\n"
    "    float y1 = min(y0 + 1.0, uVolumeRes.z - 1.0);\n"
    "    float fy = fract(y);\n"
    "    fy = fy * fy * (3.0 - 2.0 * fy);\n"
    "    vec4 a = sampleLayer(y0, uvw.xz);\n"
    "    vec4 b = sampleLayer(y1, uvw.xz);\n"
    "    return mix(a, b, fy);\n"
    "}\n"
    "\n"
    "vec3 volumeUVW(vec3 worldPos) {\n"
    "    return (worldPos - uVolumeMin) / (uVolumeMax - uVolumeMin);\n"
    "}\n"
    "\n"
    "float terrainHeightAt(vec2 uvXZ) {\n"
    "    return texture(texture1, clamp(uvXZ, vec2(0.001), vec2(0.999))).r * uVolumeMax.y;\n"
    "}\n"
    "\n"
    "bool intersectBox(vec3 ro, vec3 rd, out float tNear, out float tFar) {\n"
    "    vec3 invDir = 1.0 / max(abs(rd), vec3(0.0001)) * sign(rd);\n"
    "    vec3 t0 = (uVolumeMin - ro) * invDir;\n"
    "    vec3 t1 = (uVolumeMax - ro) * invDir;\n"
    "    vec3 tsmaller = min(t0, t1);\n"
    "    vec3 tbigger = max(t0, t1);\n"
    "    tNear = max(max(tsmaller.x, tsmaller.y), tsmaller.z);\n"
    "    tFar = min(min(tbigger.x, tbigger.y), tbigger.z);\n"
    "    return tFar > max(tNear, 0.0);\n"
    "}\n"
    "\n"
    "float densityFromSample(vec4 s, vec3 worldPos) {\n"
    "    vec3 uvw = volumeUVW(worldPos);\n"
    "    float condensate = s.r;\n"
    "    float precip = s.g;\n"
    "    float macroField = condensate * 1.45 + precip * 0.30;\n"
    "    float macro = max(macroField - uCloudParams.y, 0.0);\n"
    "    macro *= smoothstep(uCloudParams.y, uCloudParams.y + 0.06, macroField);\n"
    "    if (macro <= 0.0001) return 0.0;\n"
    "\n"
    "    float terrainY = terrainHeightAt(uvw.xz);\n"
    "    float groundFade = smoothstep(terrainY + 0.20, terrainY + 1.30, worldPos.y);\n"
    "    if (groundFade <= 0.0001) return 0.0;\n"
    "\n"
    "    float verticalFade = smoothstep(0.02, 0.10, uvw.y) * (1.0 - smoothstep(0.88, 1.0, uvw.y));\n"
    "    float detail = fbm(worldPos * vec3(0.030, 0.055, 0.030));\n"
    "    float erosion = fbm(worldPos * vec3(0.075, 0.095, 0.075) + vec3(41.0, 17.0, 9.0));\n"
    "    float shaped = macro * mix(0.72, 1.28, detail);\n"
    "    shaped -= max(erosion - 0.58, 0.0) * 0.24;\n"
    "    return max(shaped * verticalFade * groundFade, 0.0);\n"
    "}\n"
    "\n"
    "float sampleShadow(vec3 pos) {\n"
    "    float tNear;\n"
    "    float tFar;\n"
    "    if (!intersectBox(pos + uSunDir * 0.02, uSunDir, tNear, tFar)) return 1.0;\n"
    "    float startT = max(tNear, 0.0);\n"
    "    float endT = max(tFar, 0.0);\n"
    "    float stepLen = max((endT - startT) / 10.0, 0.18);\n"
    "    float transmission = 1.0;\n"
    "    float t = startT;\n"
    "    for (int i = 0; i < 10; i++) {\n"
    "        if (t >= endT || transmission <= 0.04) break;\n"
        "        vec3 samplePos = pos + uSunDir * t;\n"
        "        vec4 s = sampleVolume(samplePos);\n"
        "        float density = densityFromSample(s, samplePos);\n"
        "        transmission *= exp(-density * uCloudParams.x * stepLen * 0.85);\n"
        "        t += stepLen;\n"
    "    }\n"
    "    return clamp(transmission, 0.0, 1.0);\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    vec3 rayDir = normalize(fragWorldPos - uCameraPos);\n"
    "    float tNear;\n"
    "    float tFar;\n"
    "    if (!intersectBox(uCameraPos, rayDir, tNear, tFar)) discard;\n"
    "    float startT = max(tNear, 0.0) + 0.02;\n"
    "    float endT = max(tFar, startT);\n"
    "    float stepLen = max((endT - startT) / 56.0, 0.12);\n"
    "    startT += stepLen * (hash13(fragWorldPos * 0.21 + uCameraPos * 0.07) - 0.5) * 0.38;\n"
    "    vec4 accum = vec4(0.0);\n"
    "    float t = startT;\n"
    "    for (int i = 0; i < 56; i++) {\n"
    "        if (t > endT || accum.a > 0.98) break;\n"
    "        vec3 pos = uCameraPos + rayDir * t;\n"
    "        vec4 s = sampleVolume(pos);\n"
    "        float density = densityFromSample(s, pos);\n"
    "        if (density > 0.001) {\n"
    "            float light = clamp(s.b, 0.0, 1.0);\n"
    "            float temp = clamp(s.a, 0.0, 1.0);\n"
    "            float precip = clamp(s.g, 0.0, 1.0);\n"
    "            float shadow = sampleShadow(pos);\n"
    "            float thickShade = clamp(1.0 - density * 0.45, 0.20, 1.0);\n"
    "            vec3 coolCol = vec3(0.72, 0.80, 0.92);\n"
    "            vec3 warmCol = vec3(0.97, 0.88, 0.77);\n"
    "            vec3 wetCol = vec3(0.64, 0.70, 0.80);\n"
    "            vec3 baseCol = mix(coolCol, warmCol, clamp((temp - 0.30) * 1.25, 0.0, 1.0));\n"
    "            baseCol = mix(baseCol, wetCol, clamp(precip * 1.6, 0.0, 1.0));\n"
    "            float ambient = mix(0.08, 0.34, light);\n"
    "            float direct = mix(0.16, 1.0, light) * shadow;\n"
    "            vec3 sampleCol = baseCol * (ambient + direct * thickShade);\n"
    "            sampleCol = min(sampleCol, vec3(0.98));\n"
    "            float alpha = (1.0 - exp(-density * uCloudParams.x * stepLen)) * uCloudParams.z;\n"
    "            accum.rgb += (1.0 - accum.a) * sampleCol * alpha;\n"
    "            accum.a += (1.0 - accum.a) * alpha;\n"
    "        }\n"
    "        t += stepLen;\n"
    "    }\n"
    "    if (accum.a <= 0.002) discard;\n"
    "    finalColor = accum;\n"
    "}\n";

static const char *kSkyVs =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "in vec2 vertexTexCoord;\n"
    "in vec4 vertexColor;\n"
    "uniform mat4 mvp;\n"
    "out vec2 fragTexCoord;\n"
    "out vec4 fragColor;\n"
    "void main() {\n"
    "    fragTexCoord = vertexTexCoord;\n"
    "    fragColor = vertexColor;\n"
    "    gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
    "}\n";

static const char *kSkyFs =
    "#version 330\n"
    "in vec2 fragTexCoord;\n"
    "in vec4 fragColor;\n"
    "out vec4 finalColor;\n"
    "uniform vec3 uSunDir;\n"
    "uniform vec3 uCamRight;\n"
    "uniform vec3 uCamUp;\n"
    "uniform vec3 uCamForward;\n"
    "uniform float uAspect;\n"
    "uniform float uTanHalfFov;\n"
    "uniform float uSunStrength;\n"
    "\n"
    "void main() {\n"
    "    vec2 ndc = fragTexCoord * 2.0 - 1.0;\n"
    "    ndc.y = -ndc.y;\n"
    "    vec3 viewDir = normalize(uCamForward + ndc.x * uAspect * uTanHalfFov * uCamRight + ndc.y * uTanHalfFov * uCamUp);\n"
    "    float sunDot = max(dot(viewDir, normalize(uSunDir)), 0.0);\n"
    "    float horizon = clamp(1.0 - max(viewDir.y, -0.25), 0.0, 1.0);\n"
    "    float zenith = clamp(viewDir.y * 0.5 + 0.5, 0.0, 1.0);\n"
    "    float sunHeight = clamp(uSunDir.y * 0.5 + 0.5, 0.0, 1.0);\n"
    "\n"
    "    vec3 deepBlue = vec3(0.03, 0.08, 0.18);\n"
    "    vec3 skyBlue = vec3(0.24, 0.50, 0.92);\n"
    "    vec3 horizonBlue = vec3(0.62, 0.76, 0.92);\n"
    "    vec3 dusk = vec3(0.98, 0.56, 0.26);\n"
    "    vec3 warmHaze = mix(dusk, vec3(1.0, 0.86, 0.66), sunHeight);\n"
    "\n"
    "    vec3 sky = mix(deepBlue, skyBlue, pow(zenith, 0.55));\n"
    "    sky = mix(horizonBlue, sky, pow(zenith, 1.4));\n"
    "    sky += warmHaze * pow(horizon, 3.0) * (0.18 + 0.42 * uSunStrength);\n"
    "    sky += mix(vec3(1.0, 0.55, 0.22), vec3(1.0, 0.96, 0.86), sunHeight) * pow(sunDot, 28.0) * (0.28 + 0.55 * uSunStrength);\n"
    "    sky += vec3(1.0, 0.82, 0.58) * pow(sunDot, 180.0) * (0.45 + 1.1 * uSunStrength);\n"
    "    sky = mix(sky * 0.35, sky, clamp(zenith + 0.1, 0.0, 1.0));\n"
    "    finalColor = vec4(sky, 1.0) * fragColor;\n"
    "}\n";

static const char *kTerrainVs =
    "#version 330\n"
    "in vec3 vertexPosition;\n"
    "in vec2 vertexTexCoord;\n"
    "in vec3 vertexNormal;\n"
    "in vec4 vertexColor;\n"
    "uniform mat4 mvp;\n"
    "uniform mat4 matModel;\n"
    "out vec3 fragWorldPos;\n"
    "out vec3 fragNormal;\n"
    "out vec4 fragColor;\n"
    "void main() {\n"
    "    vec4 worldPos = matModel * vec4(vertexPosition, 1.0);\n"
    "    fragWorldPos = worldPos.xyz;\n"
    "    fragNormal = normalize(mat3(matModel) * vertexNormal);\n"
    "    fragColor = vertexColor;\n"
    "    gl_Position = mvp * vec4(vertexPosition, 1.0);\n"
    "}\n";

static const char *kTerrainFs =
    "#version 330\n"
    "in vec3 fragWorldPos;\n"
    "in vec3 fragNormal;\n"
    "in vec4 fragColor;\n"
    "out vec4 finalColor;\n"
    "uniform sampler2D texture0;\n"
    "uniform sampler2D texture1;\n"
    "uniform sampler2D texture2;\n"
    "uniform sampler2D texture3;\n"
    "uniform float uTerrainMinY;\n"
    "uniform float uTerrainMaxY;\n"
    "uniform float uRockThreshold;\n"
    "uniform float uTexScale;\n"
    "uniform float uMacroScale;\n"
    "uniform vec3 uSunDir;\n"
    "uniform vec4 colDiffuse;\n"
    "\n"
    "float hash12(vec2 p) {\n"
    "    vec3 p3 = fract(vec3(p.xyx) * 0.1031);\n"
    "    p3 += dot(p3, p3.yzx + 33.33);\n"
    "    return fract((p3.x + p3.y) * p3.z);\n"
    "}\n"
    "\n"
    "float valueNoise(vec2 p) {\n"
    "    vec2 i = floor(p);\n"
    "    vec2 f = fract(p);\n"
    "    f = f * f * (3.0 - 2.0 * f);\n"
    "    float a = hash12(i);\n"
    "    float b = hash12(i + vec2(1.0, 0.0));\n"
    "    float c = hash12(i + vec2(0.0, 1.0));\n"
    "    float d = hash12(i + vec2(1.0, 1.0));\n"
    "    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);\n"
    "}\n"
    "\n"
    "vec3 sampleTriplanar(sampler2D tex, vec3 worldPos, vec3 blendWeights, float scale) {\n"
    "    vec3 xProj = texture(tex, worldPos.yz * scale).rgb;\n"
    "    vec3 yProj = texture(tex, worldPos.xz * scale).rgb;\n"
    "    vec3 zProj = texture(tex, worldPos.xy * scale).rgb;\n"
    "    return xProj * blendWeights.x + yProj * blendWeights.y + zProj * blendWeights.z;\n"
    "}\n"
    "\n"
    "void main() {\n"
    "    vec3 normal = normalize(fragNormal);\n"
    "    vec3 blend = pow(abs(normal), vec3(6.0));\n"
    "    blend /= max(dot(blend, vec3(1.0)), 0.0001);\n"
    "\n"
    "    float heightNorm = clamp((fragWorldPos.y - uTerrainMinY) / max(uTerrainMaxY - uTerrainMinY, 0.001), 0.0, 1.0);\n"
    "    float slope = 1.0 - clamp(normal.y, 0.0, 1.0);\n"
    "    float macroNoise = valueNoise(fragWorldPos.xz * uMacroScale);\n"
    "    float macroSignedNoise = (macroNoise - 0.5) * 2.0;\n"
    "    float meadowNoise = valueNoise(fragWorldPos.xz * (uMacroScale * 5.6) + vec2(37.0, 19.0));\n"
    "\n"
    "    vec3 dirtTexA = sampleTriplanar(texture0, fragWorldPos, blend, uTexScale * 1.14);\n"
    "    vec3 dirtTexB = sampleTriplanar(texture0, fragWorldPos + vec3(13.0, 5.0, 21.0), blend, uTexScale * 2.06);\n"
    "    vec3 dirtTex = mix(dirtTexA, dirtTexB, 0.28 + 0.14 * macroNoise);\n"
    "    vec3 grassTexA = sampleTriplanar(texture1, fragWorldPos, blend, uTexScale * 1.46);\n"
    "    vec3 grassTexB = sampleTriplanar(texture1, fragWorldPos + vec3(27.0, 9.0, 17.0), blend, uTexScale * 2.78);\n"
    "    vec3 grassTex = mix(grassTexA, grassTexB, 0.38 + 0.10 * macroNoise);\n"
    "    grassTex *= mix(0.92, 1.10, meadowNoise);\n"
    "    grassTex = mix(grassTex, grassTex * fragColor.rgb * vec3(0.92, 1.12, 0.88), 0.34);\n"
    "    vec3 rockTexA = sampleTriplanar(texture2, fragWorldPos, blend, uTexScale * 1.38);\n"
    "    vec3 rockTexB = sampleTriplanar(texture2, fragWorldPos + vec3(19.0, 11.0, 31.0), blend, uTexScale * 2.06);\n"
    "    vec3 rockTex = mix(rockTexA, rockTexB, 0.52 + 0.14 * meadowNoise);\n"
    "    rockTex *= mix(0.92, 1.06, macroNoise);\n"
    "    vec3 snowTexA = sampleTriplanar(texture3, fragWorldPos, blend, uTexScale * 1.12);\n"
    "    vec3 snowTexB = sampleTriplanar(texture3, fragWorldPos + vec3(9.0, 13.0, 5.0), blend, uTexScale * 1.82);\n"
    "    vec3 snowTex = mix(snowTexA, snowTexB, 0.35 + 0.12 * macroNoise);\n"
    "\n"
    "    float rockW = smoothstep(uRockThreshold - 0.12, min(uRockThreshold + 0.12, 1.0), slope + max(heightNorm - 0.34, 0.0) * 0.16 + macroSignedNoise * 0.10);\n"
    "    float flatness = smoothstep(0.48, 0.97, normal.y);\n"
    "    float grassW = (1.0 - rockW) * flatness * (1.0 - smoothstep(0.46, 0.90, heightNorm)) * smoothstep(0.18, 0.78, meadowNoise + macroSignedNoise * 0.14);\n"
    "    float dirtW = (1.0 - rockW) * (1.0 - smoothstep(0.10, 0.44, heightNorm + macroSignedNoise * 0.04)) * (1.0 - flatness * 0.90) * (0.55 + 0.25 * (1.0 - meadowNoise));\n"
    "    float coastW = max(1.0 - grassW - dirtW - rockW, 0.0);\n"
    "    float weightSum = max(grassW + dirtW + coastW + rockW, 0.0001);\n"
    "    vec3 lowMix = mix(grassTex, dirtTex, 0.36 + 0.18 * (1.0 - meadowNoise));\n"
    "    vec3 albedo = (grassTex * grassW + dirtTex * dirtW + lowMix * coastW + rockTex * rockW) / weightSum;\n"
    "\n"
    "    vec3 terrainTint = mix(vec3(1.0), fragColor.rgb * 1.12, 0.35);\n"
    "    albedo *= terrainTint;\n"
    "    albedo *= mix(0.92, 1.08, macroNoise);\n"
    "\n"
    "    float snowW = smoothstep(0.73, 0.95, heightNorm + macroSignedNoise * 0.06 - slope * 0.55);\n"
    "    snowW *= smoothstep(0.12, 0.58, normal.y + 0.18);\n"
    "    vec3 snowColor = mix(snowTex, snowTex * vec3(1.03, 1.04, 1.05), 0.25 + 0.10 * macroNoise);\n"
    "    albedo = mix(albedo, snowColor, snowW);\n"
    "\n"
    "    vec3 sunDir = normalize(uSunDir);\n"
    "    float diffuse = max(dot(normal, sunDir), 0.0);\n"
    "    float ambient = 0.34;\n"
    "    float backLight = 0.10 * max(dot(normal, normalize(vec3(-sunDir.x, 0.35, -sunDir.z))), 0.0);\n"
    "    vec3 lit = albedo * (ambient + diffuse * 0.82 + backLight);\n"
    "    finalColor = vec4(lit * colDiffuse.rgb, colDiffuse.a);\n"
    "}\n";

static void UnloadTerrainMaterialResources(AtmosphereSim *sim) {
    if (sim->terrainTexDirt.id != 0) UnloadTexture(sim->terrainTexDirt);
    if (sim->terrainTexCoast.id != 0) UnloadTexture(sim->terrainTexCoast);
    if (sim->terrainTexRock.id != 0) UnloadTexture(sim->terrainTexRock);
    if (sim->terrainTexSnow.id != 0) UnloadTexture(sim->terrainTexSnow);
    if (sim->terrainShader.id != 0) UnloadShader(sim->terrainShader);

    sim->terrainTexDirt = (Texture2D) { 0 };
    sim->terrainTexCoast = (Texture2D) { 0 };
    sim->terrainTexRock = (Texture2D) { 0 };
    sim->terrainTexSnow = (Texture2D) { 0 };
    sim->terrainShader = (Shader) { 0 };
    sim->terrainLocTexDirt = -1;
    sim->terrainLocTexCoast = -1;
    sim->terrainLocTexRock = -1;
    sim->terrainLocMinY = -1;
    sim->terrainLocMaxY = -1;
    sim->terrainLocRockThreshold = -1;
    sim->terrainLocTexScale = -1;
    sim->terrainLocMacroScale = -1;
    sim->terrainLocSunDir = -1;
}

static bool ResolveTerrainAssetPath(const char *relativePath, char *outPath, size_t outPathSize) {
    char altPath[1024];
    snprintf(altPath, sizeof(altPath), "../%s", relativePath);

    const char *candidatePaths[2] = { relativePath, altPath };
    for (int i = 0; i < 2; i++) {
        if (!FileExists(candidatePaths[i])) continue;
        strncpy(outPath, candidatePaths[i], outPathSize - 1);
        outPath[outPathSize - 1] = '\0';
        return true;
    }

    outPath[0] = '\0';
    return false;
}

static Texture2D LoadTerrainTextureAsset(const char *relativePath) {
    char resolvedPath[1024];
    if (!ResolveTerrainAssetPath(relativePath, resolvedPath, sizeof(resolvedPath))) return (Texture2D) { 0 };

    Texture2D texture = LoadTexture(resolvedPath);
    if (texture.id != 0) {
        GenTextureMipmaps(&texture);
        SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);
        SetTextureWrap(texture, TEXTURE_WRAP_REPEAT);
    }

    return texture;
}

static Texture2D LoadBlendedTerrainTextureAssets(const char *relativePathA, const char *relativePathB) {
    char resolvedPathA[1024];
    char resolvedPathB[1024];
    if (!ResolveTerrainAssetPath(relativePathA, resolvedPathA, sizeof(resolvedPathA))) return (Texture2D) { 0 };
    if (!ResolveTerrainAssetPath(relativePathB, resolvedPathB, sizeof(resolvedPathB))) return (Texture2D) { 0 };

    Image imageA = LoadImage(resolvedPathA);
    Image imageB = LoadImage(resolvedPathB);
    if (imageA.data == NULL || imageB.data == NULL) {
        if (imageA.data != NULL) UnloadImage(imageA);
        if (imageB.data != NULL) UnloadImage(imageB);
        return (Texture2D) { 0 };
    }

    if (imageB.width != imageA.width || imageB.height != imageA.height) {
        ImageResize(&imageB, imageA.width, imageA.height);
    }

    ImageFormat(&imageA, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);
    ImageFormat(&imageB, PIXELFORMAT_UNCOMPRESSED_R8G8B8A8);

    Color *pixelsA = LoadImageColors(imageA);
    Color *pixelsB = LoadImageColors(imageB);
    int pixelCount = imageA.width * imageA.height;
    Image mixedImage = { 0 };
    Texture2D texture = { 0 };

    if (pixelsA != NULL && pixelsB != NULL) {
        Color *mixedPixels = (Color *)MemAlloc((size_t)pixelCount * sizeof(Color));
        if (mixedPixels != NULL) {
            for (int i = 0; i < pixelCount; i++) {
                int x = i % imageA.width;
                int y = i / imageA.width;
                float localNoise = 0.5f + 0.5f * sinf((float)x * 0.061f + (float)y * 0.037f) * cosf((float)x * 0.019f - (float)y * 0.053f);
                float blend = Clamp(0.38f + 0.32f * localNoise, 0.18f, 0.82f);
                mixedPixels[i].r = (unsigned char)Clamp(pixelsA[i].r * (1.0f - blend) + pixelsB[i].r * blend, 0.0f, 255.0f);
                mixedPixels[i].g = (unsigned char)Clamp(pixelsA[i].g * (1.0f - blend) + pixelsB[i].g * blend, 0.0f, 255.0f);
                mixedPixels[i].b = (unsigned char)Clamp(pixelsA[i].b * (1.0f - blend) + pixelsB[i].b * blend, 0.0f, 255.0f);
                mixedPixels[i].a = 255;
            }

            mixedImage.data = mixedPixels;
            mixedImage.width = imageA.width;
            mixedImage.height = imageA.height;
            mixedImage.mipmaps = 1;
            mixedImage.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
            texture = LoadTextureFromImage(mixedImage);
            UnloadImage(mixedImage);
        }
    }

    if (pixelsA != NULL) UnloadImageColors(pixelsA);
    if (pixelsB != NULL) UnloadImageColors(pixelsB);
    UnloadImage(imageA);
    UnloadImage(imageB);

    if (texture.id != 0) {
        GenTextureMipmaps(&texture);
        SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);
        SetTextureWrap(texture, TEXTURE_WRAP_REPEAT);
    }

    return texture;
}

static bool SetupTerrainMaterialResources(AtmosphereSim *sim) {
    UnloadTerrainMaterialResources(sim);

    sim->terrainTexDirt = LoadTerrainTextureAsset("textures/dirt_1k.blend/textures/dirt_diff_1k.jpg");
    sim->terrainTexCoast = LoadTerrainTextureAsset("textures/coast_sand_rocks_02_1k.blend/textures/coast_sand_rocks_02_diff_1k.jpg");
    sim->terrainTexRock = LoadBlendedTerrainTextureAssets(
        "textures/rock_ground_1k.blend/textures/rock_ground_diff_1k.jpg",
        "textures/rocks_ground_06_1k.blend/textures/rocks_ground_06_diff_1k.jpg");
    sim->terrainTexSnow = LoadTerrainTextureAsset("textures/snow_02_1k.blend/textures/snow_02_diff_1k.jpg");
    if (sim->terrainTexDirt.id == 0 || sim->terrainTexCoast.id == 0 || sim->terrainTexRock.id == 0 || sim->terrainTexSnow.id == 0) {
        UnloadTerrainMaterialResources(sim);
        return false;
    }

    sim->terrainShader = LoadShaderFromMemory(kTerrainVs, kTerrainFs);
    if (sim->terrainShader.id == 0) {
        UnloadTerrainMaterialResources(sim);
        return false;
    }

    sim->terrainLocMinY = GetShaderLocation(sim->terrainShader, "uTerrainMinY");
    sim->terrainLocMaxY = GetShaderLocation(sim->terrainShader, "uTerrainMaxY");
    sim->terrainLocRockThreshold = GetShaderLocation(sim->terrainShader, "uRockThreshold");
    sim->terrainLocTexScale = GetShaderLocation(sim->terrainShader, "uTexScale");
    sim->terrainLocMacroScale = GetShaderLocation(sim->terrainShader, "uMacroScale");
    sim->terrainLocSunDir = GetShaderLocation(sim->terrainShader, "uSunDir");
    sim->terrainShader.locs[SHADER_LOC_MAP_DIFFUSE] = GetShaderLocation(sim->terrainShader, "texture0");
    sim->terrainShader.locs[SHADER_LOC_MAP_SPECULAR] = GetShaderLocation(sim->terrainShader, "texture1");
    sim->terrainShader.locs[SHADER_LOC_MAP_NORMAL] = GetShaderLocation(sim->terrainShader, "texture2");
    sim->terrainShader.locs[SHADER_LOC_MAP_ROUGHNESS] = GetShaderLocation(sim->terrainShader, "texture3");
    return true;
}

static void ApplyTerrainMaterial(AtmosphereSim *sim) {
    if (sim->terrainModel.meshCount == 0 || sim->terrainShader.id == 0) return;

    Material *material = &sim->terrainModel.materials[0];
    material->shader = sim->terrainShader;
    material->maps[MATERIAL_MAP_DIFFUSE].color = WHITE;
    material->maps[MATERIAL_MAP_DIFFUSE].texture = sim->terrainTexDirt;
    material->maps[MATERIAL_MAP_SPECULAR].texture = sim->terrainTexCoast;
    material->maps[MATERIAL_MAP_NORMAL].texture = sim->terrainTexRock;
    material->maps[MATERIAL_MAP_ROUGHNESS].texture = sim->terrainTexSnow;
}

static void UpdateTerrainMaterialUniforms(const AtmosphereSim *sim) {
    if (sim->terrainShader.id == 0) return;

    float minY = sim->terrain.minTerrainY;
    float maxY = sim->terrain.maxTerrainY;
    float rockThreshold = sim->terrainGen.rockThreshold;
    float texScale = 0.054f;
    float macroScale = 0.016f;
    Vector3 lightDir = sim->sunDir;
    if (Vector3Length(lightDir) < 0.001f) lightDir = (Vector3) { 0.35f, 1.0f, 0.45f };
    if (lightDir.y < 0.18f) lightDir.y = 0.18f;
    lightDir = Vector3Normalize(lightDir);

    if (sim->terrainLocMinY >= 0) SetShaderValue(sim->terrainShader, sim->terrainLocMinY, &minY, SHADER_UNIFORM_FLOAT);
    if (sim->terrainLocMaxY >= 0) SetShaderValue(sim->terrainShader, sim->terrainLocMaxY, &maxY, SHADER_UNIFORM_FLOAT);
    if (sim->terrainLocRockThreshold >= 0) SetShaderValue(sim->terrainShader, sim->terrainLocRockThreshold, &rockThreshold, SHADER_UNIFORM_FLOAT);
    if (sim->terrainLocTexScale >= 0) SetShaderValue(sim->terrainShader, sim->terrainLocTexScale, &texScale, SHADER_UNIFORM_FLOAT);
    if (sim->terrainLocMacroScale >= 0) SetShaderValue(sim->terrainShader, sim->terrainLocMacroScale, &macroScale, SHADER_UNIFORM_FLOAT);
    if (sim->terrainLocSunDir >= 0) SetShaderValue(sim->terrainShader, sim->terrainLocSunDir, &lightDir.x, SHADER_UNIFORM_VEC3);

}

static bool SetupCloudVolumeResources(AtmosphereSim *sim) {
    sim->volumeAtlasCols = 4;
    sim->volumeAtlasRows = (sim->ny + sim->volumeAtlasCols - 1) / sim->volumeAtlasCols;
    int atlasWidth = sim->nx * sim->volumeAtlasCols;
    int atlasHeight = sim->nz * sim->volumeAtlasRows;

    sim->volumeFieldPixels = (Vector4 *)calloc((size_t)atlasWidth * (size_t)atlasHeight, sizeof(Vector4));
    if (sim->volumeFieldPixels == NULL) return false;

    Image image = {
        .data = sim->volumeFieldPixels,
        .width = atlasWidth,
        .height = atlasHeight,
        .mipmaps = 1,
        .format = PIXELFORMAT_UNCOMPRESSED_R32G32B32A32
    };
    sim->volumeFieldTex = LoadTextureFromImage(image);
    if (sim->volumeFieldTex.id == 0) return false;

    SetTextureFilter(sim->volumeFieldTex, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(sim->volumeFieldTex, TEXTURE_WRAP_CLAMP);

    sim->cloudVolumeModel = LoadModelFromMesh(GenMeshCube(sim->worldWidth, sim->topWorldY, sim->worldDepth));
    if (sim->cloudVolumeModel.meshCount == 0) return false;

    sim->cloudVolumeShader = LoadShaderFromMemory(kCloudVolumeVs, kCloudVolumeFs);
    if (sim->cloudVolumeShader.id == 0) return false;

    sim->cloudVolumeLocMin = GetShaderLocation(sim->cloudVolumeShader, "uVolumeMin");
    sim->cloudVolumeLocMax = GetShaderLocation(sim->cloudVolumeShader, "uVolumeMax");
    sim->cloudVolumeLocCamera = GetShaderLocation(sim->cloudVolumeShader, "uCameraPos");
    sim->cloudVolumeLocRes = GetShaderLocation(sim->cloudVolumeShader, "uVolumeRes");
    sim->cloudVolumeLocAtlas = GetShaderLocation(sim->cloudVolumeShader, "uAtlasGrid");
    sim->cloudVolumeLocParams = GetShaderLocation(sim->cloudVolumeShader, "uCloudParams");
    sim->cloudVolumeLocSun = GetShaderLocation(sim->cloudVolumeShader, "uSunDir");
    sim->cloudVolumeModel.materials[0].shader = sim->cloudVolumeShader;
    sim->cloudVolumeModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = sim->volumeFieldTex;
    return true;
}

static bool SetupSkyShaderResources(AtmosphereSim *sim) {
    sim->skyShader = LoadShaderFromMemory(kSkyVs, kSkyFs);
    if (sim->skyShader.id == 0) return false;

    sim->skyLocSunDir = GetShaderLocation(sim->skyShader, "uSunDir");
    sim->skyLocCamRight = GetShaderLocation(sim->skyShader, "uCamRight");
    sim->skyLocCamUp = GetShaderLocation(sim->skyShader, "uCamUp");
    sim->skyLocCamForward = GetShaderLocation(sim->skyShader, "uCamForward");
    sim->skyLocAspect = GetShaderLocation(sim->skyShader, "uAspect");
    sim->skyLocTanHalfFov = GetShaderLocation(sim->skyShader, "uTanHalfFov");
    sim->skyLocSunStrength = GetShaderLocation(sim->skyShader, "uSunStrength");
    return true;
}

static bool UpdateCloudTerrainTexture(AtmosphereSim *sim) {
    if (sim->terrain.width <= 0 || sim->terrain.height <= 0 || sim->terrain.terrainY == NULL) return false;

    int pixelCount = sim->terrain.width * sim->terrain.height;
    Color *pixels = (Color *)MemAlloc((size_t)pixelCount * sizeof(Color));
    if (pixels == NULL) return false;

    for (int z = 0; z < sim->terrain.height; z++) {
        for (int x = 0; x < sim->terrain.width; x++) {
            int i = z * sim->terrain.width + x;
            float terrainWorldY = sim->terrain.terrainY[i];
            float normalized = Clamp(terrainWorldY / fmaxf(sim->topWorldY, 0.001f), 0.0f, 1.0f);
            unsigned char packed = (unsigned char)(normalized * 255.0f);
            pixels[i] = (Color) { packed, packed, packed, 255 };
        }
    }

    Image image = {
        .data = pixels,
        .width = sim->terrain.width,
        .height = sim->terrain.height,
        .mipmaps = 1,
        .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8
    };

    if (sim->cloudTerrainTex.id != 0) UnloadTexture(sim->cloudTerrainTex);
    sim->cloudTerrainTex = LoadTextureFromImage(image);
    UnloadImage(image);

    if (sim->cloudTerrainTex.id == 0) return false;

    SetTextureFilter(sim->cloudTerrainTex, TEXTURE_FILTER_BILINEAR);
    SetTextureWrap(sim->cloudTerrainTex, TEXTURE_WRAP_CLAMP);
    if (sim->cloudVolumeModel.materialCount > 0) {
        sim->cloudVolumeModel.materials[0].maps[MATERIAL_MAP_SPECULAR].texture = sim->cloudTerrainTex;
    }
    return true;
}

static int TerrainIndex(const TerrainGrid *terrain, int x, int z) {
    return z * terrain->width + x;
}

static int ColumnIndex(const AtmosphereSim *sim, int x, int z) {
    return z * sim->nx + x;
}

static int AtmosIndex(const AtmosphereSim *sim, int x, int y, int z) {
    return ((y * sim->nz) + z) * sim->nx + x;
}

static void SwapFields(float **a, float **b) {
    float *tmp = *a;
    *a = *b;
    *b = tmp;
}

static float BilinearSample(const float *field, int width, int height, float x, float z) {
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

static bool IsFluidCell(const AtmosphereSim *sim, int x, int y, int z) {
    int column = ColumnIndex(sim, x, z);
    return y > sim->groundLayer[column];
}

static int GroundLayerForAltitude(const AtmosphereSim *sim, float terrainMeters) {
    int layer = (int)floorf((terrainMeters - 0.5f * sim->dyMeters) / sim->dyMeters);
    if (layer < -1) layer = -1;
    if (layer >= sim->ny) layer = sim->ny - 1;
    return layer;
}

static float CellAltitudeMeters(const AtmosphereSim *sim, int y) {
    return ((float)y + 0.5f) * sim->dyMeters;
}

static float WorldYFromMeters(float meters) {
    return meters / OBJ_VERTICAL_METERS;
}

static float SampleAtmosField(const AtmosphereSim *sim, const float *field, float x, float y, float z) {
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

static void FreeTerrain(TerrainGrid *terrain) {
    free(terrain->terrainY);
    free(terrain->terrainMeters);
    free(terrain->terrainDx);
    free(terrain->terrainDz);
    free(terrain->terrainSlope);
    memset(terrain, 0, sizeof(*terrain));
}

static void ComputeTerrainDerivatives(TerrainGrid *terrain) {
    for (int z = 0; z < terrain->height; z++) {
        for (int x = 0; x < terrain->width; x++) {
            int xl = (x > 0) ? x - 1 : x;
            int xr = (x + 1 < terrain->width) ? x + 1 : x;
            int zd = (z > 0) ? z - 1 : z;
            int zu = (z + 1 < terrain->height) ? z + 1 : z;

            float hL = terrain->terrainMeters[TerrainIndex(terrain, xl, z)];
            float hR = terrain->terrainMeters[TerrainIndex(terrain, xr, z)];
            float hD = terrain->terrainMeters[TerrainIndex(terrain, x, zd)];
            float hU = terrain->terrainMeters[TerrainIndex(terrain, x, zu)];

            float dx = (hR - hL) / (fabsf((float)(xr - xl)) * OBJ_HORIZONTAL_METERS + 0.0001f);
            float dz = (hU - hD) / (fabsf((float)(zu - zd)) * OBJ_HORIZONTAL_METERS + 0.0001f);
            int i = TerrainIndex(terrain, x, z);
            terrain->terrainDx[i] = dx;
            terrain->terrainDz[i] = dz;
            terrain->terrainSlope[i] = sqrtf(dx * dx + dz * dz);
        }
    }
}

static int TerrainGenIndex(int x, int z) {
    return z * TERRAIN_GEN_SIZE + x;
}

static void DefaultTerrainGenSettings(TerrainGenSettings *settings, int seed) {
    settings->seed = (seed <= 0) ? 1337 : seed;
    settings->frequency = 0.004f;
    settings->amplitude = 1.0f;
    settings->heightPower = 1.5f;
    settings->octaves = 4;
    settings->lacunarity = 1.9f;
    settings->persistence = 0.5f;
    settings->droplets = 700000;
    settings->lifetime = 24;
    settings->inertia = 0.02f;
    settings->capacity = 2.0f;
    settings->minSlope = 0.001f;
    settings->erodeRate = 0.04f;
    settings->depositRate = 0.04f;
    settings->evaporate = 0.10f;
    settings->maxStep = 1.0f;
    settings->smoothPasses = 8;
    settings->rockThreshold = 0.42f;
}

static unsigned int TerrainRngNext(unsigned int *state) {
    *state = (*state * 1664525u) + 1013904223u;
    return *state;
}

static float TerrainRng01(unsigned int *state) {
    return (float)(TerrainRngNext(state) >> 8) * (1.0f / 16777216.0f);
}

static int TerrainHash(int x, int y, int seed) {
    int h = x * 374761393 + y * 668265263 + seed * 362437;
    h = (h ^ (h >> 13)) * 1274126177;
    return h & 1023;
}

static float TerrainSmoothstep(float t) {
    return t * t * (3.0f - 2.0f * t);
}

static float TerrainGradient(int hash, float x, float y) {
    switch (hash & 3) {
        case 0: return x + y;
        case 1: return -x + y;
        case 2: return x - y;
        default: return -x - y;
    }
}

static float TerrainPerlin(float x, float y, int seed) {
    int X = (int)floorf(x);
    int Y = (int)floorf(y);
    float xf = x - floorf(x);
    float yf = y - floorf(y);

    int h00 = TerrainHash(X, Y, seed);
    int h10 = TerrainHash(X + 1, Y, seed);
    int h01 = TerrainHash(X, Y + 1, seed);
    int h11 = TerrainHash(X + 1, Y + 1, seed);

    float u = TerrainSmoothstep(xf);
    float v = TerrainSmoothstep(yf);

    float n00 = TerrainGradient(h00, xf, yf);
    float n10 = TerrainGradient(h10, xf - 1.0f, yf);
    float n01 = TerrainGradient(h01, xf, yf - 1.0f);
    float n11 = TerrainGradient(h11, xf - 1.0f, yf - 1.0f);

    return Lerp(Lerp(n00, n10, u), Lerp(n01, n11, u), v);
}

static void NormalizeTerrainMap(float *map, int count) {
    float minValue = FLT_MAX;
    float maxValue = -FLT_MAX;
    for (int i = 0; i < count; i++) {
        if (map[i] < minValue) minValue = map[i];
        if (map[i] > maxValue) maxValue = map[i];
    }

    float span = fmaxf(maxValue - minValue, 0.0001f);
    for (int i = 0; i < count; i++) {
        map[i] = Clamp((map[i] - minValue) / span, 0.0f, 1.0f);
    }
}

static void GenerateNoiseHeightmap(float *map, const TerrainGenSettings *settings) {
    unsigned int rng = (unsigned int)settings->seed;
    float offsetX = TerrainRng01(&rng) * 10000.0f;
    float offsetZ = TerrainRng01(&rng) * 10000.0f;
    int count = TERRAIN_GEN_SIZE * TERRAIN_GEN_SIZE;

    for (int z = 0; z < TERRAIN_GEN_SIZE; z++) {
        for (int x = 0; x < TERRAIN_GEN_SIZE; x++) {
            float noise = 0.0f;
            float frequency = settings->frequency * settings->lacunarity;
            float amplitude = settings->amplitude;
            float totalAmplitude = 0.0f;

            for (int octave = 0; octave < settings->octaves; octave++) {
                float nx = ((float)x + offsetX) * frequency;
                float nz = ((float)z + offsetZ) * frequency;
                noise += TerrainPerlin(nx, nz, settings->seed + octave * 101) * amplitude;
                totalAmplitude += amplitude;
                amplitude *= settings->persistence;
                frequency *= settings->lacunarity;
            }

            int index = TerrainGenIndex(x, z);
            map[index] = (totalAmplitude > 0.0f) ? noise / totalAmplitude : 0.0f;
            map[index] = (map[index] + 1.0f) * 0.5f;
        }
    }

    NormalizeTerrainMap(map, count);

    for (int i = 0; i < count; i++) {
        map[i] = powf(Clamp(map[i], 0.0f, 1.0f), settings->heightPower);
    }

    NormalizeTerrainMap(map, count);
}

static void ApplyHydroErosionToMap(float *map, const TerrainGenSettings *settings) {
    unsigned int rng = (unsigned int)(settings->seed * 9781 + 17);

    for (int drop = 0; drop < settings->droplets; drop++) {
        float x = TerrainRng01(&rng) * (float)(TERRAIN_GEN_SIZE - 2) + 0.5f;
        float z = TerrainRng01(&rng) * (float)(TERRAIN_GEN_SIZE - 2) + 0.5f;
        float dirX = 0.0f;
        float dirZ = 0.0f;
        float speed = 1.0f;
        float water = 1.0f;
        float sediment = 0.0f;

        for (int life = 0; life < settings->lifetime; life++) {
            int xi = (int)x;
            int zi = (int)z;
            if (xi < 0 || xi >= TERRAIN_GEN_SIZE - 1 || zi < 0 || zi >= TERRAIN_GEN_SIZE - 1) break;

            float fx = x - (float)xi;
            float fz = z - (float)zi;
            float h00 = map[TerrainGenIndex(xi, zi)];
            float h10 = map[TerrainGenIndex(xi + 1, zi)];
            float h01 = map[TerrainGenIndex(xi, zi + 1)];
            float h11 = map[TerrainGenIndex(xi + 1, zi + 1)];
            float height = h00 * (1.0f - fx) * (1.0f - fz) +
                           h10 * fx * (1.0f - fz) +
                           h01 * (1.0f - fx) * fz +
                           h11 * fx * fz;
            float gradX = (h10 - h00) * (1.0f - fz) + (h11 - h01) * fz;
            float gradZ = (h01 - h00) * (1.0f - fx) + (h11 - h10) * fx;

            dirX = dirX * settings->inertia - gradX * (1.0f - settings->inertia);
            dirZ = dirZ * settings->inertia - gradZ * (1.0f - settings->inertia);
            float len = sqrtf(dirX * dirX + dirZ * dirZ);
            if (len > 0.00001f) {
                dirX /= len;
                dirZ /= len;
            } else {
                dirX = (TerrainRng01(&rng) - 0.5f) * 2.0f;
                dirZ = (TerrainRng01(&rng) - 0.5f) * 2.0f;
            }

            x += dirX;
            z += dirZ;
            if (x < 0.0f || x >= TERRAIN_GEN_SIZE - 1 || z < 0.0f || z >= TERRAIN_GEN_SIZE - 1) break;

            xi = (int)x;
            zi = (int)z;
            fx = x - (float)xi;
            fz = z - (float)zi;
            float nh00 = map[TerrainGenIndex(xi, zi)];
            float nh10 = map[TerrainGenIndex(xi + 1, zi)];
            float nh01 = map[TerrainGenIndex(xi, zi + 1)];
            float nh11 = map[TerrainGenIndex(xi + 1, zi + 1)];
            float newHeight = nh00 * (1.0f - fx) * (1.0f - fz) +
                              nh10 * fx * (1.0f - fz) +
                              nh01 * (1.0f - fx) * fz +
                              nh11 * fx * fz;
            float delta = newHeight - height;

            float capacity = fmaxf(-delta * speed * water * settings->capacity, settings->minSlope);
            if (sediment > capacity || delta > 0.0f) {
                float deposit = (delta > 0.0f) ? fminf(sediment, delta) : (sediment - capacity) * settings->depositRate;
                deposit = fminf(deposit, settings->maxStep);
                sediment -= deposit;

                map[TerrainGenIndex(xi, zi)] = Clamp(map[TerrainGenIndex(xi, zi)] + deposit * (1.0f - fx) * (1.0f - fz), 0.0f, 1.0f);
                map[TerrainGenIndex(xi + 1, zi)] = Clamp(map[TerrainGenIndex(xi + 1, zi)] + deposit * fx * (1.0f - fz), 0.0f, 1.0f);
                map[TerrainGenIndex(xi, zi + 1)] = Clamp(map[TerrainGenIndex(xi, zi + 1)] + deposit * (1.0f - fx) * fz, 0.0f, 1.0f);
                map[TerrainGenIndex(xi + 1, zi + 1)] = Clamp(map[TerrainGenIndex(xi + 1, zi + 1)] + deposit * fx * fz, 0.0f, 1.0f);
            } else {
                float erode = fminf((capacity - sediment) * settings->erodeRate, -delta);
                erode = fminf(erode, settings->maxStep);

                for (int ox = 0; ox <= 1; ox++) {
                    for (int oz = 0; oz <= 1; oz++) {
                        float weight = ((ox == 0) ? (1.0f - fx) : fx) * ((oz == 0) ? (1.0f - fz) : fz);
                        int index = TerrainGenIndex(xi + ox, zi + oz);
                        float deltaCell = fminf(map[index], erode * weight);
                        map[index] = Clamp(map[index] - deltaCell, 0.0f, 1.0f);
                        sediment += deltaCell;
                    }
                }
            }

            speed = sqrtf(fmaxf(speed * speed + fabsf(delta) * 0.1f, 0.0f));
            water *= (1.0f - settings->evaporate);
            if (water < 0.01f) break;
        }
    }
}

static void SmoothTerrainMap(float *map, int passes) {
    int count = TERRAIN_GEN_SIZE * TERRAIN_GEN_SIZE;
    float *scratch = (float *)malloc((size_t)count * sizeof(float));
    if (scratch == NULL) return;

    for (int pass = 0; pass < passes; pass++) {
        memcpy(scratch, map, (size_t)count * sizeof(float));
        for (int z = 1; z < TERRAIN_GEN_SIZE - 1; z++) {
            for (int x = 1; x < TERRAIN_GEN_SIZE - 1; x++) {
                float sum = 0.0f;
                for (int dz = -1; dz <= 1; dz++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        sum += scratch[TerrainGenIndex(x + dx, z + dz)];
                    }
                }
                map[TerrainGenIndex(x, z)] = sum / 9.0f;
            }
        }
    }

    free(scratch);
}

static void FillTerrainPits(float *map) {
    const float epsilon = 1e-5f;
    for (int pass = 0; pass < 3; pass++) {
        for (int z = 1; z < TERRAIN_GEN_SIZE - 1; z++) {
            for (int x = 1; x < TERRAIN_GEN_SIZE - 1; x++) {
                float h = map[TerrainGenIndex(x, z)];
                float localMin = h;
                for (int dz = -1; dz <= 1; dz++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        float neighbor = map[TerrainGenIndex(x + dx, z + dz)];
                        if (neighbor < localMin) localMin = neighbor;
                    }
                }
                if (h < localMin - epsilon) {
                    map[TerrainGenIndex(x, z)] = localMin;
                }
            }
        }
    }
}

static bool GenerateProceduralTerrain(TerrainGrid *terrain, const TerrainGenSettings *settings) {
    int count = TERRAIN_GEN_SIZE * TERRAIN_GEN_SIZE;
    float *map = (float *)malloc((size_t)count * sizeof(float));
    terrain->width = TERRAIN_GEN_SIZE;
    terrain->height = TERRAIN_GEN_SIZE;
    terrain->terrainY = (float *)calloc((size_t)count, sizeof(float));
    terrain->terrainMeters = (float *)calloc((size_t)count, sizeof(float));
    terrain->terrainDx = (float *)calloc((size_t)count, sizeof(float));
    terrain->terrainDz = (float *)calloc((size_t)count, sizeof(float));
    terrain->terrainSlope = (float *)calloc((size_t)count, sizeof(float));

    if (map == NULL || !terrain->terrainY || !terrain->terrainMeters || !terrain->terrainDx ||
        !terrain->terrainDz || !terrain->terrainSlope) {
        free(map);
        FreeTerrain(terrain);
        return false;
    }

    GenerateNoiseHeightmap(map, settings);
    ApplyHydroErosionToMap(map, settings);
    SmoothTerrainMap(map, settings->smoothPasses);
    FillTerrainPits(map);
    NormalizeTerrainMap(map, count);

    terrain->minTerrainY = FLT_MAX;
    terrain->maxTerrainY = -FLT_MAX;
    terrain->minTerrainMeters = FLT_MAX;
    terrain->maxTerrainMeters = -FLT_MAX;

    for (int z = 0; z < TERRAIN_GEN_SIZE; z++) {
        for (int x = 0; x < TERRAIN_GEN_SIZE; x++) {
            int index = TerrainGenIndex(x, z);
            float worldHeight = map[index] * TERRAIN_GEN_HEIGHT_WORLD;
            terrain->terrainY[index] = worldHeight;
            terrain->terrainMeters[index] = worldHeight * OBJ_VERTICAL_METERS;

            if (terrain->terrainY[index] < terrain->minTerrainY) terrain->minTerrainY = terrain->terrainY[index];
            if (terrain->terrainY[index] > terrain->maxTerrainY) terrain->maxTerrainY = terrain->terrainY[index];
            if (terrain->terrainMeters[index] < terrain->minTerrainMeters) terrain->minTerrainMeters = terrain->terrainMeters[index];
            if (terrain->terrainMeters[index] > terrain->maxTerrainMeters) terrain->maxTerrainMeters = terrain->terrainMeters[index];
        }
    }

    free(map);
    ComputeTerrainDerivatives(terrain);
    return true;
}

static bool LoadOBJHeightField(const char *path, TerrainGrid *terrain) {
    FILE *file = fopen(path, "r");
    if (!file) return false;

    int capacity = 65536;
    int count = 0;
    Vector3 *vertices = (Vector3 *)malloc((size_t)capacity * sizeof(Vector3));
    char line[256];

    if (!vertices) {
        fclose(file);
        return false;
    }

    while (fgets(line, sizeof(line), file)) {
        if (line[0] == 'v' && line[1] == ' ') {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            if (sscanf(line, "v %f %f %f", &x, &y, &z) == 3) {
                if (count >= capacity) {
                    capacity *= 2;
                    Vector3 *grown = (Vector3 *)realloc(vertices, (size_t)capacity * sizeof(Vector3));
                    if (!grown) {
                        free(vertices);
                        fclose(file);
                        return false;
                    }
                    vertices = grown;
                }
                vertices[count++] = (Vector3) { x, y, z };
            }
        }
    }

    fclose(file);
    if (count < 4) {
        free(vertices);
        return false;
    }

    int width = 1;
    float firstZ = vertices[0].z;
    while (width < count && fabsf(vertices[width].z - firstZ) < 0.0001f) width++;

    if (width <= 1 || (count % width) != 0) {
        int side = (int)(sqrtf((float)count) + 0.5f);
        if (side * side != count) {
            free(vertices);
            return false;
        }
        width = side;
    }

    int height = count / width;
    int cells = width * height;

    terrain->width = width;
    terrain->height = height;
    terrain->terrainY = (float *)calloc((size_t)cells, sizeof(float));
    terrain->terrainMeters = (float *)calloc((size_t)cells, sizeof(float));
    terrain->terrainDx = (float *)calloc((size_t)cells, sizeof(float));
    terrain->terrainDz = (float *)calloc((size_t)cells, sizeof(float));
    terrain->terrainSlope = (float *)calloc((size_t)cells, sizeof(float));

    if (!terrain->terrainY || !terrain->terrainMeters || !terrain->terrainDx ||
        !terrain->terrainDz || !terrain->terrainSlope) {
        free(vertices);
        FreeTerrain(terrain);
        return false;
    }

    terrain->minTerrainY = FLT_MAX;
    terrain->maxTerrainY = -FLT_MAX;
    terrain->minTerrainMeters = FLT_MAX;
    terrain->maxTerrainMeters = -FLT_MAX;

    for (int i = 0; i < cells; i++) {
        terrain->terrainY[i] = vertices[i].y;
        terrain->terrainMeters[i] = vertices[i].y * OBJ_VERTICAL_METERS;
        if (terrain->terrainY[i] < terrain->minTerrainY) terrain->minTerrainY = terrain->terrainY[i];
        if (terrain->terrainY[i] > terrain->maxTerrainY) terrain->maxTerrainY = terrain->terrainY[i];
        if (terrain->terrainMeters[i] < terrain->minTerrainMeters) terrain->minTerrainMeters = terrain->terrainMeters[i];
        if (terrain->terrainMeters[i] > terrain->maxTerrainMeters) terrain->maxTerrainMeters = terrain->terrainMeters[i];
    }

    free(vertices);
    ComputeTerrainDerivatives(terrain);
    return true;
}

static Color TerrainColor(float heightNorm, float slope, float shade, float rockThreshold) {
    Color lowGrass = (Color) { 58, 93, 54, 255 };
    Color alpine = (Color) { 122, 130, 82, 255 };
    Color warmRock = (Color) { 133, 118, 103, 255 };
    Color coldRock = (Color) { 173, 171, 168, 255 };
    Color snow = (Color) { 236, 242, 247, 255 };
    Color base;

    if (heightNorm > 0.86f) base = LerpColor(coldRock, snow, NormalizeRange(heightNorm, 0.86f, 1.0f));
    else if (slope > rockThreshold) base = LerpColor(warmRock, coldRock, NormalizeRange(slope, rockThreshold, 1.0f));
    else if (heightNorm > 0.52f) base = LerpColor(alpine, warmRock, NormalizeRange(heightNorm, 0.52f, 0.86f));
    else base = LerpColor(lowGrass, alpine, NormalizeRange(heightNorm, 0.18f, 0.52f));

    float lit = 0.32f + 0.68f * shade;
    base.r = (unsigned char)Clamp(base.r * lit, 0.0f, 255.0f);
    base.g = (unsigned char)Clamp(base.g * lit, 0.0f, 255.0f);
    base.b = (unsigned char)Clamp(base.b * lit, 0.0f, 255.0f);
    return base;
}

static Model BuildTerrainModel(const TerrainGrid *terrain, const TerrainGenSettings *settings) {
    const int vertexCount = terrain->width * terrain->height;
    const int indexCount = (terrain->width - 1) * (terrain->height - 1) * 6;
    Vector3 *vertices = (Vector3 *)MemAlloc((size_t)vertexCount * sizeof(Vector3));
    Vector3 *normals = (Vector3 *)MemAlloc((size_t)vertexCount * sizeof(Vector3));
    Color *colors = (Color *)MemAlloc((size_t)vertexCount * sizeof(Color));
    int *indices = (int *)MemAlloc((size_t)indexCount * sizeof(int));

    if (!vertices || !normals || !colors || !indices || vertexCount > 65536) {
        if (vertices) MemFree(vertices);
        if (normals) MemFree(normals);
        if (colors) MemFree(colors);
        if (indices) MemFree(indices);
        return (Model) { 0 };
    }

    for (int z = 0; z < terrain->height; z++) {
        for (int x = 0; x < terrain->width; x++) {
            int i = TerrainIndex(terrain, x, z);
            vertices[i] = (Vector3) { (float)x, terrain->terrainY[i], (float)z };
            normals[i] = (Vector3) { 0.0f, 0.0f, 0.0f };
        }
    }

    int cursor = 0;
    for (int z = 0; z < terrain->height - 1; z++) {
        for (int x = 0; x < terrain->width - 1; x++) {
            int i0 = TerrainIndex(terrain, x, z);
            int i1 = TerrainIndex(terrain, x + 1, z);
            int i2 = TerrainIndex(terrain, x, z + 1);
            int i3 = TerrainIndex(terrain, x + 1, z + 1);
            indices[cursor++] = i0; indices[cursor++] = i2; indices[cursor++] = i1;
            indices[cursor++] = i1; indices[cursor++] = i2; indices[cursor++] = i3;
        }
    }

    for (int i = 0; i < indexCount; i += 3) {
        int i0 = indices[i];
        int i1 = indices[i + 1];
        int i2 = indices[i + 2];
        Vector3 edge1 = Vector3Subtract(vertices[i1], vertices[i0]);
        Vector3 edge2 = Vector3Subtract(vertices[i2], vertices[i0]);
        Vector3 n = Vector3Normalize(Vector3CrossProduct(edge1, edge2));
        normals[i0] = Vector3Add(normals[i0], n);
        normals[i1] = Vector3Add(normals[i1], n);
        normals[i2] = Vector3Add(normals[i2], n);
    }

    const Vector3 lightDir = Vector3Normalize((Vector3) { 0.35f, 1.0f, 0.45f });
    for (int i = 0; i < vertexCount; i++) {
        normals[i] = Vector3Normalize(normals[i]);
        float slope = 1.0f - Clamp(Vector3DotProduct(normals[i], (Vector3) { 0.0f, 1.0f, 0.0f }), 0.0f, 1.0f);
        float shade = Clamp(Vector3DotProduct(normals[i], lightDir), 0.0f, 1.0f);
        float hNorm = NormalizeRange(vertices[i].y, terrain->minTerrainY, terrain->maxTerrainY);
        float rockThreshold = (settings != NULL) ? settings->rockThreshold : 0.58f;
        colors[i] = TerrainColor(hNorm, slope, shade, rockThreshold);
    }

    Mesh mesh = { 0 };
    mesh.vertexCount = vertexCount;
    mesh.triangleCount = indexCount / 3;
    mesh.vertices = (float *)MemAlloc((size_t)vertexCount * 3 * sizeof(float));
    mesh.normals = (float *)MemAlloc((size_t)vertexCount * 3 * sizeof(float));
    mesh.colors = (unsigned char *)MemAlloc((size_t)vertexCount * 4 * sizeof(unsigned char));
    mesh.indices = (unsigned short *)MemAlloc((size_t)indexCount * sizeof(unsigned short));

    if (!mesh.vertices || !mesh.normals || !mesh.colors || !mesh.indices) {
        if (mesh.vertices) MemFree(mesh.vertices);
        if (mesh.normals) MemFree(mesh.normals);
        if (mesh.colors) MemFree(mesh.colors);
        if (mesh.indices) MemFree(mesh.indices);
        MemFree(vertices);
        MemFree(normals);
        MemFree(colors);
        MemFree(indices);
        return (Model) { 0 };
    }

    for (int i = 0; i < vertexCount; i++) {
        mesh.vertices[i * 3 + 0] = vertices[i].x;
        mesh.vertices[i * 3 + 1] = vertices[i].y;
        mesh.vertices[i * 3 + 2] = vertices[i].z;
        mesh.normals[i * 3 + 0] = normals[i].x;
        mesh.normals[i * 3 + 1] = normals[i].y;
        mesh.normals[i * 3 + 2] = normals[i].z;
        mesh.colors[i * 4 + 0] = colors[i].r;
        mesh.colors[i * 4 + 1] = colors[i].g;
        mesh.colors[i * 4 + 2] = colors[i].b;
        mesh.colors[i * 4 + 3] = colors[i].a;
    }
    for (int i = 0; i < indexCount; i++) mesh.indices[i] = (unsigned short)indices[i];

    UploadMesh(&mesh, true);

    MemFree(vertices);
    MemFree(normals);
    MemFree(colors);
    MemFree(indices);
    return LoadModelFromMesh(mesh);
}

static float BaseSeaLevelTemp(float simSeconds) {
    float dayProgress = fmodf(simSeconds, DAY_SECONDS) / DAY_SECONDS;
    float solar = fmaxf(0.0f, sinf(TAU * (dayProgress - 0.25f)));
    return 6.0f + 15.0f * solar;
}

static float BasePressureAtAltitude(float altitudeMeters) {
    return 1013.25f * expf(-altitudeMeters / 8300.0f);
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

static float BackgroundRelativeHumidity(float simSeconds, float altNorm) {
    float dayProgress = fmodf(simSeconds, DAY_SECONDS) / DAY_SECONDS;
    float diurnal = -0.05f * sinf(TAU * (dayProgress - 0.18f));
    float boundaryLayer = 0.80f - 0.30f * altNorm;
    float midLevelMoisture = 0.10f * expf(-powf((altNorm - 0.45f) / 0.22f, 2.0f));
    return Clamp(boundaryLayer + midLevelMoisture + diurnal, 0.20f, 0.97f);
}

static float BackgroundMixingRatio(float simSeconds, float altNorm, float tempC, float pressureHpa) {
    return SaturationMixingRatio(tempC, pressureHpa) * BackgroundRelativeHumidity(simSeconds, altNorm);
}

static float VirtualTemperatureK(float tempC, float vaporMixingRatio, float cloudMixingRatio, float rainMixingRatio) {
    float tempK = tempC + 273.15f;
    float condensed = fmaxf(cloudMixingRatio + rainMixingRatio, 0.0f);
    return tempK * (1.0f + 0.61f * vaporMixingRatio - condensed);
}

static float VerticalSecondDerivative(const AtmosphereSim *sim, const float *field, int x, int y, int z) {
    int i = AtmosIndex(sim, x, y, z);
    float center = field[i];

    float below = center;
    float above = center;
    if (y > 0 && IsFluidCell(sim, x, y - 1, z)) below = field[AtmosIndex(sim, x, y - 1, z)];
    if (y + 1 < sim->ny && IsFluidCell(sim, x, y + 1, z)) above = field[AtmosIndex(sim, x, y + 1, z)];

    float dy2 = sim->dyMeters * sim->dyMeters;
    return (above - 2.0f * center + below) / fmaxf(dy2, 1.0f);
}

static float PressureAnomalySample(const AtmosphereSim *sim, const float *pressureField, int x, int y, int z) {
    if (x < 0 || x >= sim->nx || y < 0 || y >= sim->ny || z < 0 || z >= sim->nz) return 0.0f;
    if (!IsFluidCell(sim, x, y, z)) return 0.0f;
    return pressureField[AtmosIndex(sim, x, y, z)] - BasePressureAtAltitude(CellAltitudeMeters(sim, y));
}

static ImVec2_c ImVec2Make(float x, float y) {
    ImVec2_c value = { x, y };
    return value;
}

static ImVec4_c ImVec4Make(float x, float y, float z, float w) {
    ImVec4_c value = { x, y, z, w };
    return value;
}

static Rectangle GetControlPanelRect(void) {
    const float width = 434.0f;
    const float margin = 12.0f;
    return (Rectangle) { GetScreenWidth() - width - margin, margin, width, GetScreenHeight() - margin * 2.0f };
}

static Rectangle GetUiToggleRect(void) {
    const float margin = 12.0f;
    return (Rectangle) { GetScreenWidth() - 124.0f - margin, margin, 124.0f, 36.0f };
}

static void ApplyImGuiTheme(void) {
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

static void DestroySim(AtmosphereSim *sim) {
    if (sim->terrainModel.meshCount > 0) UnloadModel(sim->terrainModel);
    if (sim->horizontalSliceModel.meshCount > 0) UnloadModel(sim->horizontalSliceModel);
    if (sim->verticalSliceModelX.meshCount > 0) UnloadModel(sim->verticalSliceModelX);
    if (sim->verticalSliceModelZ.meshCount > 0) UnloadModel(sim->verticalSliceModelZ);
    if (sim->cloudVolumeModel.meshCount > 0) UnloadModel(sim->cloudVolumeModel);
    if (sim->horizontalTex.id != 0) UnloadTexture(sim->horizontalTex);
    if (sim->verticalTex.id != 0) UnloadTexture(sim->verticalTex);
    if (sim->volumeFieldTex.id != 0) UnloadTexture(sim->volumeFieldTex);
    if (sim->cloudTerrainTex.id != 0) UnloadTexture(sim->cloudTerrainTex);
    if (sim->cloudVolumeShader.id != 0) UnloadShader(sim->cloudVolumeShader);
    if (sim->skyShader.id != 0) UnloadShader(sim->skyShader);
    UnloadTerrainMaterialResources(sim);

    free(sim->columnTerrainMeters);
    free(sim->columnTerrainWorldY);
    free(sim->columnDx);
    free(sim->columnDz);
    free(sim->columnSlope);
    free(sim->surfaceTemp);
    free(sim->surfaceRain);
    free(sim->groundLayer);

    free(sim->temp);
    free(sim->pressure);
    free(sim->vapor);
    free(sim->cloud);
    free(sim->rain);
    free(sim->u);
    free(sim->v);
    free(sim->w);
    free(sim->buoyancy);
    free(sim->sunlight);
    free(sim->divergence);
    free(sim->pressureSolve);
    free(sim->pressureScratch);

    free(sim->tempNext);
    free(sim->pressureNext);
    free(sim->vaporNext);
    free(sim->cloudNext);
    free(sim->rainNext);
    free(sim->uNext);
    free(sim->vNext);
    free(sim->wNext);

    free(sim->horizontalPixels);
    free(sim->verticalPixels);
    free(sim->volumeFieldPixels);
    FreeTerrain(&sim->terrain);
    memset(sim, 0, sizeof(*sim));
}

static bool AllocateSim(AtmosphereSim *sim) {
    sim->cells2D = sim->nx * sim->nz;
    sim->cells3D = sim->cells2D * sim->ny;

    sim->columnTerrainMeters = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->columnTerrainWorldY = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->columnDx = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->columnDz = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->columnSlope = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->surfaceTemp = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->surfaceRain = (float *)calloc((size_t)sim->cells2D, sizeof(float));
    sim->groundLayer = (int *)calloc((size_t)sim->cells2D, sizeof(int));

    sim->temp = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->pressure = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->vapor = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->cloud = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->rain = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->u = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->v = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->w = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->buoyancy = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->sunlight = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->divergence = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->pressureSolve = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->pressureScratch = (float *)calloc((size_t)sim->cells3D, sizeof(float));

    sim->tempNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->pressureNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->vaporNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->cloudNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->rainNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->uNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->vNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));
    sim->wNext = (float *)calloc((size_t)sim->cells3D, sizeof(float));

    int horizontalTexWidth = sim->nx * SLICE_TEX_SCALE;
    int horizontalTexHeight = sim->nz * SLICE_TEX_SCALE;
    int verticalTexWidth = ((sim->nx > sim->nz) ? sim->nx : sim->nz) * SLICE_TEX_SCALE;
    int verticalTexHeight = sim->ny * SLICE_TEX_SCALE;

    sim->horizontalPixels = (Color *)calloc((size_t)horizontalTexWidth * (size_t)horizontalTexHeight, sizeof(Color));
    sim->verticalPixels = (Color *)calloc((size_t)verticalTexWidth * (size_t)verticalTexHeight, sizeof(Color));

    if (!sim->columnTerrainMeters || !sim->columnTerrainWorldY || !sim->columnDx || !sim->columnDz ||
        !sim->columnSlope || !sim->surfaceTemp || !sim->surfaceRain || !sim->groundLayer ||
        !sim->temp || !sim->pressure || !sim->vapor || !sim->cloud || !sim->rain || !sim->u ||
        !sim->v || !sim->w || !sim->buoyancy || !sim->sunlight || !sim->divergence || !sim->pressureSolve ||
        !sim->pressureScratch || !sim->tempNext || !sim->pressureNext || !sim->vaporNext ||
        !sim->cloudNext || !sim->rainNext || !sim->uNext || !sim->vNext || !sim->wNext ||
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

    if (!SetupCloudVolumeResources(sim)) {
        return false;
    }
    if (!SetupSkyShaderResources(sim)) {
        return false;
    }

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
        .showClouds = sim->showClouds,
        .showVolume = sim->showVolume,
        .paused = sim->paused,
        .showUI = sim->showUI,
        .terrainAlpha = sim->terrainAlpha,
        .cloudRenderAlpha = sim->cloudRenderAlpha,
        .cloudRenderThreshold = sim->cloudRenderThreshold,
        .cloudDensityScale = sim->cloudDensityScale,
        .timeScale = sim->timeScale,
        .volumeThreshold = sim->volumeThreshold,
        .volumeAlpha = sim->volumeAlpha,
        .volumeStride = sim->volumeStride,
        .activeControlTab = sim->activeControlTab
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
    sim->showClouds = settings->showClouds;
    sim->showVolume = settings->showVolume;
    sim->paused = settings->paused;
    sim->showUI = settings->showUI;
    sim->terrainAlpha = settings->terrainAlpha;
    sim->cloudRenderAlpha = settings->cloudRenderAlpha;
    sim->cloudRenderThreshold = settings->cloudRenderThreshold;
    sim->cloudDensityScale = settings->cloudDensityScale;
    sim->timeScale = settings->timeScale;
    sim->volumeThreshold = settings->volumeThreshold;
    sim->volumeAlpha = settings->volumeAlpha;
    sim->volumeStride = settings->volumeStride;
    sim->activeControlTab = settings->activeControlTab;
}

static void ResetAtmosphere(AtmosphereSim *sim) {
    sim->simSeconds = 11.0f * 3600.0f;
    sim->timeScale = DEFAULT_TIME_SCALE;
    sim->prevailingU = 7.5f;
    sim->prevailingV = 2.5f;
    sim->terrainAlpha = 1.0f;
    sim->fieldMode = FIELD_TEMPERATURE;
    sim->verticalAxis = SLICE_Z;
    sim->horizontalLayer = sim->ny / 2;
    sim->verticalSlice = sim->nx / 2;
    sim->showHorizontalSlice = true;
    sim->showVerticalSlice = true;
    sim->showTerrain = true;
    sim->showWireframe = false;
    sim->showWindArrows = true;
    sim->showClouds = false;
    sim->showVolume = false;
    sim->paused = false;
    sim->showUI = true;
    sim->cloudRenderAlpha = 0.64f;
    sim->cloudRenderThreshold = 0.08f;
    sim->cloudDensityScale = 1.10f;
    sim->volumeThreshold = 0.42f;
    sim->volumeAlpha = 0.42f;
    sim->volumeStride = 5;
    sim->frameCounter = 0;
    sim->sunDir = Vector3Normalize((Vector3) { 0.2f, 1.0f, 0.3f });
    sim->solarStrength = 0.0f;

    float seaLevelTemp = BaseSeaLevelTemp(sim->simSeconds);

    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            int column = ColumnIndex(sim, x, z);
            float terrainMeters = sim->columnTerrainMeters[column];
            float terrainNorm = NormalizeRange(terrainMeters, sim->terrain.minTerrainMeters, sim->terrain.maxTerrainMeters);
            float slope = Clamp(sim->columnSlope[column] * 2.2f, 0.0f, 1.0f);
            float noiseA = NoiseCoord((float)x, (float)z, 7.0f) - 0.5f;
            sim->surfaceTemp[column] = seaLevelTemp - 0.0063f * terrainMeters + 2.2f * (1.0f - terrainNorm) - slope * 0.8f + noiseA * 1.6f;
            sim->surfaceRain[column] = 0.0f;

            for (int y = 0; y < sim->ny; y++) {
                int i = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) {
                    sim->temp[i] = 0.0f;
                    sim->pressure[i] = 0.0f;
                    sim->vapor[i] = 0.0f;
                    sim->cloud[i] = 0.0f;
                    sim->rain[i] = 0.0f;
                    sim->u[i] = 0.0f;
                    sim->v[i] = 0.0f;
                    sim->w[i] = 0.0f;
                    sim->buoyancy[i] = 0.0f;
                    sim->sunlight[i] = 0.0f;
                    continue;
                }

                float altitude = CellAltitudeMeters(sim, y);
                float altNorm = altitude / sim->topMeters;
                float baseTemp = seaLevelTemp - 0.0065f * altitude;
                float basePressure = BasePressureAtAltitude(altitude);
                float humidityFactor = 1.0f + (NoiseCoord((float)x, (float)y, (float)z) - 0.5f) * 0.18f;
                float qv = Clamp(BackgroundMixingRatio(sim->simSeconds, altNorm, baseTemp, basePressure) * humidityFactor,
                                 0.0002f, SaturationMixingRatio(baseTemp, basePressure) * 0.995f);

                sim->temp[i] = baseTemp + (NoiseCoord((float)x, (float)y * 1.7f, (float)z * 0.8f) - 0.5f) * 1.1f;
                sim->pressure[i] = basePressure + (NoiseCoord((float)x, (float)y, (float)z * 1.9f) - 0.5f) * 0.8f;
                sim->vapor[i] = qv;
                sim->cloud[i] = 0.0f;
                sim->rain[i] = 0.0f;
                sim->u[i] = sim->prevailingU * (0.76f + 0.24f * altNorm) + sim->columnDz[column] * 18.0f;
                sim->v[i] = sim->prevailingV * (0.76f + 0.24f * altNorm) - sim->columnDx[column] * 18.0f;
                sim->w[i] = 0.0f;
                sim->buoyancy[i] = 0.0f;
                sim->sunlight[i] = 0.0f;
            }
        }
    }

    if (sim->volumeFieldTex.id != 0) UpdateVolumeFieldTexture(sim);
}

static void UpdateStats(AtmosphereSim *sim) {
    double tempSum = 0.0;
    double pressureSum = 0.0;
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
                if (sim->cloud[i] > maxCloud) maxCloud = sim->cloud[i];
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
}

static float NeighborAverage3D(const AtmosphereSim *sim, const float *field, int x, int y, int z) {
    int xm = (x > 0) ? x - 1 : x;
    int xp = (x + 1 < sim->nx) ? x + 1 : x;
    int ym = (y > 0) ? y - 1 : y;
    int yp = (y + 1 < sim->ny) ? y + 1 : y;
    int zm = (z > 0) ? z - 1 : z;
    int zp = (z + 1 < sim->nz) ? z + 1 : z;

    float sum = 0.0f;
    int count = 0;

    if (IsFluidCell(sim, xm, y, z)) { sum += field[AtmosIndex(sim, xm, y, z)]; count++; }
    if (IsFluidCell(sim, xp, y, z)) { sum += field[AtmosIndex(sim, xp, y, z)]; count++; }
    if (IsFluidCell(sim, x, ym, z)) { sum += field[AtmosIndex(sim, x, ym, z)]; count++; }
    if (IsFluidCell(sim, x, yp, z)) { sum += field[AtmosIndex(sim, x, yp, z)]; count++; }
    if (IsFluidCell(sim, x, y, zm)) { sum += field[AtmosIndex(sim, x, y, zm)]; count++; }
    if (IsFluidCell(sim, x, y, zp)) { sum += field[AtmosIndex(sim, x, y, zp)]; count++; }

    if (count == 0) return field[AtmosIndex(sim, x, y, z)];
    return sum / (float)count;
}

static float FluidFieldSample(const AtmosphereSim *sim, const float *field, int x, int y, int z) {
    if (x < 0 || x >= sim->nx || y < 0 || y >= sim->ny || z < 0 || z >= sim->nz) return 0.0f;
    if (!IsFluidCell(sim, x, y, z)) return 0.0f;
    return field[AtmosIndex(sim, x, y, z)];
}

static void AdvectScalar3D(const AtmosphereSim *sim, const float *src, const float *uField, const float *vField,
                           const float *wField, float terminalFall, float dt, float *dst) {
    float xScale = dt / sim->dxMeters;
    float zScale = dt / sim->dzMeters;
    float yScale = dt / sim->dyMeters;

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int i = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) {
                    dst[i] = 0.0f;
                    continue;
                }

                float sx = (float)x - uField[i] * xScale;
                float sy = (float)y - (wField[i] - terminalFall) * yScale;
                float sz = (float)z - vField[i] * zScale;
                dst[i] = SampleAtmosField(sim, src, sx, sy, sz);
            }
        }
    }
}

static void UpdateSurfaceHeating(AtmosphereSim *sim, float dt, float solar, Vector3 sunDir) {
    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            int column = ColumnIndex(sim, x, z);
            float terrainAlt = sim->columnTerrainMeters[column];
            Vector3 normal = Vector3Normalize((Vector3) {
                -sim->columnDx[column],
                1.0f,
                -sim->columnDz[column]
            });
            float exposure = fmaxf(0.0f, Vector3DotProduct(normal, sunDir));
            float terrainNorm = NormalizeRange(terrainAlt, sim->terrain.minTerrainMeters, sim->terrain.maxTerrainMeters);
            float ambient = BaseSeaLevelTemp(sim->simSeconds) - 0.0063f * terrainAlt;
            float lowCloud = 0.0f;
            int samples = 0;

            for (int y = sim->groundLayer[column] + 1; y <= sim->groundLayer[column] + 2; y++) {
                if (y >= 0 && y < sim->ny) {
                    lowCloud += sim->cloud[AtmosIndex(sim, x, y, z)];
                    samples++;
                }
            }
            if (samples > 0) lowCloud /= (float)samples;

            float heating = (0.0040f + solar * 0.013f) * (0.28f + 0.72f * exposure) * (1.0f - Clamp(lowCloud * 280.0f, 0.0f, 0.82f));
            float cooling = 0.0020f + (1.0f - solar) * 0.0060f + terrainNorm * 0.0014f;
            sim->surfaceTemp[column] += dt * (heating - cooling - 0.010f * (sim->surfaceTemp[column] - ambient));
            sim->surfaceTemp[column] = Clamp(sim->surfaceTemp[column], -28.0f, 38.0f);
            sim->surfaceRain[column] = 0.0f;
        }
    }
}

static void ApplyRadiationPass(AtmosphereSim *sim, float dt, float solar, Vector3 sunDir) {
    sim->sunDir = sunDir;
    sim->solarStrength = solar;

    float sunShiftX = -sunDir.x * 0.50f;
    float sunShiftZ = -sunDir.z * 0.50f;

    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            float incoming = solar;
            float sampleX = (float)x;
            float sampleZ = (float)z;

            for (int y = sim->ny - 1; y >= 0; y--) {
                if (!IsFluidCell(sim, x, y, z)) {
                    if (y >= 0) sim->sunlight[AtmosIndex(sim, x, y, z)] = 0.0f;
                    sampleX += sunShiftX;
                    sampleZ += sunShiftZ;
                    continue;
                }

                int sx = Clamp((int)lroundf(sampleX), 0, sim->nx - 1);
                int sz = Clamp((int)lroundf(sampleZ), 0, sim->nz - 1);
                int i = AtmosIndex(sim, x, y, z);
                int si = AtmosIndex(sim, sx, y, sz);

                float opticalDepth = sim->cloudNext[si] * 210.0f + sim->rainNext[si] * 135.0f + sim->vaporNext[si] * 8.0f;
                float transmission = expf(-opticalDepth * sim->dyMeters * 0.00018f);
                transmission = Clamp(transmission, 0.02f, 1.0f);

                float absorbedShortwave = incoming * (1.0f - transmission);
                float humidity = RelativeHumidity(sim->tempNext[i], sim->pressureNext[i], sim->vaporNext[i]) * 0.01f;
                float greenhouse = Clamp(sim->cloudNext[i] * 190.0f + sim->vaporNext[i] * 28.0f, 0.0f, 0.96f);
                float altitude = CellAltitudeMeters(sim, y);
                float altitudeNorm = altitude / sim->topMeters;

                float shortwaveHeating = absorbedShortwave * dt * (0.82f + 0.18f * humidity) * (0.75f + 0.25f * (1.0f - altitudeNorm));
                float longwaveCooling = dt * (0.0010f + 0.0019f * (1.0f - greenhouse) + 0.0006f * altitudeNorm);
                sim->tempNext[i] += shortwaveHeating - longwaveCooling;
                sim->sunlight[i] = incoming;

                incoming *= transmission;
                incoming += absorbedShortwave * 0.035f;
                sampleX += sunShiftX;
                sampleZ += sunShiftZ;
            }
        }
    }
}

static void ProjectVelocityField(AtmosphereSim *sim, float dt) {
    float invDx2 = 1.0f / fmaxf(sim->dxMeters * sim->dxMeters, 1.0f);
    float invDy2 = 1.0f / fmaxf(sim->dyMeters * sim->dyMeters, 1.0f);
    float invDz2 = 1.0f / fmaxf(sim->dzMeters * sim->dzMeters, 1.0f);

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int i = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) {
                    sim->divergence[i] = 0.0f;
                    sim->pressureSolve[i] = 0.0f;
                    sim->pressureScratch[i] = 0.0f;
                    continue;
                }

                float du = (FluidFieldSample(sim, sim->uNext, x + 1, y, z) - FluidFieldSample(sim, sim->uNext, x - 1, y, z)) / (2.0f * sim->dxMeters);
                float dv = (FluidFieldSample(sim, sim->vNext, x, y, z + 1) - FluidFieldSample(sim, sim->vNext, x, y, z - 1)) / (2.0f * sim->dzMeters);
                float dw = (FluidFieldSample(sim, sim->wNext, x, y + 1, z) - FluidFieldSample(sim, sim->wNext, x, y - 1, z)) / (2.0f * sim->dyMeters);
                sim->divergence[i] = du + dv + dw;
                sim->pressureSolve[i] = 0.0f;
                sim->pressureScratch[i] = 0.0f;
            }
        }
    }

    float *solveA = sim->pressureSolve;
    float *solveB = sim->pressureScratch;
    for (int iter = 0; iter < 8; iter++) {
        for (int z = 0; z < sim->nz; z++) {
            for (int y = 0; y < sim->ny; y++) {
                for (int x = 0; x < sim->nx; x++) {
                    int i = AtmosIndex(sim, x, y, z);
                    if (!IsFluidCell(sim, x, y, z)) {
                        solveB[i] = 0.0f;
                        continue;
                    }

                    float sum = 0.0f;
                    float denom = 0.0f;
                    if (x > 0 && IsFluidCell(sim, x - 1, y, z)) {
                        sum += solveA[AtmosIndex(sim, x - 1, y, z)] * invDx2;
                        denom += invDx2;
                    }
                    if (x + 1 < sim->nx && IsFluidCell(sim, x + 1, y, z)) {
                        sum += solveA[AtmosIndex(sim, x + 1, y, z)] * invDx2;
                        denom += invDx2;
                    }
                    if (y > 0 && IsFluidCell(sim, x, y - 1, z)) {
                        sum += solveA[AtmosIndex(sim, x, y - 1, z)] * invDy2;
                        denom += invDy2;
                    }
                    if (y + 1 < sim->ny && IsFluidCell(sim, x, y + 1, z)) {
                        sum += solveA[AtmosIndex(sim, x, y + 1, z)] * invDy2;
                        denom += invDy2;
                    }
                    if (z > 0 && IsFluidCell(sim, x, y, z - 1)) {
                        sum += solveA[AtmosIndex(sim, x, y, z - 1)] * invDz2;
                        denom += invDz2;
                    }
                    if (z + 1 < sim->nz && IsFluidCell(sim, x, y, z + 1)) {
                        sum += solveA[AtmosIndex(sim, x, y, z + 1)] * invDz2;
                        denom += invDz2;
                    }

                    if (denom <= 0.0f) {
                        solveB[i] = 0.0f;
                    } else {
                        float rhs = sim->divergence[i] / fmaxf(dt, 0.2f);
                        solveB[i] = (sum - rhs) / denom;
                    }
                }
            }
        }

        float *tmp = solveA;
        solveA = solveB;
        solveB = tmp;
    }

    if (solveA != sim->pressureSolve) memcpy(sim->pressureSolve, solveA, (size_t)sim->cells3D * sizeof(float));

    const float projectionStrength = 0.38f;
    const float verticalStrength = 0.32f;
    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                int i = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) continue;

                float gradPx = (FluidFieldSample(sim, sim->pressureSolve, x + 1, y, z) - FluidFieldSample(sim, sim->pressureSolve, x - 1, y, z)) / (2.0f * sim->dxMeters);
                float gradPz = (FluidFieldSample(sim, sim->pressureSolve, x, y, z + 1) - FluidFieldSample(sim, sim->pressureSolve, x, y, z - 1)) / (2.0f * sim->dzMeters);
                float gradPy = (FluidFieldSample(sim, sim->pressureSolve, x, y + 1, z) - FluidFieldSample(sim, sim->pressureSolve, x, y - 1, z)) / (2.0f * sim->dyMeters);

                sim->uNext[i] -= projectionStrength * dt * gradPx;
                sim->vNext[i] -= projectionStrength * dt * gradPz;
                sim->wNext[i] -= verticalStrength * dt * gradPy;

                float du = (FluidFieldSample(sim, sim->uNext, x + 1, y, z) - FluidFieldSample(sim, sim->uNext, x - 1, y, z)) / (2.0f * sim->dxMeters);
                float dv = (FluidFieldSample(sim, sim->vNext, x, y, z + 1) - FluidFieldSample(sim, sim->vNext, x, y, z - 1)) / (2.0f * sim->dzMeters);
                float dw = (FluidFieldSample(sim, sim->wNext, x, y + 1, z) - FluidFieldSample(sim, sim->wNext, x, y - 1, z)) / (2.0f * sim->dyMeters);
                sim->divergence[i] = du + dv + dw;
            }
        }
    }
}

static void StepAtmosphere(AtmosphereSim *sim, float dt) {
    float dayProgress = fmodf(sim->simSeconds, DAY_SECONDS) / DAY_SECONDS;
    float solar = fmaxf(0.0f, sinf(TAU * (dayProgress - 0.25f)));
    Vector3 sunDir = Vector3Normalize((Vector3) {
        cosf(TAU * (dayProgress - 0.25f)),
        0.25f + 0.95f * solar,
        0.35f + 0.55f * sinf(TAU * (dayProgress - 0.12f))
    });
    sim->sunDir = sunDir;
    sim->solarStrength = solar;

    float synopticPhase = sim->simSeconds / (DAY_SECONDS * 1.2f);
    float synopticDir = 0.35f + 0.65f * sinf(synopticPhase * 0.85f);
    float synopticSpeed = 7.0f + 3.0f * sinf(synopticPhase + 1.3f);
    sim->prevailingU = cosf(synopticDir) * synopticSpeed;
    sim->prevailingV = sinf(synopticDir) * synopticSpeed * 0.75f;

    UpdateSurfaceHeating(sim, dt, solar, sunDir);

    AdvectScalar3D(sim, sim->temp, sim->u, sim->v, sim->w, 0.0f, dt, sim->tempNext);
    AdvectScalar3D(sim, sim->vapor, sim->u, sim->v, sim->w, 0.0f, dt, sim->vaporNext);
    AdvectScalar3D(sim, sim->cloud, sim->u, sim->v, sim->w, 0.0f, dt, sim->cloudNext);
    AdvectScalar3D(sim, sim->rain, sim->u, sim->v, sim->w, 6.0f, dt, sim->rainNext);
    AdvectScalar3D(sim, sim->u, sim->u, sim->v, sim->w, 0.0f, dt, sim->uNext);
    AdvectScalar3D(sim, sim->v, sim->u, sim->v, sim->w, 0.0f, dt, sim->vNext);
    AdvectScalar3D(sim, sim->w, sim->u, sim->v, sim->w, 0.0f, dt, sim->wNext);
    memcpy(sim->pressureNext, sim->pressure, (size_t)sim->cells3D * sizeof(float));

    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            int column = ColumnIndex(sim, x, z);
            int ground = sim->groundLayer[column];
            float terrainAlt = sim->columnTerrainMeters[column];
            float terrainNorm = NormalizeRange(terrainAlt, sim->terrain.minTerrainMeters, sim->terrain.maxTerrainMeters);

            for (int y = 0; y < sim->ny; y++) {
                int i = AtmosIndex(sim, x, y, z);

                if (!IsFluidCell(sim, x, y, z)) {
                    sim->tempNext[i] = 0.0f;
                    sim->pressureNext[i] = 0.0f;
                    sim->vaporNext[i] = 0.0f;
                    sim->cloudNext[i] = 0.0f;
                    sim->rainNext[i] = 0.0f;
                    sim->uNext[i] = 0.0f;
                    sim->vNext[i] = 0.0f;
                    sim->wNext[i] = 0.0f;
                    sim->buoyancy[i] = 0.0f;
                    sim->sunlight[i] = 0.0f;
                    sim->divergence[i] = 0.0f;
                    continue;
                }

                float altitude = CellAltitudeMeters(sim, y);
                float altNorm = altitude / sim->topMeters;
                float backgroundTemp = BaseSeaLevelTemp(sim->simSeconds) - 0.0065f * altitude;
                float basePressure = BasePressureAtAltitude(altitude);
                float envVapor = BackgroundMixingRatio(sim->simSeconds, altNorm, backgroundTemp, basePressure);
                float targetU = sim->prevailingU * (0.68f + 0.32f * altNorm);
                float targetV = sim->prevailingV * (0.68f + 0.32f * altNorm);
                float edgeDist = (float)x;
                if ((float)(sim->nx - 1 - x) < edgeDist) edgeDist = (float)(sim->nx - 1 - x);
                if ((float)z < edgeDist) edgeDist = (float)z;
                if ((float)(sim->nz - 1 - z) < edgeDist) edgeDist = (float)(sim->nz - 1 - z);
                float edgeBlend = Clamp((4.0f - edgeDist) / 4.0f, 0.0f, 1.0f);
                float topBlend = Clamp(((float)y - (float)(sim->ny - 4)) / 3.0f, 0.0f, 1.0f);
                float localLayer = (float)(y - ground);

                if (localLayer > 0.0f && localLayer <= 2.5f) {
                    float nearGroundWind = sqrtf(sim->uNext[i] * sim->uNext[i] + sim->vNext[i] * sim->vNext[i]);
                    float exchange = Clamp(dt * (0.015f + 0.012f * sqrtf(nearGroundWind + 0.5f)) / localLayer, 0.0f, 0.60f);
                    float qsatSurface = SaturationMixingRatio(sim->surfaceTemp[column], sim->pressureNext[i]);
                    float surfaceRH = Clamp(0.72f + 0.12f * (1.0f - solar) + 0.08f * (1.0f - terrainNorm), 0.45f, 0.96f);
                    float targetVapor = Clamp(qsatSurface * surfaceRH, 0.0002f, 0.024f);
                    sim->tempNext[i] = Lerp(sim->tempNext[i], sim->surfaceTemp[column], exchange);
                    sim->vaporNext[i] = Lerp(sim->vaporNext[i], targetVapor, exchange * (0.86f - 0.18f * terrainNorm));

                    float terrainLift = sim->uNext[i] * sim->columnDx[column] + sim->vNext[i] * sim->columnDz[column];
                    float terrainLiftGain = (fmaxf(terrainLift, 0.0f) * 1.10f) - (fmaxf(-terrainLift, 0.0f) * 0.35f);
                    sim->wNext[i] += dt * terrainLiftGain / localLayer;

                    float slopeFlow = Clamp(sim->surfaceTemp[column] - backgroundTemp, -8.0f, 8.0f);
                    sim->uNext[i] += dt * (-sim->columnDx[column] * slopeFlow * 0.015f) / localLayer;
                    sim->vNext[i] += dt * (-sim->columnDz[column] * slopeFlow * 0.015f) / localLayer;
                }

                float avgTemp = NeighborAverage3D(sim, sim->temp, x, y, z);
                float avgVapor = NeighborAverage3D(sim, sim->vapor, x, y, z);
                sim->tempNext[i] += dt * 0.007f * (avgTemp - sim->tempNext[i]);
                sim->vaporNext[i] += dt * 0.004f * (avgVapor - sim->vaporNext[i]);

                float windSpeed = sqrtf(sim->u[i] * sim->u[i] + sim->v[i] * sim->v[i]);
                float instability = Clamp((sim->surfaceTemp[column] - backgroundTemp) / 10.0f, -0.8f, 1.5f);
                float eddyDiff = 18.0f + 150.0f * (1.0f - altNorm) * (1.0f - altNorm) +
                                 95.0f * fmaxf(instability, 0.0f) + 24.0f * Clamp(windSpeed / 12.0f, 0.0f, 1.0f);
                float momentumDiff = eddyDiff * 0.62f;
                float condensateDiff = eddyDiff * 0.40f;
                sim->tempNext[i] += dt * eddyDiff * VerticalSecondDerivative(sim, sim->temp, x, y, z);
                sim->vaporNext[i] += dt * (eddyDiff * 0.82f) * VerticalSecondDerivative(sim, sim->vapor, x, y, z);
                sim->cloudNext[i] += dt * condensateDiff * VerticalSecondDerivative(sim, sim->cloud, x, y, z);
                sim->rainNext[i] += dt * (condensateDiff * 0.20f) * VerticalSecondDerivative(sim, sim->rain, x, y, z);
                sim->uNext[i] += dt * momentumDiff * VerticalSecondDerivative(sim, sim->u, x, y, z);
                sim->vNext[i] += dt * momentumDiff * VerticalSecondDerivative(sim, sim->v, x, y, z);
                sim->wNext[i] += dt * (momentumDiff * 0.55f) * VerticalSecondDerivative(sim, sim->w, x, y, z);

                float humidBlend = Clamp(RelativeHumidity(sim->tempNext[i], fmaxf(sim->pressureNext[i], 600.0f), sim->vaporNext[i]) * 0.01f +
                                         sim->cloudNext[i] * 140.0f, 0.0f, 1.0f);
                float adiabaticRate = Lerp(0.0098f, 0.0060f, humidBlend);
                sim->tempNext[i] -= dt * adiabaticRate * sim->wNext[i];

                float parcelTv = VirtualTemperatureK(sim->tempNext[i], sim->vaporNext[i], sim->cloudNext[i], sim->rainNext[i]);
                float envTv = VirtualTemperatureK(backgroundTemp, envVapor, 0.0f, 0.0f);
                float buoyancy = 9.81f * (parcelTv - envTv) / fmaxf(envTv, 180.0f);
                buoyancy = Clamp(buoyancy, -1.5f, 1.5f);
                sim->buoyancy[i] = buoyancy;
                sim->wNext[i] += dt * (buoyancy - (0.10f + 0.12f * topBlend) * sim->wNext[i]);

                int xm = (x > 0) ? x - 1 : x;
                int xp = (x + 1 < sim->nx) ? x + 1 : x;
                int ym = (y > 0) ? y - 1 : y;
                int yp = (y + 1 < sim->ny) ? y + 1 : y;
                int zm = (z > 0) ? z - 1 : z;
                int zp = (z + 1 < sim->nz) ? z + 1 : z;

                float div = (sim->uNext[AtmosIndex(sim, xp, y, z)] - sim->uNext[AtmosIndex(sim, xm, y, z)]) / (2.0f * sim->dxMeters) +
                            (sim->vNext[AtmosIndex(sim, x, y, zp)] - sim->vNext[AtmosIndex(sim, x, y, zm)]) / (2.0f * sim->dzMeters) +
                            (sim->wNext[AtmosIndex(sim, x, yp, z)] - sim->wNext[AtmosIndex(sim, x, ym, z)]) / (2.0f * sim->dyMeters);
                sim->divergence[i] = div;
                float pressureAnomaly = sim->pressureNext[i] - basePressure;
                sim->pressureNext[i] += dt * (-220.0f * div - 0.055f * pressureAnomaly);
                sim->pressureNext[i] = Clamp(sim->pressureNext[i], 520.0f, 1025.0f);

                float gradPx = (PressureAnomalySample(sim, sim->pressureNext, xp, y, z) -
                                PressureAnomalySample(sim, sim->pressureNext, xm, y, z)) / (2.0f * sim->dxMeters);
                float gradPz = (PressureAnomalySample(sim, sim->pressureNext, x, y, zp) -
                                PressureAnomalySample(sim, sim->pressureNext, x, y, zm)) / (2.0f * sim->dzMeters);
                float gradPy = (PressureAnomalySample(sim, sim->pressureNext, x, yp, z) -
                                PressureAnomalySample(sim, sim->pressureNext, x, ym, z)) / (2.0f * sim->dyMeters);
                float density = Clamp((sim->pressureNext[i] * 100.0f) / (287.05f * fmaxf(parcelTv, 180.0f)), 0.55f, 1.40f);
                float drag = 0.016f + 0.020f * Clamp(sim->columnSlope[column] * 2.2f, 0.0f, 1.0f) + 0.018f * topBlend;
                float geostrophicRelax = 0.008f + 0.008f * edgeBlend;
                sim->uNext[i] += dt * (-(100.0f / density) * gradPx - drag * sim->uNext[i] + geostrophicRelax * (targetU - sim->uNext[i]));
                sim->vNext[i] += dt * (-(100.0f / density) * gradPz - drag * sim->vNext[i] + geostrophicRelax * (targetV - sim->vNext[i]));
                sim->wNext[i] += dt * (-(72.0f / density) * gradPy - (0.12f + 0.12f * topBlend) * sim->wNext[i]);

                float qsat = SaturationMixingRatio(sim->tempNext[i], sim->pressureNext[i]);
                float relHum = Clamp(sim->vaporNext[i] / fmaxf(qsat, 0.000001f), 0.0f, 1.60f);
                float supersat = sim->vaporNext[i] - qsat;
                if (supersat > 0.0f) {
                    float condense = supersat * Clamp(dt * (0.14f + 0.12f * Clamp(sim->wNext[i], 0.0f, 4.0f) + 0.06f * relHum), 0.0f, 0.88f);
                    sim->vaporNext[i] -= condense;
                    sim->cloudNext[i] += condense;
                    sim->tempNext[i] += condense * 2480.0f;
                } else {
                    float deficit = -supersat;
                    float dryAir = Clamp(1.0f - relHum, 0.0f, 1.0f);
                    float evapCloud = fminf(sim->cloudNext[i], deficit * Clamp(dt * (0.04f + 0.08f * dryAir), 0.0f, 0.45f));
                    float evapRain = fminf(sim->rainNext[i], deficit * Clamp(dt * (0.02f + 0.10f * dryAir + 0.04f * Clamp(-sim->wNext[i], 0.0f, 3.0f)), 0.0f, 0.35f));
                    sim->cloudNext[i] -= evapCloud;
                    sim->rainNext[i] -= evapRain;
                    sim->vaporNext[i] += evapCloud + evapRain;
                    sim->tempNext[i] -= (evapCloud + evapRain) * 2450.0f;
                }

                float cloudThreshold = 0.00035f + 0.00020f * altNorm;
                float autoRain = fmaxf(sim->cloudNext[i] - cloudThreshold, 0.0f) *
                                 Clamp(dt * (0.09f + 0.05f * Clamp(sim->rainNext[i] * 900.0f, 0.0f, 1.0f)), 0.0f, 0.40f);
                float accretion = fminf(fmaxf(sim->cloudNext[i] - autoRain, 0.0f), sim->cloudNext[i] * sim->rainNext[i] * dt * 90.0f);
                sim->cloudNext[i] -= autoRain + accretion;
                sim->rainNext[i] += autoRain + accretion;

                if ((y - ground) == 1) {
                    sim->surfaceRain[column] += sim->rainNext[i] * 850000.0f;
                }

                if (edgeBlend > 0.0f || topBlend > 0.0f) {
                    float relax = fmaxf(edgeBlend * 0.10f, topBlend * 0.18f) * dt;
                    relax = Clamp(relax, 0.0f, 0.90f);
                    sim->tempNext[i] = Lerp(sim->tempNext[i], backgroundTemp, relax);
                    sim->pressureNext[i] = Lerp(sim->pressureNext[i], basePressure, relax);
                    sim->vaporNext[i] = Lerp(sim->vaporNext[i], envVapor, relax * 0.85f);
                    sim->uNext[i] = Lerp(sim->uNext[i], targetU, relax);
                    sim->vNext[i] = Lerp(sim->vNext[i], targetV, relax);
                    sim->wNext[i] *= (1.0f - relax);
                    sim->cloudNext[i] *= (1.0f - relax * 0.35f);
                    sim->rainNext[i] *= (1.0f - relax * 0.25f);
                }

                sim->tempNext[i] = Clamp(sim->tempNext[i], -42.0f, 34.0f);
                sim->pressureNext[i] = Clamp(sim->pressureNext[i], 520.0f, 1025.0f);
                sim->vaporNext[i] = Clamp(sim->vaporNext[i], 0.00001f, 0.030f);
                sim->cloudNext[i] = Clamp(sim->cloudNext[i], 0.0f, 0.0060f);
                sim->rainNext[i] = Clamp(sim->rainNext[i], 0.0f, 0.010f);
                sim->uNext[i] = Clamp(sim->uNext[i], -35.0f, 35.0f);
                sim->vNext[i] = Clamp(sim->vNext[i], -35.0f, 35.0f);
                sim->wNext[i] = Clamp(sim->wNext[i], -10.0f, 10.0f);
            }
        }
    }

    ProjectVelocityField(sim, dt);

    for (int z = 0; z < sim->nz; z++) {
        for (int x = 0; x < sim->nx; x++) {
            float incoming = solar;
            for (int y = sim->ny - 1; y >= 0; y--) {
                int i = AtmosIndex(sim, x, y, z);
                if (!IsFluidCell(sim, x, y, z)) {
                    sim->sunlight[i] = 0.0f;
                    continue;
                }

                float altNorm = CellAltitudeMeters(sim, y) / sim->topMeters;
                float opticalDepth = sim->cloudNext[i] * 175.0f + sim->rainNext[i] * 120.0f + sim->vaporNext[i] * 4.0f;
                float transmission = Clamp(expf(-opticalDepth * sim->dyMeters * 0.00016f), 0.03f, 1.0f);
                float absorbed = incoming * (1.0f - transmission);
                float greenhouse = Clamp(sim->cloudNext[i] * 170.0f + sim->vaporNext[i] * 14.0f, 0.0f, 0.95f);

                sim->sunlight[i] = incoming;
                sim->tempNext[i] += dt * (absorbed * (0.12f + 0.06f * (1.0f - altNorm)) -
                                          (0.0007f + 0.0011f * (1.0f - greenhouse) + 0.0004f * altNorm));
                incoming *= transmission;
            }
        }
    }

    for (int i = 0; i < sim->cells3D; i++) {
        sim->tempNext[i] = Clamp(sim->tempNext[i], -45.0f, 36.0f);
        sim->pressureNext[i] = Clamp(sim->pressureNext[i], 520.0f, 1030.0f);
        sim->vaporNext[i] = Clamp(sim->vaporNext[i], 0.00001f, 0.030f);
        sim->cloudNext[i] = Clamp(sim->cloudNext[i], 0.0f, 0.0060f);
        sim->rainNext[i] = Clamp(sim->rainNext[i], 0.0f, 0.010f);
        sim->uNext[i] = Clamp(sim->uNext[i], -35.0f, 35.0f);
        sim->vNext[i] = Clamp(sim->vNext[i], -35.0f, 35.0f);
        sim->wNext[i] = Clamp(sim->wNext[i], -10.0f, 10.0f);
    }

    SwapFields(&sim->temp, &sim->tempNext);
    SwapFields(&sim->pressure, &sim->pressureNext);
    SwapFields(&sim->vapor, &sim->vaporNext);
    SwapFields(&sim->cloud, &sim->cloudNext);
    SwapFields(&sim->rain, &sim->rainNext);
    SwapFields(&sim->u, &sim->uNext);
    SwapFields(&sim->v, &sim->vNext);
    SwapFields(&sim->w, &sim->wNext);

    sim->simSeconds += dt;
}

static float EstimateStableStepSeconds(const AtmosphereSim *sim) {
    float maxU = 1.0f;
    float maxV = 1.0f;
    float maxW = 1.0f;

    for (int z = 0; z < sim->nz; z++) {
        for (int y = 0; y < sim->ny; y++) {
            for (int x = 0; x < sim->nx; x++) {
                if (!IsFluidCell(sim, x, y, z)) continue;

                int i = AtmosIndex(sim, x, y, z);
                float u = fabsf(sim->u[i]);
                float v = fabsf(sim->v[i]);
                float w = fabsf(sim->w[i]);
                if (u > maxU) maxU = u;
                if (v > maxV) maxV = v;
                if (w > maxW) maxW = w;
            }
        }
    }

    float dtX = 0.34f * sim->dxMeters / maxU;
    float dtZ = 0.34f * sim->dzMeters / maxV;
    float dtY = 0.28f * sim->dyMeters / (maxW + 6.0f);
    float stableDt = fminf(dtX, fminf(dtY, dtZ));
    return Clamp(stableDt, 0.6f, MAX_STEP_SECONDS);
}

static void UpdateAtmosphere(AtmosphereSim *sim, float frameDt) {
    float simulated = Clamp(frameDt, 0.0f, 0.05f) * sim->timeScale;
    if (simulated <= 0.00001f) {
        UpdateStats(sim);
        return;
    }

    float stableDt = EstimateStableStepSeconds(sim);
    int steps = (int)ceilf(simulated / stableDt);
    if (steps < 1) steps = 1;
    if (steps > 8) steps = 8;
    float dt = simulated / (float)steps;

    for (int i = 0; i < steps; i++) StepAtmosphere(sim, dt);
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
        case FIELD_REL_HUMIDITY: return RelativeHumidity(sim->temp[i], sim->pressure[i], sim->vapor[i]);
        case FIELD_DEWPOINT: return DewPointC(sim->pressure[i], sim->vapor[i]);
        case FIELD_VAPOR: return sim->vapor[i] * 1000.0f;
        case FIELD_CLOUD: return sim->cloud[i] * 1000.0f;
        case FIELD_RAIN: return sim->rain[i] * 1000.0f;
        case FIELD_WIND: return sqrtf(sim->u[i] * sim->u[i] + sim->v[i] * sim->v[i] + sim->w[i] * sim->w[i]);
        case FIELD_VERTICAL_WIND: return sim->w[i];
        case FIELD_BUOYANCY: return sim->buoyancy[i];
        default: return 0.0f;
    }
}

static float FieldDisplaySample(const AtmosphereSim *sim, float x, float y, float z, FieldMode mode) {
    float temp = SampleAtmosField(sim, sim->temp, x, y, z);
    float pressure = SampleAtmosField(sim, sim->pressure, x, y, z);
    float vapor = SampleAtmosField(sim, sim->vapor, x, y, z);
    float cloud = SampleAtmosField(sim, sim->cloud, x, y, z);
    float rain = SampleAtmosField(sim, sim->rain, x, y, z);
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
            return RampColor(t, kPressureStops, (int)(sizeof(kPressureStops) / sizeof(kPressureStops[0])));
        case FIELD_REL_HUMIDITY:
            return RampColor(t, kHumidityStops, (int)(sizeof(kHumidityStops) / sizeof(kHumidityStops[0])));
        case FIELD_DEWPOINT:
            return RampColor(t, kDewPointStops, (int)(sizeof(kDewPointStops) / sizeof(kDewPointStops[0])));
        case FIELD_VAPOR:
            return RampColor(t, kVaporStops, (int)(sizeof(kVaporStops) / sizeof(kVaporStops[0])));
        case FIELD_CLOUD:
            return RampColor(t, kCloudStops, (int)(sizeof(kCloudStops) / sizeof(kCloudStops[0])));
        case FIELD_RAIN:
            return RampColor(t, kRainStops, (int)(sizeof(kRainStops) / sizeof(kRainStops[0])));
        case FIELD_WIND:
            return RampColor(t, kWindStops, (int)(sizeof(kWindStops) / sizeof(kWindStops[0])));
        case FIELD_VERTICAL_WIND:
            return RampColor(t, kDivergingStops, (int)(sizeof(kDivergingStops) / sizeof(kDivergingStops[0])));
        case FIELD_BUOYANCY:
            return RampColor(t, kDivergingStops, (int)(sizeof(kDivergingStops) / sizeof(kDivergingStops[0])));
        default:
            return WHITE;
    }
}

static float FieldVolumeWeight(const AtmosphereSim *sim, int x, int y, int z, FieldMode mode) {
    float value = FieldDisplayValue(sim, x, y, z, mode);
    switch (mode) {
        case FIELD_CLOUD: return NormalizeRange(value, 0.05f, 3.0f);
        case FIELD_RAIN: return NormalizeRange(value, 0.02f, 5.0f);
        case FIELD_REL_HUMIDITY: return NormalizeRange(value, 55.0f, 100.0f);
        case FIELD_VAPOR: return NormalizeRange(value, 2.0f, 14.0f);
        case FIELD_WIND: return NormalizeRange(value, 3.0f, 25.0f);
        case FIELD_VERTICAL_WIND: return NormalizeRange(fabsf(value), 0.4f, 8.0f);
        case FIELD_BUOYANCY: return NormalizeRange(fabsf(value), 0.05f, 0.9f);
        case FIELD_DEWPOINT: return NormalizeRange(value, -18.0f, 16.0f);
        case FIELD_PRESSURE: return NormalizeRange(fabsf(value - 760.0f), 8.0f, 180.0f);
        case FIELD_TEMPERATURE: return NormalizeRange(fabsf(value - 2.0f), 1.0f, 22.0f);
        default: return 0.0f;
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

static void UpdateVolumeFieldTexture(AtmosphereSim *sim) {
    if (sim->volumeFieldTex.id == 0 || sim->volumeFieldPixels == NULL) return;

    int atlasWidth = sim->nx * sim->volumeAtlasCols;
    int atlasHeight = sim->nz * sim->volumeAtlasRows;
    memset(sim->volumeFieldPixels, 0, (size_t)atlasWidth * (size_t)atlasHeight * sizeof(Vector4));

    for (int y = 0; y < sim->ny; y++) {
        int tileX = y % sim->volumeAtlasCols;
        int tileY = y / sim->volumeAtlasCols;

        for (int z = 0; z < sim->nz; z++) {
            for (int x = 0; x < sim->nx; x++) {
                int atlasX = tileX * sim->nx + x;
                int atlasY = tileY * sim->nz + z;
                int dst = atlasY * atlasWidth + atlasX;

                if (!IsFluidCell(sim, x, y, z)) {
                    sim->volumeFieldPixels[dst] = (Vector4) { 0.0f, 0.0f, 0.0f, 0.0f };
                    continue;
                }

                float sx = (float)x;
                float sy = (float)y;
                float sz = (float)z;
                float cloudSmooth =
                    SampleAtmosField(sim, sim->cloud, sx, sy, sz) * 0.28f +
                    SampleAtmosField(sim, sim->cloud, sx, sy - 0.75f, sz) * 0.15f +
                    SampleAtmosField(sim, sim->cloud, sx, sy + 0.75f, sz) * 0.15f +
                    SampleAtmosField(sim, sim->cloud, sx - 0.65f, sy, sz) * 0.10f +
                    SampleAtmosField(sim, sim->cloud, sx + 0.65f, sy, sz) * 0.10f +
                    SampleAtmosField(sim, sim->cloud, sx, sy, sz - 0.65f) * 0.07f +
                    SampleAtmosField(sim, sim->cloud, sx, sy, sz + 0.65f) * 0.07f +
                    SampleAtmosField(sim, sim->cloud, sx - 0.75f, sy, sz - 0.75f) * 0.02f +
                    SampleAtmosField(sim, sim->cloud, sx + 0.75f, sy, sz - 0.75f) * 0.02f +
                    SampleAtmosField(sim, sim->cloud, sx - 0.75f, sy, sz + 0.75f) * 0.02f +
                    SampleAtmosField(sim, sim->cloud, sx + 0.75f, sy, sz + 0.75f) * 0.02f;
                float rainSmooth =
                    SampleAtmosField(sim, sim->rain, sx, sy, sz) * 0.48f +
                    SampleAtmosField(sim, sim->rain, sx, sy - 0.80f, sz) * 0.18f +
                    SampleAtmosField(sim, sim->rain, sx, sy + 0.60f, sz) * 0.14f +
                    SampleAtmosField(sim, sim->rain, sx - 0.55f, sy, sz) * 0.10f +
                    SampleAtmosField(sim, sim->rain, sx + 0.55f, sy, sz) * 0.10f;
                float lightSmooth =
                    SampleAtmosField(sim, sim->sunlight, sx, sy, sz) * 0.52f +
                    SampleAtmosField(sim, sim->sunlight, sx, sy + 0.75f, sz) * 0.28f +
                    SampleAtmosField(sim, sim->sunlight, sx, sy - 0.75f, sz) * 0.20f;
                float tempSmooth =
                    SampleAtmosField(sim, sim->temp, sx, sy, sz) * 0.54f +
                    SampleAtmosField(sim, sim->temp, sx, sy - 0.80f, sz) * 0.23f +
                    SampleAtmosField(sim, sim->temp, sx, sy + 0.80f, sz) * 0.23f;

                float cloudNorm = NormalizeRange(cloudSmooth * 1000.0f, 0.0f, 3.0f);
                float rainNorm = NormalizeRange(rainSmooth * 1000.0f, 0.0f, 5.0f);
                float lightNorm = Clamp(lightSmooth, 0.0f, 1.0f);
                float tempNorm = NormalizeRange(tempSmooth, -30.0f, 24.0f);
                sim->volumeFieldPixels[dst] = (Vector4) { cloudNorm, rainNorm, lightNorm, tempNorm };
            }
        }
    }

    UpdateTexture(sim->volumeFieldTex, sim->volumeFieldPixels);
}

static void UpdateSliceTextures(AtmosphereSim *sim) {
    if (sim->showHorizontalSlice) UpdateHorizontalSliceTexture(sim);
    if (sim->showVerticalSlice) UpdateVerticalSliceTexture(sim);
    if (sim->showClouds) UpdateVolumeFieldTexture(sim);
}

static Color SkyColor(const AtmosphereSim *sim) {
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

static void DrawAtmosphereBackground(const AtmosphereSim *sim, const Camera3D *camera) {
    if (sim->skyShader.id == 0) {
        ClearBackground(SkyColor(sim));
        return;
    }

    Vector3 forward = Vector3Normalize(Vector3Subtract(camera->target, camera->position));
    Vector3 right = Vector3Normalize(Vector3CrossProduct(forward, camera->up));
    Vector3 up = Vector3Normalize(Vector3CrossProduct(right, forward));
    float aspect = (float)GetScreenWidth() / fmaxf((float)GetScreenHeight(), 1.0f);
    float tanHalfFov = tanf(camera->fovy * DEG2RAD * 0.5f);
    float sunDir[3] = { sim->sunDir.x, sim->sunDir.y, sim->sunDir.z };
    float camRight[3] = { right.x, right.y, right.z };
    float camUp[3] = { up.x, up.y, up.z };
    float camForward[3] = { forward.x, forward.y, forward.z };
    float sunStrength = Clamp(sim->solarStrength, 0.0f, 1.0f);

    ClearBackground(BLACK);
    SetShaderValue(sim->skyShader, sim->skyLocSunDir, sunDir, SHADER_UNIFORM_VEC3);
    SetShaderValue(sim->skyShader, sim->skyLocCamRight, camRight, SHADER_UNIFORM_VEC3);
    SetShaderValue(sim->skyShader, sim->skyLocCamUp, camUp, SHADER_UNIFORM_VEC3);
    SetShaderValue(sim->skyShader, sim->skyLocCamForward, camForward, SHADER_UNIFORM_VEC3);
    SetShaderValue(sim->skyShader, sim->skyLocAspect, &aspect, SHADER_UNIFORM_FLOAT);
    SetShaderValue(sim->skyShader, sim->skyLocTanHalfFov, &tanHalfFov, SHADER_UNIFORM_FLOAT);
    SetShaderValue(sim->skyShader, sim->skyLocSunStrength, &sunStrength, SHADER_UNIFORM_FLOAT);

    BeginShaderMode(sim->skyShader);
    DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), WHITE);
    EndShaderMode();
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

static void DrawWindArrowGlyph(Vector3 start, Vector3 tip, unsigned char alpha) {
    Vector3 delta = Vector3Subtract(tip, start);
    float length = Vector3Length(delta);
    if (length <= 0.08f) return;

    float headLength = fminf(WIND_ARROW_HEAD_LENGTH, length * 0.45f);
    Vector3 dir = Vector3Scale(delta, 1.0f / length);
    Vector3 shaftEnd = Vector3Add(start, Vector3Scale(dir, length - headLength));
    Color shaftColor = (Color) { 242, 58, 58, alpha };
    Color headColor = (Color) { 255, 90, 90, alpha };

    DrawCylinderEx(start, shaftEnd, WIND_ARROW_SHAFT_RADIUS, WIND_ARROW_SHAFT_RADIUS * 0.88f, 7, shaftColor);
    DrawCylinderEx(shaftEnd, tip, WIND_ARROW_HEAD_RADIUS, 0.0f, 7, headColor);
}

static void DrawWindArrowsOnHorizontalSlice(const AtmosphereSim *sim) {
    int layer = Clamp(sim->horizontalLayer, 0, sim->ny - 1);
    float y = WorldYFromMeters(CellAltitudeMeters(sim, layer)) + 0.08f;

    for (int z = 0; z < sim->nz; z += HSLICE_ARROW_STEP) {
        for (int x = 0; x < sim->nx; x += HSLICE_ARROW_STEP) {
            if (!IsFluidCell(sim, x, layer, z)) continue;
            int i = AtmosIndex(sim, x, layer, z);
            float speed = sqrtf(sim->u[i] * sim->u[i] + sim->v[i] * sim->v[i] + sim->w[i] * sim->w[i]);
            if (speed < 1.2f) continue;

            Vector3 start = { (float)x * sim->dxWorld, y, (float)z * sim->dzWorld };
            Vector3 tip = {
                start.x + sim->u[i] * WIND_ARROW_SCALE,
                start.y + sim->w[i] * (WIND_ARROW_SCALE * 0.72f),
                start.z + sim->v[i] * WIND_ARROW_SCALE
            };
            DrawWindArrowGlyph(start, tip, 228);
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
                Vector3 start = { xWorld + 0.08f, WorldYFromMeters(CellAltitudeMeters(sim, y)), (float)z * sim->dzWorld };
                Vector3 tip = {
                    start.x,
                    start.y + sim->w[i] * (WIND_ARROW_SCALE * 0.90f),
                    start.z + sim->v[i] * WIND_ARROW_SCALE
                };
                DrawWindArrowGlyph(start, tip, 228);
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
                Vector3 start = { (float)x * sim->dxWorld, WorldYFromMeters(CellAltitudeMeters(sim, y)), zWorld + 0.08f };
                Vector3 tip = {
                    start.x + sim->u[i] * WIND_ARROW_SCALE,
                    start.y + sim->w[i] * (WIND_ARROW_SCALE * 0.90f),
                    start.z
                };
                DrawWindArrowGlyph(start, tip, 228);
            }
        }
    }
}

static void DrawCloudVolume(const AtmosphereSim *sim, Camera camera) {
    if (!sim->showClouds) return;
    if (sim->cloudVolumeShader.id == 0 || sim->volumeFieldTex.id == 0 || sim->cloudVolumeModel.meshCount == 0) return;
    if (sim->cloudTerrainTex.id == 0) return;

    float volumeMin[3] = { 0.0f, 0.0f, 0.0f };
    float volumeMax[3] = { sim->worldWidth, sim->topWorldY, sim->worldDepth };
    float cameraPos[3] = { camera.position.x, camera.position.y, camera.position.z };
    float volumeRes[3] = { (float)sim->nx, (float)sim->nz, (float)sim->ny };
    float atlasGrid[2] = { (float)sim->volumeAtlasCols, (float)sim->volumeAtlasRows };
    float params[4] = {
        sim->cloudDensityScale,
        NormalizeRange(sim->cloudRenderThreshold, 0.0f, 3.0f),
        sim->cloudRenderAlpha,
        1.0f
    };
    float sunDir[3] = { sim->sunDir.x, sim->sunDir.y, sim->sunDir.z };

    SetShaderValue(sim->cloudVolumeShader, sim->cloudVolumeLocMin, volumeMin, SHADER_UNIFORM_VEC3);
    SetShaderValue(sim->cloudVolumeShader, sim->cloudVolumeLocMax, volumeMax, SHADER_UNIFORM_VEC3);
    SetShaderValue(sim->cloudVolumeShader, sim->cloudVolumeLocCamera, cameraPos, SHADER_UNIFORM_VEC3);
    SetShaderValue(sim->cloudVolumeShader, sim->cloudVolumeLocRes, volumeRes, SHADER_UNIFORM_VEC3);
    SetShaderValue(sim->cloudVolumeShader, sim->cloudVolumeLocAtlas, atlasGrid, SHADER_UNIFORM_VEC2);
    SetShaderValue(sim->cloudVolumeShader, sim->cloudVolumeLocParams, params, SHADER_UNIFORM_VEC4);
    SetShaderValue(sim->cloudVolumeShader, sim->cloudVolumeLocSun, sunDir, SHADER_UNIFORM_VEC3);

    rlDisableBackfaceCulling();
    rlSetBlendMode(BLEND_ALPHA);
    rlDisableDepthMask();

    DrawModel(sim->cloudVolumeModel,
              (Vector3) { sim->worldWidth * 0.5f, sim->topWorldY * 0.5f, sim->worldDepth * 0.5f },
              1.0f,
              WHITE);

    rlEnableDepthMask();
    rlEnableBackfaceCulling();
}

static void DrawVolumeField3D(const AtmosphereSim *sim) {
    if (!sim->showVolume) return;

    int strideXY = (sim->volumeStride < 2) ? 2 : sim->volumeStride;
    int strideY = (strideXY > 3) ? strideXY / 2 : 1;

    if (sim->fieldMode == FIELD_WIND || sim->fieldMode == FIELD_VERTICAL_WIND) {
        for (int z = 0; z < sim->nz; z += strideXY) {
            for (int y = 0; y < sim->ny; y += strideY) {
                for (int x = 0; x < sim->nx; x += strideXY) {
                    if (!IsFluidCell(sim, x, y, z)) continue;
                    int i = AtmosIndex(sim, x, y, z);
                    float speed = FieldDisplayValue(sim, x, y, z, sim->fieldMode);
                    float weight = FieldVolumeWeight(sim, x, y, z, sim->fieldMode);
                    if (weight < sim->volumeThreshold) continue;

                    Vector3 start = {
                        (float)x * sim->dxWorld,
                        WorldYFromMeters(CellAltitudeMeters(sim, y)),
                        (float)z * sim->dzWorld
                    };
                    Vector3 tip = {
                        start.x + sim->u[i] * WIND_ARROW_SCALE,
                        start.y + sim->w[i] * (WIND_ARROW_SCALE * 0.72f),
                        start.z + sim->v[i] * WIND_ARROW_SCALE
                    };
                    unsigned char alpha = (unsigned char)(255.0f * Clamp(sim->volumeAlpha * weight, 0.12f, 0.90f));
                    DrawWindArrowGlyph(start, tip, alpha);
                }
            }
        }
        return;
    }

    Vector3 cellSize = {
        sim->dxWorld * 0.78f,
        WorldYFromMeters(sim->dyMeters) * 0.82f,
        sim->dzWorld * 0.78f
    };

    for (int z = 0; z < sim->nz; z += strideXY) {
        for (int y = 0; y < sim->ny; y += strideY) {
            for (int x = 0; x < sim->nx; x += strideXY) {
                if (!IsFluidCell(sim, x, y, z)) continue;
                int i = AtmosIndex(sim, x, y, z);
                float value = FieldDisplayValue(sim, x, y, z, sim->fieldMode);
                float weight = FieldVolumeWeight(sim, x, y, z, sim->fieldMode);
                if (weight < sim->volumeThreshold) continue;

                unsigned char alpha = (unsigned char)(255.0f * Clamp(sim->volumeAlpha * weight, 0.08f, 0.88f));
                Color color = WithAlpha(FieldColorFromValue(sim, sim->fieldMode, value, sim->u[i], sim->v[i]), alpha);
                Vector3 position = {
                    (float)x * sim->dxWorld,
                    WorldYFromMeters(CellAltitudeMeters(sim, y)),
                    (float)z * sim->dzWorld
                };

                DrawCubeV(position, cellSize, color);
            }
        }
    }
}

static void DrawSlices3D(const AtmosphereSim *sim, Camera camera) {
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

static void DrawImGuiHud(const AtmosphereSim *sim) {
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
        igText("Layered Mountain Atmosphere");
        igSeparator();
        igText("Field  %s", FieldName(sim->fieldMode));
        igText("Scale  %.1f to %.1f %s", fieldMin, fieldMax, FieldUnits(sim->fieldMode));
        igText("Time   %02d:%02d", hour, minute);
        if (sim->paused) igTextColored(ImVec4Make(1.00f, 0.71f, 0.20f, 1.00f), "Paused");
        igText("Surface %.1f C", sim->meanSurfaceTemp);
        igText("Pressure %.1f hPa", sim->meanPressure);
        igText("Wind %.1f m/s", sim->maxWind);
        igText("Rain %.1f mm/h", sim->maxRain);
        igText("Cloud %.2f g/kg", sim->maxCloud * 1000.0f);
        ImGuiIO *io = igGetIO_Nil();
        if (io != NULL) igText("FPS %.0f", io->Framerate);
        igSeparator();
        DrawFieldLegendHud(sim, sim->fieldMode);
    }
    igEnd();
}

static bool DrawImGuiControls(AtmosphereSim *sim) {
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

    if (igBegin("Atmosphere Controls", NULL, flags)) {
        if (igButton(sim->paused ? "Resume" : "Pause", ImVec2Make(110.0f, 0.0f))) sim->paused = !sim->paused;
        igSameLine(0.0f, 8.0f);
        if (igButton("Reset Weather", ImVec2Make(132.0f, 0.0f))) ResetAtmosphere(sim);
        igSameLine(0.0f, 8.0f);
        if (igButton("Hide Panel", ImVec2Make(106.0f, 0.0f))) sim->showUI = false;

        igSeparator();
        if (igBeginChild_Str("##AtmosphereScroll", ImVec2Make(0.0f, 0.0f), ImGuiChildFlags_Borders, ImGuiWindowFlags_None)) {
            igPushItemWidth(-FLT_MIN);

            if (igBeginTabBar("##ControlTabs", ImGuiTabBarFlags_None)) {
                if (igBeginTabItem("Weather", NULL, ImGuiTabItemFlags_None)) {
                    igSeparatorText("Display");
                    int fieldMode = (int)sim->fieldMode;
                    if (DrawLabeledCombo("Field", "##FieldMode", &fieldMode, kFieldLabels, FIELD_COUNT)) {
                        sim->fieldMode = (FieldMode)fieldMode;
                    }
                    igCheckbox("Show terrain", &sim->showTerrain);
                    igCheckbox("Wireframe terrain", &sim->showWireframe);
                    igCheckbox("Show wind vectors", &sim->showWindArrows);
                    igCheckbox("Render clouds", &sim->showClouds);
                    igBeginDisabled(!sim->showTerrain);
                    DrawLabeledSliderFloat("Terrain opacity", "##TerrainOpacity", &sim->terrainAlpha, 0.05f, 1.0f, "%.2f");
                    igEndDisabled();

                    igSeparatorText("Clouds");
                    igBeginDisabled(!sim->showClouds);
                    DrawLabeledSliderFloat("Cloud opacity", "##CloudOpacity", &sim->cloudRenderAlpha, 0.20f, 0.95f, "%.2f");
                    DrawLabeledSliderFloat("Cloud density", "##CloudDensity", &sim->cloudDensityScale, 0.50f, 1.80f, "%.2f");
                    DrawLabeledSliderFloat("Cloud threshold", "##CloudThreshold", &sim->cloudRenderThreshold, 0.00f, 0.30f, "%.2f");
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

                    igSeparatorText("Simulation");
                    DrawLabeledSliderFloat("Time scale", "##TimeScale", &sim->timeScale, 30.0f, 1200.0f, "%.0fx");
                    igTextDisabled("Current units");
                    igText("%s", FieldUnits(sim->fieldMode));
                    igTextDisabled("Statistics");
                    igText("Surface temp  %.1f C", sim->meanSurfaceTemp);
                    igText("Pressure      %.1f hPa", sim->meanPressure);
                    igText("Peak wind     %.1f m/s", sim->maxWind);
                    igText("Peak rain     %.1f mm/h", sim->maxRain);
                    igText("Peak cloud    %.2f g/kg", sim->maxCloud * 1000.0f);

                    igSeparatorText("Controls");
                    igPushTextWrapPos(0.0f);
                    igTextWrapped("The panel captures the mouse while you hover or drag inside it, so camera orbit and zoom stay locked out during UI interaction.");
                    igPopTextWrapPos();
                    igBulletText("Left mouse drag: orbit");
                    igBulletText("Mouse wheel: zoom");
                    igBulletText("Space: pause");
                    igBulletText("R: reset weather");
                    igBulletText("Tab: cycle field");
                    igBulletText("C: toggle clouds");
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
                    igTextWrapped("Generate Terrain rebuilds the mountain mesh from the current settings and resets the weather simulation over the new terrain.");
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
    ApplyTerrainMaterial(sim);

    BuildAtmosColumns(sim);
    if (!UpdateCloudTerrainTexture(sim)) {
        return false;
    }
    ResetAtmosphere(sim);
    if (preserveViewSettings) {
        RestoreSimViewSettings(sim, &viewSettings);
    }
    UpdateStats(sim);
    return true;
}

static bool CreateSimFromGeneratedTerrain(AtmosphereSim *sim) {
    InitializeSimDimensions(sim);
    DefaultTerrainGenSettings(&sim->terrainGen, GetRandomValue(1, 999999));
    sim->activeControlTab = 0;

    if (!AllocateSim(sim)) {
        DestroySim(sim);
        return false;
    }

    if (!SetupTerrainMaterialResources(sim)) {
        fprintf(stderr, "Warning: failed to load terrain texture resources, using vertex colors only.\n");
    }

    return RebuildGeneratedTerrain(sim);
}

int main(int argc, char **argv) {
    (void)argc;
    (void)argv;
    AtmosphereSim sim = { 0 };

    InitWindow(1360, 820, "Mountain Atmosphere Slices");
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
        sim.frameCounter++;
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
        if (!blockShortcuts && IsKeyPressed(KEY_TAB)) sim.fieldMode = (FieldMode)((sim.fieldMode + 1) % FIELD_COUNT);
        if (!blockShortcuts && IsKeyPressed(KEY_C)) sim.showClouds = !sim.showClouds;
        if (!blockShortcuts && IsKeyPressed(KEY_H)) sim.showHorizontalSlice = !sim.showHorizontalSlice;
        if (!blockShortcuts && IsKeyPressed(KEY_J)) sim.showVerticalSlice = !sim.showVerticalSlice;
        if (!blockShortcuts && IsKeyPressed(KEY_T)) sim.showTerrain = !sim.showTerrain;

        if (!sim.paused) UpdateAtmosphere(&sim, frameDt);
        UpdateSliceTextures(&sim);

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
        if (sim.showTerrain) {
            UpdateTerrainMaterialUniforms(&sim);
            DrawModel(sim.terrainModel, (Vector3) { 0.0f, 0.0f, 0.0f }, 1.0f, Fade(WHITE, sim.terrainAlpha));
        }
        if (sim.showWireframe) DrawModelWires(sim.terrainModel, (Vector3) { 0.0f, 0.0f, 0.0f }, 1.0f, Fade(BLACK, 0.25f));
        DrawCloudVolume(&sim, camera);
        DrawSlices3D(&sim, camera);
        EndMode3D();

        if (sim.paused) {
            DrawRectangle(GetScreenWidth() / 2 - 62, 18, 124, 36, Fade(BLACK, 0.52f));
            DrawRectangleLines(GetScreenWidth() / 2 - 62, 18, 124, 36, Fade(RAYWHITE, 0.18f));
            DrawText("PAUSED", GetScreenWidth() / 2 - 30, 27, 20, ORANGE);
        }

        rlImGuiBeginDelta(frameDt);
        if (sim.showUI) DrawImGuiHud(&sim);
        bool terrainChanged = DrawImGuiControls(&sim);
        ImGuiIO *io = igGetIO_Nil();
        imguiWantsMouse = (io != NULL) ? io->WantCaptureMouse : false;
        imguiWantsKeyboard = (io != NULL) ? io->WantCaptureKeyboard : false;
        rlImGuiEnd();

        if (terrainChanged) {
            target = (Vector3) {
                sim.worldWidth * 0.5f,
                Lerp(sim.terrain.minTerrainY, sim.terrain.maxTerrainY, 0.45f),
                sim.worldDepth * 0.5f
            };
        }

        EndDrawing();
    }

    rlImGuiShutdown();
    DestroySim(&sim);
    CloseWindow();
    return 0;
}
