#include "weather.h"
#include "raymath.h"
#include "rlgl.h"
#define CIMGUI_DEFINE_ENUMS_AND_STRUCTS
#include "cimgui.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define TAU 6.28318530717958647692f
#define DAY_SECONDS 86400.0f
#define SLICE_TEX_SCALE 4
#define SLICE_UPDATE_RATE_HZ 20.0
#define HSLICE_ARROW_STEP 8
#define VSLICE_ARROW_STEP 4

static const char *const kFieldLabels[WEATHER_FIELD_COUNT] = {
    "Temperature",
    "Humidity",
    "Cloud",
    "Rain",
    "Wind Speed",
    "Vertical Wind"
};

static const char *const kFieldUnits[WEATHER_FIELD_COUNT] = {
    "deg C",
    "%",
    "g/kg",
    "g/kg",
    "m/s",
    "m/s"
};

static const Color kTemperatureStops[] = {
    { 37, 32, 97, 255 },
    { 52, 99, 192, 255 },
    { 71, 182, 212, 255 },
    { 127, 209, 124, 255 },
    { 244, 214, 87, 255 },
    { 239, 136, 55, 255 },
    { 191, 58, 48, 255 }
};

static const Color kHumidityStops[] = {
    { 100, 76, 45, 255 },
    { 151, 130, 80, 255 },
    { 97, 165, 143, 255 },
    { 102, 192, 222, 255 },
    { 240, 248, 255, 255 }
};

static const Color kCloudStops[] = {
    { 17, 23, 44, 255 },
    { 59, 72, 121, 255 },
    { 129, 148, 196, 255 },
    { 211, 222, 239, 255 },
    { 248, 252, 255, 255 }
};

static const Color kDivergingStops[] = {
    { 188, 76, 42, 255 },
    { 233, 196, 118, 255 },
    { 240, 242, 245, 255 },
    { 95, 187, 223, 255 },
    { 38, 78, 189, 255 }
};

static float Clamp01(float value) {
    return Clamp(value, 0.0f, 1.0f);
}

static int ColumnIndex(const WeatherSystem *weather, int x, int z) {
    return z * weather->config.nx + x;
}

static int CellIndex(const WeatherSystem *weather, int x, int y, int z) {
    return ((y * weather->config.nz) + z) * weather->config.nx + x;
}

static void SwapFields(float **a, float **b) {
    float *tmp = *a;
    *a = *b;
    *b = tmp;
}

static float LerpColorChannel(float a, float b, float t) {
    return a + (b - a) * t;
}

static Color LerpColor(Color a, Color b, float t) {
    t = Clamp01(t);
    return (Color){
        (unsigned char)LerpColorChannel((float)a.r, (float)b.r, t),
        (unsigned char)LerpColorChannel((float)a.g, (float)b.g, t),
        (unsigned char)LerpColorChannel((float)a.b, (float)b.b, t),
        (unsigned char)LerpColorChannel((float)a.a, (float)b.a, t)
    };
}

static Color RampColor(float t, const Color *stops, int stopCount) {
    t = Clamp01(t);
    if (stopCount <= 1) return stops[0];

    float scaled = t * (float)(stopCount - 1);
    int index = (int)floorf(scaled);
    if (index >= stopCount - 1) return stops[stopCount - 1];
    return LerpColor(stops[index], stops[index + 1], scaled - (float)index);
}

static Color WithAlpha(Color color, unsigned char alpha) {
    color.a = alpha;
    return color;
}

static float NormalizeRange(float value, float minValue, float maxValue) {
    if (fabsf(maxValue - minValue) <= 0.0001f) return 0.0f;
    return Clamp01((value - minValue) / (maxValue - minValue));
}

static float Fract(float value) {
    return value - floorf(value);
}

static float Noise3D(float x, float y, float z) {
    return Fract(sinf(x * 127.1f + y * 311.7f + z * 201.3f) * 43758.5453f);
}

static float BaseSeaLevelTemp(float simTimeSeconds) {
    float dayProgress = fmodf(simTimeSeconds, DAY_SECONDS) / DAY_SECONDS;
    float solar = fmaxf(0.0f, sinf(TAU * (dayProgress - 0.25f)));
    return 4.0f + 16.0f * solar;
}

static float CellAltitude(const WeatherSystem *weather, int y) {
    return ((float)y + 0.5f) * weather->dy;
}

static bool IsFluidCell(const WeatherSystem *weather, int x, int y, int z) {
    if (x < 0 || x >= weather->config.nx || y < 0 || y >= weather->config.ny || z < 0 || z >= weather->config.nz) {
        return false;
    }
    return y > weather->groundLayer[ColumnIndex(weather, x, z)];
}

static float SampleField(const WeatherSystem *weather, const float *field, float x, float y, float z) {
    x = Clamp(x, 0.0f, (float)(weather->config.nx - 1));
    y = Clamp(y, 0.0f, (float)(weather->config.ny - 1));
    z = Clamp(z, 0.0f, (float)(weather->config.nz - 1));

    int x0 = (int)floorf(x);
    int y0 = (int)floorf(y);
    int z0 = (int)floorf(z);
    int x1 = (x0 + 1 < weather->config.nx) ? x0 + 1 : x0;
    int y1 = (y0 + 1 < weather->config.ny) ? y0 + 1 : y0;
    int z1 = (z0 + 1 < weather->config.nz) ? z0 + 1 : z0;

    float tx = x - (float)x0;
    float ty = y - (float)y0;
    float tz = z - (float)z0;

    float c000 = field[CellIndex(weather, x0, y0, z0)];
    float c100 = field[CellIndex(weather, x1, y0, z0)];
    float c010 = field[CellIndex(weather, x0, y1, z0)];
    float c110 = field[CellIndex(weather, x1, y1, z0)];
    float c001 = field[CellIndex(weather, x0, y0, z1)];
    float c101 = field[CellIndex(weather, x1, y0, z1)];
    float c011 = field[CellIndex(weather, x0, y1, z1)];
    float c111 = field[CellIndex(weather, x1, y1, z1)];

    float c00 = Lerp(c000, c100, tx);
    float c10 = Lerp(c010, c110, tx);
    float c01 = Lerp(c001, c101, tx);
    float c11 = Lerp(c011, c111, tx);
    float c0 = Lerp(c00, c10, ty);
    float c1 = Lerp(c01, c11, ty);
    return Lerp(c0, c1, tz);
}

static float DiffuseField6(const WeatherSystem *weather, const float *field, int x, int y, int z, float fallback) {
    float sum = 0.0f;
    int count = 0;
    static const int kNeighbors[6][3] = {
        { -1, 0, 0 }, { 1, 0, 0 }, { 0, -1, 0 }, { 0, 1, 0 }, { 0, 0, -1 }, { 0, 0, 1 }
    };

    for (int i = 0; i < 6; i++) {
        int nx = x + kNeighbors[i][0];
        int ny = y + kNeighbors[i][1];
        int nz = z + kNeighbors[i][2];
        if (!IsFluidCell(weather, nx, ny, nz)) continue;
        sum += field[CellIndex(weather, nx, ny, nz)];
        count++;
    }

    return (count > 0) ? (sum / (float)count) : fallback;
}

static void BuildTerrainColumns(WeatherSystem *weather, const TerrainSurface *terrain) {
    for (int z = 0; z < weather->config.nz; z++) {
        for (int x = 0; x < weather->config.nx; x++) {
            float worldX = (float)x * weather->dx;
            float worldZ = (float)z * weather->dz;
            int column = ColumnIndex(weather, x, z);
            TerrainSample sample = Terrain_Sample(terrain, worldX, worldZ);
            weather->terrainHeight[column] = sample.height;
            weather->terrainGradX[column] = sample.gradX;
            weather->terrainGradZ[column] = sample.gradZ;
            weather->terrainSlope[column] = sample.slope;
            weather->groundLayer[column] = (int)floorf(sample.height / weather->dy);
            if (weather->groundLayer[column] < -1) weather->groundLayer[column] = -1;
            if (weather->groundLayer[column] >= weather->config.ny) weather->groundLayer[column] = weather->config.ny - 1;
        }
    }
}

static void UpdateSurfaceState(WeatherSystem *weather) {
    float dayProgress = fmodf(weather->simTimeSeconds, DAY_SECONDS) / DAY_SECONDS;
    float solar = fmaxf(0.0f, sinf(TAU * (dayProgress - 0.25f)));
    float seaLevelTemp = BaseSeaLevelTemp(weather->simTimeSeconds);
    Vector3 sunDir = Vector3Normalize((Vector3) {
        cosf(TAU * (dayProgress - 0.25f)),
        0.35f + 0.85f * solar,
        0.25f + 0.55f * sinf(TAU * (dayProgress - 0.18f))
    });

    for (int column = 0; column < weather->cells2D; column++) {
        float heightNorm = NormalizeRange(weather->terrainHeight[column], 0.0f, weather->config.topHeight);
        Vector3 normal = Vector3Normalize((Vector3) {
            -weather->terrainGradX[column], 1.0f, -weather->terrainGradZ[column]
        });
        float exposure = fmaxf(0.0f, Vector3DotProduct(normal, sunDir));
        float valley = Clamp01((1.0f - heightNorm) * (1.0f - weather->terrainSlope[column] * 3.0f));
        float ridge = Clamp01(heightNorm * 0.7f + weather->terrainSlope[column] * 1.4f);

        weather->surfaceTemp[column] = seaLevelTemp - weather->terrainHeight[column] * 0.045f +
                                       solar * exposure * 7.0f -
                                       ridge * 1.6f +
                                       valley * 0.9f;
        weather->surfaceMoisture[column] = Clamp(0.26f + valley * 0.42f + (1.0f - solar) * 0.18f + weather->surfaceRain[column] * 0.006f,
                                                 0.06f,
                                                 1.0f);
        weather->surfaceRain[column] *= 0.975f;
    }
}

static void InitializeState(WeatherSystem *weather) {
    UpdateSurfaceState(weather);

    float synopticPhase = weather->simTimeSeconds / (DAY_SECONDS * 1.5f);
    float synopticDir = 0.5f + 0.6f * sinf(synopticPhase * 0.7f);
    float synopticSpeed = 7.0f + 2.8f * sinf(synopticPhase + 0.8f);
    float synopticU = cosf(synopticDir) * synopticSpeed;
    float synopticV = sinf(synopticDir) * synopticSpeed * 0.8f;

    memset(weather->temperature, 0, (size_t)weather->cells3D * sizeof(float));
    memset(weather->humidity, 0, (size_t)weather->cells3D * sizeof(float));
    memset(weather->cloud, 0, (size_t)weather->cells3D * sizeof(float));
    memset(weather->rain, 0, (size_t)weather->cells3D * sizeof(float));
    memset(weather->windU, 0, (size_t)weather->cells3D * sizeof(float));
    memset(weather->windV, 0, (size_t)weather->cells3D * sizeof(float));
    memset(weather->windW, 0, (size_t)weather->cells3D * sizeof(float));

    for (int z = 0; z < weather->config.nz; z++) {
        for (int y = 0; y < weather->config.ny; y++) {
            for (int x = 0; x < weather->config.nx; x++) {
                int index = CellIndex(weather, x, y, z);
                if (!IsFluidCell(weather, x, y, z)) continue;

                int column = ColumnIndex(weather, x, z);
                float altitude = CellAltitude(weather, y);
                float altNorm = Clamp01(altitude / weather->config.topHeight);
                float layerAboveGround = (float)(y - weather->groundLayer[column]);
                float boundary = expf(-0.45f * fmaxf(layerAboveGround - 1.0f, 0.0f));
                float noise = (Noise3D((float)x * 0.13f, (float)y * 0.18f, (float)z * 0.11f + weather->simTimeSeconds * 0.00003f) - 0.5f);
                float backgroundTemp = BaseSeaLevelTemp(weather->simTimeSeconds) - altitude * 0.0062f;
                float profile = 0.58f + 0.52f * altNorm;

                weather->temperature[index] = backgroundTemp +
                                              boundary * (weather->surfaceTemp[column] - backgroundTemp) * 0.26f +
                                              noise * (1.2f - altNorm);
                weather->humidity[index] = Clamp(0.34f +
                                                 weather->surfaceMoisture[column] * (0.32f + boundary * 0.18f) -
                                                 altNorm * 0.30f +
                                                 noise * 0.08f,
                                                 0.05f,
                                                 1.0f);
                weather->windU[index] = synopticU * profile;
                weather->windV[index] = synopticV * profile;
                weather->windW[index] = 0.0f;
            }
        }
    }
}

static void UpdateStats(WeatherSystem *weather) {
    double surfaceTempSum = 0.0;
    double humiditySum = 0.0;
    int humidityCount = 0;
    float maxCloud = 0.0f;
    float maxRain = 0.0f;
    float maxWind = 0.0f;

    for (int z = 0; z < weather->config.nz; z++) {
        for (int x = 0; x < weather->config.nx; x++) {
            int column = ColumnIndex(weather, x, z);
            surfaceTempSum += weather->surfaceTemp[column];
            if (weather->surfaceRain[column] > maxRain) maxRain = weather->surfaceRain[column];
        }
    }

    for (int z = 0; z < weather->config.nz; z++) {
        for (int y = 0; y < weather->config.ny; y++) {
            for (int x = 0; x < weather->config.nx; x++) {
                if (!IsFluidCell(weather, x, y, z)) continue;
                int index = CellIndex(weather, x, y, z);
                humiditySum += weather->humidity[index];
                humidityCount++;
                if (weather->cloud[index] > maxCloud) maxCloud = weather->cloud[index];
                if (weather->rain[index] > maxRain) maxRain = weather->rain[index] * 1000.0f;
                float wind = sqrtf(weather->windU[index] * weather->windU[index] +
                                   weather->windV[index] * weather->windV[index] +
                                   weather->windW[index] * weather->windW[index]);
                if (wind > maxWind) maxWind = wind;
            }
        }
    }

    weather->stats.meanSurfaceTemp = (float)(surfaceTempSum / (double)weather->cells2D);
    weather->stats.meanHumidity = (humidityCount > 0) ? (float)(humiditySum / (double)humidityCount) * 100.0f : 0.0f;
    weather->stats.maxCloud = maxCloud;
    weather->stats.maxRain = maxRain;
    weather->stats.maxWind = maxWind;
    weather->stats.backlogSeconds = weather->accumulatorSeconds;
}

static void StepWeather(WeatherSystem *weather) {
    UpdateSurfaceState(weather);

    float dt = weather->config.fixedDt;
    float synopticPhase = weather->simTimeSeconds / (DAY_SECONDS * 1.5f);
    float synopticDir = 0.5f + 0.6f * sinf(synopticPhase * 0.7f);
    float synopticSpeed = 7.0f + 2.8f * sinf(synopticPhase + 0.8f);
    float synopticU = cosf(synopticDir) * synopticSpeed;
    float synopticV = sinf(synopticDir) * synopticSpeed * 0.8f;

    for (int z = 0; z < weather->config.nz; z++) {
        for (int y = 0; y < weather->config.ny; y++) {
            for (int x = 0; x < weather->config.nx; x++) {
                int index = CellIndex(weather, x, y, z);
                if (!IsFluidCell(weather, x, y, z)) {
                    weather->temperatureNext[index] = 0.0f;
                    weather->humidityNext[index] = 0.0f;
                    weather->cloudNext[index] = 0.0f;
                    weather->rainNext[index] = 0.0f;
                    weather->windUNext[index] = 0.0f;
                    weather->windVNext[index] = 0.0f;
                    weather->windWNext[index] = 0.0f;
                    continue;
                }

                int column = ColumnIndex(weather, x, z);
                float altitude = CellAltitude(weather, y);
                float altNorm = Clamp01(altitude / weather->config.topHeight);
                float layerAboveGround = (float)(y - weather->groundLayer[column]);
                float boundary = expf(-0.42f * fmaxf(layerAboveGround - 1.0f, 0.0f));
                float baseTemp = BaseSeaLevelTemp(weather->simTimeSeconds) - altitude * 0.0061f;
                float profile = 0.58f + 0.52f * altNorm;

                float velU = weather->windU[index];
                float velV = weather->windV[index];
                float velW = weather->windW[index];
                float backX = (float)x - velU * dt / fmaxf(weather->dx, 0.0001f);
                float backY = (float)y - velW * dt / fmaxf(weather->dy, 0.0001f);
                float backZ = (float)z - velV * dt / fmaxf(weather->dz, 0.0001f);

                float advTemp = SampleField(weather, weather->temperature, backX, backY, backZ);
                float advHumidity = SampleField(weather, weather->humidity, backX, backY, backZ);
                float advCloud = SampleField(weather, weather->cloud, backX, backY, backZ);
                float advRain = SampleField(weather, weather->rain, backX, backY, backZ);
                float advU = SampleField(weather, weather->windU, backX, backY, backZ);
                float advV = SampleField(weather, weather->windV, backX, backY, backZ);
                float advW = SampleField(weather, weather->windW, backX, backY, backZ);

                float diffTemp = DiffuseField6(weather, weather->temperature, x, y, z, advTemp);
                float diffHumidity = DiffuseField6(weather, weather->humidity, x, y, z, advHumidity);
                float diffCloud = DiffuseField6(weather, weather->cloud, x, y, z, advCloud);
                float diffRain = DiffuseField6(weather, weather->rain, x, y, z, advRain);

                float slopeLift = (advU * weather->terrainGradX[column] + advV * weather->terrainGradZ[column]) * boundary;
                float thermalLift = (weather->surfaceTemp[column] - advTemp) * boundary * 0.08f;
                float buoyancy = (advTemp - baseTemp) * 0.035f;
                float targetW = Clamp(slopeLift * 2.4f + thermalLift + buoyancy, -3.5f, 5.0f);
                float targetU = synopticU * profile * (1.0f - boundary * weather->terrainSlope[column] * 0.42f);
                float targetV = synopticV * profile * (1.0f - boundary * weather->terrainSlope[column] * 0.42f);

                weather->windUNext[index] = Lerp(advU, targetU, 0.08f) * (1.0f - boundary * 0.04f);
                weather->windVNext[index] = Lerp(advV, targetV, 0.08f) * (1.0f - boundary * 0.04f);
                weather->windWNext[index] = Lerp(advW, targetW, 0.12f) * (0.96f - altNorm * 0.02f);

                weather->temperatureNext[index] = advTemp +
                                                  (diffTemp - advTemp) * 0.08f +
                                                  (baseTemp - advTemp) * 0.012f +
                                                  boundary * (weather->surfaceTemp[column] - advTemp) * 0.018f;

                weather->humidityNext[index] = Clamp(advHumidity +
                                                     (diffHumidity - advHumidity) * 0.10f +
                                                     boundary * weather->surfaceMoisture[column] * 0.012f -
                                                     fmaxf(weather->windWNext[index], 0.0f) * 0.002f,
                                                     0.01f,
                                                     1.0f);

                float saturation = Clamp01((weather->humidityNext[index] - 0.76f) / 0.24f);
                float cloudGain = saturation * saturation * (0.18f + 0.18f * fmaxf(weather->windWNext[index], 0.0f));
                float cloudLoss = 0.015f + altNorm * 0.01f;
                weather->cloudNext[index] = Clamp(advCloud +
                                                  (diffCloud - advCloud) * 0.12f +
                                                  (cloudGain - cloudLoss * advCloud) * dt,
                                                  0.0f,
                                                  0.0045f);

                float rainGain = fmaxf(weather->cloudNext[index] - 0.0011f, 0.0f) * 0.22f;
                float rainLoss = 0.06f + fmaxf(0.0f, 0.70f - weather->humidityNext[index]) * 0.08f;
                weather->rainNext[index] = Clamp(advRain +
                                                 (diffRain - advRain) * 0.08f +
                                                 (rainGain - rainLoss * advRain) * dt,
                                                 0.0f,
                                                 0.0038f);
            }
        }
    }

    memset(weather->surfaceRain, 0, (size_t)weather->cells2D * sizeof(float));
    memset(weather->rain, 0, (size_t)weather->cells3D * sizeof(float));

    for (int z = 0; z < weather->config.nz; z++) {
        for (int x = 0; x < weather->config.nx; x++) {
            int column = ColumnIndex(weather, x, z);
            for (int y = weather->config.ny - 1; y >= 0; y--) {
                int index = CellIndex(weather, x, y, z);
                if (!IsFluidCell(weather, x, y, z)) continue;

                float rainValue = weather->rainNext[index];
                if (rainValue <= 0.0f) continue;
                float fallFraction = Clamp(0.15f + rainValue * 160.0f, 0.0f, 0.78f);
                float stay = rainValue * (1.0f - fallFraction);
                float fall = rainValue * fallFraction;
                weather->rain[index] += stay;

                if (IsFluidCell(weather, x, y - 1, z)) {
                    weather->rain[CellIndex(weather, x, y - 1, z)] += fall;
                } else {
                    weather->surfaceRain[column] += fall * 8200.0f;
                }
            }
        }
    }

    SwapFields(&weather->temperature, &weather->temperatureNext);
    SwapFields(&weather->humidity, &weather->humidityNext);
    SwapFields(&weather->cloud, &weather->cloudNext);
    SwapFields(&weather->windU, &weather->windUNext);
    SwapFields(&weather->windV, &weather->windVNext);
    SwapFields(&weather->windW, &weather->windWNext);

    weather->simTimeSeconds = fmodf(weather->simTimeSeconds + dt, DAY_SECONDS);
    if (weather->simTimeSeconds < 0.0f) weather->simTimeSeconds += DAY_SECONDS;
}

static float FieldSampleAt(const WeatherSystem *weather, float x, float y, float z, WeatherField field) {
    switch (field) {
        case WEATHER_FIELD_TEMPERATURE: return SampleField(weather, weather->temperature, x, y, z);
        case WEATHER_FIELD_HUMIDITY: return SampleField(weather, weather->humidity, x, y, z) * 100.0f;
        case WEATHER_FIELD_CLOUD: return SampleField(weather, weather->cloud, x, y, z) * 1000.0f;
        case WEATHER_FIELD_RAIN: return SampleField(weather, weather->rain, x, y, z) * 1000.0f;
        case WEATHER_FIELD_WIND: {
            float u = SampleField(weather, weather->windU, x, y, z);
            float v = SampleField(weather, weather->windV, x, y, z);
            float w = SampleField(weather, weather->windW, x, y, z);
            return sqrtf(u * u + v * v + w * w);
        }
        case WEATHER_FIELD_VERTICAL_WIND: return SampleField(weather, weather->windW, x, y, z);
        default: return 0.0f;
    }
}

static Color FieldColor(const WeatherSystem *weather, WeatherField field, float value) {
    float minValue = 0.0f;
    float maxValue = 1.0f;
    Weather_FieldDisplayRange(weather, field, &minValue, &maxValue);
    float t = NormalizeRange(value, minValue, maxValue);

    switch (field) {
        case WEATHER_FIELD_TEMPERATURE: return RampColor(t, kTemperatureStops, (int)(sizeof(kTemperatureStops) / sizeof(kTemperatureStops[0])));
        case WEATHER_FIELD_HUMIDITY: return RampColor(t, kHumidityStops, (int)(sizeof(kHumidityStops) / sizeof(kHumidityStops[0])));
        case WEATHER_FIELD_CLOUD: return RampColor(t, kCloudStops, (int)(sizeof(kCloudStops) / sizeof(kCloudStops[0])));
        case WEATHER_FIELD_RAIN: return RampColor(t, kTemperatureStops, (int)(sizeof(kTemperatureStops) / sizeof(kTemperatureStops[0])));
        case WEATHER_FIELD_WIND: return RampColor(t, kTemperatureStops, (int)(sizeof(kTemperatureStops) / sizeof(kTemperatureStops[0])));
        case WEATHER_FIELD_VERTICAL_WIND: return RampColor(t, kDivergingStops, (int)(sizeof(kDivergingStops) / sizeof(kDivergingStops[0])));
        default: return WHITE;
    }
}

static void UpdateHorizontalTexture(WeatherSystem *weather) {
    int layer = Clamp(weather->view.horizontalLayer, 0, weather->config.ny - 1);
    int texWidth = weather->config.nx * SLICE_TEX_SCALE;
    int texHeight = weather->config.nz * SLICE_TEX_SCALE;

    for (int z = 0; z < texHeight; z++) {
        float sampleZ = (texHeight > 1) ? ((float)z / (float)(texHeight - 1)) * (float)(weather->config.nz - 1) : 0.0f;
        int cellZ = Clamp((int)lroundf(sampleZ), 0, weather->config.nz - 1);

        for (int x = 0; x < texWidth; x++) {
            float sampleX = (texWidth > 1) ? ((float)x / (float)(texWidth - 1)) * (float)(weather->config.nx - 1) : 0.0f;
            int cellX = Clamp((int)lroundf(sampleX), 0, weather->config.nx - 1);
            int dst = z * texWidth + x;

            if (!IsFluidCell(weather, cellX, layer, cellZ)) {
                weather->horizontalPixels[dst] = BLANK;
                continue;
            }

            float value = FieldSampleAt(weather, sampleX, (float)layer, sampleZ, weather->view.field);
            Color color = FieldColor(weather, weather->view.field, value);
            weather->horizontalPixels[dst] = WithAlpha(color, 230);
        }
    }

    UpdateTexture(weather->horizontalTexture, weather->horizontalPixels);
}

static Color TerrainSliceColor(float heightNorm) {
    if (heightNorm > 0.82f) return (Color){ 216, 221, 228, 255 };
    if (heightNorm > 0.54f) return (Color){ 142, 126, 108, 255 };
    return (Color){ 83, 102, 73, 255 };
}

static void UpdateVerticalTexture(WeatherSystem *weather) {
    int slice = Clamp(weather->view.verticalSlice,
                      0,
                      (weather->view.verticalAxis == WEATHER_SLICE_X) ? weather->config.nx - 1 : weather->config.nz - 1);
    int logicalWidth = (weather->view.verticalAxis == WEATHER_SLICE_X) ? weather->config.nz : weather->config.nx;
    int texWidth = ((weather->config.nx > weather->config.nz) ? weather->config.nx : weather->config.nz) * SLICE_TEX_SCALE;
    int logicalTexWidth = logicalWidth * SLICE_TEX_SCALE;
    int texHeight = weather->config.ny * SLICE_TEX_SCALE;

    for (int y = 0; y < texHeight; y++) {
        float sampleY = (texHeight > 1) ? ((float)(texHeight - 1 - y) / (float)(texHeight - 1)) * (float)(weather->config.ny - 1) : 0.0f;
        int cellY = Clamp((int)lroundf(sampleY), 0, weather->config.ny - 1);

        for (int j = 0; j < texWidth; j++) {
            int dst = y * texWidth + j;
            if (j >= logicalTexWidth) {
                weather->verticalPixels[dst] = BLANK;
                continue;
            }

            float sampleJ = (logicalTexWidth > 1) ? ((float)j / (float)(logicalTexWidth - 1)) * (float)(logicalWidth - 1) : 0.0f;
            float sampleX = (weather->view.verticalAxis == WEATHER_SLICE_X) ? (float)slice : sampleJ;
            float sampleZ = (weather->view.verticalAxis == WEATHER_SLICE_X) ? sampleJ : (float)slice;
            int cellX = Clamp((int)lroundf(sampleX), 0, weather->config.nx - 1);
            int cellZ = Clamp((int)lroundf(sampleZ), 0, weather->config.nz - 1);
            int column = ColumnIndex(weather, cellX, cellZ);

            if (!IsFluidCell(weather, cellX, cellY, cellZ)) {
                float heightNorm = NormalizeRange(weather->terrainHeight[column], 0.0f, weather->config.topHeight);
                weather->verticalPixels[dst] = TerrainSliceColor(heightNorm);
                continue;
            }

            float value = FieldSampleAt(weather, sampleX, sampleY, sampleZ, weather->view.field);
            Color color = FieldColor(weather, weather->view.field, value);
            weather->verticalPixels[dst] = WithAlpha(color, 235);
        }
    }

    UpdateTexture(weather->verticalTexture, weather->verticalPixels);
}

void WeatherConfig_Default(WeatherConfig *config, const TerrainSurface *terrain) {
    if (config == NULL) return;
    config->nx = 72;
    config->ny = 28;
    config->nz = 72;
    config->topHeight = (terrain != NULL) ? terrain->maxHeight + 180.0f : 280.0f;
    config->fixedDt = 0.35f;
    config->maxCatchup = 6.0f;
    config->frameBudgetSec = 0.0055f;
    config->timeScale = 180.0f;
}

bool Weather_Init(WeatherSystem *weather, const WeatherConfig *config, const TerrainSurface *terrain) {
    if (weather == NULL || config == NULL || terrain == NULL) return false;
    memset(weather, 0, sizeof(*weather));

    weather->config = *config;
    weather->view.field = WEATHER_FIELD_TEMPERATURE;
    weather->view.verticalAxis = WEATHER_SLICE_Z;
    weather->view.horizontalLayer = weather->config.ny / 3;
    weather->view.verticalSlice = weather->config.nz / 2;
    weather->view.showHorizontalSlice = true;
    weather->view.showVerticalSlice = true;
    weather->view.showWindVectors = true;
    weather->view.paused = false;

    weather->cells2D = weather->config.nx * weather->config.nz;
    weather->cells3D = weather->cells2D * weather->config.ny;
    weather->worldWidth = terrain->worldWidth;
    weather->worldDepth = terrain->worldDepth;
    weather->dx = weather->worldWidth / (float)(weather->config.nx - 1);
    weather->dz = weather->worldDepth / (float)(weather->config.nz - 1);
    weather->dy = weather->config.topHeight / (float)weather->config.ny;

    weather->surfaceTemp = (float *)calloc((size_t)weather->cells2D, sizeof(float));
    weather->surfaceMoisture = (float *)calloc((size_t)weather->cells2D, sizeof(float));
    weather->surfaceRain = (float *)calloc((size_t)weather->cells2D, sizeof(float));
    weather->terrainHeight = (float *)calloc((size_t)weather->cells2D, sizeof(float));
    weather->terrainGradX = (float *)calloc((size_t)weather->cells2D, sizeof(float));
    weather->terrainGradZ = (float *)calloc((size_t)weather->cells2D, sizeof(float));
    weather->terrainSlope = (float *)calloc((size_t)weather->cells2D, sizeof(float));
    weather->groundLayer = (int *)calloc((size_t)weather->cells2D, sizeof(int));

    weather->temperature = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->humidity = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->cloud = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->rain = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->windU = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->windV = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->windW = (float *)calloc((size_t)weather->cells3D, sizeof(float));

    weather->temperatureNext = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->humidityNext = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->cloudNext = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->rainNext = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->windUNext = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->windVNext = (float *)calloc((size_t)weather->cells3D, sizeof(float));
    weather->windWNext = (float *)calloc((size_t)weather->cells3D, sizeof(float));

    int horizontalTexWidth = weather->config.nx * SLICE_TEX_SCALE;
    int horizontalTexHeight = weather->config.nz * SLICE_TEX_SCALE;
    int verticalTexWidth = ((weather->config.nx > weather->config.nz) ? weather->config.nx : weather->config.nz) * SLICE_TEX_SCALE;
    int verticalTexHeight = weather->config.ny * SLICE_TEX_SCALE;

    weather->horizontalPixels = (Color *)calloc((size_t)horizontalTexWidth * (size_t)horizontalTexHeight, sizeof(Color));
    weather->verticalPixels = (Color *)calloc((size_t)verticalTexWidth * (size_t)verticalTexHeight, sizeof(Color));

    if (!weather->surfaceTemp || !weather->surfaceMoisture || !weather->surfaceRain ||
        !weather->terrainHeight || !weather->terrainGradX || !weather->terrainGradZ || !weather->terrainSlope ||
        !weather->groundLayer || !weather->temperature || !weather->humidity || !weather->cloud || !weather->rain ||
        !weather->windU || !weather->windV || !weather->windW || !weather->temperatureNext ||
        !weather->humidityNext || !weather->cloudNext || !weather->rainNext || !weather->windUNext ||
        !weather->windVNext || !weather->windWNext || !weather->horizontalPixels || !weather->verticalPixels) {
        Weather_Shutdown(weather);
        return false;
    }

    Image hImage = GenImageColor(horizontalTexWidth, horizontalTexHeight, BLANK);
    Image vImage = GenImageColor(verticalTexWidth, verticalTexHeight, BLANK);
    weather->horizontalTexture = LoadTextureFromImage(hImage);
    weather->verticalTexture = LoadTextureFromImage(vImage);
    UnloadImage(hImage);
    UnloadImage(vImage);
    if (weather->horizontalTexture.id == 0 || weather->verticalTexture.id == 0) {
        Weather_Shutdown(weather);
        return false;
    }

    SetTextureFilter(weather->horizontalTexture, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(weather->verticalTexture, TEXTURE_FILTER_BILINEAR);
    weather->horizontalSliceModel = LoadModelFromMesh(GenMeshPlane(weather->worldWidth, weather->worldDepth, 1, 1));
    weather->verticalSliceModelX = LoadModelFromMesh(GenMeshPlane(weather->config.topHeight, weather->worldDepth, 1, 1));
    weather->verticalSliceModelZ = LoadModelFromMesh(GenMeshPlane(weather->worldWidth, weather->config.topHeight, 1, 1));
    if (weather->horizontalSliceModel.meshCount == 0 || weather->verticalSliceModelX.meshCount == 0 || weather->verticalSliceModelZ.meshCount == 0) {
        Weather_Shutdown(weather);
        return false;
    }

    weather->horizontalSliceModel.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = weather->horizontalTexture;
    weather->verticalSliceModelX.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = weather->verticalTexture;
    weather->verticalSliceModelZ.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = weather->verticalTexture;

    Weather_Reset(weather, terrain);
    return true;
}

void Weather_Shutdown(WeatherSystem *weather) {
    if (weather == NULL) return;
    if (weather->horizontalSliceModel.meshCount > 0) UnloadModel(weather->horizontalSliceModel);
    if (weather->verticalSliceModelX.meshCount > 0) UnloadModel(weather->verticalSliceModelX);
    if (weather->verticalSliceModelZ.meshCount > 0) UnloadModel(weather->verticalSliceModelZ);
    if (weather->horizontalTexture.id != 0) UnloadTexture(weather->horizontalTexture);
    if (weather->verticalTexture.id != 0) UnloadTexture(weather->verticalTexture);

    free(weather->surfaceTemp);
    free(weather->surfaceMoisture);
    free(weather->surfaceRain);
    free(weather->terrainHeight);
    free(weather->terrainGradX);
    free(weather->terrainGradZ);
    free(weather->terrainSlope);
    free(weather->groundLayer);
    free(weather->temperature);
    free(weather->humidity);
    free(weather->cloud);
    free(weather->rain);
    free(weather->windU);
    free(weather->windV);
    free(weather->windW);
    free(weather->temperatureNext);
    free(weather->humidityNext);
    free(weather->cloudNext);
    free(weather->rainNext);
    free(weather->windUNext);
    free(weather->windVNext);
    free(weather->windWNext);
    free(weather->horizontalPixels);
    free(weather->verticalPixels);
    memset(weather, 0, sizeof(*weather));
}

void Weather_Reset(WeatherSystem *weather, const TerrainSurface *terrain) {
    if (weather == NULL || terrain == NULL) return;
    weather->simTimeSeconds = 11.0f * 3600.0f;
    weather->accumulatorSeconds = 0.0f;
    weather->stats.stepsLastFrame = 0;
    weather->stats.solverCpuMilliseconds = 0.0;
    weather->sliceDirty = true;
    weather->lastSliceUpdateTime = 0.0;

    BuildTerrainColumns(weather, terrain);
    InitializeState(weather);
    UpdateStats(weather);
}

void Weather_Update(WeatherSystem *weather, const TerrainSurface *terrain, float frameDt) {
    (void)terrain;
    if (weather == NULL) return;

    float scaledDt = Clamp(frameDt, 0.0f, 0.05f) * weather->config.timeScale;
    weather->accumulatorSeconds += scaledDt;
    if (weather->accumulatorSeconds > weather->config.maxCatchup) {
        weather->accumulatorSeconds = weather->config.maxCatchup;
    }

    double start = GetTime();
    int steps = 0;
    while (weather->accumulatorSeconds >= weather->config.fixedDt) {
        if ((GetTime() - start) >= weather->config.frameBudgetSec) break;
        StepWeather(weather);
        weather->accumulatorSeconds -= weather->config.fixedDt;
        steps++;
    }

    weather->stats.stepsLastFrame = steps;
    weather->stats.solverCpuMilliseconds = (GetTime() - start) * 1000.0;
    if (steps > 0) weather->sliceDirty = true;
    UpdateStats(weather);
}

void Weather_MarkViewDirty(WeatherSystem *weather) {
    if (weather == NULL) return;
    weather->sliceDirty = true;
}

void Weather_UpdateSliceTextures(WeatherSystem *weather, bool force) {
    if (weather == NULL) return;
    double now = GetTime();
    double minInterval = 1.0 / SLICE_UPDATE_RATE_HZ;
    bool ready = force || weather->lastSliceUpdateTime <= 0.0 || (now - weather->lastSliceUpdateTime) >= minInterval;
    if (!weather->sliceDirty || !ready) return;

    if (weather->view.showHorizontalSlice) UpdateHorizontalTexture(weather);
    if (weather->view.showVerticalSlice) UpdateVerticalTexture(weather);
    weather->sliceDirty = false;
    weather->lastSliceUpdateTime = now;
}

void Weather_DrawSlices3D(const WeatherSystem *weather) {
    if (weather == NULL) return;

    rlDisableBackfaceCulling();
    if (weather->view.showHorizontalSlice) {
        float y = CellAltitude(weather, weather->view.horizontalLayer);
        DrawModel(weather->horizontalSliceModel,
                  (Vector3){ weather->worldWidth * 0.5f, y, weather->worldDepth * 0.5f },
                  1.0f,
                  WHITE);
    }

    if (weather->view.showVerticalSlice) {
        if (weather->view.verticalAxis == WEATHER_SLICE_X) {
            float x = (float)Clamp(weather->view.verticalSlice, 0, weather->config.nx - 1) * weather->dx;
            DrawModelEx(weather->verticalSliceModelX,
                        (Vector3){ x, weather->config.topHeight * 0.5f, weather->worldDepth * 0.5f },
                        (Vector3){ 0.0f, 0.0f, 1.0f },
                        90.0f,
                        (Vector3){ 1.0f, 1.0f, 1.0f },
                        WHITE);
        } else {
            float z = (float)Clamp(weather->view.verticalSlice, 0, weather->config.nz - 1) * weather->dz;
            DrawModelEx(weather->verticalSliceModelZ,
                        (Vector3){ weather->worldWidth * 0.5f, weather->config.topHeight * 0.5f, z },
                        (Vector3){ 1.0f, 0.0f, 0.0f },
                        90.0f,
                        (Vector3){ 1.0f, 1.0f, 1.0f },
                        WHITE);
        }
    }
    rlEnableBackfaceCulling();
}

void Weather_DrawWindVectors(const WeatherSystem *weather) {
    if (weather == NULL || !weather->view.showWindVectors) return;

    if (weather->view.showHorizontalSlice) {
        int layer = Clamp(weather->view.horizontalLayer, 0, weather->config.ny - 1);
        float y = CellAltitude(weather, layer);
        for (int z = 0; z < weather->config.nz; z += HSLICE_ARROW_STEP) {
            for (int x = 0; x < weather->config.nx; x += HSLICE_ARROW_STEP) {
                if (!IsFluidCell(weather, x, layer, z)) continue;
                int index = CellIndex(weather, x, layer, z);
                float speed = sqrtf(weather->windU[index] * weather->windU[index] +
                                    weather->windV[index] * weather->windV[index] +
                                    weather->windW[index] * weather->windW[index]);
                if (speed < 0.8f) continue;
                Vector3 start = { (float)x * weather->dx, y, (float)z * weather->dz };
                Vector3 tip = {
                    start.x + weather->windU[index] * 0.18f,
                    start.y + weather->windW[index] * 0.18f,
                    start.z + weather->windV[index] * 0.18f
                };
                DrawLine3D(start, tip, SKYBLUE);
            }
        }
    }

    if (weather->view.showVerticalSlice) {
        int slice = Clamp(weather->view.verticalSlice,
                          0,
                          (weather->view.verticalAxis == WEATHER_SLICE_X) ? weather->config.nx - 1 : weather->config.nz - 1);

        if (weather->view.verticalAxis == WEATHER_SLICE_X) {
            float xWorld = (float)slice * weather->dx;
            for (int z = 0; z < weather->config.nz; z += VSLICE_ARROW_STEP) {
                for (int y = 0; y < weather->config.ny; y += VSLICE_ARROW_STEP) {
                    if (!IsFluidCell(weather, slice, y, z)) continue;
                    int index = CellIndex(weather, slice, y, z);
                    float speed = sqrtf(weather->windV[index] * weather->windV[index] + weather->windW[index] * weather->windW[index]);
                    if (speed < 0.6f) continue;
                    Vector3 start = { xWorld, CellAltitude(weather, y), (float)z * weather->dz };
                    Vector3 tip = { start.x, start.y + weather->windW[index] * 0.2f, start.z + weather->windV[index] * 0.18f };
                    DrawLine3D(start, tip, SKYBLUE);
                }
            }
        } else {
            float zWorld = (float)slice * weather->dz;
            for (int x = 0; x < weather->config.nx; x += VSLICE_ARROW_STEP) {
                for (int y = 0; y < weather->config.ny; y += VSLICE_ARROW_STEP) {
                    if (!IsFluidCell(weather, x, y, slice)) continue;
                    int index = CellIndex(weather, x, y, slice);
                    float speed = sqrtf(weather->windU[index] * weather->windU[index] + weather->windW[index] * weather->windW[index]);
                    if (speed < 0.6f) continue;
                    Vector3 start = { (float)x * weather->dx, CellAltitude(weather, y), zWorld };
                    Vector3 tip = { start.x + weather->windU[index] * 0.18f, start.y + weather->windW[index] * 0.2f, start.z };
                    DrawLine3D(start, tip, SKYBLUE);
                }
            }
        }
    }
}

const char *Weather_FieldLabel(WeatherField field) {
    return (field >= 0 && field < WEATHER_FIELD_COUNT) ? kFieldLabels[field] : "Field";
}

const char *Weather_FieldUnits(WeatherField field) {
    return (field >= 0 && field < WEATHER_FIELD_COUNT) ? kFieldUnits[field] : "";
}

void Weather_FieldDisplayRange(const WeatherSystem *weather, WeatherField field, float *outMin, float *outMax) {
    (void)weather;
    switch (field) {
        case WEATHER_FIELD_TEMPERATURE: *outMin = -20.0f; *outMax = 24.0f; break;
        case WEATHER_FIELD_HUMIDITY: *outMin = 0.0f; *outMax = 100.0f; break;
        case WEATHER_FIELD_CLOUD: *outMin = 0.0f; *outMax = 4.5f; break;
        case WEATHER_FIELD_RAIN: *outMin = 0.0f; *outMax = 4.0f; break;
        case WEATHER_FIELD_WIND: *outMin = 0.0f; *outMax = 18.0f; break;
        case WEATHER_FIELD_VERTICAL_WIND: *outMin = -4.0f; *outMax = 4.0f; break;
        default: *outMin = 0.0f; *outMax = 1.0f; break;
    }
}
