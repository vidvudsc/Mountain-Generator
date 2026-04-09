#ifndef WEATHER_H
#define WEATHER_H

#include "raylib.h"
#include "terrain.h"
#include <stdbool.h>

typedef enum WeatherField {
    WEATHER_FIELD_TEMPERATURE = 0,
    WEATHER_FIELD_HUMIDITY,
    WEATHER_FIELD_CLOUD,
    WEATHER_FIELD_RAIN,
    WEATHER_FIELD_WIND,
    WEATHER_FIELD_VERTICAL_WIND,
    WEATHER_FIELD_COUNT
} WeatherField;

typedef enum WeatherSliceAxis {
    WEATHER_SLICE_X = 0,
    WEATHER_SLICE_Z
} WeatherSliceAxis;

typedef struct WeatherConfig {
    int nx;
    int ny;
    int nz;
    float topHeight;
    float fixedDt;
    float maxCatchup;
    float frameBudgetSec;
    float timeScale;
} WeatherConfig;

typedef struct WeatherViewSettings {
    WeatherField field;
    WeatherSliceAxis verticalAxis;
    int horizontalLayer;
    int verticalSlice;
    bool showHorizontalSlice;
    bool showVerticalSlice;
    bool showWindVectors;
    bool paused;
} WeatherViewSettings;

typedef struct WeatherStats {
    float meanSurfaceTemp;
    float meanHumidity;
    float maxCloud;
    float maxRain;
    float maxWind;
    int stepsLastFrame;
    float backlogSeconds;
    double solverCpuMilliseconds;
} WeatherStats;

typedef struct WeatherSystem {
    WeatherConfig config;
    WeatherViewSettings view;
    WeatherStats stats;

    int cells2D;
    int cells3D;
    float worldWidth;
    float worldDepth;
    float dx;
    float dy;
    float dz;
    float simTimeSeconds;
    float accumulatorSeconds;

    float *surfaceTemp;
    float *surfaceMoisture;
    float *surfaceRain;
    float *terrainHeight;
    float *terrainGradX;
    float *terrainGradZ;
    float *terrainSlope;
    int *groundLayer;

    float *temperature;
    float *humidity;
    float *cloud;
    float *rain;
    float *windU;
    float *windV;
    float *windW;

    float *temperatureNext;
    float *humidityNext;
    float *cloudNext;
    float *rainNext;
    float *windUNext;
    float *windVNext;
    float *windWNext;

    Texture2D horizontalTexture;
    Texture2D verticalTexture;
    Color *horizontalPixels;
    Color *verticalPixels;
    Model horizontalSliceModel;
    Model verticalSliceModelX;
    Model verticalSliceModelZ;
    bool sliceDirty;
    double lastSliceUpdateTime;
} WeatherSystem;

void WeatherConfig_Default(WeatherConfig *config, const TerrainSurface *terrain);
bool Weather_Init(WeatherSystem *weather, const WeatherConfig *config, const TerrainSurface *terrain);
void Weather_Shutdown(WeatherSystem *weather);
void Weather_Reset(WeatherSystem *weather, const TerrainSurface *terrain);
void Weather_Update(WeatherSystem *weather, const TerrainSurface *terrain, float frameDt);
void Weather_MarkViewDirty(WeatherSystem *weather);
void Weather_UpdateSliceTextures(WeatherSystem *weather, bool force);
void Weather_DrawSlices3D(const WeatherSystem *weather);
void Weather_DrawWindVectors(const WeatherSystem *weather);
const char *Weather_FieldLabel(WeatherField field);
const char *Weather_FieldUnits(WeatherField field);
void Weather_FieldDisplayRange(const WeatherSystem *weather, WeatherField field, float *outMin, float *outMax);

#endif
