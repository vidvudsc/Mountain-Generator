#ifndef TERRAIN_H
#define TERRAIN_H

#include "raylib.h"
#include <stdbool.h>

typedef struct TerrainConfig {
    int seed;
    int resolution;
    int octaves;
    int smoothingPasses;
    int thermalPasses;
    float baseFrequency;
    float warpFrequency;
    float lacunarity;
    float gain;
    float ridgeSharpness;
    float peakBias;
    float thermalTalus;
    float worldScale;
    float heightScale;
    float baseHeight;
} TerrainConfig;

typedef struct TerrainSample {
    float height;
    float gradX;
    float gradZ;
    float slope;
    Vector3 normal;
} TerrainSample;

typedef struct TerrainSurface {
    TerrainConfig config;
    int width;
    int height;
    float worldWidth;
    float worldDepth;
    float minHeight;
    float maxHeight;
    float *heightMap;
    float *gradX;
    float *gradZ;
    float *slope;
    Model model;
} TerrainSurface;

void TerrainConfig_Default(TerrainConfig *config, int seed);
bool Terrain_Init(TerrainSurface *terrain, const TerrainConfig *config);
bool Terrain_Rebuild(TerrainSurface *terrain, const TerrainConfig *config);
void Terrain_Shutdown(TerrainSurface *terrain);
void Terrain_Draw(const TerrainSurface *terrain, float alpha, bool wireframe);
TerrainSample Terrain_Sample(const TerrainSurface *terrain, float x, float z);

#endif
