#include "terrain.h"
#include "raymath.h"
#include <float.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

static int TerrainIndex(const TerrainSurface *terrain, int x, int z) {
    return z * terrain->width + x;
}

static float Clamp01(float value) {
    return Clamp(value, 0.0f, 1.0f);
}

static Color LerpColor(Color a, Color b, float t) {
    t = Clamp01(t);
    return (Color) {
        (unsigned char)Lerp((float)a.r, (float)b.r, t),
        (unsigned char)Lerp((float)a.g, (float)b.g, t),
        (unsigned char)Lerp((float)a.b, (float)b.b, t),
        (unsigned char)Lerp((float)a.a, (float)b.a, t)
    };
}

static unsigned int Hash2D(int x, int z, int seed) {
    unsigned int h = (unsigned int)(x * 0x8da6b343u) ^ (unsigned int)(z * 0xd8163841u) ^ (unsigned int)(seed * 0xcb1ab31fu);
    h ^= h >> 13;
    h *= 0x85ebca6bu;
    h ^= h >> 16;
    return h;
}

static float Gradient2D(unsigned int hash, float x, float z) {
    switch (hash & 7u) {
        case 0: return x + z;
        case 1: return x - z;
        case 2: return -x + z;
        case 3: return -x - z;
        case 4: return x;
        case 5: return -x;
        case 6: return z;
        default: return -z;
    }
}

static float SmoothStep5(float t) {
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static float Perlin2D(float x, float z, int seed) {
    int x0 = (int)floorf(x);
    int z0 = (int)floorf(z);
    int x1 = x0 + 1;
    int z1 = z0 + 1;

    float tx = x - (float)x0;
    float tz = z - (float)z0;
    float u = SmoothStep5(tx);
    float v = SmoothStep5(tz);

    float n00 = Gradient2D(Hash2D(x0, z0, seed), tx, tz);
    float n10 = Gradient2D(Hash2D(x1, z0, seed), tx - 1.0f, tz);
    float n01 = Gradient2D(Hash2D(x0, z1, seed), tx, tz - 1.0f);
    float n11 = Gradient2D(Hash2D(x1, z1, seed), tx - 1.0f, tz - 1.0f);

    return Lerp(Lerp(n00, n10, u), Lerp(n01, n11, u), v);
}

static float Fbm2D(float x, float z, int seed, int octaves, float lacunarity, float gain) {
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float sum = 0.0f;
    float norm = 0.0f;

    for (int octave = 0; octave < octaves; octave++) {
        sum += Perlin2D(x * frequency, z * frequency, seed + octave * 37) * amplitude;
        norm += amplitude;
        amplitude *= gain;
        frequency *= lacunarity;
    }

    return (norm > 0.0f) ? (sum / norm) : 0.0f;
}

static float Ridged2D(float x, float z, int seed, int octaves, float lacunarity, float gain, float sharpness) {
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float sum = 0.0f;
    float norm = 0.0f;

    for (int octave = 0; octave < octaves; octave++) {
        float value = 1.0f - fabsf(Perlin2D(x * frequency, z * frequency, seed + octave * 53));
        value = powf(Clamp01(value), sharpness);
        sum += value * amplitude;
        norm += amplitude;
        amplitude *= gain;
        frequency *= lacunarity;
    }

    return (norm > 0.0f) ? (sum / norm) : 0.0f;
}

static void NormalizeField(float *values, int count) {
    float minValue = FLT_MAX;
    float maxValue = -FLT_MAX;
    for (int i = 0; i < count; i++) {
        if (values[i] < minValue) minValue = values[i];
        if (values[i] > maxValue) maxValue = values[i];
    }

    float span = fmaxf(maxValue - minValue, 0.0001f);
    for (int i = 0; i < count; i++) {
        values[i] = Clamp01((values[i] - minValue) / span);
    }
}

static void SmoothField(float *values, int width, int height, int passes) {
    if (passes <= 0) return;

    int count = width * height;
    float *scratch = (float *)malloc((size_t)count * sizeof(float));
    if (scratch == NULL) return;

    for (int pass = 0; pass < passes; pass++) {
        memcpy(scratch, values, (size_t)count * sizeof(float));

        for (int z = 1; z < height - 1; z++) {
            for (int x = 1; x < width - 1; x++) {
                float center = scratch[z * width + x] * 4.0f;
                float cardinal = scratch[z * width + x - 1] +
                                 scratch[z * width + x + 1] +
                                 scratch[(z - 1) * width + x] +
                                 scratch[(z + 1) * width + x];
                float diagonal = scratch[(z - 1) * width + x - 1] +
                                 scratch[(z - 1) * width + x + 1] +
                                 scratch[(z + 1) * width + x - 1] +
                                 scratch[(z + 1) * width + x + 1];
                values[z * width + x] = (center + cardinal * 2.0f + diagonal) / 16.0f;
            }
        }
    }

    free(scratch);
}

static void ThermalErode(float *values, int width, int height, int passes, float talus) {
    if (passes <= 0) return;

    int count = width * height;
    float *source = (float *)malloc((size_t)count * sizeof(float));
    float *dest = (float *)malloc((size_t)count * sizeof(float));
    if (source == NULL || dest == NULL) {
        free(source);
        free(dest);
        return;
    }

    for (int pass = 0; pass < passes; pass++) {
        memcpy(source, values, (size_t)count * sizeof(float));
        memcpy(dest, source, (size_t)count * sizeof(float));

        for (int z = 1; z < height - 1; z++) {
            for (int x = 1; x < width - 1; x++) {
                int i = z * width + x;
                static const int kOffsets[4][2] = {
                    { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 }
                };

                for (int neighbor = 0; neighbor < 4; neighbor++) {
                    int nx = x + kOffsets[neighbor][0];
                    int nz = z + kOffsets[neighbor][1];
                    int n = nz * width + nx;
                    float drop = source[i] - source[n];
                    if (drop <= talus) continue;

                    float move = (drop - talus) * 0.18f;
                    dest[i] -= move;
                    dest[n] += move;
                }
            }
        }

        memcpy(values, dest, (size_t)count * sizeof(float));
    }

    free(source);
    free(dest);
}

static void ComputeDerivatives(TerrainSurface *terrain) {
    terrain->minHeight = FLT_MAX;
    terrain->maxHeight = -FLT_MAX;

    for (int z = 0; z < terrain->height; z++) {
        for (int x = 0; x < terrain->width; x++) {
            int xl = (x > 0) ? x - 1 : x;
            int xr = (x + 1 < terrain->width) ? x + 1 : x;
            int zd = (z > 0) ? z - 1 : z;
            int zu = (z + 1 < terrain->height) ? z + 1 : z;

            float h = terrain->heightMap[TerrainIndex(terrain, x, z)];
            float hL = terrain->heightMap[TerrainIndex(terrain, xl, z)];
            float hR = terrain->heightMap[TerrainIndex(terrain, xr, z)];
            float hD = terrain->heightMap[TerrainIndex(terrain, x, zd)];
            float hU = terrain->heightMap[TerrainIndex(terrain, x, zu)];

            float gradX = (hR - hL) / fmaxf((float)(xr - xl) * terrain->config.worldScale, 0.0001f);
            float gradZ = (hU - hD) / fmaxf((float)(zu - zd) * terrain->config.worldScale, 0.0001f);
            int i = TerrainIndex(terrain, x, z);
            terrain->gradX[i] = gradX;
            terrain->gradZ[i] = gradZ;
            terrain->slope[i] = sqrtf(gradX * gradX + gradZ * gradZ);

            if (h < terrain->minHeight) terrain->minHeight = h;
            if (h > terrain->maxHeight) terrain->maxHeight = h;
        }
    }
}

static Color TerrainColor(float heightNorm, float slope, float shade) {
    Color valley = (Color){ 42, 78, 52, 255 };
    Color meadow = (Color){ 94, 132, 74, 255 };
    Color alpine = (Color){ 136, 126, 98, 255 };
    Color rock = (Color){ 122, 114, 108, 255 };
    Color snow = (Color){ 231, 236, 240, 255 };
    Color base;

    if (heightNorm > 0.84f) base = LerpColor(rock, snow, Clamp01((heightNorm - 0.84f) / 0.16f));
    else if (slope > 0.68f) base = rock;
    else if (heightNorm > 0.56f) base = LerpColor(meadow, alpine, Clamp01((heightNorm - 0.56f) / 0.28f));
    else base = LerpColor(valley, meadow, Clamp01(heightNorm / 0.56f));

    float lit = 0.28f + 0.72f * shade;
    base.r = (unsigned char)Clamp((float)base.r * lit, 0.0f, 255.0f);
    base.g = (unsigned char)Clamp((float)base.g * lit, 0.0f, 255.0f);
    base.b = (unsigned char)Clamp((float)base.b * lit, 0.0f, 255.0f);
    return base;
}

static Model BuildModel(const TerrainSurface *terrain) {
    const int vertexCount = terrain->width * terrain->height;
    const int triangleCount = (terrain->width - 1) * (terrain->height - 1) * 2;
    const int indexCount = triangleCount * 3;

    Vector3 *vertices = (Vector3 *)MemAlloc((size_t)vertexCount * sizeof(Vector3));
    Vector3 *normals = (Vector3 *)MemAlloc((size_t)vertexCount * sizeof(Vector3));
    Color *colors = (Color *)MemAlloc((size_t)vertexCount * sizeof(Color));
    int *indices = (int *)MemAlloc((size_t)indexCount * sizeof(int));
    if (!vertices || !normals || !colors || !indices || vertexCount > 65536) {
        if (vertices) MemFree(vertices);
        if (normals) MemFree(normals);
        if (colors) MemFree(colors);
        if (indices) MemFree(indices);
        return (Model){ 0 };
    }

    for (int z = 0; z < terrain->height; z++) {
        for (int x = 0; x < terrain->width; x++) {
            int i = TerrainIndex(terrain, x, z);
            vertices[i] = (Vector3) {
                (float)x * terrain->config.worldScale,
                terrain->heightMap[i],
                (float)z * terrain->config.worldScale
            };
            normals[i] = (Vector3){ 0.0f, 0.0f, 0.0f };
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
        Vector3 normal = Vector3Normalize(Vector3CrossProduct(edge1, edge2));
        normals[i0] = Vector3Add(normals[i0], normal);
        normals[i1] = Vector3Add(normals[i1], normal);
        normals[i2] = Vector3Add(normals[i2], normal);
    }

    Vector3 lightDir = Vector3Normalize((Vector3){ 0.42f, 1.0f, 0.36f });
    for (int i = 0; i < vertexCount; i++) {
        normals[i] = Vector3Normalize(normals[i]);
        float shade = Clamp(Vector3DotProduct(normals[i], lightDir), 0.0f, 1.0f);
        float slope = 1.0f - Clamp(Vector3DotProduct(normals[i], (Vector3){ 0.0f, 1.0f, 0.0f }), 0.0f, 1.0f);
        float heightNorm = Clamp01((vertices[i].y - terrain->minHeight) / fmaxf(terrain->maxHeight - terrain->minHeight, 0.0001f));
        colors[i] = TerrainColor(heightNorm, slope, shade);
    }

    Mesh mesh = { 0 };
    mesh.vertexCount = vertexCount;
    mesh.triangleCount = triangleCount;
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
        return (Model){ 0 };
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

    for (int i = 0; i < indexCount; i++) {
        mesh.indices[i] = (unsigned short)indices[i];
    }

    UploadMesh(&mesh, true);

    MemFree(vertices);
    MemFree(normals);
    MemFree(colors);
    MemFree(indices);
    return LoadModelFromMesh(mesh);
}

static void GenerateHeightMap(TerrainSurface *terrain) {
    int count = terrain->width * terrain->height;
    float *values = terrain->heightMap;
    const TerrainConfig *config = &terrain->config;

    for (int z = 0; z < terrain->height; z++) {
        for (int x = 0; x < terrain->width; x++) {
            float fx = (float)x / (float)(terrain->width - 1);
            float fz = (float)z / (float)(terrain->height - 1);

            float warpX = Fbm2D(fx * config->warpFrequency + 13.7f,
                                fz * config->warpFrequency - 5.3f,
                                config->seed + 11,
                                3,
                                2.0f,
                                0.5f) * 0.14f;
            float warpZ = Fbm2D(fx * config->warpFrequency - 7.9f,
                                fz * config->warpFrequency + 3.1f,
                                config->seed + 23,
                                3,
                                2.0f,
                                0.5f) * 0.14f;

            float nx = fx + warpX;
            float nz = fz + warpZ;

            float continent = 0.5f + 0.5f * Fbm2D(nx * config->baseFrequency * 0.38f,
                                                  nz * config->baseFrequency * 0.38f,
                                                  config->seed + 101,
                                                  4,
                                                  2.1f,
                                                  0.52f);
            float ridges = Ridged2D(nx * config->baseFrequency,
                                    nz * config->baseFrequency,
                                    config->seed + 151,
                                    config->octaves,
                                    config->lacunarity,
                                    config->gain,
                                    config->ridgeSharpness);
            float peaks = Ridged2D((nx - warpZ * 0.4f) * config->baseFrequency * 1.85f,
                                   (nz + warpX * 0.4f) * config->baseFrequency * 1.85f,
                                   config->seed + 211,
                                   3,
                                   2.05f,
                                   0.55f,
                                   config->ridgeSharpness + 0.55f);
            float valleys = 0.5f + 0.5f * Fbm2D(nx * config->baseFrequency * 0.72f,
                                                nz * config->baseFrequency * 0.72f,
                                                config->seed + 307,
                                                3,
                                                1.9f,
                                                0.55f);

            float mountainMask = powf(Clamp01(continent), 1.45f);
            float value = ridges * (0.55f + 0.70f * mountainMask) +
                          peaks * 0.22f +
                          continent * 0.33f -
                          (1.0f - valleys) * 0.12f;
            values[z * terrain->width + x] = value;
        }
    }

    NormalizeField(values, count);
    for (int i = 0; i < count; i++) {
        values[i] = powf(values[i], config->peakBias);
    }

    ThermalErode(values, terrain->width, terrain->height, config->thermalPasses, config->thermalTalus);
    SmoothField(values, terrain->width, terrain->height, config->smoothingPasses);
    NormalizeField(values, count);

    for (int i = 0; i < count; i++) {
        values[i] = config->baseHeight + values[i] * config->heightScale;
    }
}

void TerrainConfig_Default(TerrainConfig *config, int seed) {
    if (config == NULL) return;
    config->seed = (seed <= 0) ? 1337 : seed;
    config->resolution = 256;
    config->octaves = 5;
    config->smoothingPasses = 3;
    config->thermalPasses = 18;
    config->baseFrequency = 5.8f;
    config->warpFrequency = 3.3f;
    config->lacunarity = 2.05f;
    config->gain = 0.52f;
    config->ridgeSharpness = 1.85f;
    config->peakBias = 1.45f;
    config->thermalTalus = 0.015f;
    config->worldScale = 1.0f;
    config->heightScale = 118.0f;
    config->baseHeight = 4.0f;
}

bool Terrain_Init(TerrainSurface *terrain, const TerrainConfig *config) {
    if (terrain == NULL || config == NULL) return false;
    memset(terrain, 0, sizeof(*terrain));
    return Terrain_Rebuild(terrain, config);
}

bool Terrain_Rebuild(TerrainSurface *terrain, const TerrainConfig *config) {
    if (terrain == NULL || config == NULL) return false;

    if (terrain->model.meshCount > 0) {
        UnloadModel(terrain->model);
        terrain->model = (Model){ 0 };
    }
    free(terrain->heightMap);
    free(terrain->gradX);
    free(terrain->gradZ);
    free(terrain->slope);
    terrain->heightMap = NULL;
    terrain->gradX = NULL;
    terrain->gradZ = NULL;
    terrain->slope = NULL;

    terrain->config = *config;
    terrain->width = config->resolution;
    terrain->height = config->resolution;
    terrain->worldWidth = (float)(terrain->width - 1) * config->worldScale;
    terrain->worldDepth = (float)(terrain->height - 1) * config->worldScale;

    int count = terrain->width * terrain->height;
    terrain->heightMap = (float *)calloc((size_t)count, sizeof(float));
    terrain->gradX = (float *)calloc((size_t)count, sizeof(float));
    terrain->gradZ = (float *)calloc((size_t)count, sizeof(float));
    terrain->slope = (float *)calloc((size_t)count, sizeof(float));
    if (!terrain->heightMap || !terrain->gradX || !terrain->gradZ || !terrain->slope) {
        Terrain_Shutdown(terrain);
        return false;
    }

    GenerateHeightMap(terrain);
    ComputeDerivatives(terrain);
    terrain->model = BuildModel(terrain);
    return terrain->model.meshCount > 0;
}

void Terrain_Shutdown(TerrainSurface *terrain) {
    if (terrain == NULL) return;
    if (terrain->model.meshCount > 0) UnloadModel(terrain->model);
    free(terrain->heightMap);
    free(terrain->gradX);
    free(terrain->gradZ);
    free(terrain->slope);
    memset(terrain, 0, sizeof(*terrain));
}

void Terrain_Draw(const TerrainSurface *terrain, float alpha, bool wireframe) {
    if (terrain == NULL || terrain->model.meshCount == 0) return;
    DrawModel(terrain->model, (Vector3){ 0.0f, 0.0f, 0.0f }, 1.0f, Fade(WHITE, alpha));
    if (wireframe) {
        DrawModelWires(terrain->model, (Vector3){ 0.0f, 0.0f, 0.0f }, 1.0f, Fade(BLACK, 0.22f));
    }
}

TerrainSample Terrain_Sample(const TerrainSurface *terrain, float x, float z) {
    TerrainSample sample = { 0 };
    if (terrain == NULL || terrain->heightMap == NULL) return sample;

    float gridX = Clamp(x / fmaxf(terrain->config.worldScale, 0.0001f), 0.0f, (float)(terrain->width - 1));
    float gridZ = Clamp(z / fmaxf(terrain->config.worldScale, 0.0001f), 0.0f, (float)(terrain->height - 1));
    int x0 = (int)floorf(gridX);
    int z0 = (int)floorf(gridZ);
    int x1 = (x0 + 1 < terrain->width) ? x0 + 1 : x0;
    int z1 = (z0 + 1 < terrain->height) ? z0 + 1 : z0;
    float tx = gridX - (float)x0;
    float tz = gridZ - (float)z0;

    int i00 = TerrainIndex(terrain, x0, z0);
    int i10 = TerrainIndex(terrain, x1, z0);
    int i01 = TerrainIndex(terrain, x0, z1);
    int i11 = TerrainIndex(terrain, x1, z1);

    sample.height = Lerp(Lerp(terrain->heightMap[i00], terrain->heightMap[i10], tx),
                         Lerp(terrain->heightMap[i01], terrain->heightMap[i11], tx),
                         tz);
    sample.gradX = Lerp(Lerp(terrain->gradX[i00], terrain->gradX[i10], tx),
                        Lerp(terrain->gradX[i01], terrain->gradX[i11], tx),
                        tz);
    sample.gradZ = Lerp(Lerp(terrain->gradZ[i00], terrain->gradZ[i10], tx),
                        Lerp(terrain->gradZ[i01], terrain->gradZ[i11], tx),
                        tz);
    sample.slope = Lerp(Lerp(terrain->slope[i00], terrain->slope[i10], tx),
                        Lerp(terrain->slope[i01], terrain->slope[i11], tx),
                        tz);
    sample.normal = Vector3Normalize((Vector3){ -sample.gradX, 1.0f, -sample.gradZ });
    return sample;
}
