#include "terrain.h"

void FreeTerrain(TerrainGrid *terrain) {
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

void DefaultTerrainGenSettings(TerrainGenSettings *settings, int seed) {
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

bool GenerateProceduralTerrain(TerrainGrid *terrain, const TerrainGenSettings *settings) {
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

Model BuildTerrainModel(const TerrainGrid *terrain, const TerrainGenSettings *settings) {
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
