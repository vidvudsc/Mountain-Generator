#ifndef TERRAIN_H
#define TERRAIN_H

#include "sim.h"

void FreeTerrain(TerrainGrid *terrain);
void DefaultTerrainGenSettings(TerrainGenSettings *settings, int seed);
bool GenerateProceduralTerrain(TerrainGrid *terrain, const TerrainGenSettings *settings);
Model BuildTerrainModel(const TerrainGrid *terrain, const TerrainGenSettings *settings);

#endif
