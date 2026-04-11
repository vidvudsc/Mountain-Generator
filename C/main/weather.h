#ifndef WEATHER_H
#define WEATHER_H

#include "terrain.h"

Rectangle GetControlPanelRect(void);
Rectangle GetUiToggleRect(void);
void ApplyImGuiTheme(void);

void DestroySim(AtmosphereSim *sim);
void ResetAtmosphere(AtmosphereSim *sim);
void UpdateAtmosphere(AtmosphereSim *sim, float frameDt);
void UpdateSliceTexturesIfNeeded(AtmosphereSim *sim, bool force);
Color SkyColor(const AtmosphereSim *sim);
void DrawSlices3D(const AtmosphereSim *sim, Camera camera);
void DrawImGuiHud(const AtmosphereSim *sim);
bool DrawImGuiControls(AtmosphereSim *sim);
bool CreateSimFromGeneratedTerrain(AtmosphereSim *sim);

#endif
