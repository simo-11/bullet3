#ifndef PLASTICITY_STATISTICS_H
#define PLASTICITY_STATISTICS_H
#include <list>
#include "PlasticityData.h"
class PlasticityStatistics* setupPlasticityWindow(struct GwenInternalData* data);
void processPlasticityData(PlasticityStatistics* window, bool idle);
void plasticityStatisticsWindowSetVisible(PlasticityStatistics* window, bool visible);
void destroyPlasticityWindow(PlasticityStatistics* window);
#endif//PLASTICITY_STATISTICS_H


