/**
Helper for TimeSeriesPlotting
Simo Nikula, 2017
*/
#ifndef PLASTICITY_TIME_SERIES_H
#define PLASTICITY_TIME_SERIES_H
#include "btBulletDynamicsCommon.h"
#include "../RenderingExamples/TimeSeriesCanvas.h"
#include "../CharpyDemo/btElasticPlasticConstraint.h"
enum SourceType { ElasticPlasticConstraint, RigidBody };
class PlasticityTimeSeries
{
public:
	int width = 300;
	int height = 300;
	int ticksPerSecond=60;
	char * title;
	SourceType sourceType;
	btElasticPlasticConstraint* epc;
	btRigidBody* rb;
	int dof;
	TimeSeriesCanvas* tsc=0;
	PlasticityTimeSeries();
	void deleteTs(){
		if (tsc != 0){
			delete tsc;
			tsc = 0;
		}
	}
	~PlasticityTimeSeries(){
		deleteTs();
	}
	bool connectToPrevious;
	static void clear(btAlignedObjectArray<PlasticityTimeSeries*>);
	void plot();
	float maxValue;
	float getScale();
	float getValue();
};
#endif // PLASTICITY_TIME_SERIES_H
