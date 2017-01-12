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
	int stepCount = 0;
	char * title="PlasticityTimeSeries";
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
	static void deleteTs(btAlignedObjectArray<PlasticityTimeSeries*>);
	static void clear(btAlignedObjectArray<PlasticityTimeSeries*>);
	static void plot(btAlignedObjectArray<PlasticityTimeSeries*>);
	static void update(btAlignedObjectArray<PlasticityTimeSeries*>,int ticksPerSecond);
	void plot();
	float maxValue=0;
	float getScale();
	float getValue();
};
#endif // PLASTICITY_TIME_SERIES_H
