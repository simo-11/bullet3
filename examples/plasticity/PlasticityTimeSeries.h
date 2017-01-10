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
};
#endif // PLASTICITY_TIME_SERIES_H
