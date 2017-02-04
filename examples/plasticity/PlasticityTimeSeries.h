/**
Helper for TimeSeriesPlotting
Provides 
 - automatic scaling on restarts
 - intitialization and cleanup
 - callback provides actual value
Simo Nikula, 2017
*/
#ifndef PLASTICITY_TIME_SERIES_H
#define PLASTICITY_TIME_SERIES_H
#include "btBulletDynamicsCommon.h"
#include "../RenderingExamples/TimeSeriesCanvas.h"
#include "../CharpyDemo/btElasticPlasticConstraint.h"
enum SourceType { ElasticPlasticConstraint, RigidBody, Energy, Custom };
class PlasticityTimeSeries
{
public:
	int width = 400;
	int height = 300;
	int ticksPerSecond=60;
	char * title="PlasticityTimeSeries";
	float(*cb)(PlasticityTimeSeries*) = 0;
	SourceType sourceType;
	btElasticPlasticConstraint* epc=0;
	btRigidBody* rb=0;
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
	btAlignedObjectArray<const char *> dataSourceLabels;
	int dataSourceIndex;
	/** 
	Deletes TimeSeriesCanvas:es within PlasticityTimeSeries but
	keeps e.g. scaling
	*/
	static void deleteTs(btAlignedObjectArray<PlasticityTimeSeries*>&);
	/**
	Complete deletion and clearing of btAlignedObjectArray
	*/
	static void clear(btAlignedObjectArray<PlasticityTimeSeries*>&);
	static void plot(btAlignedObjectArray<PlasticityTimeSeries*>&);
	static void updateParameters(btAlignedObjectArray<PlasticityTimeSeries*>&,
		int ticksPerSecond, float (*cb)(PlasticityTimeSeries* pts)=0);
	void plot();
	float maxValue=0;
	float scale;
	/** stores current maxValue to scale and return scale */
	float getScale(); 
	float getValue();
#define PTS_WINDOW_TITLE_LEN 50
	char windowTitleBuffer[PTS_WINDOW_TITLE_LEN];
	const char * getWindowTitle();
};
#endif // PLASTICITY_TIME_SERIES_H
