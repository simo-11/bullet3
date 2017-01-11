#include "PlasticityTimeSeries.h"
#include "../plasticity/PlasticityExampleBrowser.h"
void PlasticityTimeSeries::plot(){
	if (0 == tsc){
		CommonGraphicsApp * app = PlasticityExampleBrowser::getApp();
		tsc = new TimeSeriesCanvas
			(app->m_2dCanvasInterface, width, height, title);
		tsc->setupTimeSeries(getScale(), ticksPerSecond, 0);
		tsc->addDataSource("", 255, 0, 0);
	}
	tsc->insertDataAtCurrentTime(getValue(), 0, connectToPrevious);
	tsc->nextTick();
}

/** TODO: add callback to actual value */
float PlasticityTimeSeries::getValue(){
	float val = 3.0f;
	float absVal = btFabs(val);
	if (absVal>maxValue){
		maxValue = absVal;
	}
	return val;
}
/**
provide clean scaling value
*/
float PlasticityTimeSeries::getScale(){
	float v = ceilf(maxValue);
	return v;
}

void PlasticityTimeSeries::clear(btAlignedObjectArray<PlasticityTimeSeries*> tsArray){
	for (int j = 0; j<tsArray.size(); j++)
	{
		PlasticityTimeSeries* ts = tsArray[j];
		delete ts;
	}
	tsArray.clear();
}

