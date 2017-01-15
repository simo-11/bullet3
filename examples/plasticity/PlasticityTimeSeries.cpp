#include "PlasticityTimeSeries.h"
#include "../plasticity/PlasticityExampleBrowser.h"
PlasticityTimeSeries::PlasticityTimeSeries(){

}
void PlasticityTimeSeries::plot(){
	float value = getValue();
	if (0 == tsc && maxValue>0){
		CommonGraphicsApp * app = PlasticityExampleBrowser::getApp();
		tsc = new TimeSeriesCanvas
			(app->m_2dCanvasInterface, width, height, title);
		tsc->setupTimeSeries(getScale(), ticksPerSecond, 0);
		maxValue = btFabs(value);
		tsc->addDataSource("", 255, 0, 0);
	}
	if (tsc != 0){
		tsc->insertDataAtCurrentTime(value, 0, connectToPrevious);
		tsc->nextTick();
	}
}

float PlasticityTimeSeries::getValue(){
	stepCount++;
	float val = (*cb)(this);
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
	float v = PlasticityUtils::roundWithDigits(maxValue,2);
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

void PlasticityTimeSeries::deleteTs(btAlignedObjectArray<PlasticityTimeSeries*> tsArray){
	for (int j = 0; j < tsArray.size(); j++)
	{
		PlasticityTimeSeries* ts = tsArray[j];
		ts->deleteTs();
	}
}

void PlasticityTimeSeries::plot(btAlignedObjectArray<PlasticityTimeSeries*> tsArray){
	for (int j = 0; j < tsArray.size(); j++)
	{
		PlasticityTimeSeries* ts = tsArray[j];
		ts->plot();
	}
}

void PlasticityTimeSeries::updateParameters(btAlignedObjectArray<PlasticityTimeSeries*> tsArray, 
	int ticksPerSecond,
	float(*cb)(PlasticityTimeSeries* pts)){
	for (int j = 0; j < tsArray.size(); j++)
	{
		PlasticityTimeSeries* ts = tsArray[j];
		if (ticksPerSecond != 0){
			ts->ticksPerSecond = ticksPerSecond;
		}
		if (cb != 0){
			ts->cb = cb;
		}
	}
}
