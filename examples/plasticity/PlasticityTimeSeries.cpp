#include "PlasticityTimeSeries.h"
#include "../plasticity/PlasticityExampleBrowser.h"
PlasticityTimeSeries::PlasticityTimeSeries(){

}
void PlasticityTimeSeries::plot(){
	if (dataSourceLabels.size() == 0){
		dataSourceLabels.push_back("");
	}
	btAlignedObjectArray<float> values;
	for (int i = 0; i < dataSourceLabels.size(); i++){
		dataSourceIndex = i;
		values.push_back(getValue());
	}
	if (0 == tsc && maxValue>0){
		getScale();
		CommonGraphicsApp * app = PlasticityExampleBrowser::getApp();
		tsc = new TimeSeriesCanvas
			(app->m_2dCanvasInterface, width, height, getWindowTitle());
		tsc->setupTimeSeries(100., ticksPerSecond, 0);
		maxValue = 0.f;
		int dataSources = dataSourceLabels.size();
		for (int i = 0; i < dataSources; i++){
			unsigned char rgbs[3];
			PlasticityUtils::fillRgbs(rgbs,dataSources,i);
			tsc->addDataSource(dataSourceLabels[i], rgbs[0], rgbs[1], rgbs[2]);
			maxValue = btMax(btFabs(values[i]), maxValue);
		}
	}
	if (tsc != 0){
		for (int i = 0; i < dataSourceLabels.size(); i++){
			tsc->insertDataAtCurrentTime(100.f*values[i]/scale, i, connectToPrevious);
		}
		tsc->nextTick();
	}
}

float PlasticityTimeSeries::getValue(){
	float val = (*cb)(this);
	float absVal = btFabs(val);
	if (absVal>maxValue){
		maxValue = absVal;
	}
	if (absVal < 2 * scale){
		connectToPrevious = true;
	}
	else{
		connectToPrevious = false;
	}
	return val;
}

const char * PlasticityTimeSeries::getWindowTitle(){
	sprintf_s(windowTitleBuffer, PTS_WINDOW_TITLE_LEN, "%s as %% - max %8.4g", title, scale);
	return windowTitleBuffer;
}
/**
provide clean scaling value
*/
float PlasticityTimeSeries::getScale(){
	scale = PlasticityUtils::roundUpWithDigits(maxValue,2);
	return scale;
}


void PlasticityTimeSeries::clear(btAlignedObjectArray<PlasticityTimeSeries*> &tsArray){
	for (int j = 0; j<tsArray.size(); j++)
	{
		PlasticityTimeSeries* ts = tsArray[j];
		delete ts;
	}
	tsArray.clear();
}

void PlasticityTimeSeries::deleteTs(btAlignedObjectArray<PlasticityTimeSeries*> &tsArray){
	for (int j = 0; j < tsArray.size(); j++)
	{
		PlasticityTimeSeries* ts = tsArray[j];
		ts->deleteTs();
	}
}

void PlasticityTimeSeries::plot(btAlignedObjectArray<PlasticityTimeSeries*> &tsArray){
	for (int j = 0; j < tsArray.size(); j++)
	{
		PlasticityTimeSeries* ts = tsArray[j];
		ts->plot();
	}
}

void PlasticityTimeSeries::updateParameters(btAlignedObjectArray<PlasticityTimeSeries*> &tsArray, 
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
