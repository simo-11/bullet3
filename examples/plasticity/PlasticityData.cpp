#include "PlasticityData.h"
static bool collect = false;
bool PlasticityData::getCollect(){
	return collect;
}
void PlasticityData::setCollect(bool v){
	collect = v;
}
static list<PlasticityData> s_pData;
void PlasticityData::setData(list<PlasticityData> pData){
	s_pData = pData;
}
list<PlasticityData> PlasticityData::getData(){
	return s_pData;
}

PlasticityData::PlasticityData(char * buf){
	value = buf;
}