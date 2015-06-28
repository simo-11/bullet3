#include "PlasticityData.h"
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