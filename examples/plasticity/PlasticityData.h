#ifndef PLASTICITY_DATA_H
#define PLASTICITY_DATA_H
#include <list>
#include <string>
using namespace std;

class PlasticityData
{
public:
	PlasticityData(char * value);
	string getValue(){ return value; }
	static void setData(list<PlasticityData>);
	static list<PlasticityData> getData();
private:
	string value="";
};
#endif 