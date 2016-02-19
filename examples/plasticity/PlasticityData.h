#ifndef PLASTICITY_DATA_H
#define PLASTICITY_DATA_H
#define BPT_EP2  21
#include <list>
#include <string>
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"

using namespace std;
class PlasticityData
{
public:
	PlasticityData(char * value);
	~PlasticityData();
	string getValue(){ return value; }
	static void setData(list<PlasticityData>* l);
	static list<PlasticityData>* getData();
	static void setCollect(bool v);
	static bool getCollect();
	// for plog
	static void setTime(btScalar time);
	static void setLogData(bool v);
	static bool getLogData();
	static char* getLogDataFilename();
	static void log(btTypedConstraint::btConstraintInfo2 * data, int mode);
private:
	string value="";
};
#endif 