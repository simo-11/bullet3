/*
Simo Nikula, 2016/07
*/

#ifndef _BT_ELASTIC_PLASTIC_CONSTRAINT_INTERFACE_H
#define _BT_ELASTIC_PLASTIC_CONSTRAINT_INTERFACE_H

#include "LinearMath/btScalar.h"

///Basic interface to allow common actions for various elasticPlasticConstraints
class btElasticPlasticConstraint
{
public:
	virtual btScalar getMaxPlasticRotation()=0;
	virtual btScalar getCurrentPlasticRotation() = 0;
	virtual btScalar getMaxPlasticStrain() = 0;
	virtual btScalar getCurrentPlasticStrain() = 0;
	virtual btScalar getMaxRatio() = 0;
	virtual int getMaxRatioDof() = 0;
};

#endif //_BT_ELASTIC_PLASTIC_CONSTRAINT_INTERFACE_H

