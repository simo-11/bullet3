#ifndef ENERGY_SUM_H
#define ENERGY_SUM_H
#include "btBulletDynamicsCommon.h"
#include "../CharpyDemo/bt6DofElasticPlasticConstraint.h"
#include "../CharpyDemo/bt6DofElasticPlastic2Constraint.h"

class EnergySum
{
public:
	btDynamicsWorld* m_dw = 0;
	float linear, inertia, gravitational,elastic,plastic;
	EnergySum(btDynamicsWorld* dw);
	~EnergySum();
	float getTotal();
	void update();
	static float getRotationEnergy(btScalar invInertia, btScalar angularVelocity);
	static float getElasticEnergy(btGeneric6DofSpringConstraint *sc);
	static float getElasticEnergy(btGeneric6DofSpring2Constraint*);
};
#endif