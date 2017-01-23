#include "EnergySum.h"
#include "PlasticityData.h"

EnergySum::EnergySum(btDynamicsWorld* dw){
	m_dw = dw;
}
EnergySum::~EnergySum(){

}
float EnergySum::getTotal(){
	return linear + inertia + gravitational + elastic + plastic;
}

float EnergySum::getRotationEnergy(btScalar invInertia, btScalar angularVelocity){
	if (invInertia == 0){
		return 0.f;
	}
	float inertia = 1.f / invInertia;
	float e = 0.5*inertia*angularVelocity*angularVelocity;
	return e;
}

float EnergySum::getElasticEnergy(btGeneric6DofSpringConstraint *sc){
	float energy = 0.f;
	for (int i = 0; i < 6; i++){
		btScalar k = sc->getStiffness(i);
		btScalar d;
		if (i < 3){
			d = sc->getRelativePivotPosition(i);
		}
		else{
			d = sc->getAngle(i - 3);
		}
		energy += 0.5*k*d*d;
	}
	return energy;
}
float EnergySum::getElasticEnergy(btGeneric6DofSpring2Constraint *sc){
	float energy = 0.f;
	for (int i = 0; i < 6; i++){
		btScalar k;
		btScalar d;
		if (i < 3){
			k = sc->getTranslationalLimitMotor()->m_springStiffness[i];
			d = sc->getRelativePivotPosition(i);
		}
		else{
			k = sc->getRotationalLimitMotor(i - 3)->m_springStiffness;
			d = sc->getAngle(i - 3);
		}
		energy += 0.5*k*d*d;
	}
	return energy;
}

void EnergySum::update(){
	linear = inertia = gravitational = elastic = plastic = 0;
	btVector3 gravity = m_dw->getGravity();
	const btCollisionObjectArray objects = m_dw->getCollisionObjectArray();
	for (int i = 0; i<objects.capacity(); i++)
	{
		const btCollisionObject* o = objects[i];
		const btRigidBody* ro = btRigidBody::upcast(o);
		btScalar iM = ro->getInvMass();
		if (iM == 0){
			continue;
		}
		btVector3 v = ro->getLinearVelocity();
		btScalar m = 1 / iM;
		float linearEnergy = 0.5*m*v.length2();
		btVector3 com = ro->getCenterOfMassPosition();
		float gravitationalEnergy = -m*gravity.dot(com);
		const btVector3 rv = ro->getAngularVelocity();
		const btVector3 iI = ro->getInvInertiaDiagLocal();
		float inertiaEnergy = 0;
		inertiaEnergy += getRotationEnergy(iI.getX(), rv.getX());
		inertiaEnergy += getRotationEnergy(iI.getY(), rv.getY());
		inertiaEnergy += getRotationEnergy(iI.getZ(), rv.getZ());
		linear += linearEnergy;
		inertia += inertiaEnergy;
		gravitational += gravitationalEnergy;
	}
	for (int i = m_dw->getNumConstraints() - 1; i >= 0; i--)
	{
		btTypedConstraint* constraint = m_dw->getConstraint(i);
		int type = constraint->getUserConstraintType();
		switch (type){
		case BPT_EP:
		case BPT_EP2:
		{
			btElasticPlasticConstraint* epc = dynamic_cast<btElasticPlasticConstraint*>(constraint);
			elastic += epc->getElasticEnergy();
			plastic += epc->getPlasticEnergy();
		}
			continue;
		default:
			switch (constraint->getConstraintType()){
			case D6_SPRING_CONSTRAINT_TYPE:
				{
				btGeneric6DofSpringConstraint *sc =
					dynamic_cast<btGeneric6DofSpringConstraint*>(constraint);
				elastic += getElasticEnergy(sc);
				}
				continue;
			case D6_SPRING_2_CONSTRAINT_TYPE:
				{
				btGeneric6DofSpring2Constraint *sc =
					dynamic_cast<btGeneric6DofSpring2Constraint*>(constraint);
				elastic += getElasticEnergy(sc);
				}
				break;
			default:
				continue;
			}
		}
	}
}
