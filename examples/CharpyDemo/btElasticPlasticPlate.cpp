/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
/*
Simo Nikula 2016- while studying plasticity
*/
#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "btElasticPlasticPlate.h"
#include "btBulletDynamicsCommon.h"
/**
It is assumed that rbA and rbB have same size and link is wanted between them
*/
const btTransform btElasticPlasticPlate::getConnectingFrame
	(btRigidBody& rbA, btRigidBody& rbB){
	btTransform tr;
	tr.setIdentity();
	btVector3 pA = rbA.getCenterOfMassPosition();
	btVector3 pB = rbB.getCenterOfMassPosition();
	btVector3 o=0.5*(pB-pA);
	tr.setOrigin(o);
	return tr;
}

void btElasticPlasticPlate::setMaterial(btElasticPlasticMaterial* material) {
	m_material = material;
}

void btElasticPlasticPlate::initStiffnesses
(btScalar* v, btScalar l, btScalar b, btScalar t){
	btScalar E(m_material->getE());
	btScalar G(m_material->getG());
	btScalar A(b*t);
	btScalar I1(b*t*t*t / 12);
	btScalar I2(t*b*b*b / 12);
	btScalar It(b*t*t*t/7);
	btScalar k0(E*A/l);
	btScalar k1(12 * E*I1 / l / l / l);
	btScalar k2(12 * E*I2 / l / l/ l);
	v[0] = k0;
	v[1] = k1;
	v[2] = k2;
	v[3] = G*It / l;
	v[4] = 3 * E*I1 / l;
	v[5] = 3 * E*I2 / l;
}

void btElasticPlasticPlate::initMaxForces
(btScalar* v, btScalar l, btScalar b, btScalar t){
	btScalar fy = m_material->getFy();
	btScalar w1 = fy*b*t*t / 4;
	btScalar w2(fy*b*b*t / 4);
	v[0] = fy*b*t;
	v[1] = v[0]/2;
	v[2] = v[0] / 2;
	v[3] = w1;
	v[4] = w1;
	v[5] = w2;
}


void btElasticPlasticPlate::initMaterialBasedLimits(){
	btScalar mps = m_material->getMaxPlasticStrain();
	m_maxPlasticStrain = m_ml / m_mc*mps;
	m_maxPlasticRotation = mps * 10;
}

void btElasticPlasticPlate::initStiffnesses(){
	initStiffnesses(m_stiffnessL, btScalar(m_ll / m_lc), 
			btScalar(m_ml / m_mc), m_thickness);
	initStiffnesses(m_stiffnessM, btScalar(m_ml / m_mc), 
		btScalar(m_ll / m_lc), m_thickness);

}
void btElasticPlasticPlate::initMaxForces(){
	initMaxForces(m_maxForceL, btScalar(m_ll / m_lc), 
		btScalar(m_ml / m_mc), m_thickness);
	initMaxForces(m_maxForceM, btScalar(m_ml / m_mc), 
		btScalar(m_ll / m_lc), m_thickness);
}

/**
*/
void btElasticPlasticPlate::updateConstraint(bt6DofElasticPlastic2Constraint &constraint) {
	BT_PROFILE("updateConstraint");
	btRigidBody& rbA = constraint.getRigidBodyA();
	btRigidBody& rbB = constraint.getRigidBodyB();
	btVector3 pA = rbA.getCenterOfMassPosition();
	btVector3 pB = rbB.getCenterOfMassPosition();
	btQuaternion qA = rbA.getCenterOfMassTransform().getRotation();
	btQuaternion qB = rbB.getCenterOfMassTransform().getRotation();
	btBoxShape* sA = static_cast<btBoxShape*>(rbA.getCollisionShape());
	btBoxShape* sB = static_cast<btBoxShape*>(rbB.getCollisionShape());
	// figure out local coordinate system (lcs)
	// transform to world coordinate system
	for (int i = 0; i < 6; i++){
		btScalar stiffness = m_stiffnessL[i];
		btScalar maxForce = m_maxForceL[i];
		constraint.setStiffness(i, stiffness, m_limitIfNeedeed);
		constraint.setMaxForce(i, maxForce);
	}
}

void btElasticPlasticPlate::join(btDiscreteDynamicsWorld* dw){
	initSubShape();
	initRigidBodies(dw);
	initConstraints(dw);
	dw->addAction(this);
}
/**
Select longest and divide by m_lc
middle is divived by m_mc
shortest is used as thickness
*/
void btElasticPlasticPlate::initSubShape(){
	btVector3 v = m_mainShape->getHalfExtentsWithMargin();
	m_maxAxis = v.maxAxis();
	m_minAxis = v.minAxis();
	if (m_minAxis == m_maxAxis){
		m_minAxis = 2 ^ m_maxAxis;
	}
	m_middleAxis = 3 ^ (m_maxAxis | m_minAxis);
	if (0 == m_mc){
		m_mc = (int)(m_lc*v[m_middleAxis]/ v[m_maxAxis]);
		if (0 == m_mc){
			m_mc = 1;
		}
	}
	m_ll = v[m_maxAxis];
	m_ml = v[m_middleAxis];
	btVector3 sv=v;
	sv[m_maxAxis] /= m_lc;
	sv[m_middleAxis] /= m_mc;
	m_thickness = v[m_minAxis];
	m_subShape = new btBoxShape(sv);
}

void btElasticPlasticPlate::initRigidBodies(btDiscreteDynamicsWorld* dw){
	btScalar lLen(m_ll/m_lc),mLen(m_ml/m_mc);
	btScalar mass = lLen*mLen*m_thickness*m_material->getDensity();
	btScalar lloc = (lLen - m_ll) / 2;
	btVector3 localInertia(0, 0, 0);
	m_subShape->calculateLocalInertia(mass, localInertia);
	for (int i = 0; i < m_lc; i++){
		btScalar mloc = (mLen - m_ml) / 2;
		for (int j = 0; j < m_mc; j++){
			btTransform lTrans=m_mainTransform;
			btVector3 pos = lTrans.getOrigin();
			pos[m_maxAxis] += lloc;
			pos[m_middleAxis] += mloc;
			lTrans.setOrigin(pos);
			btDefaultMotionState* mState = 
				new btDefaultMotionState(lTrans);
			btRigidBody::btRigidBodyConstructionInfo 
				cInfo(mass, mState, m_subShape, localInertia);
			btRigidBody* body = new btRigidBody(cInfo);
			dw->addRigidBody(body);
			m_rb.push_back(body);
			mloc += mLen;
		}
		lloc += lLen;
	}
}

void btElasticPlasticPlate::prepareAndAdd
	(bt6DofElasticPlastic2Constraint *sc, btDiscreteDynamicsWorld* dw){
	sc->setMaxPlasticRotation(m_maxPlasticRotation);
	sc->setMaxPlasticStrain(m_maxPlasticStrain);
	updateConstraint(*sc);
	dw->addConstraint(sc, m_disableCollisionsBetweenLinkedBodies);
	for (int i = 0; i < 6; i++)
	{
		sc->enableSpring(i, true);
	}
	for (int i = 0; i < 6; i++)
	{
		sc->setDamping(i, damping);
	}
	sc->setEquilibriumPoint();
}

void btElasticPlasticPlate::initConstraints(btDiscreteDynamicsWorld* dw){
	initStiffnesses();
	initMaxForces();
	initMaterialBasedLimits();
	// longer direction
	for (int i = 0; i < m_lc - 1; i++){
		for (int j = 0; j < m_mc; j++){
			btRigidBody *rbA = m_rb[m_mc*i + j];
			btRigidBody *rbB = m_rb[m_mc*(i + 1) + j];
			btTransform tra=getConnectingFrame(*rbA,*rbB);
			btTransform trb = getConnectingFrame(*rbB, *rbA);
			bt6DofElasticPlastic2Constraint *sc =
				new bt6DofElasticPlastic2Constraint(*rbA, *rbB, tra, trb);
			prepareAndAdd(sc,dw);
		}
	}
	// shorter direction
	for (int i = 0; i < m_lc; i++){
		for (int j = 0; j < (m_mc - 1); j++){
			btRigidBody *rbA = m_rb[m_mc*i + j];
			btRigidBody *rbB = m_rb[m_mc*i + 1 + j];
			btTransform tra = getConnectingFrame(*rbA, *rbB);
			btTransform trb = getConnectingFrame(*rbB, *rbA);
			bt6DofElasticPlastic2Constraint *sc =
				new bt6DofElasticPlastic2Constraint(*rbA, *rbB, tra, trb);
			prepareAndAdd(sc, dw);
		}
	}
}

void btElasticPlasticPlate::setTransform(btTransform& transform){
	m_mainTransform = transform;
}
void btElasticPlasticPlate::setShape(btBoxShape* shape){
	m_mainShape = shape;
}

btElasticPlasticPlate::~btElasticPlasticPlate()
{	
	if (0 != m_subShape){
		delete m_subShape;
		m_subShape = 0;
	}
	m_constraints.clear();
}


void btElasticPlasticPlate::updateAction(btCollisionWorld* collisionWorld, btScalar step)
{
	for (int i = 0; i<m_constraints.size(); i++)
	{
		bt6DofElasticPlastic2Constraint* c = m_constraints[i];
		updateConstraint(*c);
	}

}
void btElasticPlasticPlate::debugDraw(btIDebugDraw* debugDrawer){
	for (int i = 0; i<m_constraints.size(); i++)
	{
		bt6DofElasticPlastic2Constraint* c = m_constraints[i];
		c->debugDraw(debugDrawer);
	}
}

