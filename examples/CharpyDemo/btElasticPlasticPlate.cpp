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

#include "btElasticPlasticPlate.h"
#include "btBulletDynamicsCommon.h"

/**
It is assumed that rbA and rbB have same size and link is wanted between them
*/
const btTransform btElasticPlasticPlate::getConnectingFrame(btRigidBody& rbA, btRigidBody& rbB){
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

/**
*/
void btElasticPlasticPlate::updateConstraint(bt6DofElasticPlastic2Constraint &constraint) {
	BT_PROFILE("updateConstraint");
	btRigidBody rbA = constraint.getRigidBodyA();
	btRigidBody rbB = constraint.getRigidBodyB();
	btVector3 pA = rbA.getCenterOfMassPosition();
	btVector3 pB = rbB.getCenterOfMassPosition();
	btQuaternion qA = rbA.getCenterOfMassTransform().getRotation();
	btQuaternion qB = rbB.getCenterOfMassTransform().getRotation();
	btBoxShape* sA = static_cast<btBoxShape*>(rbA.getCollisionShape());
	btBoxShape* sB = static_cast<btBoxShape*>(rbB.getCollisionShape());
	// figure out local coordinate system (lcs)
	// calculate spring constants in lcs
	// calculate maximum forces in lcs 
	// transform to world coordinate system
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
	btVector3 v = m_mainShape->getImplicitShapeDimensions();
	m_maxAxis = v.maxAxis();
	m_minAxis = v.minAxis();
	if (m_minAxis == m_maxAxis){
		m_minAxis = 2 ^ m_maxAxis;
	}
	m_middleAxis = 3 ^ (m_maxAxis | m_minAxis);
	if (0 == m_mc){
		m_mc = (int)(v[m_middleAxis] / v[m_maxAxis]) * m_lc;
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
void btElasticPlasticPlate::initConstraints(btDiscreteDynamicsWorld* dw){
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

