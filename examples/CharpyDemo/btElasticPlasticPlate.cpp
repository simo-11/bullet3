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
const btTransform getPlateFrame(btRigidBody& rbA, btRigidBody& rbB){
	btTransform tr;
	tr.setIdentity();
	btVector3 pA = rbA.getCenterOfMassPosition();
	btVector3 pB = rbB.getCenterOfMassPosition();
	btVector3 o=0.5*(pB-pA);
	tr.setOrigin(o);
	return tr;
}

btElasticPlasticPlate::btElasticPlasticPlate(btRigidBody& rbA, btRigidBody& rbB)
	: bt6DofElasticPlastic2Constraint(rbA, rbB, getPlateFrame(rbA, rbB), getPlateFrame(rbB,rbA))
{
}

void btElasticPlasticPlate::setMaterial(btElasticPlasticMaterial* material) {
	m_material = material;
	updateConstraint();
}

void btElasticPlasticPlate::updateConstraint() {
	btVector3 pA = m_rbA.getCenterOfMassPosition();
	btVector3 pB = m_rbB.getCenterOfMassPosition();
	btQuaternion qA = m_rbA.getCenterOfMassTransform().getRotation();
	btQuaternion qB = m_rbB.getCenterOfMassTransform().getRotation();
	btBoxShape* sA = static_cast<btBoxShape*>(m_rbA.getCollisionShape());
	btBoxShape* sB = static_cast<btBoxShape*>(m_rbB.getCollisionShape());
}


