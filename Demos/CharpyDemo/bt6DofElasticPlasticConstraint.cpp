/*
Bullet Continuous Collision Detection and Physics Library, http://bulletphysics.org
Copyright (C) 2006, 2007 Sony Computer Entertainment Inc. 

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "bt6DofElasticPlasticConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"


bt6DofElasticPlasticConstraint::bt6DofElasticPlasticConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA)
	: btGeneric6DofConstraint(rbA, rbB, frameInA, frameInB, useLinearReferenceFrameA)
{
    init();
}


bt6DofElasticPlasticConstraint::bt6DofElasticPlasticConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameB)
        : btGeneric6DofConstraint(rbB, frameInB, useLinearReferenceFrameB)
{
    init();
}


void bt6DofElasticPlasticConstraint::init()
{
	m_objectType = D6_SPRING_CONSTRAINT_TYPE;

	for(int i = 0; i < 6; i++)
	{
		m_springEnabled[i] = false;
		m_equilibriumPoint[i] = btScalar(0.f);
		m_springStiffness[i] = btScalar(0.f);
		m_springDamping[i] = btScalar(1.f);
		m_maxForce[i] = btScalar(1.f);
		m_maxPlasticStrain = btScalar(0.f);
		m_currentPlasticStrain = btScalar(0.f);
		m_maxPlasticRotation = btScalar(0.f);
		m_currentPlasticRotation = btScalar(0.f);
	}
}


void bt6DofElasticPlasticConstraint::enableSpring(int index, bool onOff)
{
	btAssert((index >= 0) && (index < 6));
	m_springEnabled[index] = onOff;
	if(index < 3)
	{
		m_linearLimits.m_enableMotor[index] = onOff;
	}
	else
	{
		m_angularLimits[index - 3].m_enableMotor = onOff;
	}
}



void bt6DofElasticPlasticConstraint::setStiffness(int index, btScalar stiffness)
{
	btAssert((index >= 0) && (index < 6));
	m_springStiffness[index] = stiffness;
}

void bt6DofElasticPlasticConstraint::setDamping(int index, btScalar damping)
{
	btAssert((index >= 0) && (index < 6));
	m_springDamping[index] = damping;
}

void bt6DofElasticPlasticConstraint::setMaxForce(int index, btScalar maxForce)
{
	btAssert((index >= 0) && (index < 6));
	m_maxForce[index] = maxForce;
}


void bt6DofElasticPlasticConstraint::setEquilibriumPoint()
{
	calculateTransforms();
	int i;

	for( i = 0; i < 3; i++)
	{
		m_equilibriumPoint[i] = m_calculatedLinearDiff[i];
	}
	for(i = 0; i < 3; i++)
	{
		m_equilibriumPoint[i + 3] = m_calculatedAxisAngleDiff[i];
	}
}



void bt6DofElasticPlasticConstraint::setEquilibriumPoint(int index)
{
	btAssert((index >= 0) && (index < 6));
	calculateTransforms();
	if(index < 3)
	{
		m_equilibriumPoint[index] = m_calculatedLinearDiff[index];
	}
	else
	{
		m_equilibriumPoint[index] = m_calculatedAxisAngleDiff[index - 3];
	}
}

void bt6DofElasticPlasticConstraint::setEquilibriumPoint(int index, btScalar val)
{
	btAssert((index >= 0) && (index < 6));
	m_equilibriumPoint[index] = val;
}


void bt6DofElasticPlasticConstraint::internalUpdateSprings(btConstraintInfo2* info)
{
	int i;
	for(i = 0; i < 3; i++)
	{
		if(m_springEnabled[i])
		{
			// get current position of constraint
			btScalar currPos = m_calculatedLinearDiff[i];
			// calculate difference
			btScalar delta = currPos - m_equilibriumPoint[i];
			// spring force is (delta * m_stiffness) according to Hooke's Law
			btScalar force = delta * m_springStiffness[i];
			// bcc
			if (btFabs(force)>m_maxForce[i]){
				force = (delta > 0 ? m_maxForce[i] : -m_maxForce[i]);
			}
			btScalar velFactor = info->fps * m_springDamping[i] / btScalar(info->m_numIterations);
			m_linearLimits.m_targetVelocity[i] = velFactor * force;
			m_linearLimits.m_maxMotorForce[i] = btFabs(force) / info->fps;
		}
	}
	for(i = 0; i < 3; i++)
	{
		if(m_springEnabled[i + 3])
		{
			// get current position of constraint
			btScalar currPos = m_calculatedAxisAngleDiff[i];
			// calculate difference
			btScalar delta = currPos - m_equilibriumPoint[i+3];
			// spring force is (-delta * m_stiffness) according to Hooke's Law
			btScalar force = -delta * m_springStiffness[i+3];
			// bcc
			if (btFabs(force)>m_maxForce[i+3]){
				force = (delta > 0 ? -m_maxForce[i+3] : m_maxForce[i+3]);
			}
			btScalar velFactor = info->fps * m_springDamping[i+3] / btScalar(info->m_numIterations);
            m_angularLimits[i].m_targetVelocity = velFactor * force;
            m_angularLimits[i].m_maxMotorForce = btFabs(force) / info->fps;
		}
	}
}


void bt6DofElasticPlasticConstraint::getInfo2(btConstraintInfo2* info)
{
	// this will be called by constraint solver at the constraint setup stage
	// set current motor parameters
	internalUpdateSprings(info);
	// do the rest of job for constraint setup
	btGeneric6DofConstraint::getInfo2(info);
}


void bt6DofElasticPlasticConstraint::setAxis(const btVector3& axis1,const btVector3& axis2)
{
	btVector3 zAxis = axis1.normalized();
	btVector3 yAxis = axis2.normalized();
	btVector3 xAxis = yAxis.cross(zAxis); // we want right coordinate system

	btTransform frameInW;
	frameInW.setIdentity();
	frameInW.getBasis().setValue(	xAxis[0], yAxis[0], zAxis[0],	
                                xAxis[1], yAxis[1], zAxis[1],
                                xAxis[2], yAxis[2], zAxis[2]);

	// now get constraint frame in local coordinate systems
	m_frameInA = m_rbA.getCenterOfMassTransform().inverse() * frameInW;
	m_frameInB = m_rbB.getCenterOfMassTransform().inverse() * frameInW;

  calculateTransforms();
}

void bt6DofElasticPlasticConstraint::setMaxPlasticStrain(btScalar value){
	m_maxPlasticStrain = value;
}
void bt6DofElasticPlasticConstraint::setMaxPlasticRotation(btScalar value){
	m_maxPlasticRotation = value;
}
/*

*/
void bt6DofElasticPlasticConstraint::updatePlasticity(btJointFeedback& forces){
	int i;
	for (i = 0; i < 3; i++)
	{
		if (m_springEnabled[i])
		{
			btScalar currPos = m_calculatedLinearDiff[i];
			btScalar delta = currPos - m_equilibriumPoint[i];
			btScalar force = forces.m_appliedForceBodyA[i];
			if (btFabs(force)>m_maxForce[i]){
				btScalar elasticPart = m_maxForce[i] / m_springStiffness[i];
				btScalar newVal = currPos +(currPos>0?-elasticPart:elasticPart);
				setEquilibriumPoint(i, newVal);
				m_currentPlasticStrain += btFabs(delta);
			}
		}
	}
	if (m_currentPlasticStrain > m_maxPlasticStrain){
		setEnabled(false);
		return;
	}
	for (i = 0; i < 3; i++)
	{
		if (m_springEnabled[i + 3])
		{
			btScalar currPos = m_calculatedAxisAngleDiff[i];
			btScalar delta = currPos - m_equilibriumPoint[i + 3];
			btScalar force = forces.m_appliedTorqueBodyA[i];
			if (btFabs(force)>m_maxForce[i + 3]){
				btScalar elasticPart = m_maxForce[i+3] / m_springStiffness[i+3];
				btScalar newVal = currPos + (currPos>0 ? -elasticPart : elasticPart);
				setEquilibriumPoint(i+3, newVal);
				m_currentPlasticRotation += btFabs(delta);
			}
		}
	}
	if (m_currentPlasticRotation > m_maxPlasticRotation){
		setEnabled(false);
	}
}


