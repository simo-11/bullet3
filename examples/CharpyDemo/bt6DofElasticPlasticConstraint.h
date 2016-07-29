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

/*
April 2015 based on btGeneric6DofSpringConstraint 
*/
#ifndef BT_6DOF_ELASTIC_PLASTIC_CONSTRAINT_H
#define BT_6DOF_ELASTIC_PLASTIC_CONSTRAINT_H


#include "LinearMath/btVector3.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/Dynamics/btActionInterface.h"
#include "btElasticPlasticConstraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
class btRigidBody;
#include "BulletDynamics/Dynamics/btDynamicsWorld.h"
class btDynamicsWorld;

#ifdef BT_USE_DOUBLE_PRECISION
#define bt6DofElasticPlasticConstraintData2		bt6DofElasticPlasticConstraintDoubleData2
#define bt6DofElasticPlasticConstraintDataName	"bt6DofElasticPlasticConstraintDoubleData2"
#else
#define bt6DofElasticPlasticConstraintData2		bt6DofElasticPlasticConstraintData
#define bt6DofElasticPlasticConstraintDataName	"bt6DofElasticPlasticConstraintData"
#endif //BT_USE_DOUBLE_PRECISION



/// Generic 6 DOF constraint that allows to set spring motors to any translational and rotational DOF

/// DOF index used in enableSpring() and setStiffness() means:
/// 0 : translation X
/// 1 : translation Y
/// 2 : translation Z
/// 3 : rotation X (3rd Euler rotational around new position of X axis, range [-PI+epsilon, PI-epsilon] )
/// 4 : rotation Y (2nd Euler rotational around new position of Y axis, range [-PI/2+epsilon, PI/2-epsilon] )
/// 5 : rotation Z (1st Euler rotational around Z axis, range [-PI+epsilon, PI-epsilon] )

ATTRIBUTE_ALIGNED16(class) bt6DofElasticPlasticConstraint : 
public btGeneric6DofConstraint, public btActionInterface, public btElasticPlasticConstraint
{
protected:
	bool		m_springEnabled[6];
	btScalar	m_equilibriumPoint[6];
	btScalar	m_springStiffness[6];
	btScalar	m_springDamping[6];
	// bcc, new fields for plasticity
	static int idCounter;
	static void resetIdCounter();
	int id;
	int getId(){ return id; }
	btScalar	m_maxForce[6];
	btScalar	m_fpsLimit[6];
	// how many integration steps is needed for single period before
	// elastic part is activated
	// otherwise constraint tries to remain rigid
	btScalar    m_frequencyRatio = 10;
	btScalar    m_currentPlasticStrain;
	btScalar    m_maxPlasticStrain;
	btScalar    m_maxPlasticRotation = 3;
	btScalar    m_currentPlasticRotation = 0;
	btScalar m_maxRatio;
	int m_maxRatioDof;
	// bcc end of new fields for plasticity
	void init();
	void internalUpdateSprings(btConstraintInfo2* info);
public: 
	
	BT_DECLARE_ALIGNED_ALLOCATOR();
	
    bt6DofElasticPlasticConstraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB ,bool useLinearReferenceFrameA);
    bt6DofElasticPlasticConstraint(btRigidBody& rbB, const btTransform& frameInB, bool useLinearReferenceFrameB);
	void enableSpring(int index, bool onOff);
	void setStiffness(int index, btScalar stiffness);
	void setDamping(int index, btScalar damping);
	// bcc
	void setMaxForce(int index, btScalar value);
	void setMaxPlasticStrain(btScalar value);
	void setMaxPlasticRotation(btScalar value);
	void scalePlasticity(btScalar scale);
	virtual btScalar getMaxPlasticStrain();
	virtual btScalar getMaxPlasticRotation();
	virtual btScalar getCurrentPlasticStrain();
	virtual btScalar getCurrentPlasticRotation();
	virtual btScalar getMaxRatio(){ return m_maxRatio; }
	btScalar getFpsLimit(int index){ return m_fpsLimit[i]; }
	virtual int getMaxRatioDof(){ return m_maxRatioDof; }
	void updatePlasticity(btJointFeedback& forces, btCollisionWorld* collisionWorld, btScalar step);
	void calculateFpsLimit(int index);
	void setFrequencyRatio(btScalar frequencyRatio);
	btScalar getFrequencyRatio();
	btJointFeedback* myJointFeedback=0;
	// bcc
	void setEquilibriumPoint(); // set the current constraint position/orientation as an equilibrium point for all DOF
	void setEquilibriumPoint(int index);  // set the current constraint position/orientation as an equilibrium point for given DOF
	void setEquilibriumPoint(int index, btScalar val);

	virtual void setAxis( const btVector3& axis1, const btVector3& axis2);

	virtual void getInfo2 (btConstraintInfo2* info);

	virtual	int	calculateSerializeBufferSize() const;
	///fills the dataBuffer and returns the struct name (and 0 on failure)
	virtual	const char*	serialize(void* dataBuffer, btSerializer* serializer) const;
	///btActionInterface interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step)
	{
		btJointFeedback* jf = getJointFeedback();
		if (0 == jf){
			myJointFeedback = new btJointFeedback();
			jf = myJointFeedback;
			setJointFeedback(jf);
		}
		updatePlasticity(*jf, collisionWorld, step);
		if (!isEnabled()){
			btDynamicsWorld *dw = (btDynamicsWorld *)collisionWorld;
			dw->removeConstraint(this);
		}
	}
	///btActionInterface interface
	void	debugDraw(btIDebugDraw* debugDrawer);
	~bt6DofElasticPlasticConstraint(){
		if (0 != myJointFeedback){
			delete myJointFeedback;
			myJointFeedback = 0;
		}
	}
};


struct bt6DofElasticPlasticConstraintData
{
	btGeneric6DofConstraintData	m_6dofData;
	
	int			m_springEnabled[6];
	float		m_equilibriumPoint[6];
	float		m_springStiffness[6];
};

struct bt6DofElasticPlasticConstraintDoubleData2
{
	btGeneric6DofConstraintDoubleData2	m_6dofData;
	
	int			m_springEnabled[6];
	double		m_equilibriumPoint[6];
	double		m_springStiffness[6];
	double		m_springDamping[6];
};


SIMD_FORCE_INLINE	int	bt6DofElasticPlasticConstraint::calculateSerializeBufferSize() const
{
	return sizeof(bt6DofElasticPlasticConstraintData2);
}

	///fills the dataBuffer and returns the struct name (and 0 on failure)
SIMD_FORCE_INLINE	const char*	bt6DofElasticPlasticConstraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
	bt6DofElasticPlasticConstraintData2* dof = (bt6DofElasticPlasticConstraintData2*)dataBuffer;
	btGeneric6DofConstraint::serialize(&dof->m_6dofData,serializer);

	int i;
	for (i=0;i<6;i++)
	{
		dof->m_equilibriumPoint[i] = m_equilibriumPoint[i];
		dof->m_springEnabled[i] = m_springEnabled[i]? 1 : 0;
		dof->m_springStiffness[i] = m_springStiffness[i];
	}
	return bt6DofElasticPlasticConstraintDataName;
}

#endif // BT_6DOF_ELASTIC_PLASTIC_CONSTRAINT_H

