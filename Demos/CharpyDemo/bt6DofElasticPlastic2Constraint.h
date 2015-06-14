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
April 2015 based on bt6DofElasticPlastic2Constraint
*/

#ifndef BT_6DOF_ELASTIC_PLASTIC_CONSTRAINT2_H
#define BT_6DOF_ELASTIC_PLASTIC_CONSTRAINT2_H

#include "LinearMath/btVector3.h"
#include "BulletDynamics/ConstraintSolver/btJacobianEntry.h"
#include "BulletDynamics/ConstraintSolver/btTypedConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

class btRigidBody;


#ifdef BT_USE_DOUBLE_PRECISION
#define bt6DofElasticPlastic2ConstraintData2		bt6DofElasticPlastic2ConstraintDoubleData2
#define bt6DofElasticPlastic2ConstraintDataName	"bt6DofElasticPlastic2ConstraintDoubleData2"
#else
#define bt6DofElasticPlastic2ConstraintData2		bt6DofElasticPlastic2ConstraintData
#define bt6DofElasticPlastic2ConstraintDataName	"bt6DofElasticPlastic2ConstraintData"
#endif //BT_USE_DOUBLE_PRECISION


ATTRIBUTE_ALIGNED16(class) bt6DofElasticPlastic2Constraint : public btTypedConstraint
{
protected:

	btTransform m_frameInA;
	btTransform m_frameInB;

	btJacobianEntry m_jacLinear[3];
	btJacobianEntry m_jacAng[3];

	btTranslationalLimitMotor2 m_linearLimits;
	btRotationalLimitMotor2 m_angularLimits[3];

	RotateOrder m_rotateOrder;

protected:

	btTransform  m_calculatedTransformA;
	btTransform  m_calculatedTransformB;
	btVector3    m_calculatedAxisAngleDiff;
	btVector3    m_calculatedAxis[3];
	btVector3    m_calculatedLinearDiff;
	btScalar     m_factA;
	btScalar     m_factB;
	bool         m_hasStaticBody;
	int          m_flags;

	bt6DofElasticPlastic2Constraint&	operator=(bt6DofElasticPlastic2Constraint&)
	{
		btAssert(0);
		return *this;
	}

	int setAngularLimits(btConstraintInfo2 *info, int row_offset,const btTransform& transA,const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB,const btVector3& angVelA,const btVector3& angVelB);
	int setLinearLimits(btConstraintInfo2 *info, int row, const btTransform& transA,const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB,const btVector3& angVelA,const btVector3& angVelB);

	void calculateLinearInfo();
	void calculateAngleInfo();
	void testAngularLimitMotor(int axis_index);

	void calculateJacobi(btRotationalLimitMotor2* limot, const btTransform& transA,const btTransform& transB, btConstraintInfo2* info, int srow, btVector3& ax1, int rotational, int rotAllowed);
	int get_limit_motor_info2(btRotationalLimitMotor2* limot,
		const btTransform& transA,const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB,const btVector3& angVelA,const btVector3& angVelB,
		btConstraintInfo2* info, int row, btVector3& ax1, int rotational, int rotAllowed = false);

	static btScalar btGetMatrixElem(const btMatrix3x3& mat, int index);
	static bool matrixToEulerXYZ(const btMatrix3x3& mat,btVector3& xyz);
	static bool matrixToEulerXZY(const btMatrix3x3& mat,btVector3& xyz);
	static bool matrixToEulerYXZ(const btMatrix3x3& mat,btVector3& xyz);
	static bool matrixToEulerYZX(const btMatrix3x3& mat,btVector3& xyz);
	static bool matrixToEulerZXY(const btMatrix3x3& mat,btVector3& xyz);
	static bool matrixToEulerZYX(const btMatrix3x3& mat,btVector3& xyz);

public:

	BT_DECLARE_ALIGNED_ALLOCATOR();

    bt6DofElasticPlastic2Constraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB, RotateOrder rotOrder = RO_XYZ);
    bt6DofElasticPlastic2Constraint(btRigidBody& rbB, const btTransform& frameInB, RotateOrder rotOrder = RO_XYZ);

	virtual void buildJacobian() {}
	virtual void getInfo1 (btConstraintInfo1* info);
	virtual void getInfo2 (btConstraintInfo2* info);
	virtual int calculateSerializeBufferSize() const;
	virtual const char* serialize(void* dataBuffer, btSerializer* serializer) const;

	btRotationalLimitMotor2* getRotationalLimitMotor(int index) { return &m_angularLimits[index]; }
	btTranslationalLimitMotor2* getTranslationalLimitMotor() { return &m_linearLimits; }

	// Calculates the global transform for the joint offset for body A an B, and also calculates the angle differences between the bodies.
	void calculateTransforms(const btTransform& transA,const btTransform& transB);
	void calculateTransforms();

	// Gets the global transform of the offset for body A
	const btTransform & getCalculatedTransformA() const { return m_calculatedTransformA; }
	// Gets the global transform of the offset for body B
	const btTransform & getCalculatedTransformB() const { return m_calculatedTransformB; }

	const btTransform & getFrameOffsetA() const { return m_frameInA; }
	const btTransform & getFrameOffsetB() const { return m_frameInB; }

	btTransform & getFrameOffsetA() { return m_frameInA; }
	btTransform & getFrameOffsetB() { return m_frameInB; }

	// Get the rotation axis in global coordinates ( bt6DofElasticPlastic2Constraint::calculateTransforms() must be called previously )
	btVector3 getAxis(int axis_index) const { return m_calculatedAxis[axis_index]; }

	// Get the relative Euler angle ( bt6DofElasticPlastic2Constraint::calculateTransforms() must be called previously )
	btScalar getAngle(int axis_index) const { return m_calculatedAxisAngleDiff[axis_index]; }

	// Get the relative position of the constraint pivot ( bt6DofElasticPlastic2Constraint::calculateTransforms() must be called previously )
	btScalar getRelativePivotPosition(int axis_index) const { return m_calculatedLinearDiff[axis_index]; }

	void setFrames(const btTransform & frameA, const btTransform & frameB);

	void setLinearLowerLimit(const btVector3& linearLower) { m_linearLimits.m_lowerLimit = linearLower; }
	void getLinearLowerLimit(btVector3& linearLower) { linearLower = m_linearLimits.m_lowerLimit; }
	void setLinearUpperLimit(const btVector3& linearUpper) { m_linearLimits.m_upperLimit = linearUpper; }
	void getLinearUpperLimit(btVector3& linearUpper) { linearUpper = m_linearLimits.m_upperLimit; }

	void setAngularLowerLimit(const btVector3& angularLower)
	{
		for(int i = 0; i < 3; i++) 
			m_angularLimits[i].m_loLimit = btNormalizeAngle(angularLower[i]);
	}

	void setAngularLowerLimitReversed(const btVector3& angularLower)
	{
		for(int i = 0; i < 3; i++) 
			m_angularLimits[i].m_hiLimit = btNormalizeAngle(-angularLower[i]);
	}

	void getAngularLowerLimit(btVector3& angularLower)
	{
		for(int i = 0; i < 3; i++) 
			angularLower[i] = m_angularLimits[i].m_loLimit;
	}

	void getAngularLowerLimitReversed(btVector3& angularLower)
	{
		for(int i = 0; i < 3; i++)
			angularLower[i] = -m_angularLimits[i].m_hiLimit;
	}

	void setAngularUpperLimit(const btVector3& angularUpper)
	{
		for(int i = 0; i < 3; i++)
			m_angularLimits[i].m_hiLimit = btNormalizeAngle(angularUpper[i]);
	}

	void setAngularUpperLimitReversed(const btVector3& angularUpper)
	{
		for(int i = 0; i < 3; i++)
			m_angularLimits[i].m_loLimit = btNormalizeAngle(-angularUpper[i]);
	}

	void getAngularUpperLimit(btVector3& angularUpper)
	{
		for(int i = 0; i < 3; i++)
			angularUpper[i] = m_angularLimits[i].m_hiLimit;
	}

	void getAngularUpperLimitReversed(btVector3& angularUpper)
	{
		for(int i = 0; i < 3; i++)
			angularUpper[i] = -m_angularLimits[i].m_loLimit;
	}

	//first 3 are linear, next 3 are angular

	void setLimit(int axis, btScalar lo, btScalar hi)
	{
		if(axis<3)
		{
			m_linearLimits.m_lowerLimit[axis] = lo;
			m_linearLimits.m_upperLimit[axis] = hi;
		}
		else
		{
			lo = btNormalizeAngle(lo);
			hi = btNormalizeAngle(hi);
			m_angularLimits[axis-3].m_loLimit = lo;
			m_angularLimits[axis-3].m_hiLimit = hi;
		}
	}

	void setLimitReversed(int axis, btScalar lo, btScalar hi)
	{
		if(axis<3)
		{
			m_linearLimits.m_lowerLimit[axis] = lo;
			m_linearLimits.m_upperLimit[axis] = hi;
		}
		else
		{
			lo = btNormalizeAngle(lo);
			hi = btNormalizeAngle(hi);
			m_angularLimits[axis-3].m_hiLimit = -lo;
			m_angularLimits[axis-3].m_loLimit = -hi;
		}
	}

	bool isLimited(int limitIndex)
	{
		if(limitIndex<3)
		{
			return m_linearLimits.isLimited(limitIndex);
		}
		return m_angularLimits[limitIndex-3].isLimited();
	}

	void setRotationOrder(RotateOrder order) { m_rotateOrder = order; }
	RotateOrder getRotationOrder() { return m_rotateOrder; }

	void setAxis( const btVector3& axis1, const btVector3& axis2);

	void setBounce(int index, btScalar bounce);

	void enableMotor(int index, bool onOff);
	void setServo(int index, bool onOff); // set the type of the motor (servo or not) (the motor has to be turned on for servo also)
	void setTargetVelocity(int index, btScalar velocity);
	void setServoTarget(int index, btScalar target);
	void setMaxMotorForce(int index, btScalar force);

	void enableSpring(int index, bool onOff);
	void setStiffness(int index, btScalar stiffness);
	void setDamping(int index, btScalar damping);
	void setEquilibriumPoint(); // set the current constraint position/orientation as an equilibrium point for all DOF
	void setEquilibriumPoint(int index);  // set the current constraint position/orientation as an equilibrium point for given DOF
	void setEquilibriumPoint(int index, btScalar val);

	//override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
	//If no axis is provided, it uses the default axis for this constraint.
	virtual void setParam(int num, btScalar value, int axis = -1);
	virtual btScalar getParam(int num, int axis = -1) const;
};


struct bt6DofElasticPlastic2ConstraintData
{
	btTypedConstraintData m_typeConstraintData;
	btTransformFloatData m_rbAFrame;
	btTransformFloatData m_rbBFrame;

	btVector3FloatData m_linearUpperLimit;
	btVector3FloatData m_linearLowerLimit;
	btVector3FloatData m_linearBounce;
	btVector3FloatData m_linearStopERP;
	btVector3FloatData m_linearStopCFM;
	btVector3FloatData m_linearMotorERP;
	btVector3FloatData m_linearMotorCFM;
	btVector3FloatData m_linearTargetVelocity;
	btVector3FloatData m_linearMaxMotorForce;
	btVector3FloatData m_linearServoTarget;
	btVector3FloatData m_linearSpringStiffness;
	btVector3FloatData m_linearSpringDamping;
	btVector3FloatData m_linearEquilibriumPoint;
	char               m_linearEnableMotor[4];
	char               m_linearServoMotor[4];
	char               m_linearEnableSpring[4];
	char               m_padding1[4];

	btVector3FloatData m_angularUpperLimit;
	btVector3FloatData m_angularLowerLimit;
	btVector3FloatData m_angularBounce;
	btVector3FloatData m_angularStopERP;
	btVector3FloatData m_angularStopCFM;
	btVector3FloatData m_angularMotorERP;
	btVector3FloatData m_angularMotorCFM;
	btVector3FloatData m_angularTargetVelocity;
	btVector3FloatData m_angularMaxMotorForce;
	btVector3FloatData m_angularServoTarget;
	btVector3FloatData m_angularSpringStiffness;
	btVector3FloatData m_angularSpringDamping;
	btVector3FloatData m_angularEquilibriumPoint;
	char               m_angularEnableMotor[4];
	char               m_angularServoMotor[4];
	char               m_angularEnableSpring[4];
	char               m_padding2[4];

	int                m_rotateOrder;
	char               m_padding3[4];
};

struct bt6DofElasticPlastic2ConstraintDoubleData2
{
	btTypedConstraintDoubleData m_typeConstraintData;
	btTransformDoubleData m_rbAFrame;
	btTransformDoubleData m_rbBFrame;

	btVector3DoubleData m_linearUpperLimit;
	btVector3DoubleData m_linearLowerLimit;
	btVector3DoubleData m_linearBounce;
	btVector3DoubleData m_linearStopERP;
	btVector3DoubleData m_linearStopCFM;
	btVector3DoubleData m_linearMotorERP;
	btVector3DoubleData m_linearMotorCFM;
	btVector3DoubleData m_linearTargetVelocity;
	btVector3DoubleData m_linearMaxMotorForce;
	btVector3DoubleData m_linearServoTarget;
	btVector3DoubleData m_linearSpringStiffness;
	btVector3DoubleData m_linearSpringDamping;
	btVector3DoubleData m_linearEquilibriumPoint;
	char                m_linearEnableMotor[4];
	char                m_linearServoMotor[4];
	char                m_linearEnableSpring[4];
	char                m_padding1[4];

	btVector3DoubleData m_angularUpperLimit;
	btVector3DoubleData m_angularLowerLimit;
	btVector3DoubleData m_angularBounce;
	btVector3DoubleData m_angularStopERP;
	btVector3DoubleData m_angularStopCFM;
	btVector3DoubleData m_angularMotorERP;
	btVector3DoubleData m_angularMotorCFM;
	btVector3DoubleData m_angularTargetVelocity;
	btVector3DoubleData m_angularMaxMotorForce;
	btVector3DoubleData m_angularServoTarget;
	btVector3DoubleData m_angularSpringStiffness;
	btVector3DoubleData m_angularSpringDamping;
	btVector3DoubleData m_angularEquilibriumPoint;
	char                m_angularEnableMotor[4];
	char                m_angularServoMotor[4];
	char                m_angularEnableSpring[4];
	char                m_padding2[4];

	int                 m_rotateOrder;
	char                m_padding3[4];
};

SIMD_FORCE_INLINE int bt6DofElasticPlastic2Constraint::calculateSerializeBufferSize() const
{
	return sizeof(bt6DofElasticPlastic2ConstraintData2);
}

SIMD_FORCE_INLINE const char* bt6DofElasticPlastic2Constraint::serialize(void* dataBuffer, btSerializer* serializer) const
{
	bt6DofElasticPlastic2ConstraintData2* dof = (bt6DofElasticPlastic2ConstraintData2*)dataBuffer;
	btTypedConstraint::serialize(&dof->m_typeConstraintData,serializer);

	m_frameInA.serialize(dof->m_rbAFrame);
	m_frameInB.serialize(dof->m_rbBFrame);

	int i;
	for (i=0;i<3;i++)
	{
		dof->m_angularLowerLimit.m_floats[i]       = m_angularLimits[i].m_loLimit;
		dof->m_angularUpperLimit.m_floats[i]       = m_angularLimits[i].m_hiLimit;
		dof->m_angularBounce.m_floats[i]           = m_angularLimits[i].m_bounce;
		dof->m_angularStopERP.m_floats[i]          = m_angularLimits[i].m_stopERP;
		dof->m_angularStopCFM.m_floats[i]          = m_angularLimits[i].m_stopCFM;
		dof->m_angularMotorERP.m_floats[i]         = m_angularLimits[i].m_motorERP;
		dof->m_angularMotorCFM.m_floats[i]         = m_angularLimits[i].m_motorCFM;
		dof->m_angularTargetVelocity.m_floats[i]   = m_angularLimits[i].m_targetVelocity;
		dof->m_angularMaxMotorForce.m_floats[i]    = m_angularLimits[i].m_maxMotorForce;
		dof->m_angularServoTarget.m_floats[i]      = m_angularLimits[i].m_servoTarget;
		dof->m_angularSpringStiffness.m_floats[i]  = m_angularLimits[i].m_springStiffness;
		dof->m_angularSpringDamping.m_floats[i]    = m_angularLimits[i].m_springDamping;
		dof->m_angularEquilibriumPoint.m_floats[i] = m_angularLimits[i].m_equilibriumPoint;
	}
	dof->m_angularLowerLimit.m_floats[3]       = 0;
	dof->m_angularUpperLimit.m_floats[3]       = 0;
	dof->m_angularBounce.m_floats[3]           = 0;
	dof->m_angularStopERP.m_floats[3]          = 0;
	dof->m_angularStopCFM.m_floats[3]          = 0;
	dof->m_angularMotorERP.m_floats[3]         = 0;
	dof->m_angularMotorCFM.m_floats[3]         = 0;
	dof->m_angularTargetVelocity.m_floats[3]   = 0;
	dof->m_angularMaxMotorForce.m_floats[3]    = 0;
	dof->m_angularServoTarget.m_floats[3]      = 0;
	dof->m_angularSpringStiffness.m_floats[3]  = 0;
	dof->m_angularSpringDamping.m_floats[3]    = 0;
	dof->m_angularEquilibriumPoint.m_floats[3] = 0;
	for (i=0;i<4;i++)
	{
		dof->m_angularEnableMotor[i]  = i < 3 ? ( m_angularLimits[i].m_enableMotor ? 1 : 0 ) : 0;
		dof->m_angularServoMotor[i]   = i < 3 ? ( m_angularLimits[i].m_servoMotor ? 1 : 0 ) : 0;
		dof->m_angularEnableSpring[i] = i < 3 ? ( m_angularLimits[i].m_enableSpring ? 1 : 0 ) : 0;
	}

	m_linearLimits.m_lowerLimit.serialize( dof->m_linearLowerLimit );
	m_linearLimits.m_upperLimit.serialize( dof->m_linearUpperLimit );
	m_linearLimits.m_bounce.serialize( dof->m_linearBounce );
	m_linearLimits.m_stopERP.serialize( dof->m_linearStopERP );
	m_linearLimits.m_stopCFM.serialize( dof->m_linearStopCFM );
	m_linearLimits.m_motorERP.serialize( dof->m_linearMotorERP );
	m_linearLimits.m_motorCFM.serialize( dof->m_linearMotorCFM );
	m_linearLimits.m_targetVelocity.serialize( dof->m_linearTargetVelocity );
	m_linearLimits.m_maxMotorForce.serialize( dof->m_linearMaxMotorForce );
	m_linearLimits.m_servoTarget.serialize( dof->m_linearServoTarget );
	m_linearLimits.m_springStiffness.serialize( dof->m_linearSpringStiffness );
	m_linearLimits.m_springDamping.serialize( dof->m_linearSpringDamping );
	m_linearLimits.m_equilibriumPoint.serialize( dof->m_linearEquilibriumPoint );
	for (i=0;i<4;i++)
	{
		dof->m_linearEnableMotor[i]  = i < 3 ? ( m_linearLimits.m_enableMotor[i] ? 1 : 0 ) : 0;
		dof->m_linearServoMotor[i]   = i < 3 ? ( m_linearLimits.m_servoMotor[i] ? 1 : 0 ) : 0;
		dof->m_linearEnableSpring[i] = i < 3 ? ( m_linearLimits.m_enableSpring[i] ? 1 : 0 ) : 0;
	}

	dof->m_rotateOrder = m_rotateOrder;

	return bt6DofElasticPlastic2ConstraintDataName;
}





#endif //BT_6DOF_ELASTIC_PLASTIC_CONSTRAINT_H