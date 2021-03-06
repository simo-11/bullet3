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
2014 May: btGeneric6DofSpring2Constraint is created from the original (2.82.2712) btGeneric6DofConstraint by Gabor Puhr and Tamas Umenhoffer
Pros:
- Much more accurate and stable in a lot of situation. (Especially when a sleeping chain of RBs connected with 6dof2 is pulled)
- Stable and accurate spring with minimal energy loss that works with all of the solvers. (latter is not true for the original 6dof spring)
- Servo motor functionality
- Much more accurate bouncing. 0 really means zero bouncing (not true for the original 6odf) and there is only a minimal energy loss when the value is 1 (because of the solvers' precision)
- Rotation order for the Euler system can be set. (One axis' freedom is still limited to pi/2)

Cons:
- It is slower than the original 6dof. There is no exact ratio, but half speed is a good estimation. (with PGS)
- At bouncing the correct velocity is calculated, but not the correct position. (it is because of the solver can correct position or velocity, but not both.)
*/

/// 2009 March: btGeneric6DofConstraint refactored by Roman Ponomarev
/// Added support for generic constraint solver through getInfo1/getInfo2 methods

/*
2007-09-09
btGeneric6DofConstraint Refactored by Francisco Le?n
email: projectileman@yahoo.com
http://gimpact.sf.net
*/


#include "bt6DofElasticPlastic2Constraint.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"
#include "LinearMath/btTransformUtil.h"
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btQuickprof.h"
#include <new>
#include "../plasticity/PlasticityData.h"
#include "../plasticity/PlasticityDebugDrawer.h"



bt6DofElasticPlastic2Constraint::bt6DofElasticPlastic2Constraint(btRigidBody& rbA, btRigidBody& rbB, const btTransform& frameInA, const btTransform& frameInB, RotateOrder rotOrder)
	: btTypedConstraint(D6_SPRING_2_CONSTRAINT_TYPE, rbA, rbB)
	, m_frameInA(frameInA)
	, m_frameInB(frameInB)
	, m_rotateOrder(rotOrder)	
	, m_flags(0)
{
	calculateTransforms();
	initPlasticity();
}


bt6DofElasticPlastic2Constraint::bt6DofElasticPlastic2Constraint(btRigidBody& rbB, const btTransform& frameInB, RotateOrder rotOrder)
	: btTypedConstraint(D6_SPRING_2_CONSTRAINT_TYPE, btTypedConstraint::getFixedBody(), rbB)
	, m_frameInB(frameInB)
	, m_rotateOrder(rotOrder)
	, m_flags(0)
{
	///not providing rigidbody A means implicitly using worldspace for body A
	m_frameInA = rbB.getCenterOfMassTransform() * m_frameInB;
	calculateTransforms();
	initPlasticity();
}

int bt6DofElasticPlastic2Constraint::idCounter; 
void bt6DofElasticPlastic2Constraint::resetIdCounter(){
	idCounter = 0;
}
// bcc
static int monitorVelocityDirection=0;
void bt6DofElasticPlastic2Constraint::setMonitorVelocityDirection(int val){
	monitorVelocityDirection = val;
}
int bt6DofElasticPlastic2Constraint::getMonitorVelocityDirection(){
	return monitorVelocityDirection;
}


bool bt6DofElasticPlastic2Constraint::isLimitNeeded(btScalar vel, int dof)
{
	if (dof < 0){
		return false;
	}
	velDirType val = velDir[dof] << 1;
	if (vel >= 0){
		val += 1;
	}
	velDir[dof] = val;
	int changeCount = PlasticityUtils::countChanges(val);
	if (changeCount<=monitorVelocityDirection){
		return false;
	}
	return true;
}
void bt6DofElasticPlastic2Constraint::initPlasticity()
{
	id = ++idCounter;
	m_objectType = btPlasticConstraintType::BPT_EP2;
	setUserConstraintType(btPlasticConstraintType::BPT_EP2);
	for (int i = 0; i < 6; i++)
	{
		m_maxForce[i] = btScalar(SIMD_INFINITY);
		m_plasticDisplacement[i] = BT_ZERO;
		velDir[i] = 0;
	}
	m_maxPlasticStrain = BT_ZERO;
	m_currentPlasticStrain = BT_ZERO;
	m_maxPlasticRotation = BT_ZERO;
	m_currentPlasticRotation = BT_ZERO;
}


btScalar bt6DofElasticPlastic2Constraint::btGetMatrixElem(const btMatrix3x3& mat, int index)
{
	int i = index%3;
	int j = index/3;
	return mat[i][j];
}

// MatrixToEulerXYZ from http://www.geometrictools.com/LibFoundation/Mathematics/Wm4Matrix3.inl.html

bool bt6DofElasticPlastic2Constraint::matrixToEulerXYZ(const btMatrix3x3& mat,btVector3& xyz)
{
	// rot =  cy*cz          -cy*sz           sy
	//        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
	//       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy

	btScalar fi = btGetMatrixElem(mat,2);
	if (fi < btScalar(1.0f))
	{
		if (fi > btScalar(-1.0f))
		{
			xyz[0] = btAtan2(-btGetMatrixElem(mat,5),btGetMatrixElem(mat,8));
			xyz[1] = btAsin(btGetMatrixElem(mat,2));
			xyz[2] = btAtan2(-btGetMatrixElem(mat,1),btGetMatrixElem(mat,0));
			return true;
		}
		else
		{
			// WARNING.  Not unique.  XA - ZA = -atan2(r10,r11)
			xyz[0] = -btAtan2(btGetMatrixElem(mat,3),btGetMatrixElem(mat,4));
			xyz[1] = -SIMD_HALF_PI;
			xyz[2] = btScalar(0.0);
			return false;
		}
	}
	else
	{
		// WARNING.  Not unique.  XAngle + ZAngle = atan2(r10,r11)
		xyz[0] = btAtan2(btGetMatrixElem(mat,3),btGetMatrixElem(mat,4));
		xyz[1] = SIMD_HALF_PI;
		xyz[2] = 0.0;
	}
	return false;
}

bool bt6DofElasticPlastic2Constraint::matrixToEulerXZY(const btMatrix3x3& mat,btVector3& xyz)
{
	// rot =  cy*cz          -sz           sy*cz
	//        cy*cx*sz+sx*sy  cx*cz        sy*cx*sz-cy*sx
	//        cy*sx*sz-cx*sy  sx*cz        sy*sx*sz+cx*cy

	btScalar fi = btGetMatrixElem(mat,1);
	if (fi < btScalar(1.0f))
	{
		if (fi > btScalar(-1.0f))
		{
			xyz[0] = btAtan2(btGetMatrixElem(mat,7),btGetMatrixElem(mat,4));
			xyz[1] = btAtan2(btGetMatrixElem(mat,2),btGetMatrixElem(mat,0));
			xyz[2] = btAsin(-btGetMatrixElem(mat,1));
			return true;
		}
		else
		{
			xyz[0] = -btAtan2(-btGetMatrixElem(mat,6),btGetMatrixElem(mat,8));
			xyz[1] = btScalar(0.0);
			xyz[2] = SIMD_HALF_PI;
			return false;
		}
	}
	else
	{
		xyz[0] = btAtan2(-btGetMatrixElem(mat,6),btGetMatrixElem(mat,8));
		xyz[1] = 0.0;
		xyz[2] = -SIMD_HALF_PI;
	}
	return false;
}

bool bt6DofElasticPlastic2Constraint::matrixToEulerYXZ(const btMatrix3x3& mat,btVector3& xyz)
{
	// rot =  cy*cz+sy*sx*sz  cz*sy*sx-cy*sz  cx*sy
	//        cx*sz           cx*cz           -sx
	//        cy*sx*sz-cz*sy  sy*sz+cy*cz*sx  cy*cx

	btScalar fi = btGetMatrixElem(mat,5);
	if (fi < btScalar(1.0f))
	{
		if (fi > btScalar(-1.0f))
		{
			xyz[0] = btAsin(-btGetMatrixElem(mat,5));
			xyz[1] = btAtan2(btGetMatrixElem(mat,2),btGetMatrixElem(mat,8));
			xyz[2] = btAtan2(btGetMatrixElem(mat,3),btGetMatrixElem(mat,4));
			return true;
		}
		else
		{
			xyz[0] = SIMD_HALF_PI;
			xyz[1] = -btAtan2(-btGetMatrixElem(mat,1),btGetMatrixElem(mat,0));
			xyz[2] = btScalar(0.0);
			return false;
		}
	}
	else
	{
		xyz[0] = -SIMD_HALF_PI;
		xyz[1] = btAtan2(-btGetMatrixElem(mat,1),btGetMatrixElem(mat,0));
		xyz[2] = 0.0;
	}
	return false;
}

bool bt6DofElasticPlastic2Constraint::matrixToEulerYZX(const btMatrix3x3& mat,btVector3& xyz)
{
	// rot =  cy*cz   sy*sx-cy*cx*sz   cx*sy+cy*sz*sx
	//        sz           cz*cx           -cz*sx
	//        -cz*sy  cy*sx+cx*sy*sz   cy*cx-sy*sz*sx

	btScalar fi = btGetMatrixElem(mat,3);
	if (fi < btScalar(1.0f))
	{
		if (fi > btScalar(-1.0f))
		{
			xyz[0] = btAtan2(-btGetMatrixElem(mat,5),btGetMatrixElem(mat,4));
			xyz[1] = btAtan2(-btGetMatrixElem(mat,6),btGetMatrixElem(mat,0));
			xyz[2] = btAsin(btGetMatrixElem(mat,3));
			return true;
		}
		else
		{
			xyz[0] = btScalar(0.0);
			xyz[1] = -btAtan2(btGetMatrixElem(mat,7),btGetMatrixElem(mat,8));
			xyz[2] = -SIMD_HALF_PI;
			return false;
		}
	}
	else
	{
		xyz[0] = btScalar(0.0);
		xyz[1] = btAtan2(btGetMatrixElem(mat,7),btGetMatrixElem(mat,8));
		xyz[2] = SIMD_HALF_PI;
	}
	return false;
}

bool bt6DofElasticPlastic2Constraint::matrixToEulerZXY(const btMatrix3x3& mat,btVector3& xyz)
{
	// rot =  cz*cy-sz*sx*sy    -cx*sz   cz*sy+cy*sz*sx
	//        cy*sz+cz*sx*sy     cz*cx   sz*sy-cz*xy*sx
	//        -cx*sy              sx     cx*cy

	btScalar fi = btGetMatrixElem(mat,7);
	if (fi < btScalar(1.0f))
	{
		if (fi > btScalar(-1.0f))
		{
			xyz[0] = btAsin(btGetMatrixElem(mat,7));
			xyz[1] = btAtan2(-btGetMatrixElem(mat,6),btGetMatrixElem(mat,8));
			xyz[2] = btAtan2(-btGetMatrixElem(mat,1),btGetMatrixElem(mat,4));
			return true;
		}
		else
		{
			xyz[0] = -SIMD_HALF_PI;
			xyz[1] = btScalar(0.0);
			xyz[2] = -btAtan2(btGetMatrixElem(mat,2),btGetMatrixElem(mat,0));
			return false;
		}
	}
	else
	{
		xyz[0] = SIMD_HALF_PI;
		xyz[1] = btScalar(0.0);
		xyz[2] = btAtan2(btGetMatrixElem(mat,2),btGetMatrixElem(mat,0));
	}
	return false;
}

bool bt6DofElasticPlastic2Constraint::matrixToEulerZYX(const btMatrix3x3& mat,btVector3& xyz)
{
	// rot =  cz*cy   cz*sy*sx-cx*sz   sz*sx+cz*cx*sy
	//        cy*sz   cz*cx+sz*sy*sx   cx*sz*sy-cz*sx
	//        -sy          cy*sx         cy*cx

	btScalar fi = btGetMatrixElem(mat,6);
	if (fi < btScalar(1.0f))
	{
		if (fi > btScalar(-1.0f))
		{
			xyz[0] = btAtan2(btGetMatrixElem(mat,7), btGetMatrixElem(mat,8));
			xyz[1] = btAsin(-btGetMatrixElem(mat,6));
			xyz[2] = btAtan2(btGetMatrixElem(mat,3),btGetMatrixElem(mat,0));
			return true;
		}
		else
		{
			xyz[0] = btScalar(0.0);
			xyz[1] = SIMD_HALF_PI;
			xyz[2] = -btAtan2(btGetMatrixElem(mat,1),btGetMatrixElem(mat,2));
			return false;
		}
	}
	else
	{
		xyz[0] = btScalar(0.0);
		xyz[1] = -SIMD_HALF_PI;
		xyz[2] = btAtan2(-btGetMatrixElem(mat,1),-btGetMatrixElem(mat,2));
	}
	return false;
}

void bt6DofElasticPlastic2Constraint::calculateAngleInfo()
{
	btMatrix3x3 relative_frame = m_calculatedTransformA.getBasis().inverse()*m_calculatedTransformB.getBasis();
	switch (m_rotateOrder)
	{
		case RO_XYZ : matrixToEulerXYZ(relative_frame,m_calculatedAxisAngleDiff); break;
		case RO_XZY : matrixToEulerXZY(relative_frame,m_calculatedAxisAngleDiff); break;
		case RO_YXZ : matrixToEulerYXZ(relative_frame,m_calculatedAxisAngleDiff); break;
		case RO_YZX : matrixToEulerYZX(relative_frame,m_calculatedAxisAngleDiff); break;
		case RO_ZXY : matrixToEulerZXY(relative_frame,m_calculatedAxisAngleDiff); break;
		case RO_ZYX : matrixToEulerZYX(relative_frame,m_calculatedAxisAngleDiff); break;
		default : btAssert(false);
	}
	// in euler angle mode we do not actually constrain the angular velocity
	// along the axes axis[0] and axis[2] (although we do use axis[1]) :
	//
	//    to get			constrain w2-w1 along		...not
	//    ------			---------------------		------
	//    d(angle[0])/dt = 0	ax[1] x ax[2]			ax[0]
	//    d(angle[1])/dt = 0	ax[1]
	//    d(angle[2])/dt = 0	ax[0] x ax[1]			ax[2]
	//
	// constraining w2-w1 along an axis 'a' means that a'*(w2-w1)=0.
	// to prove the result for angle[0], write the expression for angle[0] from
	// GetInfo1 then take the derivative. to prove this for angle[2] it is
	// easier to take the euler rate expression for d(angle[2])/dt with respect
	// to the components of w and set that to 0.
	switch (m_rotateOrder)
	{
	case RO_XYZ :
		{
			//Is this the "line of nodes" calculation choosing planes YZ (B coordinate system) and xy (A coordinate system)? (http://en.wikipedia.org/wiki/Euler_angles)
			//The two planes are non-homologous, so this is a Tait�Bryan angle formalism and not a proper Euler
			//Extrinsic rotations are equal to the reversed order intrinsic rotations so the above xyz extrinsic rotations (axes are fixed) are the same as the zy'x" intrinsic rotations (axes are refreshed after each rotation)
			//that is why xy and YZ planes are chosen (this will describe a zy'x" intrinsic rotation) (see the figure on the left at http://en.wikipedia.org/wiki/Euler_angles under Tait�Bryan angles)
			// x' = Nperp = N.cross(axis2)
			// y' = N = axis2.cross(axis0)	
			// z' = z
			//
			// x" = X
			// y" = y'
			// z" = ??
			//in other words:
			//first rotate around z
			//second rotate around y'= z.cross(X)
			//third rotate around x" = X
			//Original XYZ extrinsic rotation order. 
			//Planes: xy and YZ normals: z, X.  Plane intersection (N) is z.cross(X)
			btVector3 axis0 = m_calculatedTransformB.getBasis().getColumn(0);
			btVector3 axis2 = m_calculatedTransformA.getBasis().getColumn(2);
			m_calculatedAxis[1] = axis2.cross(axis0);
			m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
			m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);
			break;
		}
	case RO_XZY :
		{
			//planes: xz,ZY normals: y, X
			//first rotate around y
			//second rotate around z'= y.cross(X)
			//third rotate around x" = X
			btVector3 axis0 = m_calculatedTransformB.getBasis().getColumn(0);
			btVector3 axis1 = m_calculatedTransformA.getBasis().getColumn(1);
			m_calculatedAxis[2] = axis0.cross(axis1);
			m_calculatedAxis[0] = axis1.cross(m_calculatedAxis[2]);
			m_calculatedAxis[1] = m_calculatedAxis[2].cross(axis0);
			break;
		}
	case RO_YXZ :
		{
			//planes: yx,XZ normals: z, Y
			//first rotate around z
			//second rotate around x'= z.cross(Y)
			//third rotate around y" = Y
			btVector3 axis1 = m_calculatedTransformB.getBasis().getColumn(1);
			btVector3 axis2 = m_calculatedTransformA.getBasis().getColumn(2);
			m_calculatedAxis[0] = axis1.cross(axis2);
			m_calculatedAxis[1] = axis2.cross(m_calculatedAxis[0]);
			m_calculatedAxis[2] = m_calculatedAxis[0].cross(axis1);
			break;
		}
	case RO_YZX :
		{
			//planes: yz,ZX normals: x, Y
			//first rotate around x
			//second rotate around z'= x.cross(Y)
			//third rotate around y" = Y
			btVector3 axis0 = m_calculatedTransformA.getBasis().getColumn(0);
			btVector3 axis1 = m_calculatedTransformB.getBasis().getColumn(1);
			m_calculatedAxis[2] = axis0.cross(axis1);
			m_calculatedAxis[0] = axis1.cross(m_calculatedAxis[2]);
			m_calculatedAxis[1] = m_calculatedAxis[2].cross(axis0);
			break;
		}
	case RO_ZXY :
		{
			//planes: zx,XY normals: y, Z
			//first rotate around y
			//second rotate around x'= y.cross(Z)
			//third rotate around z" = Z
			btVector3 axis1 = m_calculatedTransformA.getBasis().getColumn(1);
			btVector3 axis2 = m_calculatedTransformB.getBasis().getColumn(2);
			m_calculatedAxis[0] = axis1.cross(axis2);
			m_calculatedAxis[1] = axis2.cross(m_calculatedAxis[0]);
			m_calculatedAxis[2] = m_calculatedAxis[0].cross(axis1);
			break;
		}
	case RO_ZYX :
		{
			//planes: zy,YX normals: x, Z
			//first rotate around x
			//second rotate around y' = x.cross(Z)
			//third rotate around z" = Z
			btVector3 axis0 = m_calculatedTransformA.getBasis().getColumn(0);
			btVector3 axis2 = m_calculatedTransformB.getBasis().getColumn(2);
			m_calculatedAxis[1] = axis2.cross(axis0);
			m_calculatedAxis[0] = m_calculatedAxis[1].cross(axis2);
			m_calculatedAxis[2] = axis0.cross(m_calculatedAxis[1]);
			break;
		}
	default:
		btAssert(false);
	}

	m_calculatedAxis[0].normalize();
	m_calculatedAxis[1].normalize();
	m_calculatedAxis[2].normalize();

}

void bt6DofElasticPlastic2Constraint::calculateTransforms()
{
	calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
}

void bt6DofElasticPlastic2Constraint::calculateTransforms(const btTransform& transA,const btTransform& transB)
{
	m_calculatedTransformA = transA * m_frameInA;
	m_calculatedTransformB = transB * m_frameInB;
	calculateLinearInfo();
	calculateAngleInfo();

	btScalar miA = getRigidBodyA().getInvMass();
	btScalar miB = getRigidBodyB().getInvMass();
	m_hasStaticBody = (miA < SIMD_EPSILON) || (miB < SIMD_EPSILON);
	btScalar miS = miA + miB;
	if(miS > btScalar(0.f))
	{
		m_factA = miB / miS;
	}
	else 
	{
		m_factA = btScalar(0.5f);
	}
	m_factB = btScalar(1.0f) - m_factA;
}


void bt6DofElasticPlastic2Constraint::testAngularLimitMotor(int axis_index)
{
	btScalar angle = m_calculatedAxisAngleDiff[axis_index];
	angle = btAdjustAngleToLimits(angle, m_angularLimits[axis_index].m_loLimit, m_angularLimits[axis_index].m_hiLimit);
	m_angularLimits[axis_index].m_currentPosition = angle;
	m_angularLimits[axis_index].testLimitValue(angle);
}


void bt6DofElasticPlastic2Constraint::getInfo1 (btConstraintInfo1* info)
{
	//prepare constraint
	calculateTransforms(m_rbA.getCenterOfMassTransform(),m_rbB.getCenterOfMassTransform());
	info->m_numConstraintRows = 0;
	info->nub = 0;
	int i;
	//test linear limits
	for(i = 0; i < 3; i++)
	{
		     if (m_linearLimits.m_currentLimit[i]==4) info->m_numConstraintRows += 2;
		else if (m_linearLimits.m_currentLimit[i]!=0) info->m_numConstraintRows += 1;
		if (m_linearLimits.m_enableMotor[i] ) info->m_numConstraintRows += 1;
		if (m_linearLimits.m_enableSpring[i]) info->m_numConstraintRows += 1;
	}
	//test angular limits
	for (i=0;i<3 ;i++ )
	{
		testAngularLimitMotor(i);
		     if (m_angularLimits[i].m_currentLimit==4) info->m_numConstraintRows += 2;
		else if (m_angularLimits[i].m_currentLimit!=0) info->m_numConstraintRows += 1;
		if (m_angularLimits[i].m_enableMotor ) info->m_numConstraintRows += 1;
		if (m_angularLimits[i].m_enableSpring) info->m_numConstraintRows += 1;
	}
}


void bt6DofElasticPlastic2Constraint::getInfo2 (btConstraintInfo2* info)
{
	BT_PROFILE("bt6DofElasticPlastic2Constraint::getInfo2");
	const btTransform& transA = m_rbA.getCenterOfMassTransform();
	const btTransform& transB = m_rbB.getCenterOfMassTransform();
	const btVector3& linVelA = m_rbA.getLinearVelocity();
	const btVector3& linVelB = m_rbB.getLinearVelocity();
	const btVector3& angVelA = m_rbA.getAngularVelocity();
	const btVector3& angVelB = m_rbB.getAngularVelocity();

	// for stability better to solve angular limits first
	int row = setAngularLimits(info, 0,transA,transB,linVelA,linVelB,angVelA,angVelB);
	setLinearLimits(info, row, transA,transB,linVelA,linVelB,angVelA,angVelB);
	PlasticityData::log(info,8);
}


int bt6DofElasticPlastic2Constraint::setLinearLimits(btConstraintInfo2* info, int row, const btTransform& transA,const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB,const btVector3& angVelA,const btVector3& angVelB)
{
	//solve linear limits
	btRotationalLimitMotor2 limot;
	for (int i=0;i<3 ;i++ )
	{
		if(m_linearLimits.m_currentLimit[i] || m_linearLimits.m_enableMotor[i] || m_linearLimits.m_enableSpring[i])
		{ // re-use rotational motor code
			limot.m_bounce                 = m_linearLimits.m_bounce[i];
			limot.m_currentLimit           = m_linearLimits.m_currentLimit[i];
			limot.m_currentPosition        = m_linearLimits.m_currentLinearDiff[i];
			limot.m_currentLimitError      = m_linearLimits.m_currentLimitError[i];
			limot.m_currentLimitErrorHi    = m_linearLimits.m_currentLimitErrorHi[i];
			limot.m_enableMotor            = m_linearLimits.m_enableMotor[i];
			limot.m_servoMotor             = m_linearLimits.m_servoMotor[i];
			limot.m_servoTarget            = m_linearLimits.m_servoTarget[i];
			limot.m_enableSpring           = m_linearLimits.m_enableSpring[i];
			limot.m_springStiffness        = m_linearLimits.m_springStiffness[i];
			limot.m_springStiffnessLimited = m_linearLimits.m_springStiffnessLimited[i];
			limot.m_springDamping          = m_linearLimits.m_springDamping[i];
			limot.m_springDampingLimited   = m_linearLimits.m_springDampingLimited[i];
			limot.m_equilibriumPoint       = m_linearLimits.m_equilibriumPoint[i];
			limot.m_hiLimit                = m_linearLimits.m_upperLimit[i];
			limot.m_loLimit                = m_linearLimits.m_lowerLimit[i];
			limot.m_maxMotorForce          = m_linearLimits.m_maxMotorForce[i];
			limot.m_targetVelocity         = m_linearLimits.m_targetVelocity[i];
			btVector3 axis = m_calculatedTransformA.getBasis().getColumn(i);
			int flags = m_flags >> (i * BT_6DOF_FLAGS_AXIS_SHIFT2);
			limot.m_stopCFM  = (flags & BT_6DOF_FLAGS_CFM_STOP2) ? m_linearLimits.m_stopCFM[i] : info->cfm[0];
			limot.m_stopERP  = (flags & BT_6DOF_FLAGS_ERP_STOP2) ? m_linearLimits.m_stopERP[i] : info->erp;
			limot.m_motorCFM = (flags & BT_6DOF_FLAGS_CFM_MOTO2) ? m_linearLimits.m_motorCFM[i] : info->cfm[0];
			limot.m_motorERP = (flags & BT_6DOF_FLAGS_ERP_MOTO2) ? m_linearLimits.m_motorERP[i] : info->erp;

			//rotAllowed is a bit of a magic from the original 6dof. The calculation of it here is something that imitates the original behavior as much as possible.
			int indx1 = (i + 1) % 3;
			int indx2 = (i + 2) % 3;
			int rotAllowed = 1; // rotations around orthos to current axis (it is used only when one of the body is static)
			#define D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION 1.0e-3
			bool indx1Violated = m_angularLimits[indx1].m_currentLimit == 1 ||
				m_angularLimits[indx1].m_currentLimit == 2 ||
				( m_angularLimits[indx1].m_currentLimit == 3 && ( m_angularLimits[indx1].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx1].m_currentLimitError > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) ) ||
				( m_angularLimits[indx1].m_currentLimit == 4 && ( m_angularLimits[indx1].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx1].m_currentLimitErrorHi > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) );
			bool indx2Violated = m_angularLimits[indx2].m_currentLimit == 1 ||
				m_angularLimits[indx2].m_currentLimit == 2 ||
				( m_angularLimits[indx2].m_currentLimit == 3 && ( m_angularLimits[indx2].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx2].m_currentLimitError > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) ) ||
				( m_angularLimits[indx2].m_currentLimit == 4 && ( m_angularLimits[indx2].m_currentLimitError < -D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION || m_angularLimits[indx2].m_currentLimitErrorHi > D6_LIMIT_ERROR_THRESHOLD_FOR_ROTATION ) );
			if( indx1Violated && indx2Violated )
			{
				rotAllowed = 0;
			}
			row += get_limit_motor_info2(&limot, transA,transB,linVelA,linVelB,angVelA,angVelB, info, row, axis, 0, rotAllowed, m_maxForce[i],i);

		}
	}
	return row;
}



int bt6DofElasticPlastic2Constraint::setAngularLimits(btConstraintInfo2 *info, int row_offset, const btTransform& transA,const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB,const btVector3& angVelA,const btVector3& angVelB)
{
	int row = row_offset;

	//order of rotational constraint rows
	int cIdx[] = {0, 1, 2};
	switch(m_rotateOrder)
	{
		case RO_XYZ : cIdx[0] = 0; cIdx[1] = 1; cIdx[2] = 2; break;
		case RO_XZY : cIdx[0] = 0; cIdx[1] = 2; cIdx[2] = 1; break;
		case RO_YXZ : cIdx[0] = 1; cIdx[1] = 0; cIdx[2] = 2; break;
		case RO_YZX : cIdx[0] = 1; cIdx[1] = 2; cIdx[2] = 0; break;
		case RO_ZXY : cIdx[0] = 2; cIdx[1] = 0; cIdx[2] = 1; break;
		case RO_ZYX : cIdx[0] = 2; cIdx[1] = 1; cIdx[2] = 0; break;
		default : btAssert(false);
	}

	for (int ii = 0; ii < 3 ; ii++ )
	{
		int i = cIdx[ii];
		if(m_angularLimits[i].m_currentLimit || m_angularLimits[i].m_enableMotor || m_angularLimits[i].m_enableSpring)
		{
			btVector3 axis = getAxis(i);
			int flags = m_flags >> ((i + 3) * BT_6DOF_FLAGS_AXIS_SHIFT2);
			if(!(flags & BT_6DOF_FLAGS_CFM_STOP2))
			{
				m_angularLimits[i].m_stopCFM = info->cfm[0];
			}
			if(!(flags & BT_6DOF_FLAGS_ERP_STOP2))
			{
				m_angularLimits[i].m_stopERP = info->erp;
			}
			if(!(flags & BT_6DOF_FLAGS_CFM_MOTO2))
			{
				m_angularLimits[i].m_motorCFM = info->cfm[0];
			}
			if(!(flags & BT_6DOF_FLAGS_ERP_MOTO2))
			{
				m_angularLimits[i].m_motorERP = info->erp;
			}
			row += get_limit_motor_info2(&m_angularLimits[i],transA,transB,linVelA,linVelB,angVelA,angVelB, 
				info,row,axis,1, false, m_maxForce[ii+3],ii+3);
		}
	}

	return row;
}


void bt6DofElasticPlastic2Constraint::setFrames(const btTransform& frameA, const btTransform& frameB)
{
	m_frameInA = frameA;
	m_frameInB = frameB;
	buildJacobian();
	calculateTransforms();
}


void bt6DofElasticPlastic2Constraint::calculateLinearInfo()
{
	m_calculatedLinearDiff = m_calculatedTransformB.getOrigin() - m_calculatedTransformA.getOrigin();
	m_calculatedLinearDiff = m_calculatedTransformA.getBasis().inverse() * m_calculatedLinearDiff;
	for(int i = 0; i < 3; i++)
	{
		m_linearLimits.m_currentLinearDiff[i] = m_calculatedLinearDiff[i];
		m_linearLimits.testLimitValue(i, m_calculatedLinearDiff[i]);
	}
}

void bt6DofElasticPlastic2Constraint::calculateJacobi(btRotationalLimitMotor2 * limot, const btTransform& transA,const btTransform& transB, btConstraintInfo2 *info, int srow, btVector3& ax1, int rotational, int rotAllowed)
{
	btScalar *J1 = rotational ? info->m_J1angularAxis : info->m_J1linearAxis;
	btScalar *J2 = rotational ? info->m_J2angularAxis : info->m_J2linearAxis;

	J1[srow+0] = ax1[0];
	J1[srow+1] = ax1[1];
	J1[srow+2] = ax1[2];

	J2[srow+0] = -ax1[0];
	J2[srow+1] = -ax1[1];
	J2[srow+2] = -ax1[2];

	if(!rotational)
	{
		btVector3 tmpA, tmpB, relA, relB;
		// get vector from bodyB to frameB in WCS
		relB = m_calculatedTransformB.getOrigin() - transB.getOrigin();
		// same for bodyA
		relA = m_calculatedTransformA.getOrigin() - transA.getOrigin();
		tmpA = relA.cross(ax1);
		tmpB = relB.cross(ax1);
		if(m_hasStaticBody && (!rotAllowed))
		{
			tmpA *= m_factA;
			tmpB *= m_factB;
		}
		int i;
		for (i=0; i<3; i++) info->m_J1angularAxis[srow+i] = tmpA[i];
		for (i=0; i<3; i++) info->m_J2angularAxis[srow+i] = -tmpB[i];
	}
}

// bcc maxForce and dof added (SIMD_INFINITY replaced) 
int bt6DofElasticPlastic2Constraint::get_limit_motor_info2(
	btRotationalLimitMotor2 * limot,
	const btTransform& transA,const btTransform& transB,const btVector3& linVelA,const btVector3& linVelB,const btVector3& angVelA,const btVector3& angVelB,
	btConstraintInfo2 *info, int row, btVector3& ax1, int rotational,int rotAllowed, btScalar maxForce, int dof)
{
	BT_PROFILE("bt6DofElasticPlastic2Constraint::get_limit_motor_info2");
	bool useBcc = true;
	int count = 0;
	int srow = row * info->rowskip;
	btScalar dt = BT_ONE / info->fps;
	btScalar maxImpulse;
	if (useBcc && maxForce < SIMD_INFINITY){
		maxImpulse = maxForce*dt;
	}
	else{
		maxImpulse = SIMD_INFINITY;
	}

	if (limot->m_currentLimit==4) 
	{
		btScalar vel = rotational ? angVelA.dot(ax1) - angVelB.dot(ax1) : linVelA.dot(ax1) - linVelB.dot(ax1);

		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
		info->m_constraintError[srow] = info->fps * limot->m_stopERP * limot->m_currentLimitError * (rotational ? -1 : 1);
		if (rotational) {
			if (info->m_constraintError[srow]-vel*limot->m_stopERP > 0) {
				btScalar bounceerror = -limot->m_bounce* vel;
				if (bounceerror > info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
			}
		} else {
			if (info->m_constraintError[srow]-vel*limot->m_stopERP < 0) {
				btScalar bounceerror = -limot->m_bounce* vel;
				if (bounceerror < info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
			}
		}
		info->m_lowerLimit[srow] = rotational ? 0 : -maxImpulse;
		info->m_upperLimit[srow] = rotational ? maxImpulse : 0;
		info->cfm[srow] = limot->m_stopCFM;
		srow += info->rowskip;
		++count;

		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
		info->m_constraintError[srow] = info->fps * limot->m_stopERP * limot->m_currentLimitErrorHi * (rotational ? -1 : 1);
		if (rotational) {
			if (info->m_constraintError[srow]-vel*limot->m_stopERP < 0) {
				btScalar bounceerror = -limot->m_bounce* vel;
				if (bounceerror < info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
			}
		} else {
			if (info->m_constraintError[srow]-vel*limot->m_stopERP > 0) {
				btScalar bounceerror = -limot->m_bounce* vel;
				if (bounceerror > info->m_constraintError[srow]) info->m_constraintError[srow] = bounceerror;
			}
		}
		info->m_lowerLimit[srow] = rotational ? -maxImpulse : 0;
		info->m_upperLimit[srow] = rotational ? 0 : maxImpulse;
		info->cfm[srow] = limot->m_stopCFM;
		srow += info->rowskip;
		++count;
	} else
	if (limot->m_currentLimit==3) 
	{
		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
		info->m_constraintError[srow] = info->fps * limot->m_stopERP * limot->m_currentLimitError * (rotational ? -1 : 1);
		info->m_lowerLimit[srow] = -maxImpulse;
		info->m_upperLimit[srow] = maxImpulse;
		info->cfm[srow] = limot->m_stopCFM;
		srow += info->rowskip;
		++count;
	}

	if (limot->m_enableMotor && !limot->m_servoMotor)
	{
		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
		btScalar tag_vel = rotational ? limot->m_targetVelocity : -limot->m_targetVelocity;
		btScalar mot_fact = getMotorFactor(limot->m_currentPosition, 
			limot->m_loLimit,
			limot->m_hiLimit,
			tag_vel,
			info->fps * limot->m_motorERP);
		info->m_constraintError[srow] = mot_fact * limot->m_targetVelocity;
		info->m_lowerLimit[srow] = -limot->m_maxMotorForce;
		info->m_upperLimit[srow] = limot->m_maxMotorForce;
		info->cfm[srow] = limot->m_motorCFM;
		srow += info->rowskip;
		++count;
	}

	if (limot->m_enableMotor && limot->m_servoMotor)
	{
		btScalar error = limot->m_currentPosition - limot->m_servoTarget;
		calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
		btScalar targetvelocity = error<0 ? -limot->m_targetVelocity : limot->m_targetVelocity;
		btScalar tag_vel = -targetvelocity;
		btScalar mot_fact;
		if(error != 0)
		{
			btScalar lowLimit;
			btScalar hiLimit;
			if(limot->m_loLimit > limot->m_hiLimit)
			{
				lowLimit = error > 0 ? limot->m_servoTarget : -maxImpulse;
				hiLimit = error < 0 ? limot->m_servoTarget : maxImpulse;
			}
			else
			{
				lowLimit = error > 0 && limot->m_servoTarget>limot->m_loLimit ? limot->m_servoTarget : limot->m_loLimit;
				hiLimit  = error < 0 && limot->m_servoTarget<limot->m_hiLimit ? limot->m_servoTarget : limot->m_hiLimit;
			}
			mot_fact = getMotorFactor(limot->m_currentPosition, lowLimit, hiLimit, tag_vel, info->fps * limot->m_motorERP);
		} 
		else 
		{
			mot_fact = 0;
		}
		info->m_constraintError[srow] = mot_fact * targetvelocity * (rotational ? -1 : 1);
		info->m_lowerLimit[srow] = -limot->m_maxMotorForce;
		info->m_upperLimit[srow] = limot->m_maxMotorForce;
		info->cfm[srow] = limot->m_motorCFM;
		srow += info->rowskip;
		++count;
	}

	if (limot->m_enableSpring)
	{
		if (!useBcc){
			btScalar error = limot->m_currentPosition - limot->m_equilibriumPoint;
			calculateJacobi(limot, transA, transB, info, srow, ax1, rotational, rotAllowed);
			btScalar kd = limot->m_springDamping;
			btScalar ks = limot->m_springStiffness;
			btScalar vel = rotational ? angVelA.dot(ax1) - angVelB.dot(ax1) : linVelA.dot(ax1) - linVelB.dot(ax1);
			//		btScalar erp = 0.1;
			btScalar cfm = BT_ZERO;
			btScalar mA = BT_ONE / m_rbA.getInvMass();
			btScalar mB = BT_ONE / m_rbB.getInvMass();
			btScalar m = mA > mB ? mB : mA;
			btScalar angularfreq = sqrt(ks / m);


			//limit stiffness (the spring should not be sampled faster that the quarter of its angular frequency)
			if (limot->m_springStiffnessLimited && 0.25 < angularfreq * dt)
			{
				ks = BT_ONE / dt / dt / btScalar(16.0) * m;
			}
			//avoid damping that would blow up the spring
			if (limot->m_springDampingLimited && kd * dt > m)
			{
				kd = m / dt;
			}
			btScalar fs = ks * error * dt;
			btScalar fd = -kd * (vel)* (rotational ? -1 : 1) * dt;
			btScalar f = (fs + fd);

			info->m_constraintError[srow] = (vel + f * (rotational ? -1 : 1));

			btScalar minf = f < fd ? f : fd;
			btScalar maxf = f < fd ? fd : f;
			if (!rotational)
			{
				info->m_lowerLimit[srow] = minf > 0 ? 0 : minf;
				info->m_upperLimit[srow] = maxf < 0 ? 0 : maxf;
			}
			else
			{
				info->m_lowerLimit[srow] = -maxf > 0 ? 0 : -maxf;
				info->m_upperLimit[srow] = -minf < 0 ? 0 : -minf;
			}

			info->cfm[srow] = cfm;
			srow += info->rowskip;
			++count;
		} else{ // useBcc
			/**
			buddha springs are explained in
			http://box2d.org/files/GDC2011/GDC2011_Catto_Erin_Soft_Constraints.pdf
			Using them properly should make (at least simple) constraints
			unconditionally stable but in this case
			either implementation is not correct or theory is not valid
			*/
			bool useBuddha = false;
			calculateJacobi(limot,transA,transB,info,srow,ax1,rotational,rotAllowed);
			btScalar kd = limot->m_springDamping;
			btScalar ks = limot->m_springStiffness;
			btScalar cfm = BT_ZERO; // gamma in buddha
			btScalar erp = limot->m_stopERP; // beta in buddha
			if (useBuddha){
				cfm = BT_ONE / (kd+dt*ks);
				erp = (dt*ks) / (kd + dt*ks);
			}
			// bcc
			bool usePlasticity=false;
			bool frequencyLimited = false;
			btScalar vel = rotational ? 
				angVelA.dot(ax1) - angVelB.dot(ax1) : 
				linVelA.dot(ax1) - linVelB.dot(ax1);
			btScalar error=limot->m_currentPosition - limot->m_equilibriumPoint;
			/** 
			* for testing if expected error at end of step could be used
			*/
			bool useVelInError = false;
			if (useVelInError){
				error += vel*dt;
			}
			btScalar m;
			btScalar afdt;
			btScalar f;
			btScalar fs;
			limitReason[dof] = None;
			if (monitorVelocityDirection>0){
				if (isLimitNeeded(vel, dof)){
					limitReason[dof] = Monitor;
					frequencyLimited = true;
				}
			}
			if (!frequencyLimited && (limot->m_springStiffnessLimited || limot->m_springDampingLimited))
			{
				// bcc
				btScalar mAI;
				btScalar mBI;
				if (rotational){
					mAI = m_rbA.getInvInertiaDiagLocal().dot(ax1);
					mBI = m_rbB.getInvInertiaDiagLocal().dot(ax1);
				}
				else{
					mAI = m_rbA.getInvMass();
					mBI = m_rbB.getInvMass();
				}
				btScalar mA = (mAI>SIMD_EPSILON?BT_ONE / mAI:SIMD_INFINITY);
				btScalar mB = (mBI>SIMD_EPSILON?BT_ONE / mBI:SIMD_INFINITY);
				m = mA > mB ? mB : mA;
				// limit stiffness (the spring should not be sampled 
				// faster that the quarter of its angular frequency)
				if (limot->m_springStiffnessLimited){
					btScalar angularfreq = sqrt(ks / m);
					afdt = angularfreq * dt;
					if (0.25 < afdt){
						limitReason[dof] = Stiffness;
						if (maxForce < SIMD_INFINITY){
							frequencyLimited = true;
						}
						else{
							ks = BT_ONE / dt / dt / btScalar(16.0) * m;
						}
					}
				}
				//avoid damping that would blow up the spring
				if (limot->m_springDampingLimited)
				{
					btScalar kddt = kd*dt;
					if (kddt > m){
						limitReason[dof] = Damping;
						if (maxForce < SIMD_INFINITY){
							frequencyLimited = true;
						}
						else{
							kd = m / dt;
						}
					}
				}
			}
			// bcc
			btScalar minf;
			btScalar maxf;
			btScalar fd;
			fs = ks * error * dt;
			fd = -kd * (vel)* (rotational ? -1 : 1) * dt;
			f = (fs + fd);
			if (btFabs(f) > maxImpulse){
				limitReason[dof] = Force;
				usePlasticity = true;
			}
			else if(!frequencyLimited){
				info->m_constraintError[srow] = (vel + f * (rotational ? -1 : 1));
				minf = f < fd ? f : fd;
				maxf = f < fd ? fd : f;
			}
			/*
			constraintError is set:
			For frequencyLimited limited case same as above for constraint case 
			(upper==lower/limot->m_currentLimit==3) is used 
			If maximum impulse has been reached error is set to zero.
			*/
			if (usePlasticity || frequencyLimited){
				minf = -maxImpulse;
				maxf = maxImpulse;
				if (frequencyLimited){
					f = info->fps*erp*error;
				}
				else{
					f = 0;
				}
				info->m_constraintError[srow] = f*(rotational ? -1 : 1);
			}
			if(!rotational)
			{
				info->m_lowerLimit[srow] = minf > 0 ? 0 : minf;
				info->m_upperLimit[srow] = maxf < 0 ? 0 : maxf;
			}
			else
			{
				info->m_lowerLimit[srow] = -maxf > 0 ? 0 : -maxf;
				info->m_upperLimit[srow] = -minf < 0 ? 0 : -minf;
			}

			info->cfm[srow] = cfm;
			srow += info->rowskip;
			++count;
		}
	}

	return count;
}


//override the default global value of a parameter (such as ERP or CFM), optionally provide the axis (0..5). 
//If no axis is provided, it uses the default axis for this constraint.
void bt6DofElasticPlastic2Constraint::setParam(int num, btScalar value, int axis)
{
	if((axis >= 0) && (axis < 3))
	{
		switch(num)
		{
			case BT_CONSTRAINT_STOP_ERP : 
				m_linearLimits.m_stopERP[axis] = value;
				m_flags |= BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				m_linearLimits.m_stopCFM[axis] = value;
				m_flags |= BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_ERP : 
				m_linearLimits.m_motorERP[axis] = value;
				m_flags |= BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_CFM : 
				m_linearLimits.m_motorCFM[axis] = value;
				m_flags |= BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			default : 
				btAssertConstrParams(0);
		}
	}
	else if((axis >=3) && (axis < 6))
	{
		switch(num)
		{
			case BT_CONSTRAINT_STOP_ERP : 
				m_angularLimits[axis - 3].m_stopERP = value;
				m_flags |= BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				m_angularLimits[axis - 3].m_stopCFM = value;
				m_flags |= BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_ERP : 
				m_angularLimits[axis - 3].m_motorERP = value;
				m_flags |= BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			case BT_CONSTRAINT_CFM : 
				m_angularLimits[axis - 3].m_motorCFM = value;
				m_flags |= BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2);
				break;
			default : 
				btAssertConstrParams(0);
		}
	}
	else
	{
		btAssertConstrParams(0);
	}
}

//return the local value of parameter
btScalar bt6DofElasticPlastic2Constraint::getParam(int num, int axis) const 
{
	btScalar retVal = 0;
	if((axis >= 0) && (axis < 3))
	{
		switch(num)
		{
			case BT_CONSTRAINT_STOP_ERP : 
				btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_linearLimits.m_stopERP[axis];
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_linearLimits.m_stopCFM[axis];
				break;
			case BT_CONSTRAINT_ERP : 
				btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_linearLimits.m_motorERP[axis];
				break;
			case BT_CONSTRAINT_CFM : 
				btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_linearLimits.m_motorCFM[axis];
				break;
			default : 
				btAssertConstrParams(0);
		}
	}
	else if((axis >=3) && (axis < 6))
	{
		switch(num)
		{
			case BT_CONSTRAINT_STOP_ERP : 
				btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_angularLimits[axis - 3].m_stopERP;
				break;
			case BT_CONSTRAINT_STOP_CFM : 
				btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_STOP2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_angularLimits[axis - 3].m_stopCFM;
				break;
			case BT_CONSTRAINT_ERP : 
				btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_ERP_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_angularLimits[axis - 3].m_motorERP;
				break;
			case BT_CONSTRAINT_CFM : 
				btAssertConstrParams(m_flags & (BT_6DOF_FLAGS_CFM_MOTO2 << (axis * BT_6DOF_FLAGS_AXIS_SHIFT2)));
				retVal = m_angularLimits[axis - 3].m_motorCFM;
				break;
			default : 
				btAssertConstrParams(0);
		}
	}
	else
	{
		btAssertConstrParams(0);
	}
	return retVal;
}

 

void bt6DofElasticPlastic2Constraint::setAxis(const btVector3& axis1,const btVector3& axis2)
{
	btVector3 zAxis = axis1.normalized();
	btVector3 yAxis = axis2.normalized();
	btVector3 xAxis = yAxis.cross(zAxis); // we want right coordinate system
	
	btTransform frameInW;
	frameInW.setIdentity();
	frameInW.getBasis().setValue( xAxis[0], yAxis[0], zAxis[0],
	                              xAxis[1], yAxis[1], zAxis[1],
	                              xAxis[2], yAxis[2], zAxis[2]);
	
	// now get constraint frame in local coordinate systems
	m_frameInA = m_rbA.getCenterOfMassTransform().inverse() * frameInW;
	m_frameInB = m_rbB.getCenterOfMassTransform().inverse() * frameInW;
	
	calculateTransforms();
}

void bt6DofElasticPlastic2Constraint::setBounce(int index, btScalar bounce)
{
	btAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_bounce[index] = bounce;
	else
		m_angularLimits[index - 3].m_bounce = bounce;
}

void bt6DofElasticPlastic2Constraint::enableMotor(int index, bool onOff)
{
	btAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_enableMotor[index] = onOff;
	else
		m_angularLimits[index - 3].m_enableMotor = onOff;
}

void bt6DofElasticPlastic2Constraint::setServo(int index, bool onOff)
{
	btAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_servoMotor[index] = onOff;
	else
		m_angularLimits[index - 3].m_servoMotor = onOff;
}

void bt6DofElasticPlastic2Constraint::setTargetVelocity(int index, btScalar velocity)
{
	btAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_targetVelocity[index] = velocity;
	else
		m_angularLimits[index - 3].m_targetVelocity = velocity;
}

void bt6DofElasticPlastic2Constraint::setServoTarget(int index, btScalar target)
{
	btAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_servoTarget[index] = target;
	else
		m_angularLimits[index - 3].m_servoTarget = target;
}

void bt6DofElasticPlastic2Constraint::setMaxMotorForce(int index, btScalar force)
{
	btAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_maxMotorForce[index] = force;
	else
		m_angularLimits[index - 3].m_maxMotorForce = force;
}

void bt6DofElasticPlastic2Constraint::enableSpring(int index, bool onOff)
{
	btAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_enableSpring[index] = onOff;
	else
		m_angularLimits[index - 3] .m_enableSpring = onOff;
}

void bt6DofElasticPlastic2Constraint::setStiffness(int index, btScalar stiffness, bool limitIfNeeded)
{
	btAssert((index >= 0) && (index < 6));
	if (index<3) {
		m_linearLimits.m_springStiffness[index] = stiffness;
		m_linearLimits.m_springStiffnessLimited[index] = limitIfNeeded;
	} else {
		m_angularLimits[index - 3].m_springStiffness = stiffness;
		m_angularLimits[index - 3].m_springStiffnessLimited = limitIfNeeded;
	}
}

void bt6DofElasticPlastic2Constraint::setDamping(int index, btScalar damping, bool limitIfNeeded)
{
	btAssert((index >= 0) && (index < 6));
	if (index<3) {
		m_linearLimits.m_springDamping[index] = damping;
		m_linearLimits.m_springDampingLimited[index] = limitIfNeeded;
	} else {
		m_angularLimits[index - 3].m_springDamping = damping;
		m_angularLimits[index - 3].m_springDampingLimited = limitIfNeeded;
	}
}

void bt6DofElasticPlastic2Constraint::setEquilibriumPoint()
{
	calculateTransforms();
	int i;
	for( i = 0; i < 3; i++)
		m_linearLimits.m_equilibriumPoint[i] = m_calculatedLinearDiff[i];
	for(i = 0; i < 3; i++)
		m_angularLimits[i].m_equilibriumPoint = m_calculatedAxisAngleDiff[i];
}

void bt6DofElasticPlastic2Constraint::setEquilibriumPoint(int index)
{
	btAssert((index >= 0) && (index < 6));
	calculateTransforms();
	if (index<3)
		m_linearLimits.m_equilibriumPoint[index] = m_calculatedLinearDiff[index];
	else
		m_angularLimits[index - 3] .m_equilibriumPoint = m_calculatedAxisAngleDiff[index - 3];
}

void bt6DofElasticPlastic2Constraint::setEquilibriumPoint(int index, btScalar val)
{
	btAssert((index >= 0) && (index < 6));
	if (index<3)
		m_linearLimits.m_equilibriumPoint[index] = val;
	else
		m_angularLimits[index - 3] .m_equilibriumPoint = val;
}

/*
// bcc starts
*/
void bt6DofElasticPlastic2Constraint::setMaxForce(int index, btScalar maxForce)
{
	btAssert((index >= 0) && (index < 6));
	m_maxForce[index] = maxForce;
}
void bt6DofElasticPlastic2Constraint::setMaxPlasticStrain(btScalar value){
	m_maxPlasticStrain = value;
	/*
	upper/lower limits cannot be used at same time as plasticity
	They are set to +/- maxPlasticStrain to avoid conflict in most cases
	*/
	btScalar ul(value);
	btScalar ll(-value);
	setLinearUpperLimit(btVector3(ll, ll, ll));
	setLinearLowerLimit(btVector3(ul, ul, ul));
}
btScalar bt6DofElasticPlastic2Constraint::getMaxPlasticStrain(){
	return m_maxPlasticStrain;
}
btScalar bt6DofElasticPlastic2Constraint::getCurrentPlasticStrain(){
	return m_currentPlasticStrain;
}
void bt6DofElasticPlastic2Constraint::setMaxPlasticRotation(btScalar value){
	m_maxPlasticRotation = value;
}
btScalar bt6DofElasticPlastic2Constraint::getDisplacement(int dof){
	btVector3 vec;
	if (dof > 2){
		dof -= 3;
		vec = m_calculatedAxisAngleDiff;
	}
	else{
		vec = m_calculatedLinearDiff;
	}
	return vec.m_floats[dof];
}

btScalar bt6DofElasticPlastic2Constraint::getMaxPlasticRotation(){
	return m_maxPlasticRotation;
}
btScalar bt6DofElasticPlastic2Constraint::getCurrentPlasticRotation(){
	return m_currentPlasticRotation;
}
btTransform & bt6DofElasticPlastic2Constraint::getFrameA(){
	return m_frameInA;
}
btTransform & bt6DofElasticPlastic2Constraint::getFrameB(){
	return m_frameInB;
}
btTransform & bt6DofElasticPlastic2Constraint::getTransformA(){
	return m_calculatedTransformA;
}
btTransform & bt6DofElasticPlastic2Constraint::getTransformB(){
	return m_calculatedTransformB;
}
void bt6DofElasticPlastic2Constraint::scalePlasticity(btScalar scale){
	m_maxPlasticStrain *= scale;
	m_maxPlasticRotation *= scale;
}
btScalar bt6DofElasticPlastic2Constraint::getMaxRatio(){
	return m_maxRatio;
}
int bt6DofElasticPlastic2Constraint::getMaxRatioDof(){
	return m_maxRatioDof;
}

btScalar bt6DofElasticPlastic2Constraint::getMaxAbsForce(btJointFeedback& forces, int i){
	btScalar a = btFabs(forces.m_appliedForceBodyA[i]);
	btScalar b = btFabs(forces.m_appliedForceBodyB[i]);
	if (a > b){
		return a;
	}
	else{
		return b;
	}
}

btScalar bt6DofElasticPlastic2Constraint::getMaxAbsMoment(btJointFeedback& forces, int i){
	btScalar a = btFabs(forces.m_appliedTorqueBodyA[i]);
	btScalar b = btFabs(forces.m_appliedTorqueBodyB[i]);
	if (a > b){
		return a;
	}
	else{
		return b;
	}
}


void bt6DofElasticPlastic2Constraint::updatePlasticity(btJointFeedback& forces){
	BT_PROFILE("updatePlasticity");
	if (!isEnabled()){
		return;
	}
	m_maxRatio = 0;
	m_maxRatioDof = -1;
	int i;
	for (i = 0; i < 3; i++)
	{
		if (m_linearLimits.m_enableSpring[i])
		{
			btScalar currPos = m_calculatedLinearDiff[i];
			btScalar delta = currPos - m_linearLimits.m_equilibriumPoint[i];
			btScalar absForce = getMaxAbsForce(forces, i);
			btScalar ratio = absForce / m_maxForce[i];
			if (ratio > m_maxRatio){
				m_maxRatio = ratio;
				m_maxRatioDof = i;
			}
			if (ratio>0.99999){
				btScalar elasticPart = m_maxForce[i] / m_linearLimits.m_springStiffness[i];
				btScalar newVal = currPos;
				if (btFabs(elasticPart)<btFabs(currPos)){
					newVal += (currPos > 0 ? -elasticPart : elasticPart);
				}
				btScalar plasticDelta = btFabs(delta)-elasticPart;
				/* direction changes */
				if ((m_linearLimits.m_equilibriumPoint[i] * currPos)<0){
					plasticDelta -= elasticPart;
				}
				if (plasticDelta < 0){
					plasticDelta = 0;
				}
				setEquilibriumPoint(i, newVal);
				m_currentPlasticStrain += plasticDelta;
				m_plasticDisplacement[i] += plasticDelta;
			}
		}
	}
	for (i = 0; i < 3; i++)
	{
		if (m_angularLimits[i].m_enableSpring)
		{
			btScalar currPos = m_calculatedAxisAngleDiff[i];
			btScalar delta = currPos - m_angularLimits[i].m_equilibriumPoint;
			btScalar absForce = getMaxAbsMoment(forces,i);
			btScalar ratio = absForce / m_maxForce[i+3];
			if (ratio > m_maxRatio){
				m_maxRatio = ratio;
				m_maxRatioDof = i+3;
			}
			if (ratio>0.99999){
				btScalar elasticPart = m_maxForce[i + 3] / m_angularLimits[i].m_springStiffness;
				btScalar newVal = currPos;
				if (btFabs(elasticPart)<btFabs(currPos)){
					newVal += (currPos > 0 ? -elasticPart : elasticPart);
				}
				btScalar plasticDelta = btFabs(delta)-elasticPart;
				/* direction changes */
				if ((m_angularLimits[i].m_equilibriumPoint*currPos)<0){
					plasticDelta -= elasticPart;
				}
				if (plasticDelta < 0){
					plasticDelta = 0;
				}
				setEquilibriumPoint(i + 3, newVal);
				m_currentPlasticRotation += plasticDelta;
				m_plasticDisplacement[i+3] += plasticDelta;
			}
		}
	}
	downScale();
}
/**
1 means that constraint is broken when maximum plastic rotation or strain is exceeded
0.5 means that maximum forces are scaled down between max and twice the max strain
0 means that constraint is never broken and maximum forces remain same
*/
btScalar bt6DofElasticPlastic2Constraint::downScaleRamp(1.);
/**
Scales maximum forces down so that
elastic energy can consumed before breaking of constraint
*/
void bt6DofElasticPlastic2Constraint::downScale(){
	btScalar maxCurrent = btMax(m_currentPlasticRotation/m_maxPlasticRotation,
		m_currentPlasticStrain/m_maxPlasticStrain);
	btScalar scale = 1 - maxCurrent*downScaleRamp;
	if (scale <= 0){
		setEnabled(false);
		return;
	}
	if (downScaleRamp >= 1.){
		return;
	}
	for (int i = 0; i < 6; i++){
		m_maxForce[i] *= scale / factorForDownScale;
	}
	factorForDownScale = scale;
}
void bt6DofElasticPlastic2Constraint::debugDraw(btIDebugDraw* debugDrawer)
{
	PlasticityDebugDrawer::drawPlasticConstraint((btElasticPlasticConstraint*)(this), 
		(btTypedConstraint*)(this),
		debugDrawer);
}

LimitReason bt6DofElasticPlastic2Constraint::getLimitReason(int dof){
	return limitReason[dof];
}

void bt6DofElasticPlastic2Constraint::fillLimitReasons(char *buf){
	btElasticPlasticConstraint::fillLimitReasons(buf, limitReason);
}

btScalar bt6DofElasticPlastic2Constraint::getElasticEnergy(){
	return btElasticPlasticConstraint::getElasticEnergy(this);
}

btScalar bt6DofElasticPlastic2Constraint::getElasticEnergy(int dof){
	return btElasticPlasticConstraint::getElasticEnergy(this,dof);
}

btScalar bt6DofElasticPlastic2Constraint::getPlasticEnergy(){
	return btElasticPlasticConstraint::getPlasticEnergy(this);
}

btScalar bt6DofElasticPlastic2Constraint::getPlasticEnergy(int dof){
	return btElasticPlasticConstraint::getPlasticEnergy(this, dof);
}


btScalar bt6DofElasticPlastic2Constraint::getSpringStiffness(int dof){
	if (dof < 3){
		return getTranslationalLimitMotor()->m_springStiffness[dof];
	}
	else{
		return getRotationalLimitMotor(dof - 3)->m_springStiffness;
	}
}

btScalar bt6DofElasticPlastic2Constraint::getElasticDisplacement(int dof){
	if (!isEnabled()){
		return BT_ZERO;
	}
	btScalar currPos;
	btScalar elasticPart;
	btScalar k = getSpringStiffness(dof);
	if (dof<3)
	{
		if (!m_linearLimits.m_enableSpring[dof]){
			return BT_ZERO;
		}
		currPos = m_calculatedLinearDiff[dof];
		elasticPart = m_maxForce[dof] / k;
	}
	else{
		if (m_angularLimits[dof - 3].m_enableSpring){
			return BT_ZERO;
		}
		currPos = m_calculatedAxisAngleDiff[dof - 3];
		elasticPart = m_maxForce[dof] /k;
	}
	if (btFabs(elasticPart)<btFabs(currPos)){
		return elasticPart;
	}
	else{
		return currPos;
	}
}
btScalar bt6DofElasticPlastic2Constraint::getMaxForce(int dof){
	return m_maxForce[dof];
}
btScalar bt6DofElasticPlastic2Constraint::getPlasticDisplacement(int dof){
	return m_plasticDisplacement[dof];
}