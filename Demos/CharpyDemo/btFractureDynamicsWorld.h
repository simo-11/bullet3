#ifndef _BT_FRACTURE_DYNAMICS_WORLD_H
#define _BT_FRACTURE_DYNAMICS_WORLD_H

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "LinearMath/btAlignedObjectArray.h"

class btFractureBody;
class btCompoundShape;
class btTransform;


///If/once this implementation is stablized/tested we might merge it into btDiscreteDynamicsWorld and remove the class.
class btFractureDynamicsWorld : public btDiscreteDynamicsWorld
{
	btAlignedObjectArray<btFractureBody*> m_fractureBodies;

	btFractureBody* addNewBody(const btTransform& oldTransform,btScalar* masses, btCompoundShape* oldCompound);

	void	breakDisconnectedParts( btFractureBody* fracObj);

public:

	btFractureDynamicsWorld ( btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration);

	virtual void	addRigidBody(btRigidBody* body);

	virtual void	removeRigidBody(btRigidBody* body);

	void	solveConstraints(btContactSolverInfo& solverInfo);
	///normally those callbacks are called internally by the 'solveConstraints'
	void fractureCallback();

};

#endif //_BT_FRACTURE_DYNAMICS_WORLD_H

