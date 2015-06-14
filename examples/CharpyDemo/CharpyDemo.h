/*
Simo Nikula 2014 based on bullet3 Demos
*/
#ifndef CHARPY_DEMO_H
#define CHARPY_DEMO_H

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "LinearMath/btAlignedObjectArray.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///CharpyDemo shows basic breaking and glueing of objects
class CharpyDemo : public CommonRigidBodyBase
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
	int m_viewMode;

	void showMessage();

	public:
	CharpyDemo(struct GUIHelperInterface* helper)
			:CommonRigidBodyBase(helper)
	{
	}
	virtual ~CharpyDemo()
	{
		exitPhysics();
	}
	void	initPhysics();

	void	exitPhysics();

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();
		
	virtual void keyboardCallback(unsigned char key, int x, int y);
	virtual void specialKeyboard(int key, int x, int y);
	virtual void setViewMode(int viewMode);
	virtual void updateView();

	virtual void clientResetScene();

	void	shootBox(const btVector3& destination);
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform,
		btCollisionShape* shape);
};
class CommonExampleInterface*    CharpyDemoCreateFunc(CommonExampleOptions& options);
#endif //CHARPY_DEMO_H

