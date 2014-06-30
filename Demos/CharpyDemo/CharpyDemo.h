/*
Simo Nikula 2014 based on bullet3 App_FractureDemo
*/
#ifndef FRACTURE_DEMO_H
#define FRACTURE_DEMO_H

#ifdef _WINDOWS
#include "Win32DemoApplication.h"
#define PlatformDemoApplication Win32DemoApplication
#else
#include "GlutDemoApplication.h"
#define PlatformDemoApplication GlutDemoApplication
#endif

#include "LinearMath/btAlignedObjectArray.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

///CharpyDemo shows basic breaking and glueing of objects
class CharpyDemo : public PlatformDemoApplication
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	void showMessage();

	public:

	CharpyDemo()
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
		
	virtual void keyboardUpCallback(unsigned char key, int x, int y);

	virtual void	clientResetScene();

	static DemoApplication* Create()
	{
		CharpyDemo* demo = new CharpyDemo;
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	void	shootBox(const btVector3& destination);
	
};

#endif //FRACTURE_DEMO_H

