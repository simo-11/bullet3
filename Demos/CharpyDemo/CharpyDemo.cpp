/*
Simo Nikula 2014 based on bullet3 App_FractureDemo
*/


///CharpyDemo shows how to break objects.
///It assumes a btCompoundShaps (where the childshapes are the pre-fractured pieces)
///The btFractureBody is a class derived from btRigidBody, dealing with the collision impacts.

#include "CharpyDemo.h"
#include "GlutStuff.h"
#include "GLDebugFont.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"


#include <stdio.h> //printf debugging


int sFrameNumber = 0;

#include "btFractureBody.h"
#include "btFractureDynamicsWorld.h"





void	CharpyDemo::initPhysics()
{

	setTexturing(true);
	setShadows(true);

	setDebugMode(btIDebugDraw::DBG_DrawText|btIDebugDraw::DBG_NoHelpText);
	// we look at quite small object
	setCameraDistance(btScalar(0.5));
	m_frustumZNear=btScalar(0.01);
	m_frustumZFar=btScalar(10);

	///collision configuration contains default setup for memory, collision setup
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	//m_collisionConfiguration->setConvexConvexMultipointIterations();

	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);

	m_broadphase = new btDbvtBroadphase();

	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* sol = new btSequentialImpulseConstraintSolver;
	m_solver = sol;

	//m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);

	btFractureDynamicsWorld* fractureWorld = new btFractureDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = fractureWorld;
	//m_splitImpulse removes the penetration resolution from the applied impulse, otherwise objects might fracture due to deep penetrations.
	m_dynamicsWorld->getSolverInfo().m_splitImpulse = true;

	// floor
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(5,0.1,5));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-0.1,0));
		localCreateRigidBody(0.f,groundTransform,groundShape);
	}


	// support anvils leaving 40 mm open space between them
	{
		btCollisionShape* shape = 
			new btBoxShape(btVector3(0.05,0.1,0.02));
		m_collisionShapes.push_back(shape);
		// symmetrically around z=0 and x=0
		// top at y=0.2
		for (int i=0;i<2;i++)
		{	
			btTransform tr;
			tr.setIdentity();
			btVector3 pos(0,btScalar(0.1),btScalar((i==0?-1:1)*0.04));
			tr.setOrigin(pos);
			localCreateRigidBody(0.f,tr,shape);
		}
	}
	{
		btCollisionShape* shape = 
			new btBoxShape(btVector3(0.025,0.02,0.02));
		m_collisionShapes.push_back(shape);
		// symmetrically around z=0
		// bottom at y=0.2
		// frontsize at x=0
		for (int i=0;i<2;i++)
		{	
			btTransform tr;
			tr.setIdentity();
			btVector3 pos(btScalar(-0.025),
				btScalar(0.22),
				btScalar((i==0?-1:1)*0.04));
			tr.setOrigin(pos);
			localCreateRigidBody(0.f,tr,shape);
		}
	}

	// charpy specimen using two halfs, 
	// symmetrically around z=0
	// bottom at y=0.2
	// backside at x=0
	{
		btScalar sMass=0.01*0.01*0.0275*7800;
		btScalar halfLength=0.01375;
		btCollisionShape* shape = 
			new btBoxShape(btVector3(0.005,0.005,halfLength));
		m_collisionShapes.push_back(shape);
		btVector3 localInertia(0,0,0);
		shape->calculateLocalInertia(sMass,localInertia);
		for (int i=0;i<2;i++)
		{	
			btTransform tr;
			tr.setIdentity();
			btVector3 pos(0.005,0.205,(i==0?-1:1)*halfLength);
			tr.setOrigin(pos);
			btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(sMass,myMotionState,shape,localInertia);
			btFractureBody* body = new btFractureBody(rbInfo, m_dynamicsWorld);
			m_dynamicsWorld->addRigidBody(body);
		}
	}
	// hammer with arm
	// hammer should be able to provide impact of about 500 J
	// m*g*h=500
	// m~500/2/10~25 kg
	{
		btCompoundShape* compound = new btCompoundShape();
		btCollisionShape* hammer = 
			new btBoxShape(btVector3(0.25,0.125,0.01));
		btScalar hMass=0.5*0.25*0.02*7800;
		btTransform hTr;
		hTr.setIdentity();
		btVector3 hPos(btScalar(0),btScalar(0),btScalar(0));
		hTr.setOrigin(hPos);
		compound->addChildShape(hTr,hammer);
		btCollisionShape* arm = 
			new btBoxShape(btVector3(0.02,0.4,0.02));
		btScalar aMass=0.04*0.8*0.04*7800;
		btTransform aTr;
		aTr.setIdentity();
		btVector3 aPos(btScalar(0),btScalar(0.4+0.125),btScalar(0));
		aTr.setOrigin(aPos);
		compound->addChildShape(aTr,arm);
		btTransform cTr;
		cTr.setIdentity();
		btVector3 cPos(btScalar(0),btScalar(2),btScalar(0));
		cTr.setOrigin(cPos);
		btQuaternion cRot;
		btVector3 axis(btScalar(0),btScalar(0),btScalar(1));
		cRot.setRotation(axis,btScalar(3));
		cTr.setRotation(cRot);
		btRigidBody *hBody=localCreateRigidBody(hMass+aMass,cTr,compound);
		const btVector3 pivot(btScalar(0),btScalar(0),btScalar(0));
		btVector3 pivotAxis(btScalar(0),btScalar(0),btScalar(1)); 
		btHingeConstraint *hammerHinge=
			new btHingeConstraint( *hBody, pivot, pivotAxis );
		m_dynamicsWorld->addConstraint(hammerHinge, true);
	}

	fractureWorld->stepSimulation(1./60.,0);
}

void	CharpyDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}


void CharpyDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	///step the simulation
	if (m_dynamicsWorld)
	{
		m_dynamicsWorld->stepSimulation(ms / 1000000.f);
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();
	}



	renderme(); 

	showMessage();

	glFlush();

	swapBuffers();

}

void CharpyDemo::showMessage()
{
	if((getDebugMode() & btIDebugDraw::DBG_DrawText))
	{
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);

		int lineWidth=380;
		int xStart = m_glutScreenWidth - lineWidth;
		int yStart = 20;

		btFractureDynamicsWorld* world = (btFractureDynamicsWorld*)m_dynamicsWorld;
		yStart+=20;
		GLDebugDrawString(xStart,yStart,"space to restart");
		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}

}


void CharpyDemo::displayCallback(void) {

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	showMessage();

	//optional but useful: debug drawing to detect problems
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	swapBuffers();
}


void CharpyDemo::keyboardUpCallback(unsigned char key, int x, int y)
{
	PlatformDemoApplication::keyboardUpCallback(key,x,y);
}

// no-op
void	CharpyDemo::shootBox(const btVector3& destination)
{
}






void	CharpyDemo::exitPhysics()
{
	int i;

	//cleanup in the reverse order of creation/initialization
	for (i=m_dynamicsWorld->getNumConstraints()-1; i>=0 ;i--)
	{
		btTypedConstraint* constraint = m_dynamicsWorld->getConstraint(i);
		m_dynamicsWorld->removeConstraint(constraint);
		delete constraint;
	}
	//remove the rigidbodies from the dynamics world and delete them
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	m_collisionShapes.clear();

	delete m_dynamicsWorld;
	m_dynamicsWorld=0;

	delete m_solver;
	m_solver=0;

	delete m_broadphase;
	m_broadphase=0;

	delete m_dispatcher;
	m_dispatcher=0;

	delete m_collisionConfiguration;
	m_collisionConfiguration=0;

}




