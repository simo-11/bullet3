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

	setCameraDistance(btScalar(0.3));
	m_frustumZNear=btScalar(0.1);

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

	{
		///create a few basic rigid bodies
		btCollisionShape* groundShape = new btBoxShape(btVector3(5,0.1,5));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-0.1,0));
		localCreateRigidBody(0.f,groundTransform,groundShape);
	}

	{
		///create chary specimen using two halfs
		btScalar sMass=0.01*0.01*0.0275*7800;
		btCollisionShape* shape = new btBoxShape(btVector3(0.005,0.005,0.01375));
		m_collisionShapes.push_back(shape);
		btVector3 localInertia(0,0,0);
		shape->calculateLocalInertia(sMass,localInertia);
		for (int i=0;i<2;i++)
		{	
			btTransform tr;
			tr.setIdentity();
			btVector3 pos(0,0.2,i*0.0275);
			tr.setOrigin(pos);
			btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
			btRigidBody::btRigidBodyConstructionInfo rbInfo(sMass,myMotionState,shape,localInertia);
			btFractureBody* body = new btFractureBody(rbInfo, m_dynamicsWorld);
			m_dynamicsWorld->addRigidBody(body);
		}
	}

	fractureWorld->stepSimulation(1./60.,0);
	fractureWorld->glueCallback();

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
		char buf[124];

		int lineWidth=380;
		int xStart = m_glutScreenWidth - lineWidth;
		int yStart = 20;

		btFractureDynamicsWorld* world = (btFractureDynamicsWorld*)m_dynamicsWorld;
		if (world->getFractureMode())
		{
			sprintf(buf,"Fracture mode");
		} else
		{
			sprintf(buf,"Glue mode");
		}
		GLDebugDrawString(xStart,yStart,buf);
		sprintf(buf,"f to toggle fracture/glue mode");		
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);
		sprintf(buf,"space to restart, mouse to pick/shoot");
		yStart+=20;
		GLDebugDrawString(xStart,yStart,buf);

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
	if (key=='f')
	{
		btFractureDynamicsWorld* world = (btFractureDynamicsWorld*)m_dynamicsWorld;
		world->setFractureMode(!world->getFractureMode());
	}

	PlatformDemoApplication::keyboardUpCallback(key,x,y);

}


void	CharpyDemo::shootBox(const btVector3& destination)
{

	if (m_dynamicsWorld)
	{
		btScalar mass = .01f;
		btTransform startTransform;
		startTransform.setIdentity();
		btVector3 camPos = getCameraPosition();
		startTransform.setOrigin(camPos);

		setShootBoxShape ();

		btAssert((!m_shootBoxShape || m_shootBoxShape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			m_shootBoxShape->calculateLocalInertia(mass,localInertia);

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

		btFractureBody* body = new btFractureBody(mass,0,m_shootBoxShape,localInertia,&mass,1,m_dynamicsWorld);

		body->setWorldTransform(startTransform);

		m_dynamicsWorld->addRigidBody(body);


		body->setLinearFactor(btVector3(1,1,1));
		//body->setRestitution(1);

		btVector3 linVel(destination[0]-camPos[0],destination[1]-camPos[1],destination[2]-camPos[2]);
		linVel.normalize();
		linVel*=m_ShootBoxInitialSpeed;

		body->getWorldTransform().setOrigin(camPos);
		body->getWorldTransform().setRotation(btQuaternion(0,0,0,1));
		body->setLinearVelocity(linVel);
		body->setAngularVelocity(btVector3(0,0,0));
		body->setCcdMotionThreshold(1.);
		body->setCcdSweptSphereRadius(0.2f);

	}
}






void	CharpyDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
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




