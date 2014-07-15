/*
Simo Nikula 2014 based on bullet3 App_FractureDemo
Lots of details are described so that newcomers 
missing graphics 
or structural analysis background should be able to follow.

It also helps myself as this work has had many long pauses
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
bool firstRun=true;
btScalar startAngle(1);
btScalar timeStep(1./60);
btScalar btZero(0);

#include "btFractureBody.h"
#include "btFractureDynamicsWorld.h"





void	CharpyDemo::initPhysics()
{
	if(firstRun){
		setTexturing(true);
		setShadows(true);
		setDebugMode(btIDebugDraw::DBG_DrawText|btIDebugDraw::DBG_NoHelpText);
		// we look at quite small object
		setCameraDistance(btScalar(0.5));
		m_cameraPosition.setX(btScalar(0.3));
		m_frustumZNear=btScalar(0.01);
		m_frustumZFar=btScalar(10);
		firstRun=false;
	}
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
		btScalar l=0.055;
		btScalar halfLength=l/4;
		btCollisionShape* shape = 
			new btBoxShape(btVector3(0.005,0.005,halfLength));
		m_collisionShapes.push_back(shape);
		btVector3 localInertia(0,0,0);
		shape->calculateLocalInertia(sMass,localInertia);
		btAlignedObjectArray<btRigidBody*> ha;
		btAlignedObjectArray<btTransform> ta;
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
			ha.push_back(body);
			btTransform ctr;
			ctr.setIdentity();
			btVector3 cpos(0,0,(i==0?1:-1)*halfLength);
			ctr.setOrigin(cpos);
			ta.push_back(ctr);
		}
		btGeneric6DofSpringConstraint *sc=
						new btGeneric6DofSpringConstraint(*ha[0],*ha[1], 
			ta[0], ta[1],true);
		btScalar E(200e9);
		btScalar fu(400e6);
		btScalar b(0.01);
		btScalar h(0.008);
		btScalar I1(b*h*h*h/12);
		btScalar I2(h*b*b*b/12);
		btScalar k0(E*b*h/l/2);
		btScalar k1(48*E*I1/l/l/l);
		btScalar k2(48*E*I1/l/l/l);
		btScalar w1(fu*b*h*h/4);
		btScalar w2(fu*b*b*h/4);
		sc->setStiffness(0,k0);
		sc->setStiffness(1,k1);
		sc->setStiffness(2,k2);
		sc->setStiffness(3,w1); // not very exact
		sc->setStiffness(4,w2); 
		sc->setStiffness(5,w1); 
		m_dynamicsWorld->addConstraint(sc, true);
		for (int i=0;i<6;i++)
		{	
			sc->enableSpring(i,true);
		}
		for (int i=0;i<6;i++)
		{	
			sc->setDamping(i,btZero);
		}
		sc->setEquilibriumPoint();
	}
	// hammer with arm and hinge
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
		// create hammer at y=0
		compound->addChildShape(hTr,hammer);
		btCollisionShape* arm = 
			new btBoxShape(btVector3(0.02,0.5,0.02));
		btScalar aMass=0.04*1.0*0.04*7800;
		btTransform aTr;
		aTr.setIdentity();
		// arm above hammer
		btVector3 aPos(btZero,btScalar(0.5+0.125),btZero);
		aTr.setOrigin(aPos);
		compound->addChildShape(aTr,arm);
		btTransform cTr;
		// move compound down so that axis-position
		// corresponding to armPivot is at y=0
		// rotate and then move back up
		// up so that center of hammer is about y=0.2
		const btVector3 armPivot(btZero,
			btScalar(1),btZero);
		btVector3 cPos(btZero,btScalar(1.2),btZero);
		btTransform downTr;
		btTransform upTr;
		upTr.setIdentity();
		upTr.setOrigin(cPos);
		downTr.setIdentity();
		downTr.setOrigin(btScalar(-1)*armPivot);
		btQuaternion cRot;
		btVector3 axis(btZero,btZero,btScalar(1));
		cRot.setRotation(axis,btScalar(startAngle)); // pi means up
		btTransform axTr;
		axTr.setIdentity();
		axTr.setOrigin(cPos);
		// multiply transformations in reverse order
		cTr.setIdentity();
		cTr*=upTr;
		cTr*=btTransform(cRot);
		cTr*=downTr;
		btRigidBody *hBody=localCreateRigidBody(hMass+aMass,cTr,compound);
		btVector3 aDims(btScalar(0.01),btScalar(0.01),btScalar(0.05));
		btCollisionShape* axil = 
		new btCylinderShapeZ(aDims);
		m_collisionShapes.push_back(axil);
		btRigidBody *axilBody=localCreateRigidBody(btZero,axTr,axil);
		const btVector3 axilPivot(btZero,btZero,btZero);
		btVector3 pivotAxis(btZero,btZero,btScalar(1)); 
		btHingeConstraint *hammerHinge=
			new btHingeConstraint( *hBody,*axilBody, 
			armPivot, axilPivot, pivotAxis,pivotAxis, false );
		m_dynamicsWorld->addConstraint(hammerHinge, true);
	}

	fractureWorld->stepSimulation(timeStep,0);
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
		char buf[100];

		int lineWidth=580;
		int xStart = m_glutScreenWidth - lineWidth;
		int yStart = 20;

		btFractureDynamicsWorld* world = (btFractureDynamicsWorld*)m_dynamicsWorld;
		yStart+=20;
		GLDebugDrawString(xStart,yStart,"space to restart");
		yStart+=20;
		sprintf(buf,"+/- to change start angle, now=%1.1f",startAngle);
		GLDebugDrawString(xStart,yStart,buf);
		yStart+=20;
		sprintf(buf,"./: to change timeStep, now=%2.4f ms",timeStep*1000);
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
	switch (key)
	{
	case '+':
		{
			startAngle+=0.1;
			clientResetScene();
			break;
		}
	case '-':
		{
			startAngle-=0.1;
			clientResetScene();
			break;
		}
	case '.':
		{
			timeStep/=0.8;
			clientResetScene();
			break;
		}
	case ':':
		{
			timeStep*=0.8;
			clientResetScene();
			break;
		}
	default:
		PlatformDemoApplication::keyboardUpCallback(key,x,y);
		break;
	}
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




