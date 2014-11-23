/*
Simo Nikula 2014 based on bullet3 Demos.

For video and other links see  http://youtu.be/3xU2uohZWCk

Lots of details are described so that newcomers 
missing graphics 
or structural analysis background should be able to follow.

It also helps myself as this work has had many long pauses
*/


/**
CharpyDemo shows how to handle Charpy test.
target is to break objects using plasticity.
*/
#include "CharpyDemo.h"
#include "GlutStuff.h"
#include "GLDebugDrawer.h"
#include "GLDebugFont.h"
///btBulletDynamicsCommon.h is the main Bullet include file, contains most common include files.
#include "btBulletDynamicsCommon.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

#include <stdio.h> 
#include <time.h>

GLDebugDrawer	gDebugDrawer;
int sFrameNumber = 0;
bool firstRun=true;
btScalar startAngle(0.3);
long displayWait=10;
btScalar ccdMotionThreshHold(0.001);
btScalar margin(0.001);
btScalar floorHE(0.1);
btScalar defaultTimeStep(1./60);
btScalar timeStep(defaultTimeStep);
btScalar simulationTimeStep(timeStep);
bool variableTimeStep = false;
btScalar currentTime;
int mode=5;
const char *modes[] =
{
"None",
"single object",
"two objects and springConstraints",
"two objects and constraints with zero limits",
"two objects and spring2Constraints",
"two objects and hingeConstraint",
};
const char *viewModes[] =
{
	"None",
	"Look at anvil",
	"Follow specimen",
};
btScalar btZero(0);
btScalar btOne(1);
btScalar l(0.055);
btScalar w(0.01);
btScalar E(200E9);
btScalar fu(400e6);
btScalar damping(0.2);
float energy = 0;
float maxEnergy;
btDynamicsWorld* dw;
btVector3 y_up(0.01, 1., 0.01);
btRigidBody *specimenBody,*hammerBody;
btHingeConstraint *hammerHinge;
btJointFeedback hammerHingeJointFeedback;
btJointFeedback specimenJointFeedback;
btHingeConstraint *mode5Hinge;
btScalar w1;
float maxForces[6];
FILE *fp;
boolean openGraphFile = false;

/*
Writes sgraph value
*/
void wgv(float f){
	fprintf(fp, "%6.2f;", f);
}
/**
Writes graph data if file can be opened
*/
void writeGraphData(){
	if (!openGraphFile){
		return;
	}
	if (!fp){
		char buf[100];
		sprintf(buf, "d:/wrk/charpyGraph-%ld.txt", time(NULL));
		fp = fopen(buf, "w");
		if (!fp){
			openGraphFile = false;
			return;
		}
	}
	wgv(currentTime);
	for (int i = 0; i < 3; i++){
		wgv(specimenJointFeedback.m_appliedForceBodyA[i]);
	}
	for (int i = 0; i < 3; i++){
		wgv(specimenJointFeedback.m_appliedTorqueBodyA[i]);
	}
	fprintf(fp, "\n");
}

btScalar getBodySpeed2(const btCollisionObject* o){
	return o->getInterpolationLinearVelocity().length2();
}

btScalar getManifoldSpeed2(btPersistentManifold* manifold){
	btScalar s0=getBodySpeed2(manifold->getBody0());
	btScalar s1=getBodySpeed2(manifold->getBody0());
	return s0>s1?s0:s1;
}


void addSpringConstraints(btAlignedObjectArray<btRigidBody*> ha,
		btAlignedObjectArray<btTransform> ta){
	btGeneric6DofSpringConstraint *sc=
				new btGeneric6DofSpringConstraint(*ha[0],*ha[1], 
	ta[0], ta[1],true);
	sc->setJointFeedback(&specimenJointFeedback);
	btScalar b(w);
	btScalar h(w-0.002); // notch is 2 mm
	btScalar I1(b*h*h*h/12);
	btScalar I2(h*b*b*b/12);
	btScalar k0(E*b*h/l/2);
	btScalar k1(48*E*I1/l/l/l);
	btScalar k2(48*E*I1/l/l/l);
	w1=fu*b*h*h/4;
	btScalar w2(fu*b*b*h/4);
	sc->setStiffness(0,k0);
	sc->setStiffness(1,k1);
	sc->setStiffness(2,k2);
	sc->setStiffness(3,w1); // not very exact
	sc->setStiffness(4,w2); 
	sc->setStiffness(5,w1); 
	dw->addConstraint(sc, true);
	for (int i=0;i<6;i++)
	{	
		sc->enableSpring(i,true);
	}
	for (int i=0;i<6;i++)
	{	
		sc->setDamping(i,damping); 
	}
	sc->setEquilibriumPoint();	
}

void addSpring2Constraints(btAlignedObjectArray<btRigidBody*> ha,
	btAlignedObjectArray<btTransform> ta){
	btGeneric6DofSpring2Constraint *sc =
		new btGeneric6DofSpring2Constraint(*ha[0], *ha[1],
		ta[0], ta[1]);
	sc->setJointFeedback(&specimenJointFeedback);
	btScalar b(w);
	btScalar h(w - 0.002); // notch is 2 mm
	btScalar I1(b*h*h*h / 12);
	btScalar I2(h*b*b*b / 12);
	btScalar k0(E*b*h / l / 2);
	btScalar k1(48 * E*I1 / l / l / l);
	btScalar k2(48 * E*I1 / l / l / l);
	w1=fu*b*h*h / 4;
	btScalar w2(fu*b*b*h / 4);
	sc->setStiffness(0, k0);
	sc->setStiffness(1, k1);
	sc->setStiffness(2, k2);
	sc->setStiffness(3, w1); // not very exact
	sc->setStiffness(4, w2);
	sc->setStiffness(5, w1);
	dw->addConstraint(sc, true);
	for (int i = 0; i<6; i++)
	{
		sc->enableSpring(i, true);
	}
	for (int i = 0; i<6; i++)
	{
		sc->setDamping(i, damping);
	}
	sc->setEquilibriumPoint();
}

void addFixedConstraints(btAlignedObjectArray<btRigidBody*> ha,
		btAlignedObjectArray<btTransform> ta){
	btGeneric6DofConstraint *sc=
				new btGeneric6DofConstraint(*ha[0],*ha[1], 
	ta[0], ta[1],true);
	sc->setJointFeedback(&specimenJointFeedback);
	dw->addConstraint(sc, true);
	for (int i=0;i<6;i++){	
		sc->setLimit(i,0,0); // make fixed
	}
}

void addHingeConstraint(btAlignedObjectArray<btRigidBody*> ha){
	btVector3 pivotAxis(btZero, btScalar(1), btZero);
	btVector3 pivotInA(btZero, btZero, l/4);
	btVector3 pivotInB(btZero, btZero, -l/4);
	btHingeConstraint *sc =
		new btHingeConstraint(*ha[0], *ha[1],
		pivotInA,pivotInB ,pivotAxis,pivotAxis, false);
	mode5Hinge = sc;
	btScalar b(w);
	btScalar h(w - 0.002); // notch is 2 mm
	w1=fu*b*h*h/4;
	sc->setJointFeedback(&specimenJointFeedback);
	sc->setLimit(-SIMD_PI, SIMD_PI); // until parts are overturn
	sc->enableAngularMotor(true, 0, w1*timeStep);
	dw->addConstraint(sc, true);
}

/**
http://www.bulletphysics.org/mediawiki-1.5.8/index.php?title=Anti_tunneling_by_Motion_Clamping
*/
void resetCcdMotionThreshHold()
{
	btScalar radius(ccdMotionThreshHold/5.f);
	for (int i=dw->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = dw->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			body->setCcdMotionThreshold(ccdMotionThreshHold);
			body->setCcdSweptSphereRadius(radius);
		}
	}
}
float getRotationEnergy(btScalar invInertia, btScalar angularVelocity){
	if (invInertia == 0){
		return 0.f;
	}
	float inertia = 1.f / invInertia;
	float e=0.5*inertia*angularVelocity*angularVelocity;
	return e;
}

void updateEnergy(){
	energy = 0;
	btVector3 gravity = dw->getGravity();
	const btCollisionObjectArray objects = dw->getCollisionObjectArray();
	for (int i = 0; i<objects.capacity(); i++)
	{
		const btCollisionObject* o = objects[i];
		const btRigidBody* ro = btRigidBody::upcast(o);
		btScalar iM = ro->getInvMass();
		if (iM == 0){
			continue;
		}
		btVector3 v = ro->getLinearVelocity();
		btScalar m = 1 / iM;
		float linearEnergy = 0.5*m*v.length2();
		btVector3 com = ro->getCenterOfMassPosition();
		float gravitationalEnergy = -m*gravity.dot(com);
		const btVector3 rv = ro->getAngularVelocity();
		const btVector3 iI = ro->getInvInertiaDiagLocal();
		float inertiaEnergy = 0;
		inertiaEnergy += getRotationEnergy(iI.getX(), rv.getX());
		inertiaEnergy += getRotationEnergy(iI.getY(), rv.getY());
		inertiaEnergy += getRotationEnergy(iI.getZ(), rv.getZ());
		energy += linearEnergy;
		energy += inertiaEnergy;
		energy += gravitationalEnergy;
	}
	if (energy > maxEnergy){
		maxEnergy = energy;
	}
}

/*
X axis is horizontal, positive to direction where hammer comes from (left)
Y axis is vertical, positive up
Z axis is horizontal and Z=0 is symmetry plane
*/
void	CharpyDemo::initPhysics()
{
	if(firstRun){
		setTexturing(true);
		setShadows(true);
		setDebugMode(btIDebugDraw::DBG_DrawText|btIDebugDraw::DBG_NoHelpText);
		setViewMode(1);
		firstRun=false;
	}
	energy = 0;
	maxEnergy = energy;
	printf("startAngle=%f timeStep=%f\n",startAngle,timeStep);
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

	btDiscreteDynamicsWorld* btWorld =
		new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld = btWorld;
	dw=m_dynamicsWorld;
	// dw->getSolverInfo().m_erp2 = 0.95; // default is 0.8
	// dw->getSolverInfo().m_erp = 0.6; // default is 0.2
	// dw->getSolverInfo().m_splitImpulse = true; // default is true
	// dw->getSolverInfo().m_splitImpulsePenetrationThreshold = -0.002; // default is -0.04
	// dw->getSolverInfo().m_numIterations = 100;
	// Tuning of values above did not help
	// floor
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(5,floorHE,5));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-floorHE,0));
		localCreateRigidBody(0.f,groundTransform,groundShape);
	}


	// support anvils leaving 40 mm open space between them
	{ // lower part
		btBoxShape* shape = 
			new btBoxShape(btVector3(0.05f+w,0.1f,0.02f));
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
	{ // back support
		btBoxShape* shape = 
			new btBoxShape(btVector3(0.025+w/2,0.02+w,0.02f));
		m_collisionShapes.push_back(shape);
		// symmetrically around z=0
		// bottom at y=0.2
		// frontsize at x=0
		for (int i=0;i<2;i++)
		{	
			btTransform tr;
			tr.setIdentity();
			btVector3 pos(btScalar(-0.025-w/2),
				btScalar(0.22+w),
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
		btScalar halfLength=l/4;
		if(mode==1){
			halfLength=l/2;
		}
		btScalar sMass(w*w*halfLength*2.0f*7800.0f);
		btAlignedObjectArray<btRigidBody*> ha;
		btAlignedObjectArray<btTransform> ta;
		btBoxShape* shape = 
			new btBoxShape(btVector3(w/2,w/2,halfLength));
		m_collisionShapes.push_back(shape);
		btVector3 localInertia(0,0,0);
		shape->calculateLocalInertia(sMass,localInertia);
		// Only one object in mode 1
		for (int i=0;(mode>1?i<2:i<1);i++)
		{	
			btTransform tr;
			tr.setIdentity();
			btVector3 pos(w/2,0.2+w/2,
				(mode>1?(i==0?-1:1)*halfLength:0.f));
			tr.setOrigin(pos);
			btDefaultMotionState* myMotionState 
				= new btDefaultMotionState(tr);
			btRigidBody::btRigidBodyConstructionInfo 
				rbInfo(sMass,myMotionState,shape,localInertia);
			btRigidBody* body =
				new btRigidBody(rbInfo);
			m_dynamicsWorld->addRigidBody(body);
			ha.push_back(body);
			btTransform ctr;
			ctr.setIdentity();
			btVector3 cpos(0,0,
				(mode>1?(i==0?1:-1)*halfLength:0));
			ctr.setOrigin(cpos);
			ta.push_back(ctr);
		}
		specimenBody = ha[0];
		switch(mode) {
		case 2:
			addSpringConstraints(ha,ta);
			break;
		case 3:
			addFixedConstraints(ha,ta);
			break;
		case 4:
			addSpring2Constraints(ha, ta);
		case 5:
			addHingeConstraint(ha);
			break;
		}
	}
	// hammer with arm and hinge
	// hammer is 0.5 m wide and 0.25 m high, thickness is 0.02
	// hammer and hinge are positioned so that impact is horizontal 
	// (global x-direction)
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
		// rotate and then move back up and in
		// x-direction so that at impact time hammer is in down position
		// up so that center of hammer is about y=0.2
		const btVector3 armPivot(btZero,
			btScalar(1),btZero);
		btVector3 cPos(btZero,btScalar(1.2),btZero);
		btVector3 lPos(btScalar(0.25 + w), btZero, btZero);
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
		axTr.setOrigin(cPos+lPos);
		//
		btTransform leftTr;
		leftTr.setIdentity();
		leftTr.setOrigin(lPos);
		// multiply transformations in reverse order
		cTr.setIdentity();
		cTr*=leftTr;
		cTr*=upTr;
		cTr*=btTransform(cRot);
		cTr*=downTr;
		btRigidBody *hBody=localCreateRigidBody(hMass+aMass,cTr,compound);
		hammerBody = hBody;
		btVector3 aDims(btScalar(0.01),btScalar(0.01),btScalar(0.05));
		btCollisionShape* axil = 
		new btCylinderShapeZ(aDims);
		m_collisionShapes.push_back(axil);
		btRigidBody *axilBody=localCreateRigidBody(btZero,axTr,axil);
		const btVector3 axilPivot(btZero,btZero,btZero);
		btVector3 pivotAxis(btZero,btZero,btScalar(1)); 
		hammerHinge=
			new btHingeConstraint( *hBody,*axilBody, 
			armPivot, axilPivot, pivotAxis,pivotAxis, false );
		hammerHinge->setJointFeedback(&hammerHingeJointFeedback);
		m_dynamicsWorld->addConstraint(hammerHinge, true);
	}
	resetCcdMotionThreshHold();
	updateEnergy();
	btWorld->stepSimulation(timeStep,0);
	currentTime=timeStep;
	dw->setDebugDrawer(&gDebugDrawer);
}


void	CharpyDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

btScalar minCurrentCollisionDistance(0);
btScalar minCollisionDistance(0);
btScalar maxImpact(0);
btScalar maxSpeed2(0);

void updateMaxForces(int baseIndex, const btVector3 &v){
	for (int i = 0; i < 3; i++){
		int j = baseIndex + i;
		float absValue = btFabs(v.m_floats[i]);
		if (absValue>maxForces[j]){
			maxForces[j] = absValue;
		}
	}
}

void addForces(char *buf, const btVector3 &v){
	sprintf(buf, "Constraint forces:X/Y/Z %9.4f/%9.4f/%9.4f N",
		v.m_floats[0], v.m_floats[1], v.m_floats[2]);
	updateMaxForces(0,v);
}

void addMoments(char *buf, const btVector3 &v){
	sprintf(buf, "Constraint moments:X/Y/Z %9.4f/%9.4f/%9.4f Nm",
		v.m_floats[0], v.m_floats[1], v.m_floats[2]);
	updateMaxForces(3, v);
}

void checkConstraints(){
}

void checkCollisions(){
	btScalar maxCurrentSpeed2 = btScalar(0);
	minCurrentCollisionDistance = btScalar(0);
	int numManifolds = dw->getDispatcher()->getNumManifolds();
	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* contactManifold =  dw->getDispatcher()->getManifoldByIndexInternal(i);
		const btCollisionObject* obA = contactManifold->getBody0();
		const btCollisionObject* obB = contactManifold->getBody1();

		int numContacts = contactManifold->getNumContacts();
		btScalar totalImpact(0);
		for (int j=0;j<numContacts;j++)
		{
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			totalImpact += pt.m_appliedImpulse;
			btScalar distance(pt.getDistance());
			if(distance<minCollisionDistance){
				minCollisionDistance=distance;
			}
			if (distance<minCurrentCollisionDistance){
				minCurrentCollisionDistance = distance;
			}
		}
		if (totalImpact>maxImpact){
			maxImpact = totalImpact;
		}
		float maxManifoldSpeed2=getManifoldSpeed2(contactManifold);
		if (maxManifoldSpeed2>maxSpeed2){
			maxSpeed2 = maxManifoldSpeed2;
		}
		if (variableTimeStep){
			if (maxManifoldSpeed2 > maxCurrentSpeed2){
				maxCurrentSpeed2 = maxManifoldSpeed2;
			}
		}
	}
	if (variableTimeStep){
		/**
		* tune timeStep so that it 1/10 of specimen width
		is done in one step
		*/
		if (maxCurrentSpeed2>0){
			btScalar speed = btSqrt(maxCurrentSpeed2);
			timeStep = w / 10 / speed;
		}else{
			/**
			* no manifolds -> increase timeStep
			*/
			if (timeStep < defaultTimeStep/1.2){
				timeStep *= 1.2;
			}
		}
	}
}

void tuneMode5(){
	mode5Hinge->setMaxMotorImpulse(w1*timeStep);
}

void beforeStepSimulation(){
	if (mode == 5){
		tuneMode5();
	}
}


void CharpyDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	///step the simulation
	if (m_dynamicsWorld)	{
		if(timeStep!=defaultTimeStep){
			simulationTimeStep=timeStep;
			beforeStepSimulation();
			m_dynamicsWorld->stepSimulation(simulationTimeStep, 30, simulationTimeStep);
		} else{
			float ms = getDeltaTimeMicroseconds();
			simulationTimeStep=btScalar(ms/1e6);
			beforeStepSimulation();
			m_dynamicsWorld->stepSimulation(simulationTimeStep, 10);
		}
		checkCollisions();
		updateEnergy();
		checkConstraints();
		m_dynamicsWorld->debugDrawWorld();
	}
	currentTime += simulationTimeStep;
	updateView();
	renderme();
	showMessage();
	glFlush();
	swapBuffers();
	Sleep(displayWait);
}

int yStart;
int xStart;
btVector3 infoColor(1,1,1);
void infoMsg(const char * buf){
	GLDebugDrawStringInternal(xStart, yStart, buf, infoColor);
	yStart += 20;
}
void CharpyDemo::showMessage()
{
	if((getDebugMode() & btIDebugDraw::DBG_DrawText))
	{	
		writeGraphData();
		setOrthographicProjection();
		glDisable(GL_LIGHTING);
		glColor3f(0, 0, 0);
		char buf[100];
		int lineWidth = 620;
		xStart = m_glutScreenWidth - lineWidth;
		yStart = 20;

		sprintf(buf, "energy:max/current/loss %9.3g/%9.3g/%9.3g J", 
			maxEnergy,energy,maxEnergy-energy);
		infoMsg(buf);
		addForces(buf, specimenJointFeedback.m_appliedForceBodyA);
		infoMsg(buf);
		addMoments(buf, specimenJointFeedback.m_appliedTorqueBodyA);
		infoMsg(buf);
		sprintf(buf, "minCollisionDistance: simulation/step %1.3f/%1.3f",
			minCollisionDistance, minCurrentCollisionDistance);
		infoMsg(buf);
		sprintf(buf, "{/} to change displayWait, now=%3ld ms", displayWait);
		infoMsg(buf);
		sprintf(buf,"+/- to change start angle, now=%1.1f",startAngle);
		infoMsg(buf);
		if (mode == 2 || mode == 4){
			sprintf(buf, "(/) to change damping, now=%1.1f", damping);
			infoMsg(buf);
		}
		sprintf(buf, "./:/,/; to change timeStep, now=%2.2f/%2.2f ms, auto(;)=%s",
		timeStep*1000,simulationTimeStep*1000,(variableTimeStep?"on":"off"));
		infoMsg(buf);
		sprintf(buf,"k/K to change specimen width, now=%1.6f m",w);
		infoMsg(buf);
		sprintf(buf,"v/V to change specimen length, now=%1.6f m",l);
		infoMsg(buf);
		sprintf(buf,"mode(F1-F5)=F%d: %s",mode,modes[mode]);
		infoMsg(buf);
		sprintf(buf, "viewMode(<Shift>F1-F2)=F%d: %s", m_viewMode, viewModes[m_viewMode]);
		infoMsg(buf);
		if (false){ // these do not currently seem interesting
			sprintf(buf, "</> to change ccdMotionThreshHold, now=%1.8f m",
				ccdMotionThreshHold);
			infoMsg(buf);
			sprintf(buf, "e/E to change margin, now=%1.8f m",margin);
			infoMsg(buf);
			sprintf(buf, "j/J to change floor half extents, now=%1.6f m",
				floorHE);
			infoMsg(buf);
		}
		sprintf(buf, "space to restart, currentTime=%3.4f s", currentTime);
		infoMsg(buf);
		resetPerspectiveProjection();
		glEnable(GL_LIGHTING);
	}

}

void CharpyDemo::updateView(){
	switch (m_viewMode){
	case 1:
		break;
	case 2:
		m_cameraPosition=hammerBody->getCenterOfMassPosition();
		m_cameraTargetPosition = specimenBody->getCenterOfMassPosition();
		break;
	}
}

void CharpyDemo::displayCallback(void) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
	updateView();
	renderme();
	showMessage();
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
	glFlush();
	swapBuffers();
	Sleep(100); // save energy
}


void resetCollisionMargin()
{
	for (int i=dw->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = dw->getCollisionObjectArray()[i];
		if (obj)
		{
			obj->getCollisionShape()->setMargin(margin);
		}
	}
}

void CharpyDemo::setViewMode(int viewMode){
	m_viewMode = viewMode;
	switch (viewMode){
	case 1:
		setCameraDistance(btScalar(0.5));
		m_cameraPosition.setX(btScalar(0.3));
		m_cameraTargetPosition = btVector3(0, 0.2, 0);
		m_cameraUp = y_up;
		m_frustumZNear = btScalar(0.001);
		m_frustumZFar = btScalar(10);
		break;
	}
}

void CharpyDemo::specialKeyboard(int key, int x, int y){
	updateModifierKeys();

	switch (key){
	case GLUT_KEY_F1:
		if (m_modifierKeys& BT_ACTIVE_SHIFT){
			setViewMode(1);
		}else{
		  mode = 1;
		  clientResetScene();
		}
		break;
	case GLUT_KEY_F2:
		if (m_modifierKeys& BT_ACTIVE_SHIFT){
			setViewMode(2);
		}
		else{
			mode = 2;
			clientResetScene();
		}
		break;
	case GLUT_KEY_F3:
		mode = 3;
		clientResetScene();
		break;
	case GLUT_KEY_F4:
		mode = 4;
		clientResetScene();
		break;
	case GLUT_KEY_F5:
		mode = 5;
		clientResetScene();
		break;
	default:
		PlatformDemoApplication::specialKeyboard(key, x, y);
		break;
	}
}

/**  no free keys */
void CharpyDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case '{':
		if (displayWait < 10 && displayWait>0){
			displayWait--;
		}else{
			displayWait =(long)(displayWait/1.2);
		}
		break;
	case '}':
		if (displayWait < 10){
			displayWait++;
		}else{
			displayWait *= 1.2;
		}
		break;
	case '+':
			startAngle+=0.1;
			clientResetScene();
			break;
	case '-':
			startAngle-=0.1;
			clientResetScene();
			break;
	case '(':
			if (damping>0.){
				damping -= 0.1;
				clientResetScene();
			}
			break;
	case ')':
			if (damping < 1){
				damping += 0.1;
				clientResetScene();
			}
			break;
	case '<':
			ccdMotionThreshHold*=0.8;
			resetCcdMotionThreshHold();
			break;
	case '>':
			ccdMotionThreshHold/=0.8;
			resetCcdMotionThreshHold();
			break;
	case 'e':
			margin*=0.8;
			resetCollisionMargin();
			break;
	case 'E':
			margin/=0.8;
			resetCollisionMargin();
			break;
	case 'j':
			floorHE*=0.8;
			clientResetScene();
			break;
	case 'J':
			floorHE/=0.8;
			clientResetScene();
			break;
	case 'k':
			w*=0.8;
			clientResetScene();
			break;
	case 'K':
			w/=0.8;
			clientResetScene();
			break;
	case 'v':
			l*=0.8;
			clientResetScene();
			break;
	case 'V':
			l/=0.8;
			clientResetScene();
			break;
	case ':':
			timeStep=btScalar(timeStep/0.8);
			break;
	case '.':
			timeStep=btScalar(timeStep*0.8);
			break;
	case ',':
			timeStep=defaultTimeStep;
			break;
	case ';':
			variableTimeStep = !variableTimeStep;
			break;
	case 'i':
		getDeltaTimeMicroseconds(); // get net time
		PlatformDemoApplication::keyboardCallback(key, x, y);
		break;
	default:
		PlatformDemoApplication::keyboardCallback(key,x,y);
		break;
	}
}

// no-op
void	CharpyDemo::shootBox(const btVector3& destination)
{
}

void	CharpyDemo::exitPhysics()
{
	printf("maxCollision was %f m\n",(float)(-1.*minCollisionDistance));
	printf("maxImpact was %f J\n",maxImpact);
	printf("maxSpeed was %f m/s\n",sqrtf(maxSpeed2));
	printf("maximum constraint forces were:");
	for (int i = 0; i < 6; i++){
	   printf(" %6.2f", maxForces[i]);
	}
	printf("\n");
	if (fp){
		fclose(fp);
		fp = NULL;
	}
	maxSpeed2 = btScalar(0);
	maxImpact=btScalar(0);
	minCollisionDistance=btScalar(0.f);
	for (int i = 0; i < 6; i++){
		maxForces[i]=0.f;
	}
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




