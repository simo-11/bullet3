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

/**
Based on ForkLiftDemo by Simo Nikula 2015-
*/
#include "DemolisherDemo.h"
#include "LinearMath/btQuickprof.h"
#include <stdio.h> 
#include <string>
#ifdef _WIN32
#include <windows.h>
#endif
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "../plasticity/PlasticityExampleBrowser.h"

class btVehicleTuning;
struct btVehicleRaycaster;
class btCollisionShape;

#include "BulletDynamics/Vehicle/btRaycastVehicle.h"
#include "BulletDynamics/ConstraintSolver/btHingeConstraint.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../ExampleBrowser/GwenGUISupport/gwenUserInterface.h"
#include "../ExampleBrowser/GwenGUISupport/gwenInternalData.h"

const char * PROFILE_DEMOLISHER_SLEEP = "DemolisherDemo::Sleep";

class DemolisherDemo : public Gwen::Event::Handler, public CommonRigidBodyBase
{
	public:
	class btDiscreteDynamicsWorld* m_dynamicsWorld;
	btDiscreteDynamicsWorld* getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}
	btRigidBody* m_carChassis;
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

	GUIHelperInterface* m_guiHelper;
	int m_wheelInstances[4];

//----------------------------
	btRigidBody* m_loadBody;
	btVector3	m_loadStartPos;

	bool m_useDefaultCamera;
//----------------------------


	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	class btTriangleIndexVertexArray*	m_indexVertexArrays;

	btVector3*	m_vertices;
	float	maxEngineForce = 1000.f;//this should be engine/velocity dependent
	float	maxBreakingForce = 100.f;
	btScalar	wheelFriction=1000;
	btScalar lsx, lsy, lsz;
	int gx = 10; // for labels
	int gxi = 120; // for inputs elements
	int wxi = 60; // width
	int gy;
	int gyInc = 25;
	bool restartRequested;
	bool dropFocus = false;
	GwenUserInterface* gui;
	Gwen::Controls::Canvas* canvas;
	CommonWindowInterface* window;
	int m_option;
	enum Constraint {None=0,Rigid=1,Impulse=2,Tree=3,Plastic=4};
	Constraint constraintType;
	btRaycastVehicle::btVehicleTuning	m_tuning;
	btVehicleRaycaster*	m_vehicleRayCaster;
	btRaycastVehicle*	m_vehicle;
	btCollisionShape*	m_wheelShape;

	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;

	DemolisherDemo(CommonExampleOptions & options);

	virtual ~DemolisherDemo();

	virtual void stepSimulation(float deltaTime);
	
	virtual void	resetDemolisher();
		
	virtual void clientResetScene();

	virtual void displayCallback();
	
	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y);

	virtual bool	mouseMoveCallback(float x,float y)
	{
		return false;
	}

	virtual bool	mouseButtonCallback(int button, int state, float x, float y)
	{
		return false;
	}

	virtual bool	keyboardCallback(int key, int state);

	virtual void renderScene();

	virtual void physicsDebugDraw(int debugFlags);
	

	void initPhysics();
	void exitPhysics();

	virtual void resetCamera()
	{
		float dist = 8;
		float pitch = -45;
		float yaw = 32;
		float targetPos[3]={-0.33,-0.72,4.5};
		m_guiHelper->resetCamera(dist,pitch,yaw,targetPos[0],targetPos[1],targetPos[2]);
		CommonCameraInterface* camera =
			PlasticityExampleBrowser::getRenderer()->getActiveCamera();
		camera->setFrustumZNear(0.01);
		camera->setFrustumZFar(1000);
	}
	/** new and delete redefined due to
	warning C4316: ... : object allocated on the heap may not be aligned 16
	should be available at least in gcc
	*/
	void* operator new(size_t i)
	{
		return _mm_malloc(i, 16);
	}
		void operator delete(void* p)
	{
		_mm_free(p);
	}
	btClock idleClock;
	long displayWait = 50;
	int m_viewMode = 1;
	void setViewMode(int mode){
		m_viewMode = mode;
	}
	/** Gwen controls handling.
	Just flag changes to avoid need to specify all parent references
	if calls come from Gwen
	*/
	void restartHandler(Gwen::Controls::Base* control);
	void reinit();
	void resetHandler(Gwen::Controls::Base* control); 
	btVector3 lastLocation=btVector3(0,0,0);
	btVector3 currentLocation = btVector3(0, 0, 0);
	bool isMoving(){
		btScalar d2 = currentLocation.distance2(lastLocation);
		bool isMoving = d2>1e-4;
		return isMoving;
	}
	void updateView(){
		lastLocation = currentLocation;
		currentLocation = m_carChassis->getCenterOfMassPosition();
		btVector3 loc = (lastLocation + currentLocation)/2;
		CommonCameraInterface* camera = 
			PlasticityExampleBrowser::getRenderer()->getActiveCamera();
		camera->setCameraTargetPosition(loc.x(), loc.y(), loc.z());
	}
	std::string getText(Gwen::Controls::Base* control){
		Gwen::Controls::TextBoxNumeric* box =
			static_cast<Gwen::Controls::TextBoxNumeric*>(control);
		if (!box)	{
			return "";
		}
		return Gwen::Utility::UnicodeToString(box->GetText());
	}
	void setScalar(Gwen::Controls::Base* control, btScalar * vp){
		std::string text = getText(control);
		if (text.length() == 0)	{
			return;
		}
#if defined(BT_USE_DOUBLE_PRECISION)
		double fv = std::stod(text);
#else
		float fv = std::stof(text);
#endif
		*vp = fv;
	}
	void setLong(Gwen::Controls::Base* control, long * vp){
		std::string text = getText(control);
		if (text.length() == 0)	{
			return;
		}
		long fv = std::stol(text);
		*vp = fv;
	}
	void setInt(Gwen::Controls::Base* control, int * vp){
		std::string text = getText(control);
		if (text.length() == 0)	{
			return;
		}
		int fv = std::stoi(text);
		*vp = fv;
	}
	/*
	Limited formatter
	*/
#define UIF_SIZE 10
	std::string uif(btScalar value, const char* fmt = "%.4f")
	{
		char buffer[UIF_SIZE];
		sprintf_s(buffer, UIF_SIZE, fmt, value);
		return std::string(buffer);
	}
	void setWheelFriction(Gwen::Controls::Base* control);
	void setLsx(Gwen::Controls::Base* control);
	void setLsy(Gwen::Controls::Base* control);
	void setLsz(Gwen::Controls::Base* control);
	Gwen::Controls::Base* pPage;
	Gwen::Controls::Button* pauseButton;
	Gwen::Controls::Label* addLabel(std::string txt){
		Gwen::Controls::Label* gc = new Gwen::Controls::Label(pPage);
		gc->SetText(txt);
		gc->SizeToContents();
		gc->SetPos(gx, gy);
		return gc;
	}
	void addPauseSimulationButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		pauseButton = gc;
		gc->SetText(L"Pause");
		gc->SetPos(gx, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &DemolisherDemo::handlePauseSimulation);
	}
	void addRestartButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Restart");
		gc->SetPos(gx + wxi, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &DemolisherDemo::restartHandler);
	}
	void addResetButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Reset");
		gc->SetPos(gx + 2 * wxi, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gy += gyInc;
		gc->onPress.Add(pPage, &DemolisherDemo::resetHandler);
	}
	void place(Gwen::Controls::Base* gc){
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
	}
	void addWheelFriction(){
		addLabel("wheel friction");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(wheelFriction, "%.2f");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setWheelFriction);
	}
	void addLsx(){
		addLabel("lsx");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(lsx, "%.2f");
		gc->SetToolTip("Load size in x-direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setLsx);
	}
	void addLsy(){
		addLabel("lsy");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(lsy, "%.2f");
		gc->SetToolTip("Load size in y-direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setLsy);
	}
	void addLsz(){
		addLabel("lsz");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(lsz, "%.2f");
		gc->SetToolTip("Load size in z-direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setLsz);
	}

	void DemolisherDemo::updatePauseButtonText(){
		bool pauseSimulation = PlasticityExampleBrowser::getPauseSimulation();
		if (pauseSimulation){
			pauseButton->SetText(L"Continue");
		}
		else{
			pauseButton->SetText(L"Pause");
		}
	}
	void handlePauseSimulation(Gwen::Controls::Base* control){
		Gwen::Controls::Button* gc =
			static_cast<Gwen::Controls::Button*>(control);
		bool pauseSimulation = PlasticityExampleBrowser::getPauseSimulation();
		pauseSimulation = !pauseSimulation;
		PlasticityExampleBrowser::setPauseSimulation(pauseSimulation);
	}
	void initOptions(){
		resetCamera();
		int option = m_option;
		reinit();
		constraintType = (Constraint)(option % 100);
		option /= 100;
		while ((option /= 100) > 0){
			switch (option % 100){
			case 1:
				break;
			case 2:
				break;
			case 3:
				break;
			}
		}

	}
	void initParameterUi(){
		gui = PlasticityExampleBrowser::getGui();
		window = PlasticityExampleBrowser::getWindow();
		canvas = gui->getInternalData()->pCanvas;
		pPage = gui->getInternalData()->m_demoPage->GetPage();
		gy = 5;
		addLsx();
		addLsy();
		addLsz();
		addWheelFriction();
		addPauseSimulationButton();
		addRestartButton();
		addResetButton();
	}
	void clearParameterUi(){
		if (pPage){
			pPage->RemoveAllChildren();
			pPage = 0;
		}
		Gwen::KeyboardFocus = NULL;
	}
	void restart()
	{
		clearParameterUi();
		exitPhysics();
		PlasticityExampleBrowser::getRenderer()->removeAllInstances();
		initParameterUi();
		initPhysics();
	}

};
DemolisherDemo *demo = 0;

btScalar maxMotorImpulse = 4000.f;

//the sequential impulse solver has difficulties dealing with large mass ratios (differences),
// between loadMass and the fork parts
btScalar loadMass = 350.f;//
//btScalar loadMass = 10.f;//this should work fine for the SI solver


#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif

		int rightIndex = 0;
		int upIndex = 1;
		int forwardIndex = 2;
		btVector3 wheelDirectionCS0(0,-1,0);
		btVector3 wheelAxleCS(-1,0,0);

bool useMCLPSolver = true;


#include <stdio.h> //printf debugging


#include "DemolisherDemo.h"


const int maxProxies = 32766;
const int maxOverlap = 65535;

///btRaycastVehicle is the interface for the constraint that implements the raycast vehicle
///notice that for higher-quality slow-moving vehicles, another approach might be better
///implementing explicit hinged-wheel constraints with cylinder collision, rather then raycasts
float	gEngineForce = 0.f;

float	defaultBreakingForce = 10.f;
float	gBreakingForce = 100.f;


float	gVehicleSteering = 0.f;
float	steeringIncrement = 0.04f;
float	steeringClamp = 0.3f;
float	wheelRadius = 0.5f;
float	wheelWidth = 0.4f;
float	suspensionStiffness = 20.f;
float	suspensionDamping = 2.3f;
float	suspensionCompression = 4.4f;
float	rollInfluence = 0.1f;//1.0f;


btScalar suspensionRestLength(0.6);

#define CUBE_HALF_EXTENTS 1
void DemolisherDemo::setWheelFriction(Gwen::Controls::Base* control){
	setScalar(control, &(demo->wheelFriction));
	restartHandler(control);
}
void DemolisherDemo::setLsx(Gwen::Controls::Base* control){
	setScalar(control, &(demo->lsx));
	restartHandler(control);
}
void DemolisherDemo::setLsy(Gwen::Controls::Base* control){
	setScalar(control, &(demo->lsy));
	restartHandler(control);
}
void DemolisherDemo::setLsz(Gwen::Controls::Base* control){
	setScalar(control, &(demo->lsz));
	restartHandler(control);
}

void DemolisherDemo::restartHandler(Gwen::Controls::Base* control){
	demo->restartRequested = true;
}
void DemolisherDemo::reinit(){
	maxEngineForce = 1000;
	maxBreakingForce = 100;
	wheelFriction = 1000;
	lsx = 4;
	lsy = 2;
	lsz = 2;
}
void DemolisherDemo::resetHandler(Gwen::Controls::Base* control){
	demo->reinit();
	restartHandler(control);
}


DemolisherDemo::DemolisherDemo(CommonExampleOptions & options)
	:CommonRigidBodyBase(options.m_guiHelper),
	Gwen::Event::Handler(),
	m_guiHelper(options.m_guiHelper),
m_carChassis(0),
m_loadBody(0),
m_indexVertexArrays(0),
m_vertices(0),
m_cameraHeight(4.f),
m_minCameraDistance(3.f),
m_maxCameraDistance(10.f)
{
	options.m_guiHelper->setUpAxis(1);
	m_option = options.m_option;
	m_vehicle = 0;
	m_wheelShape = 0;
	m_useDefaultCamera = false;
	initOptions();
	initParameterUi();
}


void DemolisherDemo::exitPhysics()
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

			while (body->getNumConstraintRefs())
			{
				btTypedConstraint* constraint = body->getConstraintRef(0);
				m_dynamicsWorld->removeConstraint(constraint);
				delete constraint;
			}
			delete body->getMotionState();
			m_dynamicsWorld->removeRigidBody(body);
		} else
		{
			m_dynamicsWorld->removeCollisionObject( obj );
		}
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}
	m_collisionShapes.clear();

	delete m_indexVertexArrays;
	delete m_vertices;

	//delete dynamics world
	delete m_dynamicsWorld;
	m_dynamicsWorld=0;

	delete m_vehicleRayCaster;
	m_vehicleRayCaster = 0;

	delete m_vehicle;
	m_vehicle=0;
	
	delete m_wheelShape;
	m_wheelShape=0;

	//delete solver
	delete m_constraintSolver;
	m_constraintSolver=0;

	//delete broadphase
	delete m_overlappingPairCache;
	m_overlappingPairCache=0;

	//delete dispatcher
	delete m_dispatcher;
	m_dispatcher=0;

	delete m_collisionConfiguration;
	m_collisionConfiguration=0;

}

DemolisherDemo::~DemolisherDemo()
{
	clearParameterUi();
}

void DemolisherDemo::initPhysics()
{
	int upAxis = 1;	
	m_guiHelper->setUpAxis(upAxis);
	btVector3 groundExtents(50,50,50);
	groundExtents[upAxis]=3;
	btCollisionShape* groundShape = new btBoxShape(groundExtents);
	m_collisionShapes.push_back(groundShape);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
	if (useMCLPSolver)
	{
		btDantzigSolver* mlcp = new btDantzigSolver();
		//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
		btMLCPSolver* sol = new btMLCPSolver(mlcp);
		m_constraintSolver = sol;
	} else
	{
		m_constraintSolver = new btSequentialImpulseConstraintSolver();
	}
	m_dynamicsWorld = new btDiscreteDynamicsWorld
		(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
	if (useMCLPSolver)
	{//for direct solver it is better to have a small A matrix
		m_dynamicsWorld ->getSolverInfo().m_minimumSolverBatchSize = 1;
	} else
	{//for direct solver, it is better to solve multiple objects together, small batches have high overhead
		m_dynamicsWorld ->getSolverInfo().m_minimumSolverBatchSize = 128;
	}
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
btTransform tr;
tr.setIdentity();
tr.setOrigin(btVector3(0,-3,0));
	//create ground object
	localCreateRigidBody(0,tr,groundShape);

	btCollisionShape* chassisShape = new btBoxShape(btVector3(1.f,0.5f,2.f));
	m_collisionShapes.push_back(chassisShape);

	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	//localTrans effectively shifts the center of mass with respect to the chassis
	localTrans.setOrigin(btVector3(0,1,0));
	compound->addChildShape(localTrans,chassisShape);
	tr.setOrigin(btVector3(0,0.f,0));

	m_carChassis = localCreateRigidBody(800,tr,compound);//chassisShape);
	//m_carChassis->setDamping(0.2,0.2);
	
	m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));

	m_guiHelper->createCollisionShapeGraphicsObject(m_wheelShape);
	int wheelGraphicsIndex = m_wheelShape->getUserIndex();

	const float position[4]={0,10,10,0};
	const float quaternion[4]={0,0,0,1};
	const float color[4]={0,1,0,1};
	const float scaling[4] = {1,1,1,1};

	for (int i=0;i<4;i++)
	{
		m_wheelInstances[i] = m_guiHelper->registerGraphicsInstance
			(wheelGraphicsIndex, position, quaternion, color, scaling);
	}



	{
		btCompoundShape* loadCompound = new btCompoundShape();
		m_collisionShapes.push_back(loadCompound);
		btCollisionShape* loadShapeA = new btBoxShape(btVector3(lsx/2,lsy/2,lsz/2));
		m_collisionShapes.push_back(loadShapeA);
		btTransform loadTrans;
		loadTrans.setIdentity();
		loadCompound->addChildShape(loadTrans, loadShapeA);
		m_loadStartPos = btVector3(0, lsy/2, 5);
		loadTrans.setOrigin(m_loadStartPos);
		btScalar mass=loadMass;
		switch (constraintType){
		case Rigid:
			mass = 0;
		}
		m_loadBody  = localCreateRigidBody(mass, loadTrans, loadCompound);
	}
	/// create vehicle
	{
		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);
		
		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);

		m_dynamicsWorld->addVehicle(m_vehicle);

		float connectionHeight = 1.2f;

	
		bool isFrontWheel=true;

		//choose coordinate system
		m_vehicle->setCoordinateSystem(rightIndex,upIndex,forwardIndex);

		btVector3 connectionPointCS0(CUBE_HALF_EXTENTS-(0.3*wheelWidth),
			connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);

		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),
			connectionHeight,2*CUBE_HALF_EXTENTS-wheelRadius);

		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
		connectionPointCS0 = btVector3(-CUBE_HALF_EXTENTS+(0.3*wheelWidth),
			connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
		isFrontWheel = false;
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
		connectionPointCS0 = btVector3(CUBE_HALF_EXTENTS-(0.3*wheelWidth),
			connectionHeight,-2*CUBE_HALF_EXTENTS+wheelRadius);
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);
		
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
			wheel.m_suspensionStiffness = suspensionStiffness;
			wheel.m_wheelsDampingRelaxation = suspensionDamping;
			wheel.m_wheelsDampingCompression = suspensionCompression;
			wheel.m_frictionSlip = wheelFriction;
			wheel.m_rollInfluence = rollInfluence;
		}
	}
	resetDemolisher();
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void DemolisherDemo::physicsDebugDraw(int debugFlags)
{
	if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		m_dynamicsWorld->debugDrawWorld();
	}
}

void DemolisherDemo::renderScene()
{
	m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);

	for (int i=0;i<m_vehicle->getNumWheels();i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i,true);

		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		if (renderer)
		{
			btTransform tr = m_vehicle->getWheelInfo(i).m_worldTransform;
			btVector3 pos=tr.getOrigin();
			btQuaternion orn = tr.getRotation();
			renderer->writeSingleInstanceTransformToCPU(pos,orn,m_wheelInstances[i]);
		}
	}	
	m_guiHelper->render(m_dynamicsWorld);
	ATTRIBUTE_ALIGNED16(btScalar) m[16];
	int i;
	btVector3 wheelColor(1,0,0);
	btVector3	worldBoundsMin,worldBoundsMax;
	getDynamicsWorld()->getBroadphase()->getBroadphaseAabb(worldBoundsMin,worldBoundsMax);
	for (i=0;i<m_vehicle->getNumWheels();i++)
	{
		//synchronize the wheels with the (interpolated) chassis worldtransform
		m_vehicle->updateWheelTransform(i,true);
		//draw wheels (cylinders)
		m_vehicle->getWheelInfo(i).m_worldTransform.getOpenGLMatrix(m);
//		m_shapeDrawer->drawOpenGL(m,m_wheelShape,wheelColor,getDebugMode(),worldBoundsMin,worldBoundsMax);
	}
	btScalar idleTime = idleClock.getTimeSeconds();
	if ( idleTime> 10 && !isMoving()){
#ifdef _WIN32
		if (displayWait>0){
			BT_PROFILE(PROFILE_DEMOLISHER_SLEEP);
			Sleep(displayWait);
		}
#endif
	}
	updatePauseButtonText();
}

void DemolisherDemo::stepSimulation(float deltaTime)
{
	if (restartRequested){
		restart();
		restartRequested = false;
	}
	{			
		int wheelIndex = 2;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);
		wheelIndex = 3;
		m_vehicle->applyEngineForce(gEngineForce,wheelIndex);
		m_vehicle->setBrake(gBreakingForce,wheelIndex);


		wheelIndex = 0;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		wheelIndex = 1;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);

	}


	float dt = deltaTime;
	
	if (m_dynamicsWorld)
	{
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps =  2;
		
		int numSimSteps;
        numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);

		if (m_dynamicsWorld->getConstraintSolver()->getSolverType()==BT_MLCP_SOLVER)
		{
			btMLCPSolver* sol = (btMLCPSolver*) m_dynamicsWorld->getConstraintSolver();
			int numFallbacks = sol->getNumFallbacks();
			if (numFallbacks)
			{
				static int totalFailures = 0;
				totalFailures+=numFallbacks;
				printf("MLCP solver failed %d times, \
falling back to btSequentialImpulseSolver (SI), totalFailures=%d\n", numFallbacks, totalFailures);
			}
			sol->setNumFallbacks(0);
		}
		updateView();
	}
}



void DemolisherDemo::displayCallback(void) 
{
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
}


void DemolisherDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void DemolisherDemo::resetDemolisher()
{
	gVehicleSteering = 0.f;
	gBreakingForce = defaultBreakingForce;
	gEngineForce = 0.f;

	m_carChassis->setCenterOfMassTransform(btTransform::getIdentity());
	m_carChassis->setLinearVelocity(btVector3(0,0,0));
	m_carChassis->setAngularVelocity(btVector3(0,0,0));
	m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->
		cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),
		getDynamicsWorld()->getDispatcher());
	if (m_vehicle)
	{
		m_vehicle->resetSuspension();
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i,true);
		}
	}
	btTransform loadTrans;
	loadTrans.setIdentity();
	loadTrans.setOrigin(m_loadStartPos);
	m_loadBody->activate();
	m_loadBody->setCenterOfMassTransform(loadTrans);
	m_loadBody->setLinearVelocity(btVector3(0,0,0));
	m_loadBody->setAngularVelocity(btVector3(0,0,0));

}


bool	DemolisherDemo::keyboardCallback(int key, int state)
{
	bool handled = false;
	bool isShiftPressed = m_guiHelper->getAppInterface()->m_window->isModifierKeyPressed(B3G_SHIFT);
	idleClock.reset();
	if (state)
	{
	if (isShiftPressed) 
	{
		switch (key) 
			{
			case B3G_LEFT_ARROW : 
				{
					handled = true;
					break;
				}
			case B3G_RIGHT_ARROW : 
				{
					handled = true;
					break;
				}
			case B3G_UP_ARROW :
				{
					handled = true;
					break;
				}
			case B3G_DOWN_ARROW :
				{
					handled = true;
					break;
				}
			}

	} else
	{
			switch (key) 
			{
			case B3G_LEFT_ARROW : 
				{
					handled = true;
					gVehicleSteering += steeringIncrement;
					if (	gVehicleSteering > steeringClamp)
						gVehicleSteering = steeringClamp;

					break;
				}
			case B3G_RIGHT_ARROW : 
				{
					handled = true;
					gVehicleSteering -= steeringIncrement;
					if (	gVehicleSteering < -steeringClamp)
						gVehicleSteering = -steeringClamp;

					break;
				}
			case B3G_UP_ARROW :
				{
					handled = true;
					gEngineForce = maxEngineForce;
					gBreakingForce = 0.f;
					break;
				}
			case B3G_DOWN_ARROW :
				{
					handled = true;
					gEngineForce = -maxEngineForce;
					gBreakingForce = 0.f;
					break;
				}
			case B3G_F1:
			{
				handled = true;
				setViewMode(1);
				break;
			}
			case B3G_F2:
			{
				handled = true;
				setViewMode(2);
				break;
			}
			case B3G_F7:
				{
					handled = true;
					btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*)m_dynamicsWorld;
					world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
					printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
					break;
				}
			case B3G_F6:
				{
					handled = true;
					//switch solver (needs demo restart)
					useMCLPSolver = !useMCLPSolver;
					printf("switching to useMLCPSolver = %d\n", useMCLPSolver);

					delete m_constraintSolver;
					if (useMCLPSolver)
					{
						btDantzigSolver* mlcp = new btDantzigSolver();
						//btSolveProjectedGaussSeidel* mlcp = new btSolveProjectedGaussSeidel;
						btMLCPSolver* sol = new btMLCPSolver(mlcp);
						m_constraintSolver = sol;
					} else
					{
						m_constraintSolver = new btSequentialImpulseConstraintSolver();
					}

					m_dynamicsWorld->setConstraintSolver(m_constraintSolver);
					break;
				}

			case B3G_F5:
			handled = true;
				m_useDefaultCamera = !m_useDefaultCamera;
				break;
			default:
				break;
			}
	}

	} else
	{
		switch (key) 
		{
		case B3G_UP_ARROW:
			{
				gEngineForce = 0.f;
				gBreakingForce = defaultBreakingForce; 
				handled=true;
			break;
			}
		case B3G_DOWN_ARROW:
			{
				gEngineForce = 0.f;
				gBreakingForce = defaultBreakingForce;
				handled=true;
			break;
			}
		case B3G_LEFT_ARROW:
		case B3G_RIGHT_ARROW:
			{
				handled=true;
				break;
			}
		default:
			
			break;
		}
	}
	return handled;
}

void DemolisherDemo::specialKeyboardUp(int key, int x, int y)
{
#if 0
   
#endif
}


void DemolisherDemo::specialKeyboard(int key, int x, int y)
{
}



btRigidBody* DemolisherDemo::localCreateRigidBody(btScalar mass, 
	const btTransform& startTransform, btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	// using motionstate is recommended, it provides 
	// interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

	btRigidBody* body = new btRigidBody(cInfo);
	//body->setContactProcessingThreshold(m_defaultContactProcessingThreshold);

#else
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);
	body->setWorldTransform(startTransform);
#endif//

	m_dynamicsWorld->addRigidBody(body);
	return body;
}
CommonExampleInterface*    DemolisherDemoCreateFunc(struct CommonExampleOptions& options)
{
	demo = new DemolisherDemo(options);
	return demo;

}
