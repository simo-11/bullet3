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
#define _CRT_SECURE_NO_WARNINGS
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
#include "bt6DofElasticPlastic2Constraint.h"
#include "../plasticity/PlasticityExampleBrowser.h"
#include "../plasticity/PlasticityData.h"
#include "../plasticity/PlasticityStatistics.h"

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
	btRigidBody* m_carChassis=0;
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

	GUIHelperInterface* m_guiHelper;
	int m_wheelInstances[4];

	bool m_useDefaultCamera;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	class btTriangleIndexVertexArray*	m_indexVertexArrays;
	btDantzigSolver* mlcp=0;
	btMLCPSolver* sol=0;
	btVector3*	m_vertices;
	btScalar	maxEngineForce;//this should be engine/velocity dependent
	btScalar	defaultBreakingForce;
	btScalar	wheelFriction;
	btScalar lsx, lsy, lsz;
	btScalar bridgeLsx, bridgeLsy, bridgeLsz, bridgeSupportY;
	bool twoSupportsForGate = false;
	btScalar gateLsx, gateLsy, gateLsz;
	btScalar poleLsx, poleLsy;
	btScalar poleZ=60, bridgeZ = 40, gateZ = -20, fenceZ=5, bridgeSupportX=1;
	btScalar tolerance;
	// vehicle body measures
	btScalar yhl,xhl,zhl;
	long stepCount,maxStepCount,syncedStep;
	btScalar stepTime,gravityRampUpTime;
	btVector3 m_gravity;
	bool hasFullGravity;
	int lpc;
	btScalar breakingImpulseThreshold, breakingSpeed;
	btScalar m_fixedTimeStep = btScalar(1) / btScalar(60);
	btScalar bridgeSteelScale,gateSteelScale,fenceSteelScale, poleSteelScale;
	btScalar density;
	btScalar defaultCarMass = 50000;
	btScalar carMass;
	boolean calculateMaxEngineForce = true,
		calculateDefaultBreakingForce = true,
		calculateSuspensionStiffness=true,
		calculateMaxSuspensionForce = true,
		calculateSuspensionDamping = true,
		calculateSuspensionCompression = true,
		calculateSuspensionRestLength = true;
	btScalar suspensionStiffness, suspensionMaxForce;
	btScalar suspensionDamping;
	btScalar suspensionCompression;
	btScalar suspensionRestLength, connectionHeight;
	btScalar maxPlasticRotation;
	btScalar maxPlasticStrain;
	float	gEngineForce = 0.f;
	float	gBreakingForce = 100.f;
	bool gameBindings = true;
	bool disableCollisionsBetweenLinkedBodies = true;
	bool dumpPng = false;
	char *logDir = _strdup("d:/wrk");
	int gx = 10; // for labels
	int gxi = 120; // for inputs elements
	int wxi = 60; // width
	int swxi = 30; // short width
	int gy;
	int gyInc = 25;
	bool restartRequested=false;
	bool dropFocus = false;
	GwenUserInterface* gui;
	Gwen::Controls::Canvas* canvas;
	CommonWindowInterface* window;
	int m_option;
	enum Constraint { None = 0, Rigid = 1, Impulse = 2, ElasticPlastic = 3 };
	enum CarPosition { Gate, Fence, Bridge};
	Constraint constraintType;
	btRaycastVehicle::btVehicleTuning	m_tuning;
	btVehicleRaycaster*	m_vehicleRayCaster=0;
	btRaycastVehicle*	m_vehicle=0;
	btCollisionShape*	m_wheelShape=0;

	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;
	bool useMCLPSolver = false;
	float	wheelRadius,wheelWidth;
	float	rollInfluence = 0.1f;//1.0f;

	DemolisherDemo(CommonExampleOptions & options);

	virtual ~DemolisherDemo();

	virtual void stepSimulation(float deltaTime);

	virtual void	resetDemolisher();

	virtual void clientResetScene();

	virtual void displayCallback();

	virtual void specialKeyboard(int key, int x, int y);

	virtual void specialKeyboardUp(int key, int x, int y);

	virtual bool	mouseMoveCallback(float x, float y)
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
		float dist = 30;
		float pitch = 15;
		float yaw = 38;
		float targetPos[3] = { -0.33, -0.72, 4.5 };
		m_guiHelper->resetCamera(dist, pitch, yaw, targetPos[0], targetPos[1], targetPos[2]);
		CommonCameraInterface* camera =
			PlasticityExampleBrowser::getRenderer()->getActiveCamera();
		camera->setFrustumZNear(0.01);
		camera->setFrustumZFar(1000);
	}
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btClock idleClock;
	btClock driveClock;
	long displayWait = 50;
	/** Gwen controls handling.
	Just flag changes to avoid need to specify all parent references
	if calls come from Gwen
	*/
	void restartHandler(Gwen::Controls::Base* control);
	void reinit();
	void resetHandler(Gwen::Controls::Base* control);
	btVector3 lastLocation = btVector3(0, 0, 0);
	btVector3 currentLocation = btVector3(0, 0, 0);
	float lastClock = 0;
	float currentClock = 0;
	bool isMoving(){
		btScalar d2 = currentLocation.distance2(lastLocation);
		bool isMoving = d2 > 1e-4;
		return isMoving;
	}
#define CAM_SMOOTH_SIZE 1
	btVector3 cla[CAM_SMOOTH_SIZE];
	btVector3 scl; // smoothed camera location
	int clai = 0;
	int clas = 0;
	void updateCameraLocation(){
		if (clas > 0){
			if (clas < CAM_SMOOTH_SIZE){
				scl = (scl + currentLocation) / 2;
			}
			else{
				scl = scl + currentLocation / clas;
			}
		}
		else{
			scl = currentLocation;
		}
		clas++;
		if (clas > CAM_SMOOTH_SIZE){
			scl = scl - cla[clai] / CAM_SMOOTH_SIZE;
			cla[clai] = currentLocation;
			clai++;
			if (clai >= CAM_SMOOTH_SIZE){
				clai = 0;
			}
			clas = CAM_SMOOTH_SIZE;
		}
		else{ // initial fill
			cla[clas-1] = currentLocation;
		}
		CommonCameraInterface* camera =
			PlasticityExampleBrowser::getRenderer()->getActiveCamera();
		camera->setCameraTargetPosition(scl.x(), scl.y(), scl.z());
	}
	/** update fps and kmph after interval seconds have passed */
	float speedometerUpdated;
	float updateInterval = 0.3;
	int updateViewCount = 0;
	void resetClocks(){
		driveClock.reset();
		speedometerUpdated = 0;
	}
	void updateView(){
		updateViewCount++;
		float now = driveClock.getTimeSeconds();
		lastClock = currentClock;
		currentClock = now;
		lastLocation = currentLocation;
		if (m_carChassis){
			currentLocation = m_carChassis->getCenterOfMassPosition();
		}
		else{
			currentLocation = getCarTransform().getOrigin();
		}
		updateCameraLocation();
		float timeDelta = now - speedometerUpdated;
		if (timeDelta < updateInterval){
			return;
		}
		if (m_vehicle){
			kmph = (int)(m_vehicle->getCurrentSpeedKmHour());
			mps = (int)(m_vehicle->getCurrentSpeedKmHour() / 3.6);
		}
		fps = updateViewCount / timeDelta;
		speedometerUpdated = now;
		updateViewCount = 0;
	}
	float gVehicleSteering = 0.f;
	float steeringIncrement = 0.04f;
	float steeringClamp = 0.3f;
	/* 1/3 second for full recover */
	float centeringIncrement = 3*steeringClamp *m_fixedTimeStep; 
	float steeringDelta = 0.f;
	void setSteeringDelta(float delta){
		steeringDelta = delta;
	}
	void centerSteering(){
		if (!isMoving()){
			return;
		}
		if (gVehicleSteering == 0.f){
			return;
		}
		if (gVehicleSteering > 0.f){
			if (gVehicleSteering < centeringIncrement){
				gVehicleSteering = 0.f;
			}
			else{
				gVehicleSteering -= centeringIncrement;
			}
		}
		else{
			if (gVehicleSteering > -centeringIncrement){
				gVehicleSteering = 0.f;
			}
			else{
				gVehicleSteering +=centeringIncrement;
			}
		}
	}
	void updateSteering(){
		if (steeringDelta == 0.f){
			centerSteering();
			return;
		}
		gVehicleSteering += steeringDelta;
		if (gVehicleSteering > steeringClamp){
			gVehicleSteering = steeringClamp;
		}
		else if (gVehicleSteering < -steeringClamp){
			gVehicleSteering = -steeringClamp;
		}
	}
	float pitchDelta = 0;
	void setPitchDelta(float delta){
		pitchDelta = delta;
	}
	void updatePitch(){
		if (pitchDelta == 0.f){
			return;
		}
		CommonCameraInterface* camera =
			PlasticityExampleBrowser::getRenderer()->getActiveCamera();
		float current = camera->getCameraPitch();
		current += pitchDelta;
		if (current > 360){
			current -= 360;
		}
		else if (current < 0){
			current += 360;
		}
		camera->setCameraPitch(current);
	}
	float yawDelta = 0;
	void setYawDelta(float delta){
		yawDelta = delta;
	}
	void updateYaw(){
		if (yawDelta == 0.f){
			return;
		}
		CommonCameraInterface* camera =
			PlasticityExampleBrowser::getRenderer()->getActiveCamera();
		float current = camera->getCameraYaw();
		current += yawDelta;
		if (current >= 0 && current < 90){
			camera->setCameraYaw(current);
		}
	}
	void updateEngineForce(float scale){
		gEngineForce += scale*maxEngineForce;
		gBreakingForce = 0.f;
	}
	void updateGravity(){
		if (!m_dynamicsWorld){
			return;
		}
		if (stepTime < gravityRampUpTime){
			m_dynamicsWorld->setGravity(stepTime / gravityRampUpTime*m_gravity);
		}
		else if (!hasFullGravity){
			m_dynamicsWorld->setGravity(m_gravity);
			hasFullGravity = true;
		}
	}
	/**
	https://en.wikipedia.org/wiki/Drag_(physics)
	ro is about 1 kg/m3
	area is about 2 m2
	Cd is about 1
	so we use speed2
	*/
	btVector3 drag;
	btScalar dragForce = 0;
	btScalar halfAreaForDrag;
	void updateDrag(){
		btVector3 vel = m_carChassis->getLinearVelocity();
		dragForce = -vel.length2();
		btVector3 norm;
		if (dragForce < -0.01){
			drag = vel.normalized()*dragForce;
		}
		else{
			drag = btVector3(0, 0, 0);
		}
	}
	std::string getText(Gwen::Controls::Base* control){
		Gwen::Controls::TextBoxNumeric* box =
			static_cast<Gwen::Controls::TextBoxNumeric*>(control);
		if (!box)	{
			return "";
		}
		return Gwen::Utility::UnicodeToString(box->GetText());
	}
#define FN_SIZE 512
	char* getChar(Gwen::UnicodeString s){
		const wchar_t * wcs = s.c_str();
		size_t count = wcstombs(NULL, wcs, FN_SIZE);
		if (count < 1){
			return NULL;
		}
		char * cp = (char*)malloc(count);
		size_t retValue = wcstombs(cp, wcs, count);
		if (retValue > 0){
			return cp;
		}
		else{
			free(cp);
			return NULL;
		}
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
#define UIF_SIZE 50
	std::string uif(btScalar value, const char* fmt = "%.4f")
	{
		char buffer[UIF_SIZE];
		sprintf_s(buffer, UIF_SIZE, fmt, value);
		return std::string(buffer);
	}
	void setWheelFriction(Gwen::Controls::Base* control);
	void setGravityRampUpTime(Gwen::Controls::Base* control);
	void setLpc(Gwen::Controls::Base* control);
	void setLsx(Gwen::Controls::Base* control);
	void setLsy(Gwen::Controls::Base* control);
	void setLsz(Gwen::Controls::Base* control);
	void setDensity(Gwen::Controls::Base* control);
	void setBreakingSpeed(Gwen::Controls::Base* control);
	void setCarMass(Gwen::Controls::Base* control);
	void setDefaultBreakingForce(Gwen::Controls::Base* control);
	void setMaxEngineForce(Gwen::Controls::Base* control);
	void setSuspensionStiffness(Gwen::Controls::Base* control);
	void setSuspensionMaxForce(Gwen::Controls::Base* control);
	void setSuspensionDamping(Gwen::Controls::Base* control);
	void setSuspensionCompression(Gwen::Controls::Base* control);
	void setSuspensionRestLength(Gwen::Controls::Base* control);
	void setFenceSteelScale(Gwen::Controls::Base* control);
	void setBridgeSteelScale(Gwen::Controls::Base* control);
	void setGateSteelScale(Gwen::Controls::Base* control);
	void setPoleSteelScale(Gwen::Controls::Base* control);
	void setMaxPlasticStrain(Gwen::Controls::Base* control);
	void setMaxPlasticRotation(Gwen::Controls::Base* control);
	void setGameBindings(Gwen::Controls::Base* control);
	void handlePauseSimulation(Gwen::Controls::Base* control);
	void handleSingleStep(Gwen::Controls::Base* control);
	void setDisableCollisionsBetweenLinkedBodies(Gwen::Controls::Base* control);
	void setLogDir(Gwen::Controls::Base* control);
	void setDumpPng(Gwen::Controls::Base* control);
	Gwen::Controls::Base* pPage;
	Gwen::Controls::Button* pauseButton, *singleStepButton;
	Gwen::Controls::Label *db11, *db12, *db13, *db14, *db15, *db16;
	Gwen::Controls::Label *db21, *db22, *db23;
	Gwen::Controls::Label *db31, *db32;
	int fps = 0;
	int kmph = 0;
	int mps = 0;
	void updateDashboards(){
		char buffer[UIF_SIZE];
		sprintf_s(buffer, UIF_SIZE, "%-3d ", fps);
		std::string str = std::string(buffer);
		db11->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%-3d ", kmph);
		str = std::string(buffer);
		db13->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%-3d ", mps);
		str = std::string(buffer);
		db15->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%-+9.0f ", gEngineForce);
		str = std::string(buffer);
		db21->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%9.0f ", -gBreakingForce);
		str = std::string(buffer);
		db22->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%9.0f ", -dragForce);
		str = std::string(buffer);
		db23->SetText(str);
		if (useMCLPSolver && sol){
			sprintf_s(buffer, UIF_SIZE, "%5d ", sol->getNumFallbacks());
			str = std::string(buffer);
			db32->SetText(str);
		}
	}
	void syncWheelsToGraphics(){
		CommonRenderInterface* renderer = m_guiHelper->getRenderInterface();
		if (m_vehicle && renderer){
			for (int i = 0; i < m_vehicle->getNumWheels(); i++)
			{
				btWheelInfo& wi = m_vehicle->getWheelInfo(i);
				btTransform tr = wi.m_worldTransform;
				btVector3 pos = tr.getOrigin();
				btQuaternion orn = tr.getRotation();
				renderer->writeSingleInstanceTransformToCPU
					(pos, orn, m_wheelInstances[i]);
			}
		}
	}
	void addCollisionBetweenLinkedBodies(){
		Gwen::Controls::Label* label = addLabel("disableCollisions");
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetToolTip("disableCollisionsBetweenLinkedBodies");
		gc->SetPos(gxi, gy);
		gc->SetChecked(disableCollisionsBetweenLinkedBodies);
		gy += gyInc;
		gc->onCheckChanged.Add(pPage, 
			&DemolisherDemo::setDisableCollisionsBetweenLinkedBodies);
	}

	void addGameBindings(){
		Gwen::Controls::Label* label = addLabel("gameBindings");
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetToolTip("set game style keyboard bindings for e.g. asdw");
		gc->SetPos(gxi, gy);
		gc->SetChecked(gameBindings);
		gy += gyInc;
		gc->onCheckChanged.Add(pPage, &DemolisherDemo::setGameBindings);
	}
	Gwen::Controls::CheckBox* dumpPngGc;
	void addDumpPng(){
		Gwen::Controls::Label* label = addLabel("dumpPng");
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetPos(gxi, gy);
		gc->SetChecked(dumpPng);
		dumpPngGc = gc;
		gy += gyInc;
		gc->onCheckChanged.Add(pPage, &DemolisherDemo::setDumpPng);
	}
	void addLogDir(){
		Gwen::Controls::Label* label = addLabel("logDir");
		Gwen::Controls::TextBox* gc = new Gwen::Controls::TextBox(pPage);
		gc->SetPos(gxi, gy);
		gc->SetText(logDir);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setLogDir);
	}
	void addDashboard(){
		db11 = new Gwen::Controls::Label(pPage);
		db12 = new Gwen::Controls::Label(pPage);
		db13 = new Gwen::Controls::Label(pPage);
		db14 = new Gwen::Controls::Label(pPage);
		db15 = new Gwen::Controls::Label(pPage);
		db16 = new Gwen::Controls::Label(pPage);
		db21 = new Gwen::Controls::Label(pPage);
		db22 = new Gwen::Controls::Label(pPage);
		db23 = new Gwen::Controls::Label(pPage);
		db31 = new Gwen::Controls::Label(pPage);
		db32 = new Gwen::Controls::Label(pPage);
		updateDashboards();
		db11->SizeToContents();
		db11->SetPos(gx, gy);
		db12->SetText("fps");
		db12->SizeToContents();
		db12->SetPos(gx + wxi / 2, gy);
		db13->SizeToContents();
		db13->SetPos(gx + wxi, gy);
		db14->SetText("km/h");
		db14->SizeToContents();
		db14->SetPos(gx + 14 * wxi / 10, gy);
		db15->SizeToContents();
		db15->SetPos(gx + 2*wxi, gy);
		db16->SetText("m/s");
		db16->SizeToContents();
		db16->SetPos(gx + 24 * wxi / 10, gy);
		gy += gyInc;
		db21->SizeToContents();
		db21->SetPos(gx, gy);
		db22->SizeToContents();
		db22->SetPos(gx + wxi, gy);
		db23->SizeToContents();
		db23->SetPos(gx + 2 * wxi, gy);
		gy += gyInc;
		if (useMCLPSolver){
			db31->SetText("#mlcp fails");
			db31->SizeToContents();
			db31->SetPos(gx, gy);
			db32->SizeToContents();
			db32->SetPos(gx + wxi, gy);
		}
	}
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
	void addSingleStepButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"SS");
		gc->SetToolTip(L"Single Step");
		gc->SetPos(gx + wxi, gy);
		gc->SetSize(swxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &DemolisherDemo::handleSingleStep);
	}
	void addRestartButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Restart");
		gc->SetPos(gx + wxi+swxi, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &DemolisherDemo::restartHandler);
	}
	void addResetButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Reset");
		gc->SetPos(gx + 2 * wxi+swxi, gy);
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
	void addGravityRampUpTime(){
		addLabel("gravityRampUpTime");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		gc->SetToolTip("Gravity ramp up time [s]");
		std::string text = uif(gravityRampUpTime, "%.2f");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setGravityRampUpTime);
	}
	void addLpc(){
		addLabel("lpc");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = std::to_string(lpc);
		gc->SetToolTip("Load parts in x-direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setLpc);
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
	void addDensity(){
		addLabel("density");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(density, "%.2f");
		gc->SetToolTip("Load density [kg/m3]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setDensity);
	}
	btScalar steelDensity=7800;
	btScalar getDensity(btScalar steelScale){
		return btScalar(steelScale*steelDensity+(1-steelScale)*density);
	}
	void addBreakingSpeed(){
		addLabel("breakingSpeed");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(breakingSpeed, "%.2f");
		gc->SetToolTip("breakingSpeed [m/s]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setBreakingSpeed);
	}
	void addMaxPlasticStrain(){
		addLabel("maxPlasticStrain");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(maxPlasticStrain, "%.2f");
		gc->SetToolTip("maxPlasticStrain");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setMaxPlasticStrain);
	}
	void addMaxPlasticRotation(){
		addLabel("maxPlasticRotation");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(maxPlasticRotation, "%.1f");
		gc->SetToolTip("maxPlasticRotation");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setMaxPlasticRotation);
	}
	void addGateSteelScale(){
		addLabel("gate steel%");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(gateSteelScale*100, "%.3f");
		gc->SetToolTip("Area of enforcement steel [%] [0-100]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setGateSteelScale);
	}
	void addFenceSteelScale(){
		addLabel("fence steel%");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(fenceSteelScale * 100, "%.3f");
		gc->SetToolTip("Area of enforcement steel [%] [0-100]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setFenceSteelScale);
	}
	void addBridgeSteelScale(){
		addLabel("bridge steel%");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(bridgeSteelScale * 100, "%.3f");
		gc->SetToolTip("Area of enforcement steel [%] [0-100]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setBridgeSteelScale);
	}
	void addPoleSteelScale(){
		addLabel("pole steel%");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(poleSteelScale * 100, "%.3f");
		gc->SetToolTip("Area of enforcement steel [%] [0-100]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setPoleSteelScale);
	}
	void addCarMass(){
		addLabel("car mass");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(carMass/1000, "%.0f");
		gc->SetToolTip("car mass [t]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setCarMass);
	}
	void addSuspensionStiffness(){
		addLabel("suspensionStiffness");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(suspensionStiffness, "%.1f");
		gc->SetToolTip("Suspension Stiffness [N/m]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setSuspensionStiffness);
	}
	void addSuspensionMaxForce(){
		addLabel("suspensionMaxForce");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(suspensionMaxForce, "%.0f");
		gc->SetToolTip("Suspension Maximum Force [N]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setSuspensionMaxForce);
	}
	void addSuspensionDamping(){
		addLabel("suspensionDamping");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(suspensionDamping, "%.1f");
		gc->SetToolTip("Suspension Damping");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setSuspensionDamping);
	}
	void addSuspensionCompression(){
		addLabel("suspensionCompression");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(suspensionCompression, "%.1f");
		gc->SetToolTip("Suspension Compression");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setSuspensionCompression);
	}
	void addSuspensionRestLength(){
		addLabel("suspensionRestLength");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(suspensionRestLength, "%.2f");
		gc->SetToolTip("Suspension RestLength [kg]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setSuspensionRestLength);
	}
	void addMaxEngineForce(){
		addLabel("max engine force");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(maxEngineForce, "%.0f");
		gc->SetToolTip("impulse");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setMaxEngineForce);
	}
	void addDefaultBreakingForce(){
		addLabel("default breaking force");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(defaultBreakingForce, "%.2f");
		gc->SetToolTip("impulse");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setDefaultBreakingForce);
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
		addLpc();
		addLsx();
		addLsy();
		addLsz();
		addDensity();
		switch (constraintType){
		case Impulse:
			addCollisionBetweenLinkedBodies();
			addGateSteelScale();
			addFenceSteelScale();
			addBridgeSteelScale();
			addPoleSteelScale();
			addBreakingSpeed();
			break;
		case ElasticPlastic:
			addCollisionBetweenLinkedBodies();
			addMaxPlasticRotation();
			addMaxPlasticStrain();
			addGateSteelScale();
			addFenceSteelScale();
			addBridgeSteelScale();
			addPoleSteelScale();
			break;
		}
		addCarMass();
		if (!calculateSuspensionStiffness){
			addSuspensionStiffness();
		}
		if (!calculateMaxSuspensionForce){
			addSuspensionMaxForce();
		}
		if (!calculateSuspensionDamping){
			addSuspensionDamping();
		}
		if (!calculateSuspensionCompression){
			addSuspensionCompression();
		}
		if (!calculateSuspensionRestLength){
			addSuspensionRestLength();
		}
		if (!calculateMaxEngineForce){
			addMaxEngineForce();
		}
		if (!calculateDefaultBreakingForce){
			addDefaultBreakingForce();
		}
		addWheelFriction();
		addGravityRampUpTime();
		addGameBindings();
		addDumpPng();
		addLogDir();
		addPauseSimulationButton();
		addSingleStepButton();
		addRestartButton();
		addResetButton();
		addDashboard();
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
		resetClocks();
		initPhysics();
	}
	void addFixedConstraint(btAlignedObjectArray<btRigidBody*> ha, btScalar xlen, btScalar scale){
		btVector3 cpos(xlen/2, 0, 0);
		addFixedConstraint(ha, cpos, scale);
	}
	void addFixedConstraint(btAlignedObjectArray<btRigidBody*> ha, btVector3& cpos, btScalar scale){
			int loopSize = ha.size() - 1;
		breakingImpulseThreshold = carMass*breakingSpeed / m_fixedTimeStep;
		btTransform tra;
		btTransform trb;
		tra.setIdentity();
		trb.setIdentity();
		tra.setOrigin(cpos);
		trb.setOrigin(-cpos);
		for (int i = 0; i < loopSize; i++){
			btGeneric6DofConstraint *sc =
				new btGeneric6DofConstraint(*ha[i], *ha[i + 1],
				tra, trb, true);
			sc->setBreakingImpulseThreshold(scale*breakingImpulseThreshold);
			m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
			for (int i = 0; i < 6; i++){
				sc->setLimit(i, 0, 0); // make fixed
			}
		}
	}
	/**
	*
	*/
	void addElasticPlasticConstraint(btAlignedObjectArray<btRigidBody*> ha,
		btScalar xlen, btScalar ylen, btScalar zlen, btScalar steelScale){
		btVector3 cpos(xlen / 2, 0, 0);
		addElasticPlasticConstraint(ha, cpos, xlen, ylen, zlen, steelScale);
	}
	/**
	xlen is length of body to be used for axial and torsional stiffness
	TODO, use cpos for selecting global dofs
	*/
	void addElasticPlasticConstraint(btAlignedObjectArray<btRigidBody*> ha,
		btVector3& cpos,
		btScalar xlen, btScalar ylen, btScalar zlen, btScalar steelScale){
		int loopSize = ha.size() - 1;
		btScalar E(200E9);
		btScalar G(80E9);
		btScalar fy(200E6);
		bool limitIfNeeded = true;
		btScalar damping(0.1);
		btTransform tra;
		btTransform trb;
		tra.setIdentity();
		trb.setIdentity();
		tra.setOrigin(cpos);
		trb.setOrigin(-cpos);
		btScalar k0(E*ylen*zlen*steelScale / xlen);
		// I=bh^3/12, k for end moment with fixed end is EI/l
		btScalar im(E* steelScale / xlen / 12);
		btScalar k1(ylen*ylen*ylen*zlen*im);
		btScalar k2(ylen*zlen*zlen*zlen*im);
		btScalar w0(fy*ylen*zlen*steelScale);
		btScalar w1(fy*zlen*ylen*ylen*steelScale / 4);
		btScalar w2(fy*zlen*zlen*ylen*steelScale / 4);
		for (int i = 0; i < loopSize; i++){
			bt6DofElasticPlastic2Constraint *sc =
				new bt6DofElasticPlastic2Constraint(*ha[i], *ha[i + 1],
				tra, trb);
			sc->setUserConstraintType(BPT_EP2);
			sc->setMaxPlasticRotation(maxPlasticRotation);
			sc->setMaxPlasticStrain(maxPlasticStrain);
			sc->setStiffness(2, k0, limitIfNeeded);
			sc->setMaxForce(2, w0/2);
			sc->setStiffness(0, k0, limitIfNeeded);
			sc->setMaxForce(0, w0);
			sc->setStiffness(1, k0, limitIfNeeded);
			sc->setMaxForce(1, w0/2);
			sc->setStiffness(5, k1); 
			sc->setMaxForce(5, w1); 
			sc->setStiffness(4, k1); 
			sc->setMaxForce(4, w1);
			sc->setStiffness(3, k2, limitIfNeeded); 
			sc->setMaxForce(3, w2);
			m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
			for (int i = 0; i<6; i++)
			{
				sc->enableSpring(i, true);
			}
			for (int i = 0; i<6; i++)
			{
				sc->setDamping(i, damping);
			}
			sc->setEquilibriumPoint();
			m_dynamicsWorld->addAction(sc);
		}
	}
	int pngNro = 0;
	char dumpFilename[FN_SIZE];
	void setDumpFilename(){
		CommonGraphicsApp * app = PlasticityExampleBrowser::getApp();
		if (dumpPng){
			pngNro++;
			sprintf_s(dumpFilename, FN_SIZE, "%s/demolisher-%d.png", logDir, pngNro);
			dumpPngGc->SetToolTip(dumpFilename);
			app->dumpNextFrameToPng(dumpFilename);
		}
		else{
			app->dumpNextFrameToPng(NULL);
		}
    }
	/**
	*/
	void addRamp(btScalar xloc, btScalar zloc){
		btScalar z = bridgeLsz/2;
		btScalar y = bridgeLsy + bridgeSupportY;
		btScalar x = 10*y;
		btConvexHullShape* draft = new btConvexHullShape();
		draft->addPoint(btVector3(0, y, z));
		draft->addPoint(btVector3(0, y, -z));
		draft->addPoint(btVector3(0, 0, -z));
		draft->addPoint(btVector3(0, 0, z));
		draft->addPoint(btVector3(x, 0, z));
		draft->addPoint(btVector3(x, 0, -z));
		btTransform tr;
		tr.setIdentity();
		btVector3 pos = btVector3(xloc, 0, zloc);
		tr.setOrigin(pos);
		if (xloc < 0){
			btQuaternion q(btVector3(0,1,0),SIMD_PI);
			tr.setRotation(q);
		}
		localCreateRigidBody(0, tr, draft);
	}
	CarPosition latestCarPosition = Gate;
	btTransform getCarTransform(){
		return getCarTransform(latestCarPosition);
	}
	btTransform getCarTransform(CarPosition position){
		updateGravity();
		btScalar gy = m_dynamicsWorld->getGravity().getY();
		/*
		* this scaler was obtained by fitting data
		* physical backgrund remains to be solved
		*/
		btScalar scaler = 0.00018;
		btScalar sy = scaler* gy* carMass	/ suspensionStiffness / 4;
		if (sy < -suspensionRestLength){
			sy = -suspensionRestLength;
		}
		btScalar yStart = connectionHeight+sy;
		btScalar zStart, xStart;
		btTransform tr;
		tr.setIdentity();
		switch (position){
		default:
		case Fence:
			xStart = 0;
			zStart = -zhl;
			break;
		case Bridge:
			{
				btQuaternion q(SIMD_HALF_PI,0,0);
				tr.setRotation(q);
				xStart = -bridgeLsx / 2-2*zhl ;
				zStart = bridgeZ;
				yStart += bridgeSupportY + bridgeLsy-0.2*zhl;
			}
			break;
		case Gate:
			xStart = 0;
			zStart = gateZ-3*zhl;
			break;
		}
		tr.setOrigin(btVector3(xStart, yStart, zStart));
		return tr;
	}
	void setCarPosition(CarPosition position){
		latestCarPosition = position;
		btRigidBody *b = m_carChassis;
		if (b){
			btVector3 zero(0, 0, 0);
			gEngineForce = 0.f;
			gBreakingForce = defaultBreakingForce;
			gVehicleSteering = 0.f;
			b->setLinearVelocity(zero);
			b->setAngularVelocity(zero);
			b->setCenterOfMassTransform
				(getCarTransform(position));
		}
	}
	list<PlasticityData> pData;
	PlasticityData getPlasticityData(char* buf){
		PlasticityData pd(buf);
		return pd;
	}
	void addPData(char * buf){
		pData.push_back(getPlasticityData(buf));
	}
	void infoMsg(char * buf){
		addPData(buf);
	}
	// Buffer length for sprintfs
	#define B_LEN 100
	// clean small value
#define CSV(x) ((x)<1e-6?0:(x))
	void showMessage()
	{
		if (!PlasticityData::getCollect()){
			return;
		}
		if (restartRequested){
			return;
		}
		pData.clear();
		char buf[B_LEN*2];
		sprintf_s(buf, B_LEN, "stepTime=%4.1f, gravity=%4.1f, stepCount=%d",
			stepTime, m_dynamicsWorld->getGravity().getY(),stepCount);
		infoMsg(buf);
		btDiscreteDynamicsWorld *dw = m_dynamicsWorld;
		bool headerDone = false;
		int numConstraints=dw->getNumConstraints();
		for (int i = 0; i < numConstraints; i++){
			btTypedConstraint* sc = dw->getConstraint(i);
			int type = sc->getUserConstraintType();
			if (type != BPT_EP2){
				continue;
			}
			if (!headerDone){
				sprintf_s(buf, B_LEN, "%2s %8s %5s %5s %5s %5s %5s",
					"#", "max%", "m%dof", "mpr", "cpr", "mps", "cps");
				infoMsg(buf);
				headerDone = true;
			}
			bt6DofElasticPlastic2Constraint *epc =
				static_cast<bt6DofElasticPlastic2Constraint*>(sc);
			btScalar mpr = epc->getMaxPlasticRotation(),
				cpr = epc->getCurrentPlasticRotation(),
				mps = epc->getMaxPlasticStrain(),
				cps = epc->getCurrentPlasticStrain(),
				maxr = epc->getMaxRatio();
			int	maxrd = epc->getMaxRatioDof();
				sprintf_s(buf, B_LEN*2, "%2d %8.1f %5d %5.3f %5.3f %5.3f %5.3f",
					i, maxr * 100, maxrd, mpr, cpr, mps, cps);
			infoMsg(buf);
		}
		PlasticityData::setData(&pData);
	}
	void addPole(){
		btAlignedObjectArray<btRigidBody*> ha;
		btScalar ylen = poleLsy / lpc;
		btCollisionShape* loadShape = new btBoxShape(btVector3(poleLsx / 2, ylen / 2, poleLsx / 2));
		btScalar mass;
		switch (constraintType){
		case Rigid:
			mass = 0;
			break;
		default:
			mass = poleLsx*ylen*poleLsx*getDensity(poleSteelScale) / lpc;
			break;
		}
		m_collisionShapes.push_back(loadShape);
		btScalar yloc = ylen / 2;
		for (int i = 0; i < lpc; i++){
			btTransform loadTrans;
			loadTrans.setIdentity();
			btVector3 pos = btVector3(0, yloc, poleZ);
			loadTrans.setOrigin(pos);
			ha.push_back(localCreateRigidBody(mass, loadTrans, loadShape));
			yloc += ylen;
		}
		btVector3 cpos(0, ylen / 2, 0);
		switch (constraintType){
		case Impulse:
			addFixedConstraint(ha, cpos, poleSteelScale);
			break;
		case ElasticPlastic:
			addElasticPlasticConstraint(ha, cpos, ylen, poleLsx, poleLsx, poleSteelScale);
			break;
		}
	}
	void addFence(){
		btAlignedObjectArray<btRigidBody*> ha;
		btScalar xlen = lsx / lpc;
		btCollisionShape* loadShape = new btBoxShape(btVector3(xlen / 2, lsy / 2, lsz / 2));
		btScalar mass;
		switch (constraintType){
		case Rigid:
			mass = 0;
			break;
		default:
			mass = lsx*lsy*lsz*getDensity(fenceSteelScale) / lpc;
			break;
		}
		m_collisionShapes.push_back(loadShape);
		btScalar xloc = (xlen - lsx) / 2;
		for (int i = 0; i < lpc; i++){
			btTransform loadTrans;
			loadTrans.setIdentity();
			btVector3 pos = btVector3(xloc, lsy / 2, fenceZ);
			loadTrans.setOrigin(pos);
			ha.push_back(localCreateRigidBody(mass, loadTrans, loadShape));
			xloc += xlen;
		}
		switch (constraintType){
		case Impulse:
			addFixedConstraint(ha, xlen, fenceSteelScale);
			break;
		case ElasticPlastic:
			addElasticPlasticConstraint(ha, xlen, lsy, lsz, fenceSteelScale);
			break;
		}
	}
	void addBridge(){
		btAlignedObjectArray<btRigidBody*> ha;
		btScalar xlen = bridgeLsx / lpc;
		btCollisionShape* partShape = 
			new btBoxShape(btVector3(xlen / 2, bridgeLsy / 2, bridgeLsz / 2));
		btCollisionShape* supportShape = 
			new btBoxShape(btVector3(bridgeSupportX/2, bridgeSupportY / 2, bridgeLsz / 2));
		m_collisionShapes.push_back(partShape);
		m_collisionShapes.push_back(supportShape);
		btScalar mass;
		switch (constraintType){
		case Rigid:
			mass = 0;
			break;
		default:
			mass = bridgeLsx*bridgeLsy*bridgeLsz*getDensity(bridgeSteelScale) / lpc;
			break;
		}
		btScalar xloc = (xlen - bridgeLsx) / 2;
		for (int i = 0; i < lpc; i++){
			btTransform tr;
			tr.setIdentity();
			btVector3 pos = btVector3(xloc, bridgeSupportY + bridgeLsy / 2, bridgeZ);
			tr.setOrigin(pos);
			ha.push_back(localCreateRigidBody(mass, tr, partShape));
			// end supports do not move
			if (i == 0 || i == (lpc - 1)){
				btScalar rxloc;
				btScalar sloc;
				switch (i){
				case 0:
					rxloc = xloc - xlen / 2 - tolerance;
					sloc = xloc - xlen / 2 + bridgeSupportX / 2-tolerance;
					break;
				default:
					rxloc = xloc + xlen / 2 + tolerance;
					sloc = xloc + xlen / 2 - bridgeSupportX / 2+tolerance;
					break;
				}
				btTransform tr;
				tr.setIdentity();
				btVector3 pos = btVector3(sloc, bridgeSupportY / 2, bridgeZ);
				tr.setOrigin(pos);
				localCreateRigidBody(0, tr, supportShape);
				addRamp(rxloc, bridgeZ);
			}
			xloc += xlen;
		}
		switch (constraintType){
		case Impulse:
			addFixedConstraint(ha, xlen, bridgeSteelScale);
			break;
		case ElasticPlastic:
			addElasticPlasticConstraint(ha, xlen, bridgeLsy, bridgeLsz, bridgeSteelScale);
			break;
		}
	}
	void addGate(){
		btScalar gateSupportHeight = 0.5*yhl + wheelRadius + gateLsy;
		btScalar gateY = gateSupportHeight/2;
		btScalar gateSupportWidth = 0.5 + gateLsz * 2;
		btScalar gateSupportLength = (twoSupportsForGate?gateLsx/3:gateLsx/2);
		btAlignedObjectArray<btRigidBody*> ha;
		/* end part(s) have additional mass */
		int gpc = lpc + (twoSupportsForGate?2:1);
		btScalar xlen = gateLsx / gpc;
		btCollisionShape* partShape = new btBoxShape
			(btVector3(xlen / 2, gateLsy / 2, gateLsz / 2));
		btCollisionShape* supportShape = new btBoxShape
			(btVector3(gateSupportLength / 2, gateSupportHeight / 2, gateSupportWidth / 2));
		btScalar mass;
		switch (constraintType){
		case Rigid:
			mass = 0;
			break;
		default:
			mass = gateLsx*gateLsy*gateLsz*getDensity(gateSteelScale) / gpc;
			break;
		}
		/* */
		btScalar gateSupportMass = gateSupportLength*gateSupportHeight*gateSupportWidth*density+mass;
		m_collisionShapes.push_back(partShape);
		btScalar xloc = (xlen - gateLsx) / 2;
		for (int i = 0; i < gpc; i++){
			btTransform tr;
			tr.setIdentity();
			btVector3 pos = btVector3(xloc, gateY, gateZ);
			tr.setOrigin(pos);
			if (i == 0 || (twoSupportsForGate && i == (gpc - 1))){
				// add gateSupport and connect it to gate part
				btCompoundShape* compound = new btCompoundShape();
				m_collisionShapes.push_back(compound);
				btTransform itr;
				itr.setIdentity();
				compound->addChildShape(itr, partShape);
				btTransform str;
				str.setIdentity();
				btScalar xLoc(xlen / 2 - gateSupportLength / 2);
				if (i != 0){
					xLoc = -xLoc;
				}
				btVector3 pos = btVector3(xLoc, 0, 0);
				str.setOrigin(pos);
				compound->addChildShape(str, supportShape);
				btRigidBody* rb=localCreateRigidBody(gateSupportMass, tr, compound);
				btScalar friction = rb->getFriction();
				rb->setFriction(friction*15);
				ha.push_back(rb);
			}
			else{
				ha.push_back(localCreateRigidBody(mass, tr, partShape));
			}
			xloc += xlen;
		}
		switch (constraintType){
		case Impulse:
			addFixedConstraint(ha, xlen, gateSteelScale);
			break;
		case ElasticPlastic:
			addElasticPlasticConstraint(ha, xlen, gateLsy, gateLsz, gateSteelScale);
			break;
		}
	}
	void addVehicle(){
		btCollisionShape* chassisShape = 
			new btBoxShape(btVector3(xhl, yhl, zhl));
		m_collisionShapes.push_back(chassisShape);
		btCompoundShape* compound = new btCompoundShape();
		m_collisionShapes.push_back(compound);
		btTransform localTrans;
		localTrans.setIdentity();
		localTrans.setOrigin(btVector3(0, yhl, 0));
		compound->addChildShape(localTrans, chassisShape);
		btTransform carTr = getCarTransform();
		m_carChassis = localCreateRigidBody(carMass, carTr, compound);
		m_wheelShape = new btCylinderShapeX
			(btVector3(wheelWidth, wheelRadius, wheelRadius));
		m_guiHelper->createCollisionShapeGraphicsObject(m_wheelShape);
		int wheelGraphicsIndex = m_wheelShape->getUserIndex();
		btVector3 co=carTr.getOrigin();
		const float position[4] = 
		{ (float)co.getX(), (float)co.getY(), (float)co.getZ(), 0 };
		const float quaternion[4] = { 0, 0, 0, 1 };
		const float color[4] = { 0.4, 0.4, 0.4, 1 };
		const float scaling[4] = { 1, 1, 1, 1 };
		for (int i = 0; i<4; i++)
		{
			m_wheelInstances[i] = m_guiHelper->registerGraphicsInstance
				(wheelGraphicsIndex, position, quaternion, color, scaling);
		}
		{
			m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
			m_vehicle = new btRaycastVehicle(m_tuning, m_carChassis, m_vehicleRayCaster);

			///never deactivate the vehicle
			m_carChassis->setActivationState(DISABLE_DEACTIVATION);
			m_dynamicsWorld->addAction(m_vehicle);
			bool isFrontWheel = true;
			//choose coordinate system
			int rightIndex = 0;
			int upIndex = 1;
			int forwardIndex = 2;
			m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);
			// <2 causes frequently wheel to be seen
			// >1.1 causes situation where wheels are in air
			btScalar wheelZLocScale = 0.8;
			btScalar caster = 0.05;
			btScalar camber = 0.05;
			btVector3 connectionPointCS0(xhl - (0.3*wheelWidth),
				connectionHeight, zhl - wheelZLocScale*wheelRadius);
			btVector3 wheelDirectionCS0(camber, -1, caster);
			btVector3 wheelAxleCS(-1, 0, 0);
			// left front
			m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

			connectionPointCS0 = btVector3(-xhl + (0.3*wheelWidth),
				connectionHeight, zhl - wheelZLocScale*wheelRadius);
			wheelDirectionCS0 = btVector3(-camber, -1, caster);
			// right front
			m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

			connectionPointCS0 = btVector3(-xhl + (0.3*wheelWidth),
				connectionHeight, -zhl + wheelZLocScale*wheelRadius);
			wheelDirectionCS0 = btVector3(-camber, -1, -caster);
			isFrontWheel = false;
			// right rear
			m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

			connectionPointCS0 = btVector3(xhl - (0.3*wheelWidth),
				connectionHeight, -zhl + wheelZLocScale*wheelRadius);
			wheelDirectionCS0 = btVector3(camber, -1, -caster);
			// leaft rear
			m_vehicle->addWheel(connectionPointCS0, wheelDirectionCS0, wheelAxleCS,
				suspensionRestLength, wheelRadius, m_tuning, isFrontWheel);

			for (int i = 0; i < m_vehicle->getNumWheels(); i++)
			{
				btWheelInfo& wheel = m_vehicle->getWheelInfo(i);
				wheel.m_suspensionStiffness = suspensionStiffness;
				wheel.m_wheelsDampingRelaxation = suspensionDamping;
				wheel.m_wheelsDampingCompression = suspensionCompression;
				wheel.m_frictionSlip = wheelFriction;
				wheel.m_rollInfluence = rollInfluence;
				wheel.m_maxSuspensionForce = suspensionMaxForce;
			}
		}
	}
	void switchSolver(){
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
		}
		else
		{
			m_constraintSolver = new btSequentialImpulseConstraintSolver();
		}
		m_dynamicsWorld->setConstraintSolver(m_constraintSolver);
	}
};
DemolisherDemo *demo = 0;



#include <stdio.h> //printf debugging


#include "DemolisherDemo.h"


void DemolisherDemo::handlePauseSimulation(Gwen::Controls::Base* control){
	Gwen::Controls::Button* gc =
		static_cast<Gwen::Controls::Button*>(control);
	bool pauseSimulation = PlasticityExampleBrowser::getPauseSimulation();
	pauseSimulation = !pauseSimulation;
	if (!pauseSimulation){
		demo->maxStepCount = LONG_MAX;
	}
	PlasticityExampleBrowser::setPauseSimulation(pauseSimulation);
}

void DemolisherDemo::handleSingleStep(Gwen::Controls::Base* control){
	Gwen::Controls::Button* gc =
		static_cast<Gwen::Controls::Button*>(control);
	demo->maxStepCount = demo->stepCount + 1;
	PlasticityExampleBrowser::setPauseSimulation(false);
}

void DemolisherDemo::setGameBindings(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->gameBindings = cb->IsChecked();
}
void DemolisherDemo::setDisableCollisionsBetweenLinkedBodies
(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->disableCollisionsBetweenLinkedBodies = cb->IsChecked();
}
void DemolisherDemo::setLogDir(Gwen::Controls::Base* control){
	Gwen::Controls::TextBox* cb =
		static_cast<Gwen::Controls::TextBox*>(control);
	char *cp=getChar(cb->GetText());
	if (cp != NULL){
		if (NULL != demo->logDir){
			free(demo->logDir);
		}
		demo->logDir = cp;
	}
}
void DemolisherDemo::setDumpPng(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->dumpPng = cb->IsChecked();
}
void DemolisherDemo::setWheelFriction(Gwen::Controls::Base* control){
	setScalar(control, &(demo->wheelFriction));
	restartHandler(control);
}
void DemolisherDemo::setGravityRampUpTime(Gwen::Controls::Base* control){
	setScalar(control, &(demo->gravityRampUpTime));
	restartHandler(control);
}
void DemolisherDemo::setLpc(Gwen::Controls::Base* control){
	setInt(control, &(demo->lpc));
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
void DemolisherDemo::setDensity(Gwen::Controls::Base* control){
	setScalar(control, &(demo->density));
	restartHandler(control);
}
void DemolisherDemo::setBreakingSpeed(Gwen::Controls::Base* control){
	setScalar(control, &(demo->breakingSpeed));
	restartHandler(control);
}
void DemolisherDemo::setCarMass(Gwen::Controls::Base* control){
	btScalar tv(demo->carMass/1000);
	setScalar(control, &tv);
	demo->carMass = tv *1000;
	restartHandler(control);
}
void DemolisherDemo::setSuspensionStiffness(Gwen::Controls::Base* control){
	setScalar(control, &(demo->suspensionStiffness));
	restartHandler(control);
}
void DemolisherDemo::setSuspensionMaxForce(Gwen::Controls::Base* control){
	setScalar(control, &(demo->suspensionMaxForce));
	restartHandler(control);
}
void DemolisherDemo::setSuspensionDamping(Gwen::Controls::Base* control){
	setScalar(control, &(demo->suspensionDamping));
	restartHandler(control);
}
void DemolisherDemo::setSuspensionCompression(Gwen::Controls::Base* control){
	setScalar(control, &(demo->suspensionCompression));
	restartHandler(control);
}
void DemolisherDemo::setSuspensionRestLength(Gwen::Controls::Base* control){
	setScalar(control, &(demo->suspensionRestLength));
	restartHandler(control);
}
void DemolisherDemo::setMaxEngineForce(Gwen::Controls::Base* control){
	setScalar(control, &(demo->maxEngineForce));
	restartHandler(control);
}
void DemolisherDemo::setDefaultBreakingForce(Gwen::Controls::Base* control){
	setScalar(control, &(demo->defaultBreakingForce));
	restartHandler(control);
}
void DemolisherDemo::setFenceSteelScale(Gwen::Controls::Base* control){
	btScalar tv(demo->fenceSteelScale*100);
	setScalar(control, &tv);
	demo->fenceSteelScale = tv/100;
	restartHandler(control);
}
void DemolisherDemo::setBridgeSteelScale(Gwen::Controls::Base* control){
	btScalar tv(demo->bridgeSteelScale * 100);
	setScalar(control, &tv);
	demo->bridgeSteelScale = tv / 100;
	restartHandler(control);
}
void DemolisherDemo::setGateSteelScale(Gwen::Controls::Base* control){
	btScalar tv(demo->gateSteelScale * 100);
	setScalar(control, &tv);
	demo->gateSteelScale = tv / 100;
	restartHandler(control);
}
void DemolisherDemo::setPoleSteelScale(Gwen::Controls::Base* control){
	btScalar tv(demo->poleSteelScale * 100);
	setScalar(control, &tv);
	demo->poleSteelScale = tv / 100;
	restartHandler(control);
}
void DemolisherDemo::setMaxPlasticStrain(Gwen::Controls::Base* control){
	setScalar(control, &(demo->maxPlasticStrain));
	restartHandler(control);
}
void DemolisherDemo::setMaxPlasticRotation(Gwen::Controls::Base* control){
	setScalar(control, &(demo->maxPlasticRotation));
	restartHandler(control);
}

void DemolisherDemo::restartHandler(Gwen::Controls::Base* control){
	demo->restartRequested = true;
}
void DemolisherDemo::reinit(){
	wheelFriction = 1;
	m_gravity = btVector3(0, -10, 0);
	gravityRampUpTime = 4;
	lpc = 4;
	lsx = 30;
	lsy = 3;
	lsz = 2;
	density = 2000;
	breakingSpeed = 5;
	carMass = defaultCarMass;
	maxEngineForce = 100000;
	defaultBreakingForce = 1000;
	suspensionStiffness = 30;
	suspensionMaxForce = 2000000; // allows static load of 800 tons
	suspensionDamping = .5;
	suspensionCompression=0.3;
	suspensionRestLength=0.8;
	gateSteelScale = 0.05;
	fenceSteelScale = 0.01;
	bridgeSteelScale = 0.05;
	poleSteelScale = 0.01;
	maxPlasticStrain = 0.2;
	maxPlasticRotation = 3;
	gameBindings = true;
	maxStepCount = LONG_MAX;
	resetClocks();
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
	if (m_indexVertexArrays){
		delete m_indexVertexArrays;
		m_indexVertexArrays = 0;
	}
	if (m_vertices){
		delete m_vertices;
		m_vertices = 0;
	}
	if (m_vehicleRayCaster){
		delete m_vehicleRayCaster;
		m_vehicleRayCaster = 0;
	}
	if (m_vehicle){
		delete m_vehicle;
		m_vehicle = 0;
		delete m_wheelShape;
		m_wheelShape = 0;
	}
	if (m_constraintSolver){
		delete m_constraintSolver;
		m_constraintSolver = 0;
	}
	if (m_dynamicsWorld){
		delete m_dynamicsWorld;
		m_dynamicsWorld = 0;
	}
	if (m_overlappingPairCache){
		delete m_overlappingPairCache;
		m_overlappingPairCache = 0;
	}
	if (m_dispatcher){
		delete m_dispatcher;
		m_dispatcher = 0;
	}
	if (m_collisionConfiguration){
		delete m_collisionConfiguration;
		m_collisionConfiguration = 0;
	}
	PlasticityData::setData(0);
}

DemolisherDemo::~DemolisherDemo()
{
	clearParameterUi();
}

void DemolisherDemo::initPhysics()
{	
	hasFullGravity = false;
	if (maxStepCount != LONG_MAX){
		// Single step is active
		maxStepCount = 1;
	}
	else{
		maxStepCount = LONG_MAX;
	}
	stepCount = 0;
	syncedStep = 0;
	stepTime = 0;
	if (calculateMaxEngineForce){
		maxEngineForce = 4*carMass*wheelFriction;
	}
	if (calculateMaxEngineForce){
		defaultBreakingForce = 0.01*carMass;
	}
	if (calculateSuspensionStiffness){
		suspensionStiffness = 30;
	}
	if (calculateMaxSuspensionForce){
		// 4 wheels, so car can take additional load of 2 time of own mass
		suspensionMaxForce = (1+2)*10*carMass/4;
	}
	if (calculateSuspensionDamping){
		suspensionDamping = .5;
	}
	if (calculateSuspensionCompression){
		suspensionCompression = suspensionDamping-0.2;
	}
	{ /* car dimensions are scaled if mass is changed
	  third root would be correct for homogenous solid
	  so it is used
	  */
		btScalar mr=pow(carMass / defaultCarMass,0.33);
		yhl = mr;
		xhl = 1.5*mr;
		zhl = 3*mr;
		wheelRadius = zhl / 3;
		wheelWidth = xhl / 3;
	}
	if (calculateSuspensionRestLength){
		suspensionRestLength = 0.8*yhl;
	}
	connectionHeight = 1.2*yhl;
	bridgeLsx = 5*lsx/6;
	tolerance = 0.003*bridgeLsx;
	bridgeLsy = 0.0007*bridgeLsx*bridgeLsx;
	bridgeLsz = 6 * xhl;
	bridgeSupportY = max(2.5*lsy,2*yhl+wheelRadius);
	b3Printf("bridgeLsx=%.1f, bridgeLsy=%.1f, bridgeLsz=%.1f", 
		bridgeLsx, bridgeLsy, bridgeLsz);
	gateLsx = lsx/3;
	gateLsy = 0.2*lsy;
	gateLsz = 0.2*lsz;
	b3Printf("gateLsx=%.1f, gateLsy=%.1f, gateLsz=%.1f",
		gateLsx, gateLsy, gateLsz);
	poleLsx = 0.1;
	poleLsy = lsx;
	b3Printf("poleLsx=%.1f, poleLsy=%.1f",
		poleLsx, poleLsy);
	int upAxis = 1;
	m_guiHelper->setUpAxis(upAxis);
	btVector3 groundExtents(200,200,200);
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
		mlcp = new btDantzigSolver();
		sol = new btMLCPSolver(mlcp);
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
	{//for iterative solver, it is better to solve multiple objects together, small batches have high overhead
		m_dynamicsWorld ->getSolverInfo().m_minimumSolverBatchSize = 128;
	}
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-3,0));
	//create ground object
	btRigidBody* ground=localCreateRigidBody(0, tr, groundShape);
	ground->setFriction(1);
	halfAreaForDrag = xhl*yhl * 2;
	if (gateSteelScale >= 0){
		addGate();
	}
	if (fenceSteelScale >= 0){
		addFence();
	}
	if (bridgeSteelScale >= 0){
		addBridge();
	}
	if (poleSteelScale >= 0){
		addPole();
	}
	if (wheelFriction >= 0){
		addVehicle();
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
	if (demo->restartRequested || stepCount<1){
		return;
	}
	updatePitch();
	updateYaw();
	updatePauseButtonText();
	setDumpFilename();
	{
		BT_PROFILE("DemolisherDemo::showMessage");
		showMessage();
	}
	updateDashboards();
	{
		BT_PROFILE("m_guiHelper::render");
		m_guiHelper->render(m_dynamicsWorld);
	}
	if (stepCount > syncedStep){
		BT_PROFILE("m_guiHelper::syncPhysicsToGraphics");
		syncWheelsToGraphics();
		m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
		syncedStep = stepCount;
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
}

void DemolisherDemo::stepSimulation(float deltaTime)
{
	if (restartRequested){
		restart();
		restartRequested = false;
	}
	if(m_vehicle){
		updateSteering();
		btScalar kmph = m_vehicle->getCurrentSpeedKmHour();
		bool reverseSteeringForBackWheels = (btFabs(kmph)<40);
		for (int i = 0; i < 4; i++){
			btScalar steering;
			bool frontWheel = false;
			switch (i){
			case 0:
			case 1:
				frontWheel = true;
				break;
			default:
				break;
			}
			if (frontWheel){
				steering = gVehicleSteering;
			}
			else{
				if (reverseSteeringForBackWheels){
					steering = -gVehicleSteering/2;
				}
				else{
					steering = gVehicleSteering/5;
				}
			}
			m_vehicle->setSteeringValue(steering, i);
			// limit speed
			if (btFabs(kmph) < 95 || kmph*gEngineForce<0) {
				m_vehicle->applyEngineForce(gEngineForce, i);
			}
			else{
				m_vehicle->applyEngineForce(0, i);
			}
			m_vehicle->setBrake(gBreakingForce, i);
		}
	}
	if(m_carChassis){
		updateDrag();
		m_carChassis->applyCentralForce(drag);
	}
	if (m_dynamicsWorld)
	{
		int maxSimSubSteps =  10;
		updateGravity();
		btScalar timeStep = (btScalar)deltaTime;
		stepCount += m_dynamicsWorld->stepSimulation(timeStep, maxSimSubSteps, m_fixedTimeStep);
		if (stepCount > maxStepCount){
			PlasticityExampleBrowser::setPauseSimulation(true);
		}
		stepTime += timeStep;
		updateView();
	}
	else{
		kmph = 0;
		mps = 0;
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
	clas = 0;
	clai = 0;
	if (m_carChassis){
		m_carChassis->setCenterOfMassTransform(getCarTransform());
		m_carChassis->setLinearVelocity(btVector3(0, 0, 0));
		m_carChassis->setAngularVelocity(btVector3(0, 0, 0));
		m_dynamicsWorld->getBroadphase()->getOverlappingPairCache()->
			cleanProxyFromPairs(m_carChassis->getBroadphaseHandle(),
			getDynamicsWorld()->getDispatcher());
	}
	if (m_vehicle)
	{
		m_vehicle->resetSuspension();
		for (int i=0;i<m_vehicle->getNumWheels();i++)
		{
			//synchronize the wheels with the (interpolated) chassis worldtransform
			m_vehicle->updateWheelTransform(i,true);
		}
	}
}


bool	DemolisherDemo::keyboardCallback(int key, int state)
{
	bool handled = false;
	CommonWindowInterface * win=m_guiHelper->getAppInterface()->m_window;
	bool isShiftPressed = win->isModifierKeyPressed(B3G_SHIFT);
	bool isControlPressed = win->isModifierKeyPressed(B3G_CONTROL);
	idleClock.reset();
	if (state)
	{
	if (isShiftPressed||gameBindings)
	{
		switch (key) 
			{
			case B3G_LEFT_ARROW : 
				{	
					float increment = 2;
					if (isControlPressed){
						increment = 6;
					}
					setPitchDelta(increment);
					handled = true;
					break;
				}
			case B3G_RIGHT_ARROW : 
				{
					float increment = -2;
					if (isControlPressed){
						increment = 6;
					}
					setPitchDelta(increment);
					handled = true;
					break;
				}
			case B3G_UP_ARROW :
				{
					setYawDelta(1);
					handled = true;
					break;
				}
			case B3G_DOWN_ARROW :
				{
					setYawDelta(-1);
					handled = true;
					break;
				}
			}

	} 
	if (!handled)
	{
		switch (key) 
		{
		case 'a':if (!gameBindings){break;}
		case B3G_LEFT_ARROW :
			{
				setSteeringDelta(steeringIncrement);
				handled = true;
				break;
			}
		case 'd':if (!gameBindings){ break; }
		case B3G_RIGHT_ARROW:
			{
				setSteeringDelta(-steeringIncrement);
				handled = true;
				break;
			}
		case 'w':if (!gameBindings){ break; }
		case B3G_UP_ARROW:
			{
				handled = true;
				gEngineForce = maxEngineForce;
				gBreakingForce = 0.f;
				break;
			}
		case 's':if (!gameBindings){ break; }
		case B3G_DOWN_ARROW:
			{
				handled = true;
				gEngineForce = -maxEngineForce;
				gBreakingForce = 0.f;
				break;
			}
		case 'r':
		{
			handled = true;
			updateEngineForce(0.1);
			break;
		}
		case 'f':
		{
			handled = true;
			updateEngineForce(-0.1);
			break;
		}
		case B3G_F1:
		{
			handled = true;
			setCarPosition(Gate);
			break;
		}
		case B3G_F2:
		{
			handled = true;
			setCarPosition(Fence);
			break;
		}
		case B3G_F3:
		{
			handled = true;
			setCarPosition(Bridge);
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
				switchSolver();
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

	} else /* handling of releases of arrow keys and game bound keys */
	{
		bool isArrow = false;
		switch (key){
			case B3G_UP_ARROW:
			case B3G_DOWN_ARROW:
			case B3G_LEFT_ARROW:
			case B3G_RIGHT_ARROW:
				isArrow = true;
				break;
		}
		bool isCommon = !(gameBindings&&isArrow);
		if (gameBindings || isArrow){
			switch (key)
			{
			case 'w':
			case B3G_UP_ARROW:
			{
				if (isArrow){
					setYawDelta(0.f);
				}
				if (isCommon){
					gEngineForce = 0.f;
					gBreakingForce = defaultBreakingForce;
				}
				handled = true;
				break;
			}
			case 's':
			case B3G_DOWN_ARROW:
			{
				if (isArrow){
					setYawDelta(0.f);
				}
				if (isCommon){
					gEngineForce = 0.f;
					gBreakingForce = defaultBreakingForce;
				}
				handled = true;
				break;
			}
			case 'a':
			case 'd':
			case B3G_LEFT_ARROW:
			case B3G_RIGHT_ARROW:
			{
				if (isArrow){
					setPitchDelta(0.f);
				}
				if (isCommon){
					setSteeringDelta(0.f);
				}
				handled = true;
				break;
			}
			default:
				break;
			}
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
