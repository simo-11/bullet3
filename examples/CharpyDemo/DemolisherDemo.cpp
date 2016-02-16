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
#include "bt6DofElasticPlastic2Constraint.h"
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

	bool m_useDefaultCamera;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	class btTriangleIndexVertexArray*	m_indexVertexArrays;

	btVector3*	m_vertices;
	btScalar	maxEngineForce;//this should be engine/velocity dependent
	btScalar	defaultBreakingForce;
	btScalar	wheelFriction;
	btScalar lsx, lsy, lsz;
	btScalar bridgeLsx,bridgeLsy, bridgeLsz;
	btScalar bridgeZ=40;
	btScalar yhl=1;
	int lpc;
	btScalar breakingImpulseThreshold;
	btScalar bridgeSteelScale=30,steelScale;
	btScalar density;
	btScalar carMass;
	btScalar suspensionStiffness, suspensionMaxForce;
	btScalar suspensionDamping;
	btScalar suspensionCompression;
	btScalar suspensionRestLength;
	btScalar steelArea;
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
	int gy;
	int gyInc = 25;
	bool restartRequested;
	bool dropFocus = false;
	GwenUserInterface* gui;
	Gwen::Controls::Canvas* canvas;
	CommonWindowInterface* window;
	int m_option;
	enum Constraint { None = 0, Rigid = 1, Impulse = 2, ElasticPlastic = 3 };
	Constraint constraintType;
	btRaycastVehicle::btVehicleTuning	m_tuning;
	btVehicleRaycaster*	m_vehicleRayCaster;
	btRaycastVehicle*	m_vehicle;
	btCollisionShape*	m_wheelShape;

	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;
	bool useMCLPSolver = true;
	float	wheelRadius = 1;
	float	wheelWidth = 0.6;
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
		float dist = 16;
		float pitch = -45;
		float yaw = 32;
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
	btVector3 lastLocation = btVector3(0, 0, 0);
	btVector3 currentLocation = btVector3(0, 0, 0);
	float lastClock = 0;
	float currentClock = 0;
	bool isMoving(){
		btScalar d2 = currentLocation.distance2(lastLocation);
		bool isMoving = d2 > 1e-4;
		return isMoving;
	}
#define CAM_SMOOTH_SIZE 10
	btVector3 cla[CAM_SMOOTH_SIZE];
	btVector3 scl; // smoothed camera location
	int clai = 0;
	int clas = 0;
	void updateCameraLocation(){
		if (clas > 0){
			scl = scl + currentLocation / clas;
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
	btVector3 speedometerLocation = lastLocation;
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
		currentLocation = m_carChassis->getCenterOfMassPosition();
		updateCameraLocation();
		float timeDelta = now - speedometerUpdated;
		if (timeDelta < updateInterval){
			return;
		}
		mps = (int)(currentLocation.distance(speedometerLocation) / timeDelta);
		kmph = (int)(3.6*currentLocation.distance(speedometerLocation) / timeDelta);
		fps = updateViewCount / timeDelta;
		speedometerLocation = currentLocation;
		speedometerUpdated = now;
		updateViewCount = 0;
	}
	float gVehicleSteering = 0.f;
	float steeringIncrement = 0.04f;
	float steeringClamp = 0.3f;
	float steeringDelta = 0.f;
	void setSteeringDelta(float delta){
		steeringDelta = delta;
	}
	void updateSteering(){
		if (steeringDelta == 0.f){
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
	void setLpc(Gwen::Controls::Base* control);
	void setLsx(Gwen::Controls::Base* control);
	void setLsy(Gwen::Controls::Base* control);
	void setLsz(Gwen::Controls::Base* control);
	void setDensity(Gwen::Controls::Base* control);
	void setBreakingImpulseThreshold(Gwen::Controls::Base* control);
	void setCarMass(Gwen::Controls::Base* control);
	void setDefaultBreakingForce(Gwen::Controls::Base* control);
	void setMaxEngineForce(Gwen::Controls::Base* control);
	void setSuspensionStiffness(Gwen::Controls::Base* control);
	void setSuspensionMaxForce(Gwen::Controls::Base* control);
	void setSuspensionDamping(Gwen::Controls::Base* control);
	void setSuspensionCompression(Gwen::Controls::Base* control);
	void setSuspensionRestLength(Gwen::Controls::Base* control);
	void setSteelArea(Gwen::Controls::Base* control);
	void setMaxPlasticStrain(Gwen::Controls::Base* control);
	void setMaxPlasticRotation(Gwen::Controls::Base* control);
	void setGameBindings(Gwen::Controls::Base* control);
	void setDisableCollisionsBetweenLinkedBodies(Gwen::Controls::Base* control);
	void setLogDir(Gwen::Controls::Base* control);
	void setDumpPng(Gwen::Controls::Base* control);
	Gwen::Controls::Base* pPage;
	Gwen::Controls::Button* pauseButton;
	Gwen::Controls::Label *db11, *db12, *db13, *db14, *db15, *db16;
	Gwen::Controls::Label *db21, *db22, *db23;
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
	void addBreakingImpulseThreshold(){
		addLabel("breakingImpulse");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(breakingImpulseThreshold, "%.0f");
		gc->SetToolTip("breakingImpulseThreshold for load parts [Ns]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setBreakingImpulseThreshold);
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
	void addSteelArea(){
		addLabel("steelArea");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(steelArea, "%.5f");
		gc->SetToolTip("Area of enforcement steel [m2]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setSteelArea);
	}
	void addCarMass(){
		addLabel("car mass");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(carMass, "%.0f");
		gc->SetToolTip("car mass [kg]");
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
		addLpc();
		addLsx();
		addLsy();
		addLsz();
		addDensity();
		switch (constraintType){
		case Impulse:
			addCollisionBetweenLinkedBodies();
			addBreakingImpulseThreshold();
			break;
		case ElasticPlastic:
			addCollisionBetweenLinkedBodies();
			addMaxPlasticRotation();
			addMaxPlasticStrain();
			addSteelArea();
			break;
		}
		addCarMass();
		addSuspensionStiffness();
		addSuspensionMaxForce();
		addSuspensionDamping();
		addSuspensionCompression();
		addSuspensionRestLength();
		addMaxEngineForce();
		addDefaultBreakingForce();
		addWheelFriction();
		addGameBindings();
		addDumpPng();
		addLogDir();
		addPauseSimulationButton();
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
	void addFixedConstraint(btAlignedObjectArray<btRigidBody*> ha, btScalar xlen){
		int loopSize = ha.size() - 1;
		btScalar halfLength = xlen / 2;
		btTransform tra;
		btTransform trb;
		tra.setIdentity();
		trb.setIdentity();
		btVector3 cpos(halfLength, 0, 0);
		tra.setOrigin(cpos);
		trb.setOrigin(-cpos);
		for (int i = 0; i < loopSize; i++){
			btGeneric6DofConstraint *sc =
				new btGeneric6DofConstraint(*ha[i], *ha[i + 1],
				tra, trb, true);
			sc->setBreakingImpulseThreshold(steelScale*breakingImpulseThreshold);
			m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
			for (int i = 0; i < 6; i++){
				sc->setLimit(i, 0, 0); // make fixed
			}
		}
	}
	void addElasticPlasticConstraint(btAlignedObjectArray<btRigidBody*> ha, btScalar xlen){
		int loopSize = ha.size() - 1;
		btScalar E(200E9);
		btScalar G(80E9);
		btScalar fy(200E6);
		bool limitIfNeeded = true;
		btScalar damping(0.1);
		btScalar l4s = xlen;
		btScalar halfLength = l4s / 2;
		btTransform tra;
		btTransform trb;
		tra.setIdentity();
		trb.setIdentity();
		btVector3 cpos(halfLength, 0, 0);
		tra.setOrigin(cpos);
		trb.setOrigin(-cpos);
		btScalar k0(E*steelArea*steelScale / 0.2);
		btScalar k1(E*steelArea*steelScale*lsy / 2.5);
		btScalar k2(E*steelArea*steelScale*lsz / 2.5);
		btScalar m(fy / E);
		btScalar w0(k0*m);
		btScalar w1(k1*fy/E);
		btScalar w2(k2*fy/E);
		for (int i = 0; i < loopSize; i++){
			bt6DofElasticPlastic2Constraint *sc =
				new bt6DofElasticPlastic2Constraint(*ha[i], *ha[i + 1],
				tra, trb);
			sc->setMaxPlasticRotation(maxPlasticRotation);
			sc->setMaxPlasticStrain(maxPlasticStrain);
			sc->setStiffness(2, k0, limitIfNeeded);
			sc->setMaxForce(2, w0);
			sc->setStiffness(0, k1, limitIfNeeded);
			sc->setMaxForce(0, w0/2);
			sc->setStiffness(1, k2, limitIfNeeded);
			sc->setMaxForce(1, w0/2);
			sc->setStiffness(5, k0); 
			sc->setMaxForce(5, w0); 
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
		btScalar x = bridgeLsx;
		btScalar y = bridgeLsy+lsy;
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
	btTransform getCarStartPosition(boolean chassis){
		btScalar sy = suspensionRestLength - 10 * carMass / suspensionStiffness / 4;
		if (sy < 0){
			sy = 0.06;
		}
		btScalar yStart = sy + yhl;
		if (!chassis){
			yStart+=lsy + bridgeLsy;
		}
		btScalar zStart;
		if (chassis){
			zStart = 0;
		}
		else{
			zStart = bridgeZ;
		}
		btTransform startTrans;
		startTrans.setIdentity();
		startTrans.setOrigin(btVector3(0, yStart, zStart));
		return startTrans;
	}

};
DemolisherDemo *demo = 0;



#include <stdio.h> //printf debugging


#include "DemolisherDemo.h"



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
void DemolisherDemo::setBreakingImpulseThreshold(Gwen::Controls::Base* control){
	setScalar(control, &(demo->breakingImpulseThreshold));
	restartHandler(control);
}
void DemolisherDemo::setCarMass(Gwen::Controls::Base* control){
	setScalar(control, &(demo->carMass));
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
void DemolisherDemo::setSteelArea(Gwen::Controls::Base* control){
	setScalar(control, &(demo->steelArea));
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
	maxEngineForce = 100000;
	defaultBreakingForce = 1000;
	wheelFriction = 0.8;
	lpc = 5;
	lsx = 30;
	lsy = 3;
	lsz = 2;
	bridgeLsx = 2 * lsx;
	bridgeLsy = 0.3*lsy;
	bridgeLsz = 6 * lsz;
	density = 2000;
	breakingImpulseThreshold = 50000;
	carMass = 50000;
	suspensionStiffness=30;
	suspensionMaxForce = 2000000; // allows static load of 800 tons
	suspensionDamping = .5;
	suspensionCompression=0.3;
	suspensionRestLength=0.8;
	steelArea = 0.001;
	maxPlasticStrain = 0.1;
	maxPlasticRotation = 1;
	gameBindings = true;
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
	float xhl = 1.66, zhl = 3.37;
	halfAreaForDrag = xhl*yhl * 2;
	btCollisionShape* chassisShape = new btBoxShape(btVector3(xhl,yhl,zhl));
	m_collisionShapes.push_back(chassisShape);
	btCompoundShape* compound = new btCompoundShape();
	m_collisionShapes.push_back(compound);
	btTransform localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(0, yhl, 0));
	compound->addChildShape(localTrans, chassisShape);
	m_carChassis = localCreateRigidBody(carMass, getCarStartPosition(false), compound);
	m_wheelShape = new btCylinderShapeX(btVector3(wheelWidth,wheelRadius,wheelRadius));

	m_guiHelper->createCollisionShapeGraphicsObject(m_wheelShape);
	int wheelGraphicsIndex = m_wheelShape->getUserIndex();

	const float position[4]={0,10,10,0};
	const float quaternion[4]={0,0,0,1};
	const float color[4]={0.4,0.4,0.4,1};
	const float scaling[4] = {1,1,1,1};

	for (int i=0;i<4;i++)
	{
		m_wheelInstances[i] = m_guiHelper->registerGraphicsInstance
			(wheelGraphicsIndex, position, quaternion, color, scaling);
	}

	/// create load parts
	{
		steelScale = btScalar(1);
		btAlignedObjectArray<btRigidBody*> ha;
		btScalar xlen = lsx / lpc;
		btCollisionShape* loadShape = new btBoxShape(btVector3(xlen/2, lsy / 2, lsz / 2));
		btScalar mass;
		switch (constraintType){
		case Rigid:
			mass = 0;
			break;
		default:
			mass = lsx*lsy*lsz*density / lpc;
			break;
		}
		m_collisionShapes.push_back(loadShape);
		btScalar xloc = (xlen-lsx)/2;
		for (int i = 0; i < lpc; i++){
			btTransform loadTrans;
			loadTrans.setIdentity();
			btVector3 pos = btVector3(xloc, lsy / 2, 5);
			loadTrans.setOrigin(pos);
			ha.push_back(localCreateRigidBody(mass, loadTrans, loadShape));
			xloc += xlen;
		}
		switch (constraintType){
		case Impulse:
			addFixedConstraint(ha,xlen);
			break;
		case ElasticPlastic:
			addElasticPlasticConstraint(ha,xlen);
			break;
		}
	}
	/// create bridge parts
	{
		steelScale = btScalar(bridgeSteelScale);
		btAlignedObjectArray<btRigidBody*> ha;
		btScalar xlen = bridgeLsx / lpc;
		btCollisionShape* partShape = new btBoxShape(btVector3(xlen/ 2, bridgeLsy / 2, bridgeLsz / 2));
		btCollisionShape* supportShape = new btBoxShape(btVector3(xlen / 2, lsy / 2, bridgeLsz / 2));
		btScalar mass;
		switch (constraintType){
		case Rigid:
			mass = 0;
			break;
		default:
			mass = bridgeLsx*bridgeLsy*bridgeLsz*density / lpc;
			break;
		}
		m_collisionShapes.push_back(partShape);
		btScalar xloc = (xlen - bridgeLsx) / 2;
		for (int i = 0; i < lpc; i++){
			btTransform tr;
			tr.setIdentity();
			btVector3 pos = btVector3(xloc, lsy + bridgeLsy / 2, bridgeZ);
			tr.setOrigin(pos);
			ha.push_back(localCreateRigidBody(mass, tr, partShape));
			// end supports do not move
			if (i == 0 || i == (lpc - 1)){
				btScalar rxloc;
				switch (i){
				case 0: rxloc = xloc - xlen / 2;
					break;
				default:
					rxloc = xloc + xlen / 2;
					break;
				}
				btTransform tr;
				tr.setIdentity();
				btVector3 pos = btVector3(rxloc, lsy / 2, bridgeZ);
				tr.setOrigin(pos);
				localCreateRigidBody(0, tr, supportShape);
				addRamp(rxloc, bridgeZ);
			}
			xloc += xlen;
		}
		switch (constraintType){
		case Impulse:
			addFixedConstraint(ha,xlen);
			break;
		case ElasticPlastic:
			addElasticPlasticConstraint(ha,xlen);
			break;
		}
	}
	/// create vehicle
	{
		m_vehicleRayCaster = new btDefaultVehicleRaycaster(m_dynamicsWorld);
		m_vehicle = new btRaycastVehicle(m_tuning,m_carChassis,m_vehicleRayCaster);
		
		///never deactivate the vehicle
		m_carChassis->setActivationState(DISABLE_DEACTIVATION);

		m_dynamicsWorld->addAction(m_vehicle);

		float connectionHeight = 1.2f;

	
		bool isFrontWheel=true;

		//choose coordinate system
		int rightIndex = 0;
		int upIndex = 1;
		int forwardIndex = 2;
		m_vehicle->setCoordinateSystem(rightIndex, upIndex, forwardIndex);

		btVector3 connectionPointCS0(xhl-(0.3*wheelWidth),
			connectionHeight,zhl-2*wheelRadius);
		btScalar caster = 0.05;
		btScalar camber = 0.05;
		btVector3 wheelDirectionCS0(camber, -1, caster);
		btVector3 wheelAxleCS(-1, 0, 0);
		// left front
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

		connectionPointCS0 = btVector3(-xhl+(0.3*wheelWidth),
			connectionHeight,zhl-2*wheelRadius);
		wheelDirectionCS0 = btVector3(-camber, -1, caster);
		// right front
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

		connectionPointCS0 = btVector3(-xhl+(0.3*wheelWidth),
			connectionHeight,-zhl+2*wheelRadius);
		wheelDirectionCS0 = btVector3(-camber, -1, -caster);
		isFrontWheel = false;
		// right rear
		m_vehicle->addWheel(connectionPointCS0,wheelDirectionCS0,wheelAxleCS,
			suspensionRestLength,wheelRadius,m_tuning,isFrontWheel);

		connectionPointCS0 = btVector3(xhl-(0.3*wheelWidth),
			connectionHeight,-zhl+2*wheelRadius);
		wheelDirectionCS0 = btVector3(camber, -1, -caster);
		// leaft rear
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
			wheel.m_maxSuspensionForce = suspensionMaxForce;
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
	updatePitch();
	updateYaw();
	updatePauseButtonText();
	updateDashboards();
	setDumpFilename();
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
		m_vehicle->setBrake(gBreakingForce, wheelIndex);
		wheelIndex = 1;
		m_vehicle->setSteeringValue(gVehicleSteering,wheelIndex);
		m_vehicle->setBrake(gBreakingForce, wheelIndex);

	}
	{
		updateDrag();
		m_carChassis->applyCentralForce(drag);
	}

	float dt = deltaTime;
	
	if (m_dynamicsWorld)
	{
		int maxSimSubSteps =  2;		
		int numSimSteps;
        numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);
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
	m_carChassis->setCenterOfMassTransform(getCarStartPosition(false));
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
	updateSteering();
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
