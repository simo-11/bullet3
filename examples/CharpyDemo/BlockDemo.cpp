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
Based on DemolisherDemo by Simo Nikula 2016-
*/
#define _CRT_SECURE_NO_WARNINGS
#include "BlockDemo.h"
#include "AxisMapper.h"
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
#include "bt6DofElasticPlasticConstraint.h"
#include "bt6DofElasticPlastic2Constraint.h"
#include "btElasticPlasticConstraint.h"
#include "../plasticity/PlasticityExampleBrowser.h"
#include "../plasticity/PlasticityData.h"
#include "../plasticity/PlasticityStatistics.h"
#include "../plasticity/PlasticityDebugDrawer.h"
#include "BulletDynamics/ConstraintSolver/btSliderConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../ExampleBrowser/GwenGUISupport/gwenUserInterface.h"
#include "../ExampleBrowser/GwenGUISupport/gwenInternalData.h"
#include "../RenderingExamples/TimeSeriesCanvas.h"
#include "Gwen/Utility.h"
const char * PROFILE_TENSION_SLEEP = "BlockDemo::Sleep";

#define CDBG_CALLBACK

class BlockDemo : public Gwen::Event::Handler, public CommonRigidBodyBase
{
public:
	class btDiscreteDynamicsWorld* m_dynamicsWorld;
	btDiscreteDynamicsWorld* getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

	GUIHelperInterface* m_guiHelper;

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
	btScalar lsx, lsy, lsz;
	btRigidBody* m_body=0;
	btTypedConstraint* m_constraint=0;
	btScalar maxImpulse;
	btJointFeedback jf;
	AxisMapper* axisMapper=0;
	long stepCount,maxStepCount,syncedStep;
	btScalar stepTime,targetTime,gravityRampUpTime;
	btVector3 m_gravity;
	btScalar frequencyRatio;
	btScalar E,fy,G;
	btScalar damping=0.9; // 1= no damping, 0=full damping
	btScalar damping2 = 0.1; // 0= no damping, 1=critically damped, >1 overdamped
	bool hasFullGravity;
	int intFreq = 60;
	btScalar m_fixedTimeStep = btScalar(1) / btScalar(intFreq);
	int maxSimSubSteps;
	btScalar blockSteelScale;
	btScalar density;
	btScalar maxPlasticRotation;
	btScalar maxPlasticStrain;
	bool disableCollisionsBetweenLinkedBodies = true;
	bool dumpPng = false;
	bool useCcd, limitIfNeeded, useTs;
	int shootCount = 0;
	btScalar ammoVelocity;
	void shoot();
	btBoxShape* ammoShape = 0;
	btVector4 ammoColor = btVector4(0.5, 0.5, 0.5, 0.5);
	btScalar loadCcdSweptSphereRadius,loadCcdMotionThreshold;
	btScalar ammoCcdSweptSphereRadius, ammoCcdMotionThreshold;
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
	enum Constraint { None = 0, Rigid = 1, Spring = 2, 
		Impulse = 3, Spring2 = 4, SingleDof=5,
		ElasticPlastic = 6, ElasticPlastic2 = 7 };
	Constraint constraintType;
	enum Orientation {
		X = 0, Y = 1
	};
	Orientation orientationType;
	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;
	bool useMCLPSolver = false;
	BlockDemo(CommonExampleOptions & options);

	virtual ~BlockDemo();

	virtual void stepSimulation(float deltaTime);

	virtual void	resetDemo();

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
		float dist = 14;
		float pitch = 15;
		float yaw = 13;
		float targetPos[3] = { 0, 0, 0 };
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
	void resetClocks(){
		driveClock.reset();
	}
	btScalar sumContactImpulses(){
		btScalar totalImpact = btScalar(0);
		btDispatcher *dispatcher = m_dynamicsWorld->getDispatcher();
		int numManifolds = dispatcher->getNumManifolds();
		for (int i = 0; i < numManifolds; i++)
		{
			btPersistentManifold* contactManifold = dispatcher->getManifoldByIndexInternal(i);
			int numContacts = contactManifold->getNumContacts();
			for (int j = 0; j < numContacts; j++)
			{
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				totalImpact += pt.m_appliedImpulse;
			}
		}
		return totalImpact;
	}
	void updateView(){
		if (m_constraint){
			switch (orientationType){
			case X:
				c_force = bt6DofElasticPlastic2Constraint::getMaxAbsMoment(jf,2);
				break;
			case Y:
				c_force = bt6DofElasticPlastic2Constraint::getMaxAbsForce(jf,1);
				break;
			}
			c_impulse = btFabs(c_force*m_fixedTimeStep);
			c_percent = 100.*c_impulse/maxImpulse;
		}
		else{
			c_impulse = sumContactImpulses();
			c_force = c_impulse / m_fixedTimeStep;
		}
		if (m_body){
			switch (orientationType){
			case X:
			{
				btTransform tr = m_body->getWorldTransform();
				btScalar yaw, pitch, roll;
				tr.getBasis().getEulerYPR(yaw, pitch, roll);
				b_location = yaw;
				b_velocity = m_body->getAngularVelocity().z();
			}
				break;
			case Y:
				b_location = m_body->getWorldTransform().getOrigin().y() - yStart;
				b_velocity = m_body->getLinearVelocity().y();
				break;
			}
		}
		if (useTs){
			switch (orientationType){
			case X:
				updateXTimeSeries();
				break;
			case Y:
				updateYTimeSeries();
				break;
			}
			updateErrorTimeSeries();
		}
	}
	void resetView(){
		c_impulse = 0;
		c_percent = 0;
		b_location = 0;
		b_velocity = 0;
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
		btVector3 vel = m_body->getLinearVelocity();
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
	void setMaxSimSubSteps(Gwen::Controls::Base* control);
	void setLpc(Gwen::Controls::Base* control);
	void setLsx(Gwen::Controls::Base* control);
	void setLsy(Gwen::Controls::Base* control);
	void setLsz(Gwen::Controls::Base* control);
	void setDensity(Gwen::Controls::Base* control);
	void setE(Gwen::Controls::Base* control);
	void setFrequencyRatio(Gwen::Controls::Base* control);
	void setLoadCcdSweptSphereRadius(Gwen::Controls::Base* control);
	void setLoadCcdMotionThreshold(Gwen::Controls::Base* control);
	void setAmmoCcdSweptSphereRadius(Gwen::Controls::Base* control);
	void setAmmoCcdMotionThreshold(Gwen::Controls::Base* control);
	void setAmmoVelocity(Gwen::Controls::Base* control);
	void setFy(Gwen::Controls::Base* control);
	void setBlockSteelScale(Gwen::Controls::Base* control);
	void setMaxPlasticStrain(Gwen::Controls::Base* control);
	void setMaxPlasticRotation(Gwen::Controls::Base* control);
	void handleShoot(Gwen::Controls::Base* control);
	void handlePauseSimulation(Gwen::Controls::Base* control);
	void handleSingleStep(Gwen::Controls::Base* control);
	void setDisableCollisionsBetweenLinkedBodies(Gwen::Controls::Base* control);
	void setLogDir(Gwen::Controls::Base* control);
	void setDumpPng(Gwen::Controls::Base* control);
	void setUseCcd(Gwen::Controls::Base* control);
	void setUseTs(Gwen::Controls::Base* control);
	void setLimitIfNeeded(Gwen::Controls::Base* control);
	Gwen::Controls::Base* pPage;
	Gwen::Controls::Button* pauseButton, *singleStepButton;
	Gwen::Controls::Label *db11, *db12, *db13, *db14, *db15, *db16;
	Gwen::Controls::Label *db21, *db22, *db23;
	Gwen::Controls::Label *db31, *db32;
	int fps = 0;
	btScalar c_impulse = 0, c_force=0,
		c_percent=0, 
		b_location=0, b_velocity=0, yStart=0;
	TimeSeriesCanvas *tsYLocation=0, *tsYVelocity=0, *tsYForce=0;
	TimeSeriesCanvas *tsZRLocation = 0, *tsZRVelocity = 0, *tsZMoment = 0, *tsXForce=0;
	TimeSeriesCanvas *tsVelError = 0,*tsPosError=0;
	int maxErrorIndex = 0;
#ifdef CDBG_CALLBACK
#define C_ERROR_SIZE 9
	float velError[C_ERROR_SIZE];
	float posError[C_ERROR_SIZE];
#endif
	btScalar getMass(){
		return btScalar(lsx*lsy*lsz*getDensity(blockSteelScale));
	}
	btScalar getEquilibriumPoint(){
		bool useZero = true;
		if (useZero){
			return 0;
		}
		btScalar f = -10 * getMass();
		btScalar k = axisMapper->getStiffness(1);
		btScalar val(f / k);
		b3Printf("getEquilibriumPoint: %.4f",val);
		return val;
	}
	void updateDashboards(){
		char buffer[UIF_SIZE];
		sprintf_s(buffer, UIF_SIZE, "%-3d ", fps);
		std::string str = std::string(buffer);
		db11->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%4.0f ", c_impulse);
		str = std::string(buffer);
		db13->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%4.0f ", c_percent);
		str = std::string(buffer);
		db15->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%9.6f ", b_location);
		str = std::string(buffer);
		db21->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%9.6f ", b_velocity);
		str = std::string(buffer);
		db22->SetText(str);
		if (m_body){
			sprintf_s(buffer, UIF_SIZE, "%9.0f ", m_body->getGravity().y());
			str = std::string(buffer);
			db23->SetText(str);
		}
		if (useMCLPSolver && sol){
			sprintf_s(buffer, UIF_SIZE, "%5d ", sol->getNumFallbacks());
			str = std::string(buffer);
			db32->SetText(str);
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
			&BlockDemo::setDisableCollisionsBetweenLinkedBodies);
	}

	Gwen::Controls::CheckBox* dumpPngGc;
	void addDumpPng(){
		Gwen::Controls::Label* label = addLabel("dumpPng");
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetPos(gxi, gy);
		gc->SetChecked(dumpPng);
		dumpPngGc = gc;
		gy += gyInc;
		gc->onCheckChanged.Add(pPage, &BlockDemo::setDumpPng);
	}
	template <typename T>
	void addCheckBox(string label, 
			const string & tooltip, bool value, T f){
		addLabel(label);
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetPos(gxi, gy);
		gc->SetChecked(value);
		gy += gyInc;
		gc->onCheckChanged.Add(pPage,f);
		gc->SetToolTip(tooltip);
	}
	void addUseCcd(){
		addCheckBox("useCcd", 
			"Use continous collision detection",
			useCcd, &BlockDemo::setUseCcd);
	}
	void addUseTs(){
		addCheckBox("useTs",
			"plot time seriers",
			useTs, &BlockDemo::setUseTs);
	}
	void addLimitIfNeeded(){
		addCheckBox("limitIfNeeded", 
			"limit spring stiffness if needed",
			limitIfNeeded, &BlockDemo::setLimitIfNeeded);
	}
	template <typename T>
	void addScalar(string label, const string & tooltip, btScalar value, T f){
		addLabel(label);
		Gwen::Controls::TextBoxNumeric* gc = 
			new Gwen::Controls::TextBoxNumeric(pPage);
		string text;
		if (value < 10e-3){
			text = uif(value, "%.6f");
		}
		else if (value>10e3){
			text = uif(value, "%.0f");
		}
		else if (value>10e2){
			text = uif(value, "%.1f");
		}
		else{
			text = uif(value);
		}
		gc->SetToolTip(tooltip);
		gc->SetText(text);
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, f);
	}
	void addCcdValues(){
		addScalar("loadCcdSphere","Size of ccd sphere for load",
			loadCcdSweptSphereRadius,&BlockDemo::setLoadCcdSweptSphereRadius);
		addScalar("loadCcdMotion", "Ccd motion threshold for load",
			loadCcdMotionThreshold, &BlockDemo::setLoadCcdMotionThreshold);
		addScalar("ammoCcdSphere", "Size of ccd sphere for ammo",
			ammoCcdSweptSphereRadius, &BlockDemo::setAmmoCcdSweptSphereRadius);
		addScalar("ammoCcdMotion", "Ccd motion threshold for ammo",
			ammoCcdMotionThreshold, &BlockDemo::setAmmoCcdMotionThreshold);
		addScalar("ammoVelocity", "Velocity of ammo [m/s]",
			ammoVelocity, &BlockDemo::setAmmoVelocity);
	}
	void addFrequencyRatio(){
		addLabel("frequencyRatio");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(frequencyRatio);
		gc->SetToolTip("How many simulation steps are required for spring period");
		gc->SetText(text);
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &BlockDemo::setFrequencyRatio);
	}
	void addLogDir(){
		Gwen::Controls::Label* label = addLabel("logDir");
		Gwen::Controls::TextBox* gc = new Gwen::Controls::TextBox(pPage);
		gc->SetPos(gxi, gy);
		gc->SetText(logDir);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &BlockDemo::setLogDir);
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
		db13->SetText("        ");
		db13->SizeToContents(); // c_impulse
		db13->SetToolTip("constraint impulse");
		db13->SetPos(gx + wxi, gy);
		db14->SetText("Ns");
		db14->SizeToContents();
		db14->SetPos(gx + 18 * wxi / 10, gy);
		db15->SizeToContents(); // c_percent
		db15->SetToolTip("constraint usage"); 
		db15->SetPos(gx + 22 * wxi / 10, gy);
		db16->SetText("%");
		db16->SizeToContents();
		db16->SetPos(gx + 28 * wxi / 10, gy);
		gy += gyInc;
		db21->SizeToContents();
		db21->SetPos(gx, gy);
		db21->SetToolTip("y location");
		db22->SizeToContents();
		db22->SetPos(gx + wxi, gy);
		db22->SetToolTip("y velocity");
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
	void addShootButtons(){
		ammoShape = new btBoxShape(btVector3(lsz / 10, lsz / 10, lsz / 10));
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Shoot");
		gc->SetPos(gx, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &BlockDemo::handleShoot);
		gy += gyInc;
	}
	void addPauseSimulationButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		pauseButton = gc;
		gc->SetPos(gx, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		updatePauseButtonText();
		gc->onPress.Add(pPage, &BlockDemo::handlePauseSimulation);
	}
	void addSingleStepButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"SS");
		gc->SetToolTip(L"Single Step");
		gc->SetPos(gx + wxi, gy);
		gc->SetSize(swxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &BlockDemo::handleSingleStep);
	}
	void addRestartButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Restart");
		gc->SetPos(gx + wxi+swxi, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &BlockDemo::restartHandler);
	}
	void addResetButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Reset");
		gc->SetPos(gx + 2 * wxi+swxi, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gy += gyInc;
		gc->onPress.Add(pPage, &BlockDemo::resetHandler);
	}
	void place(Gwen::Controls::Base* gc){
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
	}
	void addGravityRampUpTime(){
		addLabel("gravityRampUpTime");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		gc->SetToolTip("Gravity ramp up time [s]");
		std::string text = uif(gravityRampUpTime, "%.2f");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setGravityRampUpTime);
	}
	void addMaxSimSubSteps(){
		addLabel("maxSimSubSteps");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		gc->SetToolTip("Maximum number of simulations steps for one render");
		std::string text = std::to_string(maxSimSubSteps);
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setMaxSimSubSteps);
	}
	void addLsx(){
		addLabel("lsx");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(lsx, "%.2f");
		gc->SetToolTip("Load size in x-direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setLsx);
	}
	void addLsy(){
		addLabel("lsy");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(lsy, "%.2f");
		gc->SetToolTip("Load size in y-direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setLsy);
	}
	void addLsz(){
		addLabel("lsz");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(lsz, "%.2f");
		gc->SetToolTip("Load size in z-direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setLsz);
	}
	void addDensity(){
		addLabel("density");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(density, "%.2f");
		gc->SetToolTip("Load density [kg/m3]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setDensity);
	}
	void addE(){
		addLabel("E [MPa]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(E/1e6, "%.0f");
		gc->SetToolTip("Young's modulus");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setE);
	}
	void addFy(){
		addLabel("fy [MPa]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(fy / 1e6, "%.0f");
		gc->SetToolTip("yield stress");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setFy);
	}
	btScalar steelDensity = 7800;
	btScalar getDensity(btScalar steelScale){
		return btScalar(steelScale*steelDensity+(1-steelScale)*density);
	}
	void addMaxPlasticStrain(){
		addLabel("maxPlasticStrain");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(maxPlasticStrain, "%.2f");
		gc->SetToolTip("maxPlasticStrain");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setMaxPlasticStrain);
	}
	void addMaxPlasticRotation(){
		addLabel("maxPlasticRotation");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(maxPlasticRotation, "%.1f");
		gc->SetToolTip("maxPlasticRotation");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setMaxPlasticRotation);
	}
	void addBlockSteelScale(){
		addLabel("block steel%");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(blockSteelScale * 100, "%.6f");
		gc->SetToolTip("Area of enforcement steel [%] [0-100]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &BlockDemo::setBlockSteelScale);
	}
	void BlockDemo::updatePauseButtonText(){
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
		constraintType = (Constraint)(option % 10);
		option /= 10;
		orientationType = (Orientation)(option % 10);
		option /= 10;
		reinit();
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
		addDensity();
		addE();
		addFy();
		switch (constraintType){
		case Spring:
		case Impulse:
		case Spring2:
			if (constraintType == Spring2){
				addLimitIfNeeded();
			}
		case SingleDof:
			addBlockSteelScale();
			break;
		case ElasticPlastic:
			addFrequencyRatio();
			// nobreak
		case ElasticPlastic2:
			if (constraintType == ElasticPlastic2){
				addLimitIfNeeded();
			}
			addMaxPlasticRotation();
			addMaxPlasticStrain();
			addBlockSteelScale();
			break;
		}
		addGravityRampUpTime();
		addMaxSimSubSteps();
		addDumpPng();
		addUseCcd();
		if (useCcd){
			addCcdValues();
		}
		addUseTs();
		addLogDir();
		addPauseSimulationButton();
		addSingleStepButton();
		addRestartButton();
		addResetButton();
		if (useCcd){
			addShootButtons();
		}
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
	btTypedConstraint* addFixedConstraint(btRigidBody* rb, btVector3& cpos){
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(cpos);
		btGeneric6DofConstraint *sc =
		new btGeneric6DofConstraint(*rb, tr, true);
		m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
		for (int i = 0; i < 6; i++){
			sc->setLimit(i, 0, 0); // make fixed
		}
		return sc;
	}
	btTypedConstraint* addSpringConstraint(btRigidBody* rb, btVector3& cpos){
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(cpos);
		btGeneric6DofSpringConstraint *sc =
			new btGeneric6DofSpringConstraint(*rb, tr, true);
		m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
		for (int i = 0; i < 6; i++){
			sc->enableSpring(i,true);
			sc->setStiffness(i, axisMapper->getStiffness(i));
			sc->setDamping(i, damping);
		}
		sc->setEquilibriumPoint(1, getEquilibriumPoint());
		return sc;
	}
	btTypedConstraint* addSliderConstraint(btRigidBody* rb, btVector3& cpos){
		btTransform tr;
		tr.setIdentity();
		tr.setRotation(btQuaternion(0, 0,SIMD_HALF_PI));
		tr.setOrigin(cpos); //debug drawing is easily seen
		btSliderConstraint *sc =
			new btSliderConstraint(*rb, tr, true);
		m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
		sc->setPoweredLinMotor(true);
		sc->setMaxLinMotorForce(axisMapper->getMaxForce(1));
		sc->setTargetLinMotorVelocity(0);
		return sc;
	}
	/**
	create bodyA yourself to get proper reference later
	*/
	btTypedConstraint* addHingeConstraint(btRigidBody* rbB, btVector3& pivot, btVector3 & axis){
		btTransform trA;
		trA.setIdentity();
		trA.setOrigin(rbB->getCenterOfMassPosition() + pivot);
		btCollisionShape* fixedShape = new btSphereShape(lsz/10);
		m_collisionShapes.push_back(fixedShape);
		btRigidBody* rbA = localCreateRigidBody(0, trA, fixedShape);
		btVector3 pivotInA(0,0,0);
		btHingeConstraint *sc =
			new btHingeConstraint(*rbA, *rbB, pivotInA, pivot, axis, axis);
		m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
		sc->enableAngularMotor(true,0,axisMapper->getMaxForce(5)*m_fixedTimeStep);
		sc->setLimit(1, -1); // free
		return sc;
	}
	btTypedConstraint* addSpring2Constraint(btRigidBody* rb, btVector3& cpos){
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(cpos);
		btGeneric6DofSpring2Constraint *sc =
			new btGeneric6DofSpring2Constraint(*rb, tr);
		m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
		for (int i = 0; i < 6; i++){
			sc->enableSpring(i, true);
			sc->setStiffness(i, axisMapper->getStiffness(i), limitIfNeeded);
			sc->setDamping(i, damping2);
		}
		sc->setLinearLowerLimit(btVector3(1, 1, 1));
		sc->setLinearUpperLimit(btVector3(-1, -1, -1));
		sc->setEquilibriumPoint(1, getEquilibriumPoint());
		return sc;
	}
	/**
	*/
	btTypedConstraint* addElasticPlasticConstraint(btRigidBody* rb, btVector3& cpos){
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(cpos);
		bt6DofElasticPlasticConstraint *sc =
			new bt6DofElasticPlasticConstraint(*rb,
			tr, true);
		sc->setUserConstraintType(BPT_EP);
		sc->setMaxPlasticRotation(maxPlasticRotation);
		sc->setMaxPlasticStrain(maxPlasticStrain);
		m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
		for (int i = 0; i<6; i++)
		{
			sc->enableSpring(i, true);
			sc->setDamping(i, damping);
			sc->setStiffness(i, axisMapper->getStiffness(i));
			sc->setMaxForce(i, axisMapper->getMaxForce(i));
		}
		sc->setEquilibriumPoint(1, getEquilibriumPoint());
		b3Printf("fpsLimit[1]=%.1f",sc->getFpsLimit(1));
		sc->setFrequencyRatio(frequencyRatio);
		m_dynamicsWorld->addAction(sc);
		return sc;
	}
	btTypedConstraint* addElasticPlastic2Constraint(btRigidBody* rb, btVector3& cpos){
		btTransform trA;
		trA.setIdentity();
		trA.setOrigin(rb->getCenterOfMassPosition() + cpos);
		btCollisionShape* fixedShape = new btSphereShape(lsz / 10);
		m_collisionShapes.push_back(fixedShape);
		btRigidBody* rbA = localCreateRigidBody(0, trA, fixedShape);
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(cpos);
		trA.setIdentity();
		bt6DofElasticPlastic2Constraint *sc =
			new bt6DofElasticPlastic2Constraint(*rbA,*rb,trA,tr);
		sc->setUserConstraintType(BPT_EP2);
		sc->setMaxPlasticRotation(maxPlasticRotation);
		sc->setMaxPlasticStrain(maxPlasticStrain);
		m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
		for (int i = 0; i<6; i++)
		{
			sc->enableSpring(i, true);
			sc->setDamping(i, damping2);
			sc->setStiffness(i, axisMapper->getStiffness(i),limitIfNeeded);
			sc->setMaxForce(i, axisMapper->getMaxForce(i));
		}
		sc->setEquilibriumPoint(1, getEquilibriumPoint());
		m_dynamicsWorld->addAction(sc);
		return sc;
	}
	int pngNro = 0;
	char dumpFilename[FN_SIZE];
	void setDumpFilename(){
		CommonGraphicsApp * app = PlasticityExampleBrowser::getApp();
		if (dumpPng){
			pngNro++;
			sprintf_s(dumpFilename, FN_SIZE, "%s/tension-%d.png", logDir, pngNro);
			dumpPngGc->SetToolTip(dumpFilename);
			app->dumpNextFrameToPng(dumpFilename);
		}
		else{
			app->dumpNextFrameToPng(NULL);
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
		sprintf_s(buf, B_LEN, "stepTime=%.3f, gravity=%4.1f, stepCount=%d",
			stepTime, m_dynamicsWorld->getGravity().getY(),stepCount);
		infoMsg(buf);
		btDiscreteDynamicsWorld *dw = m_dynamicsWorld;
		bool headerDone = false;
		int numConstraints=dw->getNumConstraints();
		for (int i = 0; i < numConstraints; i++){
			btTypedConstraint* sc = dw->getConstraint(i);
			int type = sc->getUserConstraintType();
			if (type != BPT_EP2 && type!=BPT_EP){
				continue;
			}
			if (!headerDone){
				sprintf_s(buf, B_LEN, "%2s %8s %5s %5s %5s %5s %5s %6s",
					"#", "max%", "m%dof", "mpr", "cpr", "mps", "cps", "reasons");
				infoMsg(buf);
				headerDone = true;
			}
			btElasticPlasticConstraint *epc =
				dynamic_cast<btElasticPlasticConstraint*>(sc);
			char reasons[7];
			reasons[6] = '\0';
			epc->fillLimitReasons(reasons);
			btScalar mpr = epc->getMaxPlasticRotation(),
				cpr = epc->getCurrentPlasticRotation(),
				mps = epc->getMaxPlasticStrain(),
				cps = epc->getCurrentPlasticStrain(),
				maxr = epc->getMaxRatio();
			int	maxrd = epc->getMaxRatioDof();
				sprintf_s(buf, B_LEN*2, "%2d %8.1f %5d %5.3f %5.3f %5.3f %5.3f %6s",
					i, maxr * 100, maxrd, mpr, cpr, mps, cps,reasons);
			infoMsg(buf);
		}
		if (m_constraint != 0 && m_constraint->isEnabled()){
			sprintf_s(buf, B_LEN * 2, "B: FX=%8.3g FY=%8.3g MZ=%8.3g",
				jf.m_appliedForceBodyB.getX(),
				jf.m_appliedForceBodyB.getY(),
				jf.m_appliedTorqueBodyB.getZ());
			infoMsg(buf);
			sprintf_s(buf, B_LEN * 2, "A: FX=%8.3g FY=%8.3g MZ=%8.3g",
				jf.m_appliedForceBodyA.getX(),
				jf.m_appliedForceBodyA.getY(),
				jf.m_appliedTorqueBodyA.getZ());
			infoMsg(buf);
		}
		if (m_body != 0){
			btVector3 loc = m_body->getCenterOfMassPosition();
			sprintf_s(buf, B_LEN * 2, "x=%8.3g y=%8.3g z=%8.3g",
				loc.x(),
				loc.y(),
				loc.z());
			infoMsg(buf);
			btTransform tr = m_body->getCenterOfMassTransform();
			btQuaternion q = tr.getRotation();
			sprintf_s(buf, B_LEN * 2, "q={%8.3g, %8.3g, %8.3g, %8.3g}",
				q.x(),
				q.y(),
				q.z(),
				q.w());
			infoMsg(buf);
			sprintf_s(buf, B_LEN * 2, "velError[0-2]={%8.3g, %8.3g, %8.3g}",
				velError[0],
				velError[1],
				velError[2]);
			infoMsg(buf);
			sprintf_s(buf, B_LEN * 2, "velError[3-5]={%8.3g, %8.3g, %8.3g}",
				velError[3],
				velError[4],
				velError[5]);
			infoMsg(buf);
			if (maxErrorIndex > 5){
				sprintf_s(buf, B_LEN * 2, "velError[6-8]={%8.3g, %8.3g, %8.3g}",
					velError[6],
					velError[7],
					velError[8]);
				infoMsg(buf);
			}
			sprintf_s(buf, B_LEN * 2, "posError[0-2]={%8.3g, %8.3g, %8.3g}",
				posError[0],
				posError[1],
				posError[2]);
			infoMsg(buf);
			sprintf_s(buf, B_LEN * 2, "posError[3-5]={%8.3g, %8.3g, %8.3g}",
				posError[3],
				posError[4],
				posError[5]);
			infoMsg(buf);
			if (maxErrorIndex > 5){
				sprintf_s(buf, B_LEN * 2, "posError[6-8]={%8.3g, %8.3g, %8.3g}",
					posError[6],
					posError[7],
					posError[8]);
				infoMsg(buf);
			}
			btScalar yaw, pitch, roll;
			tr.getBasis().getEulerYPR(yaw, pitch, roll);
			sprintf_s(buf, B_LEN * 2, "yaw=%8.3g pitch=%8.3g roll=%8.3g",
				yaw,
				pitch,
				roll);
			infoMsg(buf);
		}
		PlasticityData::setData(&pData);
	}
	void addBlock(){
		btAlignedObjectArray<btRigidBody*> ha;
		btCollisionShape* loadShape = new btBoxShape(btVector3(lsx / 2, lsy / 2, lsz / 2));
		btScalar mass = getMass();
		m_collisionShapes.push_back(loadShape);
		btTransform loadTrans;
		btVector3 cpos(0,0,0);
		loadTrans.setIdentity();
		switch (orientationType){
		case X:
			yStart = lsx+lsy;
			cpos.setX(-lsx / 2);
			break;
		case Y:
			yStart = lsy;
			cpos.setY(lsy / 2);
			break;
		default:
			assert(0); // Fix if new orientation is added
		}
		btVector3 pos = btVector3(0, yStart, 0);
		loadTrans.setOrigin(pos);
		m_body=localCreateRigidBody(mass, loadTrans, loadShape);
		if (useCcd){
			m_body->setCcdSweptSphereRadius(loadCcdSweptSphereRadius);
			m_body->setCcdMotionThreshold(loadCcdMotionThreshold);
		}
		axisMapper=new AxisMapper(lsx, lsy, lsz, cpos);
		axisMapper->setE(E*blockSteelScale);
		axisMapper->setFy(fy*blockSteelScale);
		switch (constraintType){
		case Rigid:
			m_constraint=addFixedConstraint(m_body, cpos);
			break;
		case Spring:
			m_constraint = addSpringConstraint(m_body, cpos);
			break;
		case Impulse:
			m_constraint = addFixedConstraint(m_body, cpos);
			break;
		case Spring2:
			m_constraint = addSpring2Constraint(m_body, cpos);
			break;
		case SingleDof:
			switch (orientationType){
			case X:
				m_constraint = addHingeConstraint(m_body, cpos, btVector3(0,0,1));
				break;
			case Y:
				m_constraint = addSliderConstraint(m_body, cpos);
				break;
			}
			break;
		case ElasticPlastic:
			m_constraint = addElasticPlasticConstraint(m_body, cpos);
			break;
		case ElasticPlastic2:
			m_constraint = addElasticPlastic2Constraint(m_body, cpos);
			break;
		}
		if (m_constraint != 0){
			m_constraint->enableFeedback(true);
			m_constraint->setJointFeedback(&jf);
			switch (constraintType){
			case Rigid:
				maxImpulse = m_constraint->getBreakingImpulseThreshold();
				break;
			default:
				switch (orientationType){
				case X:
					maxImpulse = m_fixedTimeStep*axisMapper->getMaxForce(5);
					break;
				case Y:
					maxImpulse = m_fixedTimeStep*axisMapper->getMaxForce(1);
					break;
				}
				break;
			}
			switch (constraintType){
			case Impulse:
				m_constraint->setBreakingImpulseThreshold(maxImpulse);
				break;
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
	void addYTimeSeries(){
		int tsWidth = 300, tsHeight = 300;
		CommonGraphicsApp * app = PlasticityExampleBrowser::getApp();
		if (0 == tsYLocation){
			tsYLocation = new TimeSeriesCanvas
				(app->m_2dCanvasInterface, tsWidth, tsHeight, "Y location [m]");
		}
		tsYLocation->setupTimeSeries(2, intFreq, 0);
		tsYLocation->addDataSource("",255,0,0);
		if (0 == tsYVelocity){
			tsYVelocity = new TimeSeriesCanvas
				(app->m_2dCanvasInterface, tsWidth, tsHeight, "Y velocity [m/s]");
		}
		tsYVelocity->setupTimeSeries(6, intFreq, 0);
		tsYVelocity->addDataSource("",0 , 255, 0);
		btScalar maxForce = axisMapper->getMaxForce(1);
		if (0 == tsYForce){
			tsYForce = new TimeSeriesCanvas
				(app->m_2dCanvasInterface, tsWidth, tsHeight, "Y force [kN]");
		}
		tsYForce->setupTimeSeries(maxForce/1000., intFreq, 0);
		tsYForce->addDataSource("", 0, 0, 255);
	}
	void updateYTimeSeries(){
		tsYLocation->insertDataAtCurrentTime(b_location, 0, true);
		tsYLocation->nextTick();
		tsYVelocity->insertDataAtCurrentTime(b_velocity, 0, true);
		tsYVelocity->nextTick();
		if (m_constraint &&  m_constraint->isEnabled()){
			tsYForce->insertDataAtCurrentTime(c_force / 1000., 0, true);
			tsYForce->nextTick();
		}
	}
	void addXTimeSeries(){
		int tsWidth = 300, tsHeight = 300;
		CommonGraphicsApp * app = PlasticityExampleBrowser::getApp();
		if (0 == tsZRLocation){
			tsZRLocation = new TimeSeriesCanvas
				(app->m_2dCanvasInterface, tsWidth, tsHeight, "Z rotation");
		}
		tsZRLocation->setupTimeSeries(0.3, intFreq, 0);
		tsZRLocation->addDataSource("", 255, 0, 0);
		if (0 == tsZRVelocity){
			tsZRVelocity = new TimeSeriesCanvas
				(app->m_2dCanvasInterface, tsWidth, tsHeight, "Z angular velocity [1/s]");
		}
		tsZRVelocity->setupTimeSeries(1, intFreq, 0);
		tsZRVelocity->addDataSource("", 0, 255, 0);
		btScalar maxMoment = axisMapper->getMaxForce(5);
		if (0 == tsZMoment){
			tsZMoment = new TimeSeriesCanvas
				(app->m_2dCanvasInterface, tsWidth, tsHeight, "Z moment [kNm]");
		}
		tsZMoment->setupTimeSeries(maxMoment / 1000., intFreq, 0);
		tsZMoment->addDataSource("   Body", 0, 0, 255);
		tsZMoment->addDataSource("   Wall", 255, 0, 0);
		btScalar maxYForce = axisMapper->getMaxForce(1);
		if (0 == tsYForce){
			tsYForce = new TimeSeriesCanvas
				(app->m_2dCanvasInterface, tsWidth, tsHeight, "Y force [kN]");
		}
		tsYForce->setupTimeSeries(maxYForce / 1000., intFreq, 0);
		tsYForce->addDataSource("   Body", 0, 0, 255);
		tsYForce->addDataSource("   Wall", 255, 0, 0);
		btScalar maxXForce = axisMapper->getMaxForce(0)/10;
		if (0 == tsXForce){
			tsXForce = new TimeSeriesCanvas
				(app->m_2dCanvasInterface, tsWidth, tsHeight, "X force [kN]");
		}
		tsXForce->setupTimeSeries(maxXForce / 1000., intFreq, 0);
		tsXForce->addDataSource("   Body", 0, 0, 255);
		tsXForce->addDataSource("   Wall", 255, 0, 0);
	}
	void updateXTimeSeries(){
		tsZRLocation->insertDataAtCurrentTime(b_location, 0, true);
		tsZRLocation->nextTick();
		tsZRVelocity->insertDataAtCurrentTime(b_velocity, 0, true);
		tsZRVelocity->nextTick();
		if (m_constraint &&  m_constraint->isEnabled()){
			tsZMoment->insertDataAtCurrentTime
				(jf.m_appliedTorqueBodyB[2] / 1000., 0, true);
			tsZMoment->insertDataAtCurrentTime
				(jf.m_appliedTorqueBodyA[2] / 1000., 1, true);
			tsZMoment->nextTick();
			tsYForce->insertDataAtCurrentTime
				(jf.m_appliedForceBodyB[1] / 1000., 0, true);
			tsYForce->insertDataAtCurrentTime
				(jf.m_appliedForceBodyA[1] / 1000., 1, true);
			tsYForce->nextTick();
			tsXForce->insertDataAtCurrentTime
				(jf.m_appliedForceBodyB[0] / 1000., 0, true);
			tsXForce->insertDataAtCurrentTime
				(jf.m_appliedForceBodyA[0] / 1000., 1, true);
			tsXForce->nextTick();
		}
	}
	bool errorScaled = false;
	float maxVelError = 1000, currentMaxVelError;
	void updateMaxVelError(int index){
		float absValue = btFabs(velError[index]);
		if (absValue > maxVelError){
			maxVelError = absValue;
		}
	}
	float maxPosError = 1000, currentMaxPosError;
	void updateMaxPosError(int index){
		float absValue = btFabs(posError[index]);
		if (absValue > maxPosError){
			maxPosError = absValue;
		}
	}
	/**
	*/
	unsigned char getRGBVal(int k1, int b1, int i, int k2, int b2){
		int v = 255*b1 + 255*k1 * i *3/ C_ERROR_SIZE;
		if (v>255){
			v = 255*b2+255*k2*i*3/C_ERROR_SIZE;
		}
		if (v < 0){
			v = 0;
		}
		return (unsigned char)v;

	}
	unsigned char getR(int i){
		return getRGBVal(-1, 1, i,0,0);
	}
	unsigned char getG(int i){
		return getRGBVal(1, 0, i,-1,2);
	}
	unsigned char getB(int i){
		return getRGBVal(1, -1, i,0,0);
	}
	/**
	provide clean scaling value
	*/
	float getScale(float in){
		float s = 1;
		float v=ceilf(s*in);
		return v;
	}
	void addErrorTimeSeries(){
#ifdef CDBG_CALLBACK
		if (0==m_constraint){
			return;
		}
		const char *label[] = {"0","1","2","3","4","5","6","7","8"};
		int tsWidth = 300, tsHeight = 300;
		CommonGraphicsApp * app = PlasticityExampleBrowser::getApp();
		if (0 == tsVelError){
			tsVelError = new TimeSeriesCanvas
				(app->m_2dCanvasInterface, tsWidth, tsHeight, "constraint's velImpulse [Ns]");
		}
		if (0 == tsPosError){
			tsPosError = new TimeSeriesCanvas
				(app->m_2dCanvasInterface, tsWidth, tsHeight, "constraint's posImpulse [Ns]");
		}
		currentMaxVelError = maxVelError;
		tsVelError->setupTimeSeries(getScale(maxVelError), intFreq, 0);
		currentMaxPosError = maxPosError;
		tsPosError->setupTimeSeries(getScale(maxPosError), intFreq, 0);
		for (int i = 0; i < C_ERROR_SIZE; i++){
			unsigned char r = getR(i), g = getG(i), b = getB(i);
			tsVelError->addDataSource(label[i], r, g, b);
			tsPosError->addDataSource(label[i], r, g, b);
		}
		maxVelError = 0;
		maxPosError = 0;
#endif
	}
	void updateErrorTimeSeries(){
#ifdef CDBG_CALLBACK
		if (!tsVelError || !tsPosError){
			return;
		}
		for (int i = 0; i < C_ERROR_SIZE; i++){
			if (btFabs(velError[i])>0.01*currentMaxVelError){
				tsVelError->insertDataAtCurrentTime(velError[i], i, errorScaled);
			}
			if (btFabs(posError[i])>0.01*currentMaxPosError){
				tsPosError->insertDataAtCurrentTime(posError[i], i, errorScaled);
			}
		}
		tsVelError->nextTick();
		tsPosError->nextTick();
#endif
	}

	void deleteTimeSeries(){
		if (tsVelError){
			delete tsVelError;
			tsVelError = 0;
			errorScaled = false;
		}
		if (tsPosError){
			delete tsPosError;
			tsPosError = 0;
		}
		if (tsYLocation){
			delete tsYLocation;
			tsYLocation = 0;
		}
		if (tsYVelocity){
			delete tsYVelocity;
			tsYVelocity = 0;
		}
		if (tsXForce){
			delete tsXForce;
			tsXForce = 0;
		}
		if (tsYForce){
			delete tsYForce;
			tsYForce = 0;
		}
		if (tsZRLocation){
			delete tsZRLocation;
			tsZRLocation = 0;
		}
		if (tsZRVelocity){
			delete tsZRVelocity;
			tsZRVelocity = 0;
		}
		if (tsZMoment){
			delete tsZMoment;
			tsZMoment = 0;
		}
	}
	/**

	*/
	btRigidBody* ammoBody=0;
	void addAmmoBody(){
		if (ammoBody == 0){
			return;
		}
		if (ammoShape->getUserIndex() < 0){
			m_guiHelper->createCollisionShapeGraphicsObject(ammoShape);
		}
		m_guiHelper->createCollisionObjectGraphicsObject(ammoBody, ammoColor);
		ammoBody = 0;
	}

};
BlockDemo *demo = 0;



#include <stdio.h> //printf debugging


#include "BlockDemo.h"


void BlockDemo::handleShoot(Gwen::Controls::Base* control){
	Gwen::Controls::Button* gc =
		static_cast<Gwen::Controls::Button*>(control);
	demo->shootCount++;
}

void BlockDemo::handlePauseSimulation(Gwen::Controls::Base* control){
	Gwen::Controls::Button* gc =
		static_cast<Gwen::Controls::Button*>(control);
	bool pauseSimulation = PlasticityExampleBrowser::getPauseSimulation();
	pauseSimulation = !pauseSimulation;
	if (!pauseSimulation){
		demo->maxStepCount = LONG_MAX;
	}
	PlasticityExampleBrowser::setPauseSimulation(pauseSimulation);
}

void BlockDemo::handleSingleStep(Gwen::Controls::Base* control){
	Gwen::Controls::Button* gc =
		static_cast<Gwen::Controls::Button*>(control);
	demo->maxStepCount = demo->stepCount + 1;
	PlasticityExampleBrowser::setPauseSimulation(false);
}

void BlockDemo::setDisableCollisionsBetweenLinkedBodies
(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->disableCollisionsBetweenLinkedBodies = cb->IsChecked();
}
void BlockDemo::setLogDir(Gwen::Controls::Base* control){
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
void BlockDemo::setDumpPng(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->dumpPng = cb->IsChecked();
}
void BlockDemo::setUseCcd(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->useCcd = cb->IsChecked();
	restartHandler(control);
}
void BlockDemo::setUseTs(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->useTs = cb->IsChecked();
	restartHandler(control);
}
void BlockDemo::setLimitIfNeeded(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->limitIfNeeded = cb->IsChecked();
	restartHandler(control);
}
void BlockDemo::setGravityRampUpTime(Gwen::Controls::Base* control){
	setScalar(control, &(demo->gravityRampUpTime));
	restartHandler(control);
}
void BlockDemo::setMaxSimSubSteps(Gwen::Controls::Base* control){
	setInt(control, &(demo->maxSimSubSteps));
}
void BlockDemo::setLsx(Gwen::Controls::Base* control){
	setScalar(control, &(demo->lsx));
	restartHandler(control);
}
void BlockDemo::setLsy(Gwen::Controls::Base* control){
	setScalar(control, &(demo->lsy));
	restartHandler(control);
}
void BlockDemo::setLsz(Gwen::Controls::Base* control){
	setScalar(control, &(demo->lsz));
	restartHandler(control);
}
void BlockDemo::setDensity(Gwen::Controls::Base* control){
	setScalar(control, &(demo->density));
	restartHandler(control);
}
void BlockDemo::setE(Gwen::Controls::Base* control){
	btScalar tv(demo->E/1e6);
	setScalar(control, &tv);
	demo->E = tv*1e6;
	demo->G = demo->E / 2.6;
	restartHandler(control);
}
void BlockDemo::setLoadCcdSweptSphereRadius(Gwen::Controls::Base* control){
	setScalar(control, &(demo->loadCcdSweptSphereRadius));
}
void BlockDemo::setLoadCcdMotionThreshold(Gwen::Controls::Base* control){
	setScalar(control, &(demo->loadCcdMotionThreshold));
}
void BlockDemo::setAmmoCcdSweptSphereRadius(Gwen::Controls::Base* control){
	setScalar(control, &(demo->ammoCcdSweptSphereRadius));
}
void BlockDemo::setAmmoCcdMotionThreshold(Gwen::Controls::Base* control){
	setScalar(control, &(demo->ammoCcdMotionThreshold));
}
void BlockDemo::setAmmoVelocity(Gwen::Controls::Base* control){
	setScalar(control, &(demo->ammoVelocity));
}
void BlockDemo::setFrequencyRatio(Gwen::Controls::Base* control){
	setScalar(control, &(demo->frequencyRatio));
	restartHandler(control);
}
void BlockDemo::setFy(Gwen::Controls::Base* control){
	btScalar tv(demo->fy / 1e6);
	setScalar(control, &tv);
	demo->fy = tv*1e6;
	restartHandler(control);
}
void BlockDemo::setBlockSteelScale(Gwen::Controls::Base* control){
	btScalar tv(demo->blockSteelScale * 100);
	setScalar(control, &tv);
	demo->blockSteelScale = tv / 100;
	restartHandler(control);
}
void BlockDemo::setMaxPlasticStrain(Gwen::Controls::Base* control){
	setScalar(control, &(demo->maxPlasticStrain));
	restartHandler(control);
}
void BlockDemo::setMaxPlasticRotation(Gwen::Controls::Base* control){
	setScalar(control, &(demo->maxPlasticRotation));
	restartHandler(control);
}

void BlockDemo::restartHandler(Gwen::Controls::Base* control){
	demo->restartRequested = true;
}
void BlockDemo::reinit(){
	m_gravity = btVector3(0, -10, 0);
	gravityRampUpTime = 0;
	switch (orientationType){
	case X:
		lsx = 6;
		lsy = 1;
		blockSteelScale = 0.006;
		break;
	case Y:
		lsx = 1;
		lsy = 3;
		blockSteelScale = 0.0004;
		break;
	}
	btScalar minDist = 1;
	lsz = 1;
	density = 2000;
	maxPlasticStrain = 0.2;
	maxPlasticRotation = 3;
	E = 200E9;
	fy = 200E6;
	G = E / 2.6; /* isotropic nu=0.3 */
	maxStepCount = LONG_MAX;
	frequencyRatio = 10;
	resetClocks();
	useCcd = false;
	useTs = false;
	limitIfNeeded = true;
	ammoVelocity = 300;
	ammoCcdMotionThreshold=minDist/2;
	ammoCcdSweptSphereRadius = minDist / 10;
	loadCcdMotionThreshold = 1e-3;
	loadCcdSweptSphereRadius = minDist/2;
	maxSimSubSteps = 1;
}

void BlockDemo::resetHandler(Gwen::Controls::Base* control){
	demo->reinit();
	restartHandler(control);
}


BlockDemo::BlockDemo(CommonExampleOptions & options)
	:CommonRigidBodyBase(options.m_guiHelper),
	Gwen::Event::Handler(),
	m_guiHelper(options.m_guiHelper),
m_indexVertexArrays(0),
m_vertices(0),
m_cameraHeight(4.f),
m_minCameraDistance(3.f),
m_maxCameraDistance(10.f)
{
	options.m_guiHelper->setUpAxis(1);
	m_option = options.m_option;
	m_body = 0;
	m_useDefaultCamera = false;
	initOptions();
	initParameterUi();
}


void BlockDemo::exitPhysics()
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
	if (ammoShape){
		delete ammoShape;
		ammoShape = 0;
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
	m_body = 0;
	m_constraint = 0;
	if (axisMapper){
		delete axisMapper;
		axisMapper = 0;
	}
	resetView();
	PlasticityData::setData(0);
	deleteTimeSeries();
}

BlockDemo::~BlockDemo()
{
	clearParameterUi();
}

void BlockDemo::initPhysics()
{	
	hasFullGravity = false;
	maxErrorIndex = 0;
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
	int upAxis = 1;
	m_guiHelper->setUpAxis(upAxis);
	btVector3 groundExtents(10,2,10);
	groundExtents[upAxis]=3;
	btCollisionShape* groundShape = new btBoxShape(groundExtents);
	m_collisionShapes.push_back(groundShape);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btVector3 worldMin(-3*lsx,-3*lsy,-3*lsz);
	btVector3 worldMax(3 * lsx, 3 * lsy, 3 * lsz);
	m_overlappingPairCache = new btDbvtBroadphase();
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
	if (blockSteelScale >= 0){
		addBlock();
	}
	resetDemo();
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
	if (useTs){
		switch (orientationType){
		case X:
			addXTimeSeries();
			break;
		case Y:
			addYTimeSeries();
			break;
		default:
			assert(0); // Add if new orientation is added
		}
		addErrorTimeSeries();
	}
}

void BlockDemo::physicsDebugDraw(int debugFlags)
{
	if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		m_dynamicsWorld->debugDrawWorld();
		PlasticityDebugDrawer::drawPlasticConstraints(m_dynamicsWorld);
	}
}

void BlockDemo::renderScene()
{
	if (demo->restartRequested || stepCount<1){
		return;
	}
	updatePitch();
	updateYaw();
	updatePauseButtonText();
	setDumpFilename();
	{
		BT_PROFILE("BlockDemo::showMessage");
		showMessage();
	}
	updateDashboards();
	{
		BT_PROFILE("m_guiHelper::render");
		m_guiHelper->render(m_dynamicsWorld);
	}
	if (stepCount > syncedStep){
		BT_PROFILE("m_guiHelper::syncPhysicsToGraphics");
		addAmmoBody();
		m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
		syncedStep = stepCount;
	}
	btScalar idleTime = idleClock.getTimeSeconds();
	if ( idleTime> 10 && !isMoving()){
#ifdef _WIN32
		if (displayWait>0){
			BT_PROFILE(PROFILE_TENSION_SLEEP);
			Sleep(displayWait);
		}
#endif
	}
}

void BlockDemo::stepSimulation(float deltaTime)
{
	if (restartRequested){
		restart();
		restartRequested = false;
	}
	if (m_dynamicsWorld)
	{
		updateGravity();
		if (useCcd){
			shoot();
		}
		btScalar timeStep = (btScalar)deltaTime;
		if (stepCount + (deltaTime/m_fixedTimeStep)>=maxStepCount){
			timeStep = m_fixedTimeStep;
		}else if (maxSimSubSteps*m_fixedTimeStep<deltaTime){
			timeStep = maxSimSubSteps*m_fixedTimeStep;
		}
		targetTime = stepTime + timeStep;
		stepCount += m_dynamicsWorld->stepSimulation
			(timeStep, maxSimSubSteps, m_fixedTimeStep);
		if (stepCount >= maxStepCount){
			PlasticityExampleBrowser::setPauseSimulation(true);
		}
		stepTime += timeStep;
		updateView();
	}
	else{
		resetView();
	}
}

/**
*/
void BlockDemo::shoot(){
	if (shootCount < 1){
		return;
	}
	btScalar mass = getMass()/100;
	btScalar xStart = -3 * lsx;
	btTransform trans;
	trans.setIdentity();
	switch (orientationType){
	case X:
	{
		/*
		*/
		const btTransform tr = m_body->getCenterOfMassTransform();
		yStart = (tr*btVector3(lsx / 10, -lsy / 2, 0)).y();
	}
		break;
	case Y:
		yStart = 2*lsy/3;
		break;
	default:
		assert(0); // Fix if new orientation is added
	}
	btVector3 pos = btVector3(xStart, yStart, 0);
	trans.setOrigin(pos);
	btRigidBody* body = localCreateRigidBody(mass, trans, ammoShape);
	body->setCcdSweptSphereRadius(ammoCcdSweptSphereRadius);
	body->setCcdMotionThreshold(ammoCcdMotionThreshold);
	btVector3 linVel(ammoVelocity, 0, 0);
	body->setLinearVelocity(linVel);
	ammoBody = body;
	shootCount--;
}



void BlockDemo::displayCallback(void) 
{
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
}


void BlockDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void BlockDemo::resetDemo()
{
	clas = 0;
	clai = 0;
	for (int i = 0; i < C_ERROR_SIZE; i++){
		velError[i] = 0;
		posError[i] = 0;
	}
}


bool	BlockDemo::keyboardCallback(int key, int state)
{
	bool handled = false;
	CommonWindowInterface * win=m_guiHelper->getAppInterface()->m_window;
	bool isShiftPressed = win->isModifierKeyPressed(B3G_SHIFT);
	bool isControlPressed = win->isModifierKeyPressed(B3G_CONTROL);
	idleClock.reset();
	if (state)
	{
	if (true)
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
		bool isCommon = !(isArrow);
		if (isArrow){
			switch (key)
			{
			case 'w':
			case B3G_UP_ARROW:
			{
				if (isArrow){
					setYawDelta(0.f);
				}
				if (isCommon){
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

void BlockDemo::specialKeyboardUp(int key, int x, int y)
{
#if 0
   
#endif
}


void BlockDemo::specialKeyboard(int key, int x, int y)
{
}



btRigidBody* BlockDemo::localCreateRigidBody(btScalar mass, 
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
CommonExampleInterface*    BlockDemoCreateFunc(struct CommonExampleOptions& options)
{
	demo = new BlockDemo(options);
	return demo;

}

#ifdef CDBG_CALLBACK
int cdbgActive = 1;
void
cdbgCallback(
int i,
int j,
btTypedConstraint* constraint,
const btTypedConstraint::btConstraintInfo1* info1,
btTypedConstraint::btConstraintInfo2* info2,
btScalar positionalError,
btScalar velocityError,
btSolverConstraint* solverConstraint
){
	if (0==demo || 0==demo->m_constraint){
		return;
	}
	if (!cdbgActive){
		return;
	}
	if (i != 0){
		return;
	}
	btAssert(j<C_ERROR_SIZE);
	if (demo->maxErrorIndex < j){
		demo->maxErrorIndex = j;
	}
	if (velocityError == 0 && positionalError == 0){
		demo->velError[j] = 0;
		demo->posError[j] = 0;
		return;
	}
	btScalar jInv = solverConstraint->m_jacDiagABInv;
	demo->velError[j] = velocityError*jInv;
	demo->posError[j] = positionalError*jInv;
	demo->updateMaxVelError(j);
	demo->updateMaxPosError(j);
}
#endif