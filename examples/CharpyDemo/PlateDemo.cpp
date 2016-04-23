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
#include "PlateDemo.h"
#include "LinearMath/btQuickprof.h"
#include <stdio.h> 
#include <string>
#ifdef _WIN32
#include <windows.h>
#endif
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "bt6DofElasticPlastic2Constraint.h"
#include "btElasticPlasticPlate.h"
#include "btElasticPlasticMaterial.h"
#include "../plasticity/PlasticityExampleBrowser.h"
#include "../plasticity/PlasticityData.h"
#include "../plasticity/PlasticityStatistics.h"
#include "../plasticity/PlasticityDebugDrawer.h"

class btCollisionShape;

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
#include <string>

const char * PROFILE_PLATE_SLEEP = "PlateDemo::Sleep";

class PlateDemo : public Gwen::Event::Handler, public CommonRigidBodyBase
{
public:
	class btDiscreteDynamicsWorld* m_dynamicsWorld;
	class PlasticityDebugDrawer* plasticityDebugDrawer=0;
	btDiscreteDynamicsWorld* getDynamicsWorld()
	{
		return m_dynamicsWorld;
	}
	btRigidBody* m_loadBody;
	bool raiseLoad = false;
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& worldTransform, btCollisionShape* colSape);

	GUIHelperInterface* m_guiHelper;
	bool m_useDefaultCamera;

	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	class btTriangleIndexVertexArray*	m_indexVertexArrays;

	btVector3*	m_vertices;
	btScalar lsx, lsz;
	btScalar thickness, supportThickness=0.2;
	int cx,cz;
	btScalar breakingImpulseThreshold;
	btScalar density;
	btScalar initialE=200E9; // Steel
	btScalar E;
	btScalar nu=0.3; // Steel
	btScalar initialFy=200e6;
	btScalar fy;
	int initialNumIterations = 10;
	int numIterations = initialNumIterations;
	btScalar initialFixedTimeStep = 1. / 60 , m_fixedTimeStep;
	bool moveLoad;
	btScalar loadMass, loadRaise, loadRaiseX, loadRaiseZ;
	btScalar maxPlasticRotation;
	btScalar maxPlasticStrain;
	long stepCount, maxStepCount, syncedStep;
	btScalar stepTime;
	bool gameBindings;
	bool solidPlate = true;
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
	Constraint constraintType;
	float		m_cameraHeight;

	float	m_minCameraDistance;
	float	m_maxCameraDistance;
	PlateDemo(CommonExampleOptions & options);

	virtual ~PlateDemo();

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

	virtual void updateUI(); // called at end of stepSimulation
	virtual void renderScene();

	virtual void physicsDebugDraw(int debugFlags);


	void initPhysics();
	void exitPhysics();

	virtual void resetCamera()
	{
		float dist = 5;
		float pitch = -45;
		float yaw = 32;
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
	btElasticPlasticPlate *elasticPlasticPlate=0;
	long displayWait = 50;
	/** Gwen controls handling.
	Just flag changes to avoid need to specify all parent references
	if calls come from Gwen
	*/
	void restartHandler(Gwen::Controls::Base* control);
	void reinit();
	void resetHandler(Gwen::Controls::Base* control);
	float pitchDelta = 0;
	void setPitchDelta(float delta);
	void updatePitch();
	float yawDelta = 0;
	void setYawDelta(float delta);
	void updateYaw();
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
	void setCx(Gwen::Controls::Base* control);
	void setCy(Gwen::Controls::Base* control);
	void setCz(Gwen::Controls::Base* control);
	void setLsx(Gwen::Controls::Base* control);
	void setLsy(Gwen::Controls::Base* control);
	void setLsz(Gwen::Controls::Base* control);
	void setThickness(Gwen::Controls::Base* control);
	void setDensity(Gwen::Controls::Base* control);
	void setE(Gwen::Controls::Base* control);
	void setFy(Gwen::Controls::Base* control);
	void setBreakingImpulseThreshold(Gwen::Controls::Base* control);
	void setLoadMass(Gwen::Controls::Base* control);
	void setLoadRaise(Gwen::Controls::Base* control);
	void setLoadRaiseX(Gwen::Controls::Base* control);
	void setLoadRaiseZ(Gwen::Controls::Base* control);
	void setDefaultBreakingForce(Gwen::Controls::Base* control);
	void setMaxEngineForce(Gwen::Controls::Base* control);
	void setMaxPlasticStrain(Gwen::Controls::Base* control);
	void setMaxPlasticRotation(Gwen::Controls::Base* control);
	void setGameBindings(Gwen::Controls::Base* control);
	void setSolidPlate(Gwen::Controls::Base* control);
	void setDisableCollisionsBetweenLinkedBodies(Gwen::Controls::Base* control);
	void setLogDir(Gwen::Controls::Base* control);
	void setDumpPng(Gwen::Controls::Base* control);
	void updatePauseButtonText();
	Gwen::Controls::Base* pPage;
	Gwen::Controls::Button* pauseButton, *singleStepButton;
	Gwen::Controls::Button* raiseLoadButton;
	void addCollisionBetweenLinkedBodies(){
		Gwen::Controls::Label* label = addLabel("disableCollisions");
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetToolTip("disableCollisionsBetweenLinkedBodies");
		gc->SetPos(gxi, gy);
		gc->SetChecked(disableCollisionsBetweenLinkedBodies);
		gy += gyInc;
		gc->onCheckChanged.Add(pPage, &PlateDemo::setDisableCollisionsBetweenLinkedBodies);
	}
	void addGameBindings(){
		Gwen::Controls::Label* label = addLabel("gameBindings");
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetToolTip("set game style keyboard bindings for e.g. asdw");
		gc->SetPos(gxi, gy);
		gc->SetChecked(gameBindings);
		gy += gyInc;
		gc->onCheckChanged.Add(pPage, &PlateDemo::setGameBindings);
	}
	void addSolidPlate(){
		Gwen::Controls::Label* label = addLabel("solidPlate");
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetToolTip("solid steel plate instead of enforced concrete");
		gc->SetPos(gxi, gy);
		gc->SetChecked(solidPlate);
		gy += gyInc;
		gc->onCheckChanged.Add(pPage, &PlateDemo::setSolidPlate);
	}
	Gwen::Controls::CheckBox* dumpPngGc;
	void addDumpPng(){
		Gwen::Controls::Label* label = addLabel("dumpPng");
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetPos(gxi, gy);
		gc->SetChecked(dumpPng);
		dumpPngGc = gc;
		gy += gyInc;
		gc->onCheckChanged.Add(pPage, &PlateDemo::setDumpPng);
	}
	void addLogDir(){
		Gwen::Controls::Label* label = addLabel("logDir");
		Gwen::Controls::TextBox* gc = new Gwen::Controls::TextBox(pPage);
		gc->SetPos(gxi, gy);
		gc->SetText(logDir);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &PlateDemo::setLogDir);
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
		gc->onPress.Add(pPage, &PlateDemo::handlePauseSimulation);
	}
	void addSingleStepButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"SS");
		gc->SetToolTip(L"Single Step");
		gc->SetPos(gx + wxi, gy);
		gc->SetSize(swxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &PlateDemo::handleSingleStep);
	}
	void addRestartButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Restart");
		gc->SetPos(gx + wxi + swxi, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &PlateDemo::restartHandler);
	}
	void addResetButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Reset");
		gc->SetPos(gx + 2 * wxi + swxi, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gy += gyInc;
		gc->onPress.Add(pPage, &PlateDemo::resetHandler);
	}
	void addRaiseLoadButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Raise load");
		gc->SetPos(gx, gy);
		gc->SetSize(wxi + 10, gyInc - 4);
		gc->onPress.Add(pPage, &PlateDemo::handleRaiseLoad);
	}

	void place(Gwen::Controls::Base* gc){
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
	}
	void addCx(){
		addLabel("cx");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = std::to_string(cx);
		gc->SetToolTip("Parts in x direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setCx);
	}
	void addCz(){
		addLabel("cz");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = std::to_string(cz);
		gc->SetToolTip("Parts in z direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setCz);
	}
	void addLsx(){
		addLabel("lsx");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(lsx, "%.2f");
		gc->SetToolTip("Length in x-direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setLsx);
	}
	void addLsz(){
		addLabel("lsz");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(lsz, "%.2f");
		gc->SetToolTip("Length in z-direction");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setLsz);
	}
	void addThickness(){
		addLabel("thickness");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(thickness, "%.4f");
		gc->SetToolTip("Plate thickness [m]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setThickness);
	}
	void addE(){
		addLabel("E [GPa]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(E / 1e9, "%.2f");
		gc->SetText(text);
		gc->SetToolTip("Young's modulus");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &PlateDemo::setE);
	}
	void addFy(){
		addLabel("fy [MPa]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(fy / 1e6, "%.3f");
		gc->SetText(text);
		gc->SetToolTip("yield stress");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &PlateDemo::setFy);
	}
	void addDensity(){
		addLabel("density");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(density, "%.2f");
		gc->SetToolTip("Plate density [kg/m3]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setDensity);
	}
	void addBreakingImpulseThreshold(){
		addLabel("breakingImpulse");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(breakingImpulseThreshold, "%.0f");
		gc->SetToolTip("breakingImpulseThreshold for load parts [Ns]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setBreakingImpulseThreshold);
	}
	void addMaxPlasticStrain(){
		addLabel("maxPlasticStrain");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(maxPlasticStrain, "%.2f");
		gc->SetToolTip("maxPlasticStrain");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setMaxPlasticStrain);
	}
	void addMaxPlasticRotation(){
		addLabel("maxPlasticRotation");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(maxPlasticRotation, "%.1f");
		gc->SetToolTip("maxPlasticRotation");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setMaxPlasticRotation);
	}
	void addLoadMass(){
		addLabel("load mass");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(loadMass, "%.0f");
		gc->SetToolTip("load mass [kg]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setLoadMass);
	}
	void addLoadRaise(){
		addLabel("load raise");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(loadRaise, "%.2f");
		gc->SetToolTip("load raise [m]");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setLoadRaise);
	}
	void setUiNumIterations(Gwen::Controls::Base* control);
	void addNumIterations(){
		addLabel("iteration count");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = std::to_string(numIterations);
		gc->SetText(text);
		gc->SetToolTip("Number of solver iterations");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &PlateDemo::setUiNumIterations);
	}
	void setUiFixedTimeStep(Gwen::Controls::Base* control);
	void addFixedTimeStep(){
		addLabel("timeStep");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(1000*m_fixedTimeStep,"%.3f");
		gc->SetText(text);
		gc->SetToolTip("fixedTimeStep [ms]");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &PlateDemo::setUiFixedTimeStep);
	}

	Gwen::Controls::TextBoxNumeric* loadRaiseXGc;
	void updateLoadRaiseXGc(){
		std::string text = uif(loadRaiseX, "%.2f");
		loadRaiseXGc->SetText(text);
	}
	void addLoadRaiseX(){
		addLabel("load raise x");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		gc->SetToolTip("load raise x[m] (ad)");
		loadRaiseXGc = gc;
		updateLoadRaiseXGc();
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setLoadRaiseX);
	}
	Gwen::Controls::TextBoxNumeric* loadRaiseZGc;
	void updateLoadRaiseZGc(){
		std::string text = uif(loadRaiseZ, "%.2f");
		loadRaiseZGc->SetText(text);
	}
	void addLoadRaiseZ(){
		addLabel("load raise z");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		loadRaiseZGc = gc;
		updateLoadRaiseZGc();
		std::string text = uif(loadRaiseZ, "%.2f");
		gc->SetToolTip("load raise z[m] (sw)");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &PlateDemo::setLoadRaiseZ);
	}
	void pdatePauseButtonText();
	void handlePauseSimulation(Gwen::Controls::Base* control);
	void handleRaiseLoad(Gwen::Controls::Base* control);
	void handleSingleStep(Gwen::Controls::Base* control);
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
		addCx();
		addCz();
		addLsx();
		addLsz();
		addThickness();
		addE();
		addFy();
		addDensity();
		switch (constraintType){
		case Impulse:
			addBreakingImpulseThreshold();
			addCollisionBetweenLinkedBodies();
			break;
		case ElasticPlastic:
			addMaxPlasticRotation();
			addMaxPlasticStrain();
			addCollisionBetweenLinkedBodies();
			break;
		}
		addLoadMass();
		addLoadRaise();
		addLoadRaiseX();
		addLoadRaiseZ();
		addGameBindings();
		addDumpPng();
		addLogDir();
		addNumIterations();
		addFixedTimeStep();
		addPauseSimulationButton();
		addSingleStepButton();
		addRestartButton();
		addResetButton();
		addRaiseLoadButton();
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
	void prepareAndAdd(btGeneric6DofConstraint *sc){
		sc->setBreakingImpulseThreshold(breakingImpulseThreshold);
		m_dynamicsWorld->addConstraint(sc, disableCollisionsBetweenLinkedBodies);
		for (int i = 0; i < 6; i++){
			sc->setLimit(i, 0, 0); // make fixed
		}
	}
	void addFixedConstraint(btAlignedObjectArray<btRigidBody*> ha){
		// x-direction
		{
			btScalar halfLength = lsx / cx / 2;
			btTransform tra;
			btTransform trb;
			tra.setIdentity();
			trb.setIdentity();
			btVector3 cpos(halfLength, 0, 0);
			tra.setOrigin(cpos);
			trb.setOrigin(-cpos);
			for (int i = 0; i < cx - 1; i++){
				for (int j = 0; j < cz; j++){
					btGeneric6DofConstraint *sc =
						new btGeneric6DofConstraint(*ha[cz*i + j], *ha[cz*(i + 1) + j],
						tra, trb, true);
					prepareAndAdd(sc);
				}
			}
		}
		// z-direction
		{
			btScalar halfLength = lsz / cz / 2;
			btTransform tra;
			btTransform trb;
			tra.setIdentity();
			trb.setIdentity();
			btVector3 cpos(0, 0, halfLength);
			tra.setOrigin(cpos);
			trb.setOrigin(-cpos);
			for (int i = 0; i < cx ; i++){
				for (int j = 0; j < cz-1; j++){
					btGeneric6DofConstraint *sc =
						new btGeneric6DofConstraint(*ha[cz*i + j], *ha[cz*i + j+1],
						tra, trb, true);
					prepareAndAdd(sc);
				}
			}
		}
	}
	int pngNro = 0;
	char dumpFilename[FN_SIZE];
	void setDumpFilename(){
		CommonGraphicsApp * app = PlasticityExampleBrowser::getApp();
		if (dumpPng){
			pngNro++;
			sprintf_s(dumpFilename, FN_SIZE, "%s/plate-%d.png", logDir, pngNro);
			dumpPngGc->SetToolTip(dumpFilename);
			app->dumpNextFrameToPng(dumpFilename);
		}
		else{
			app->dumpNextFrameToPng(NULL);
		}
    }
	btScalar xhl = 0.5, zhl = 0.5;
	btScalar yhl;
	btBoxShape * getLoadShape(){
		switch (cz % 2){
		case 0:
			zhl = lsz /cz;
			break;
		case 1:
			zhl = lsz /cz/2;
			break;
		}
		switch (cx % 2){
		case 0:
			xhl = lsx /cz;
			break;
		case 1:
			xhl = lsx /cx/2;
			break;
		}
		yhl = loadMass / density / 2 / (2 * xhl) / (2 * zhl);
		return new btBoxShape(btVector3(xhl, yhl, zhl));
	}
	void updateLoadRaiseX(btScalar increment);
	void updateLoadRaiseZ(btScalar increment);
	btScalar getLoadRaiseY(){
		return yhl + loadRaise;
	}
	btTransform getLoadRaisePoint(){
		btTransform tr;
		tr.setIdentity();
		tr.setOrigin(btVector3
			(loadRaiseX, getLoadRaiseY(), loadRaiseZ));
		return tr;
	}
	void raiseLoadAction(){
		btVector3 zero(0, 0, 0);
		btTransform tr;
		if (moveLoad){
			tr = getLoadRaisePoint();
			moveLoad = false;
		}
		else{
			tr=m_loadBody->getCenterOfMassTransform();
			tr.getOrigin().setY(getLoadRaiseY());
		}
		m_loadBody->setCenterOfMassTransform(tr);
		m_loadBody->setAngularVelocity(zero);
		m_loadBody->setLinearVelocity(zero);
		m_loadBody->activate();
		raiseLoad = false;
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
	void headerMsg(char * buf){
		pData.push_front(getPlasticityData(buf));
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
		btDiscreteDynamicsWorld *dw = m_dynamicsWorld;
		int numConstraints = dw->getNumConstraints();
		char buf[B_LEN * 2];
		bool headerDone = false;
		int ep2count = 0;
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
			sprintf_s(buf, B_LEN * 2, "%2d %8.1f %5d %5.3f %5.3f %5.3f %5.3f",
				epc->getId(), maxr * 100, maxrd, mpr, cpr, mps, cps);
			infoMsg(buf);
			ep2count++;
		}
		sprintf_s(buf, B_LEN, "stepTime=%4.1f, stepCount=%d, ep2count=%d",
			stepTime, stepCount, ep2count);
		headerMsg(buf);
		PlasticityData::setData(&pData);
	}
};
PlateDemo *demo = 0;

#include <stdio.h> //printf debugging
#include "PlateDemo.h"

void PlateDemo::updatePauseButtonText(){
	bool pauseSimulation = PlasticityExampleBrowser::getPauseSimulation();
	if (pauseSimulation){
		demo->pauseButton->SetText(L"Continue");
	}
	else{
		demo->pauseButton->SetText(L"Pause");
	}
}

void PlateDemo::setUiNumIterations(Gwen::Controls::Base* control){
	setInt(control, &(demo->numIterations));
	if (NULL != demo->m_dynamicsWorld){
		demo->m_dynamicsWorld->getSolverInfo().m_numIterations = numIterations;
	}
	restartHandler(control);
}

void PlateDemo::setUiFixedTimeStep(Gwen::Controls::Base* control){
	btScalar tv(demo->m_fixedTimeStep*1e3);
	setScalar(control, &tv);
	demo->m_fixedTimeStep = tv/1e3;
	restartHandler(control);
}
void PlateDemo::setE(Gwen::Controls::Base* control){
	btScalar tv(demo->E / 1e9);
	setScalar(control, &tv);
	demo->E = tv*1e9;
	restartHandler(control);
}
void PlateDemo::setFy(Gwen::Controls::Base* control){
	btScalar tv(demo->fy / 1e6);
	setScalar(control, &tv);
	demo->fy = tv*1e6;
	restartHandler(control);
}

void PlateDemo::handleSingleStep(Gwen::Controls::Base* control){
	Gwen::Controls::Button* gc =
		static_cast<Gwen::Controls::Button*>(control);
	demo->maxStepCount = demo->stepCount + 1;
	PlasticityExampleBrowser::setPauseSimulation(false);
}

void PlateDemo::handlePauseSimulation(Gwen::Controls::Base* control){
	Gwen::Controls::Button* gc =
		static_cast<Gwen::Controls::Button*>(control);
	bool pauseSimulation = PlasticityExampleBrowser::getPauseSimulation();
	pauseSimulation = !pauseSimulation;
	if (!pauseSimulation){
		demo->maxStepCount = LONG_MAX;
	}
	PlasticityExampleBrowser::setPauseSimulation(pauseSimulation);
}

void PlateDemo::handleRaiseLoad(Gwen::Controls::Base* control){
	Gwen::Controls::Button* gc =
		static_cast<Gwen::Controls::Button*>(control);
	demo->raiseLoad = true;
}
void PlateDemo::updateLoadRaiseX(btScalar increment){
	demo->raiseLoad = true;
	demo->moveLoad = true;
	demo->loadRaiseX += increment;
	demo->updateLoadRaiseXGc();
}
void PlateDemo::updateLoadRaiseZ(btScalar increment){
	demo->raiseLoad = true;
	demo->moveLoad = true;
	demo->loadRaiseZ += increment;
	demo->updateLoadRaiseZGc();
}

void PlateDemo::setGameBindings(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->gameBindings = cb->IsChecked();
}
void PlateDemo::setSolidPlate(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->solidPlate = cb->IsChecked();
}
void PlateDemo::setDisableCollisionsBetweenLinkedBodies
	(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->disableCollisionsBetweenLinkedBodies = cb->IsChecked();
}
void PlateDemo::setLogDir(Gwen::Controls::Base* control){
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
void PlateDemo::setDumpPng(Gwen::Controls::Base* control){
	Gwen::Controls::CheckBox* cb =
		static_cast<Gwen::Controls::CheckBox*>(control);
	demo->dumpPng = cb->IsChecked();
}
void PlateDemo::setCx(Gwen::Controls::Base* control){
	setInt(control, &(demo->cx));
	restartHandler(control);
}
void PlateDemo::setCz(Gwen::Controls::Base* control){
	setInt(control, &(demo->cz));
	restartHandler(control);
}
void PlateDemo::setLsx(Gwen::Controls::Base* control){
	setScalar(control, &(demo->lsx));
	restartHandler(control);
}
void PlateDemo::setLsz(Gwen::Controls::Base* control){
	setScalar(control, &(demo->lsz));
	restartHandler(control);
}
void PlateDemo::setThickness(Gwen::Controls::Base* control){
	setScalar(control, &(demo->thickness));
	restartHandler(control);
}
void PlateDemo::setDensity(Gwen::Controls::Base* control){
	setScalar(control, &(demo->density));
	restartHandler(control);
}
void PlateDemo::setBreakingImpulseThreshold(Gwen::Controls::Base* control){
	setScalar(control, &(demo->breakingImpulseThreshold));
	restartHandler(control);
}
void PlateDemo::setLoadMass(Gwen::Controls::Base* control){
	setScalar(control, &(demo->loadMass));
	restartHandler(control);
}
void PlateDemo::setLoadRaise(Gwen::Controls::Base* control){
	setScalar(control, &(demo->loadRaise));
	restartHandler(control);
}
void PlateDemo::setLoadRaiseX(Gwen::Controls::Base* control){
	setScalar(control, &(demo->loadRaiseX));
	restartHandler(control);
}
void PlateDemo::setLoadRaiseZ(Gwen::Controls::Base* control){
	setScalar(control, &(demo->loadRaiseZ));
	restartHandler(control);
}
void PlateDemo::setMaxPlasticStrain(Gwen::Controls::Base* control){
	setScalar(control, &(demo->maxPlasticStrain));
	restartHandler(control);
}
void PlateDemo::setMaxPlasticRotation(Gwen::Controls::Base* control){
	setScalar(control, &(demo->maxPlasticRotation));
	restartHandler(control);
}

void PlateDemo::restartHandler(Gwen::Controls::Base* control){
	demo->restartRequested = true;
}
void PlateDemo::reinit(){
	cx = 3;
	cz = 3;
	lsx = 3;
	lsz = 3;
	E = initialE;
	fy = initialFy;
	thickness = 0.01;
	breakingImpulseThreshold = 1000;
	loadMass = 5000;
	loadRaise = 0;
	loadRaiseX = 0;
	loadRaiseZ = 0;
	maxPlasticStrain = 0.1;
	maxPlasticRotation = 1;
	driveClock.reset();
	gameBindings = false;
	solidPlate = true;
	numIterations = initialNumIterations;
	m_fixedTimeStep = initialFixedTimeStep;
	if (solidPlate){
		density = 7800;
	}
	else{
		density = 2000;
	}
	maxStepCount = LONG_MAX;
}
void PlateDemo::resetHandler(Gwen::Controls::Base* control){
	demo->reinit();
	restartHandler(control);
}


PlateDemo::PlateDemo(CommonExampleOptions & options)
	:CommonRigidBodyBase(options.m_guiHelper),
	Gwen::Event::Handler(),
	m_guiHelper(options.m_guiHelper),
m_loadBody(0),
m_indexVertexArrays(0),
m_vertices(0),
m_cameraHeight(4.f),
m_minCameraDistance(3.f),
m_maxCameraDistance(10.f)
{
	options.m_guiHelper->setUpAxis(1);
	m_option = options.m_option;
	m_useDefaultCamera = false;
	initOptions();
	initParameterUi();
}


void PlateDemo::exitPhysics()
{
	//cleanup in the reverse order of creation/initialization
	if (elasticPlasticPlate){
		delete elasticPlasticPlate->getMaterial();
		delete elasticPlasticPlate->getShape();
		delete elasticPlasticPlate;
		elasticPlasticPlate = 0;
	}
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
	if (plasticityDebugDrawer != 0){
		delete plasticityDebugDrawer;
		plasticityDebugDrawer = 0;
	}
	PlasticityData::setData(0);
}

PlateDemo::~PlateDemo()
{
	clearParameterUi();
}

void PlateDemo::initPhysics()
{
	bt6DofElasticPlastic2Constraint::resetIdCounter();
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
	btScalar xm(0.4*lsx);
	btScalar zm(0.4*lsz);
	btVector3 groundExtents(lsx+4, 0.5, lsz+4);
	btCollisionShape* groundShape = new btBoxShape(groundExtents);
	m_collisionShapes.push_back(groundShape);
	btVector3 supportExtentsX(0.5*lsx, supportThickness/2, 0.1*lsz);
	btCollisionShape* supportShapeX = new btBoxShape(supportExtentsX);
	m_collisionShapes.push_back(supportShapeX);
	btVector3 supportExtentsZ(0.1*lsx, supportThickness/2, 0.3*lsz);
	btCollisionShape* supportShapeZ = new btBoxShape(supportExtentsZ);
	m_collisionShapes.push_back(supportShapeZ);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);
	btScalar ws = 3 * max(lsx, lsz);
	btVector3 worldMin(-ws,-ws,-ws);
	btVector3 worldMax(ws,ws,ws);
	m_overlappingPairCache = new btAxisSweep3(worldMin,worldMax);
	m_constraintSolver = new btSequentialImpulseConstraintSolver();
	m_dynamicsWorld = new btDiscreteDynamicsWorld
		(m_dispatcher,m_overlappingPairCache,m_constraintSolver,m_collisionConfiguration);
	m_dynamicsWorld ->getSolverInfo().m_minimumSolverBatchSize = 128;
	bool own = true;
	if (own){
		plasticityDebugDrawer = new PlasticityDebugDrawer(PlasticityExampleBrowser::getApp());
		m_dynamicsWorld->setDebugDrawer(plasticityDebugDrawer);
		plasticityDebugDrawer->setDebugMode(
			btIDebugDraw::DBG_DrawWireframe
			+ btIDebugDraw::DBG_DrawAabb
			);
		plasticityDebugDrawer->setTextSize(0.2*ws);
	}
	else{
		m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	}
	// ground
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0, -10 - thickness - max(lsx, lsz), 0));
	localCreateRigidBody(0, tr, groundShape);
	// support objects
	btScalar supportY = -thickness - supportThickness / 2;
	tr.setOrigin(btVector3(xm, supportY, 0));
	localCreateRigidBody(0, tr, supportShapeZ);
	tr.setOrigin(btVector3(-xm, supportY, 0));
	localCreateRigidBody(0, tr, supportShapeZ);
	tr.setOrigin(btVector3(0, supportY, -zm));
	localCreateRigidBody(0, tr, supportShapeX);
	tr.setOrigin(btVector3(0, supportY, zm));
	localCreateRigidBody(0, tr, supportShapeX);
	// load
	btCollisionShape* loadShape = getLoadShape();
	m_collisionShapes.push_back(loadShape);
	m_loadBody = localCreateRigidBody(loadMass, getLoadRaisePoint(), loadShape);
	btTransform localTrans;
	localTrans.setIdentity();
	/// create plates
	switch (constraintType){
	case ElasticPlastic:
	{
		btElasticPlasticMaterial* plateMaterial = new btElasticPlasticMaterial();
		plateMaterial->setE(E);
		plateMaterial->setDensity(density);
		plateMaterial->setFy(fy);
		btTransform localTrans;
		localTrans.setIdentity();
		localTrans.setOrigin(btVector3(0, -thickness / 2, 0));
		btBoxShape* plateShape =
			new btBoxShape(btVector3(lsx / 2, thickness / 2, lsz / 2));
		elasticPlasticPlate = new btElasticPlasticPlate();
		elasticPlasticPlate->setMaterial(plateMaterial);
		elasticPlasticPlate->setShape(plateShape);
		elasticPlasticPlate->setLongCount(cx);
		elasticPlasticPlate->setMiddleCount(cz);
		elasticPlasticPlate->setTransform(localTrans);
		elasticPlasticPlate->join(m_dynamicsWorld);
	}
	break;
	default:
		/**
		* store all sequentially z-direction in inner loop
		*/
		btAlignedObjectArray<btRigidBody*> ha;
		ha.reserve(cx*cz);
		btScalar xlen = lsx / cx;
		btScalar zlen = lsz / cz;
		btCollisionShape* partShape = 
			new btBoxShape(btVector3(xlen / 2, thickness/2, zlen / 2));
		btScalar mass;
		switch (constraintType){
		case Rigid:
			mass = 0;
			break;
		default:
			mass = xlen*zlen*thickness*density;
			break;
		}
		m_collisionShapes.push_back(partShape);
		btScalar xloc = (xlen - lsx) / 2;
		for (int i = 0; i < cx; i++){
			btScalar zloc = (zlen - lsz) / 2;
			for (int j = 0; j < cz; j++){
				btTransform plateTrans;
				plateTrans.setIdentity();
				btVector3 pos = btVector3(xloc, thickness / 2, zloc);
				plateTrans.setOrigin(pos);
				ha.push_back(localCreateRigidBody(mass, plateTrans, partShape));
				zloc += zlen;
			}
			xloc += xlen;
		}
		switch (constraintType){
		case Impulse:
			addFixedConstraint(ha);
			break;
		}
	}
	resetDemo();
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

/** called at end of stepSimulation */
void PlateDemo::updateUI()
{
//	setDumpFilename();
	{
		BT_PROFILE("PlateDemo::showMessage");
		showMessage();
	}
}
void PlateDemo::physicsDebugDraw(int debugFlags)
{
	if (demo->raiseLoad){
		raiseLoadAction();
	}
	updatePitch();
	updateYaw();
	updatePauseButtonText();
	if (m_dynamicsWorld && m_dynamicsWorld->getDebugDrawer())
	{
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(debugFlags);
		m_dynamicsWorld->debugDrawWorld();
	}
#ifdef _WIN32
	btScalar idleTime = idleClock.getTimeSeconds();
	if (idleTime> 10){
		if (displayWait>0){
			BT_PROFILE(PROFILE_PLATE_SLEEP);
			Sleep(displayWait);
		}
	}
#endif
}

void PlateDemo::renderScene()
{	
	if (stepCount > syncedStep){
		BT_PROFILE("m_guiHelper::syncPhysicsToGraphics");
		m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
		syncedStep = stepCount;
	}
	{
		BT_PROFILE("m_guiHelper::render");
		m_guiHelper->render(m_dynamicsWorld);
	}
}

void PlateDemo::stepSimulation(float deltaTime)
{
	if (restartRequested){
		restart();
		restartRequested = false;
	}
	if (m_dynamicsWorld)
	{
		int maxSimSubSteps =  10;
		btScalar timeStep = (btScalar)deltaTime;
		stepCount += m_dynamicsWorld->stepSimulation(timeStep, maxSimSubSteps, m_fixedTimeStep);
		if (stepCount > maxStepCount){
			PlasticityExampleBrowser::setPauseSimulation(true);
		}
		stepTime += timeStep;
	}
	updateUI();
}



void PlateDemo::displayCallback(void) 
{
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();
}


void PlateDemo::clientResetScene()
{
	exitPhysics();
	initPhysics();
}

void PlateDemo::resetDemo()
{
}


bool	PlateDemo::keyboardCallback(int key, int state)
{
	bool handled = false;
	CommonWindowInterface * win=m_guiHelper->getAppInterface()->m_window;
	bool isShiftPressed = win->isModifierKeyPressed(B3G_SHIFT);
	bool isControlPressed = win->isModifierKeyPressed(B3G_CONTROL);
	idleClock.reset();
	if (state)
	{
	if (isShiftPressed||demo->gameBindings)
	{
		switch (key) 
			{
			case B3G_LEFT_ARROW : 
				{	
					float increment = 2;
					if (isControlPressed){
						increment = 6;
					}
					demo->setPitchDelta(increment);
					handled = true;
					break;
				}
			case B3G_RIGHT_ARROW : 
				{
					float increment = -2;
					if (isControlPressed){
						increment = 6;
					}
					demo->setPitchDelta(increment);
					handled = true;
					break;
				}
			case B3G_UP_ARROW :
				{
					demo->setYawDelta(1);
					handled = true;
					break;
				}
			case B3G_DOWN_ARROW :
				{
					demo->setYawDelta(-1);
					handled = true;
					break;
				}
			}

	} 
	if (!handled)
	{
		switch (key) 
		{
		case ' ':
			{
				handled = true;
				demo->raiseLoad = true;
				break;
			}
		case 'a':if (!demo->gameBindings){break;}
		case B3G_LEFT_ARROW :
			{
				demo->updateLoadRaiseX(lsx / cx);
				handled = true;
				break;
			}
		case 'd':if (!demo->gameBindings){ break; }
		case B3G_RIGHT_ARROW:
			{
				demo->updateLoadRaiseX(-lsx/cx);
				handled = true;
				break;
			}
		case 'w':if (!demo->gameBindings){ break; }
		case B3G_UP_ARROW:
			{
				handled = true;
				demo->updateLoadRaiseZ(lsz/cz);
				break;
			}
		case 's':if (!demo->gameBindings){ break; }
		case B3G_DOWN_ARROW:
			{
				handled = true;
				demo->updateLoadRaiseZ(-lsz / cz);
				break;
			}
		case B3G_F7:
			{
				handled = true;
				btDiscreteDynamicsWorld* world = (btDiscreteDynamicsWorld*)demo->m_dynamicsWorld;
				world->setLatencyMotionStateInterpolation(!world->getLatencyMotionStateInterpolation());
				printf("world latencyMotionStateInterpolation = %d\n", world->getLatencyMotionStateInterpolation());
				break;
			}
		case B3G_F5:
			handled = true;
			demo->m_useDefaultCamera = !demo->m_useDefaultCamera;
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
		bool isCommon = !(demo->gameBindings&&isArrow);
		if (demo->gameBindings || isArrow){
			switch (key)
			{
			case 'w':
			case B3G_UP_ARROW:
			{
				if (isArrow){
					demo->setYawDelta(0.f);
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
					demo->setYawDelta(0.f);
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
					demo->setPitchDelta(0.f);
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
void PlateDemo::setYawDelta(float delta){
	demo->yawDelta = delta;
}
void PlateDemo::updateYaw(){
	if (yawDelta == 0.f){
		return;
	}
	CommonCameraInterface* camera =
		PlasticityExampleBrowser::getRenderer()->getActiveCamera();
	float current = camera->getCameraYaw();
	current += yawDelta;
	if (current > -90 && current < 90){
		camera->setCameraYaw(current);
	}
}
void PlateDemo::setPitchDelta(float delta){
	demo->pitchDelta = delta;
}
void PlateDemo::updatePitch(){
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


void PlateDemo::specialKeyboardUp(int key, int x, int y)
{
#if 0
   
#endif
}


void PlateDemo::specialKeyboard(int key, int x, int y)
{
}



btRigidBody* PlateDemo::localCreateRigidBody(btScalar mass, 
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
CommonExampleInterface*    PlateDemoCreateFunc(struct CommonExampleOptions& options)
{
	demo = new PlateDemo(options);
	return demo;

}
