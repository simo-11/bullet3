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

	bool m_useDefaultCamera;
	btAlignedObjectArray<btCollisionShape*> m_collisionShapes;

	class btBroadphaseInterface*	m_overlappingPairCache;

	class btCollisionDispatcher*	m_dispatcher;

	class btConstraintSolver*	m_constraintSolver;

	class btDefaultCollisionConfiguration* m_collisionConfiguration;

	class btTriangleIndexVertexArray*	m_indexVertexArrays;

	btVector3*	m_vertices;
	btScalar	maxEngineForce;//this should be engine/velocity dependent
	btScalar	maxBreakingForce;
	btScalar	wheelFriction;
	btScalar lsx, lsy, lsz;
	int lpc=1;
	btScalar density;
	btScalar carMass;
	btScalar suspensionStiffness,suspensionMaxForce;
	btScalar suspensionDamping;
	btScalar suspensionCompression;
	btScalar suspensionRestLength;
	float	defaultBreakingForce = 10.f;
	float	gEngineForce = 0.f;
	float	gBreakingForce = 100.f;
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
	btVector3 lastLocation=btVector3(0,0,0);
	btVector3 currentLocation = btVector3(0, 0, 0);
	float lastClock=0;
	float currentClock=0;
	bool isMoving(){
		btScalar d2 = currentLocation.distance2(lastLocation);
		bool isMoving = d2>1e-4;
		return isMoving;
	}
	/** update fps and kmph after interval seconds have passed */
	float speedometerUpdated;
	btVector3 speedometerLocation=lastLocation;
	float updateInterval = 0.3;
	int updateViewCount = 0;
	void updateView(){
		updateViewCount++;
		float now = driveClock.getTimeSeconds();
		float timeDelta = now - lastClock;
		lastClock = currentClock;
		currentClock = now;
		lastLocation = currentLocation;
		currentLocation = m_carChassis->getCenterOfMassPosition();
		btVector3 loc = (lastLocation + currentLocation)/2;
		CommonCameraInterface* camera = 
			PlasticityExampleBrowser::getRenderer()->getActiveCamera();
		camera->setCameraTargetPosition(loc.x(), loc.y(), loc.z());
		timeDelta = now - speedometerUpdated;
		if (timeDelta < updateInterval){
			return;
		}
		kmph = (int)(3.6*currentLocation.distance(speedometerLocation) / timeDelta);
		fps = updateViewCount / timeDelta;
		speedometerLocation = currentLocation;
		speedometerUpdated = now;
		updateViewCount = 0;
	}
	void updatePitch(float delta){
		CommonCameraInterface* camera =
			PlasticityExampleBrowser::getRenderer()->getActiveCamera();
		float current = camera->getCameraPitch();
		current += delta;
		if (current > 360){
			current -= 360;
		}
		else if (current < 0){
			current += 360;
		}
		camera->setCameraPitch(current);
	}
	void updateYaw(float delta){
		CommonCameraInterface* camera =
			PlasticityExampleBrowser::getRenderer()->getActiveCamera();
		float current = camera->getCameraYaw();
		current += delta;
		if (current >= 0 && current < 90){
			camera->setCameraYaw(current);
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
	btScalar dragForce=0;
	void updateDrag(){
		btVector3 vel = m_carChassis->getLinearVelocity();
		dragForce=-vel.length2();
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
	void setCarMass(Gwen::Controls::Base* control);
	void setMaxBreakingForce(Gwen::Controls::Base* control);
	void setMaxEngineForce(Gwen::Controls::Base* control);
	void setSuspensionStiffness(Gwen::Controls::Base* control);
	void setSuspensionMaxForce(Gwen::Controls::Base* control);
	void setSuspensionDamping(Gwen::Controls::Base* control);
	void setSuspensionCompression(Gwen::Controls::Base* control);
	void setSuspensionRestLength(Gwen::Controls::Base* control);
	Gwen::Controls::Base* pPage;
	Gwen::Controls::Button* pauseButton;
	Gwen::Controls::Label *db11,*db12,*db13,*db14;
	Gwen::Controls::Label *db21,*db22,*db23;
	int fps = 0;
	int kmph=0;
	void updateDashboards(){
		char buffer[UIF_SIZE];
		sprintf_s(buffer, UIF_SIZE, "%-3d ",fps);
		std::string str=std::string(buffer);
		db11->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%-3d ", kmph);
		str = std::string(buffer);
		db13->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%-+9.0f ", gEngineForce);
		str = std::string(buffer);
		db21->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%9.0f ",-gBreakingForce);
		str = std::string(buffer);
		db22->SetText(str);
		sprintf_s(buffer, UIF_SIZE, "%9.0f ",-dragForce);
		str = std::string(buffer);
		db23->SetText(str);
	}
	void addDashboard(){
		db11 = new Gwen::Controls::Label(pPage);
		db12 = new Gwen::Controls::Label(pPage);
		db13 = new Gwen::Controls::Label(pPage);
		db14 = new Gwen::Controls::Label(pPage);
		db21 = new Gwen::Controls::Label(pPage);
		db22 = new Gwen::Controls::Label(pPage);
		db23 = new Gwen::Controls::Label(pPage);
		updateDashboards();
		db11->SizeToContents();
		db11->SetPos(gx, gy);
		db12->SetText(" fps ");
		db12->SizeToContents();
		db12->SetPos(gx+wxi/2, gy);
		db13->SizeToContents();
		db13->SetPos(gx+wxi, gy);
		db14->SetText(" km/h ");
		db14->SizeToContents();
		db14->SetPos(gx+3*wxi/2, gy);
		gy += gyInc;
		db21->SizeToContents();
		db21->SetPos(gx, gy);
		db22->SizeToContents();
		db22->SetPos(gx+wxi, gy);
		db23->SizeToContents();
		db23->SetPos(gx+2*wxi, gy);
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
	void addMaxBreakingForce(){
		addLabel("max breaking force");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		std::string text = uif(maxBreakingForce, "%.2f");
		gc->SetToolTip("impulse");
		gc->SetText(text);
		place(gc);
		gc->onReturnPressed.Add(pPage, &DemolisherDemo::setMaxEngineForce);
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
		addCarMass();
		addSuspensionStiffness();
		addSuspensionMaxForce();
		addSuspensionDamping();
		addSuspensionCompression();
		addSuspensionRestLength();
		addMaxEngineForce();
		addMaxBreakingForce();
		addWheelFriction();
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
		initPhysics();
	}

};
DemolisherDemo *demo = 0;

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


float	gVehicleSteering = 0.f;
float	steeringIncrement = 0.04f;
float	steeringClamp = 0.3f;
float	wheelRadius = 0.5f;
float	wheelWidth = 0.4f;
float	rollInfluence = 0.1f;//1.0f;



#define CUBE_HALF_EXTENTS 1
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
void DemolisherDemo::setMaxBreakingForce(Gwen::Controls::Base* control){
	setScalar(control, &(demo->maxBreakingForce));
	restartHandler(control);
}

void DemolisherDemo::restartHandler(Gwen::Controls::Base* control){
	demo->restartRequested = true;
}
void DemolisherDemo::reinit(){
	maxEngineForce = 10000;
	maxBreakingForce = 1000;
	wheelFriction = 0.8;
	lpc = 1;
	lsx = 4;
	lsy = 2;
	lsz = 2;
	density = 50;
	carMass = 1400;
	suspensionStiffness=10;
	suspensionMaxForce = 2000000; // allows static load of 800 tons
	suspensionDamping = .5;
	suspensionCompression=3;
	suspensionRestLength=1;
	driveClock.reset();
	speedometerUpdated = 0;
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
	localTrans.setOrigin(btVector3(0,suspensionRestLength+wheelRadius,0));
	compound->addChildShape(localTrans,chassisShape);
	tr.setOrigin(btVector3(0,0.f,0));
	m_carChassis = localCreateRigidBody(carMass,tr,compound);//chassisShape);	
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

	/// create load parts
	{
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
	updatePauseButtonText();
	updateDashboards();
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
				printf("MLCP solver failed %d times, falling back to SI, totalFailures=%d\n", 
					numFallbacks, totalFailures);
			}
			sol->setNumFallbacks(0);
		}
		updateView();
	}
	else{
		kmph = 0;
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
	if (isShiftPressed) 
	{
		switch (key) 
			{
			case B3G_LEFT_ARROW : 
				{	
					float increment = 1;
					if (isControlPressed){
						increment *= 5;
					}
					updatePitch(increment);
					handled = true;
					break;
				}
			case B3G_RIGHT_ARROW : 
				{
					float increment = -1;
					if (isControlPressed){
						increment *= 5;
					}
					updatePitch(increment);
					handled = true;
					break;
				}
			case B3G_UP_ARROW :
				{
					updateYaw(1);
					handled = true;
					break;
				}
			case B3G_DOWN_ARROW :
				{
					updateYaw(-1);
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
