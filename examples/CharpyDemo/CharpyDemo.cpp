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
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btQuickprof.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "BulletDynamics/MLCPSolvers/btDantzigSolver.h"
#include "BulletDynamics/MLCPSolvers/btLemkeSolver.h"
#include "BulletDynamics/MLCPSolvers/btSolveProjectedGaussSeidel.h"
#include "BulletDynamics/MLCPSolvers/btMLCPSolver.h"
#include "BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.h"
#include "btPlasticHingeConstraint.h"
#include "bt6DofElasticPlasticConstraint.h"
#include "bt6DofElasticPlastic2Constraint.h"
#include "../plasticity/PlasticityData.h"
#include "../plasticity/PlasticityStatistics.h"
#include "../plasticity/PlasticityExampleBrowser.h"
#include "../plasticity/BulletKeyToGwen.h"
#include "../ExampleBrowser/GwenGUISupport/gwenUserInterface.h"
#include "../ExampleBrowser/GwenGUISupport/gwenInternalData.h"

#include <stdio.h> 
#include <string>
#include <time.h>
#ifdef _WIN32
#include <windows.h>
#endif
// Buffer length for sprintfs
#define B_LEN 100

int sFrameNumber = 0;
bool firstRun = true;
bool hammerHitsSpecimen;
btScalar initialStartAngle(1.8);
btScalar startAngle(initialStartAngle);
long  initialDisplayWait = 50;
long displayWait = initialDisplayWait;
long setDisplayWait = displayWait;
btScalar ccdMotionThreshHold(0.001);
btScalar margin(0.001);
btScalar floorHE(0.1);
btScalar defaultTimeStep(0.005);
btScalar timeStep(defaultTimeStep);
btScalar initialTimeStep = 1e-4;
btScalar setTimeStep = initialTimeStep;
int initialNumIterations = 10;
int numIterations = initialNumIterations;
bool variableTimeStep = true;
btScalar currentTime;
const char *modes[] =
{
	"None",
	"single rigid object",
	"two objects and springConstraints",
	"two objects and constraints with zero limits",
	"two objects and spring2Constraints",
	"two objects and hingeConstraint",
	"two objects and plasticHingeConstraint",
	"two objects and 6DofElasticPlasticConstraint",
	"two objects and 6DofElasticPlastic2Constraint",
};
const char *viewModes[] =
{
	"None",
	"Look at anvil",
	"Follow specimen",
	"Follow specimen2",
};
btScalar btZero(0);
btScalar btOne(1);
/** how may parts in half specimen */
int initialSCount = 1;
int sCount = initialSCount;
btScalar initialHammerThickness(0.02);
btScalar hammerThickness(initialHammerThickness);
btScalar initialHammerWidth(0.5);
btScalar hammerWidth(initialHammerWidth);
btScalar initialHammerHeight(0.25);
btScalar hammerHeight(initialHammerHeight);
/** in [m] */
btScalar initialHammerDraft(0.04);
btScalar hammerDraft(initialHammerDraft);
btScalar initialSpaceBetweenAnvils(0.04);
btScalar spaceBetweenAnvils(initialSpaceBetweenAnvils);
btScalar initialAnvilWidth(0.04);
btScalar anvilWidth(initialAnvilWidth);
btScalar initialNotchSize(0.002);
btScalar notchSize(initialNotchSize);
btScalar initialL(0.055);
btScalar l(initialL);
btScalar initialW(0.01);
btScalar w(initialW);
btScalar initialH(0.01);
btScalar h(initialH);
btScalar initialE(200E9); // Steel
btScalar E(initialE);
btScalar nu(0.3); // Steel
btScalar G(initialE / (2 * (1 + nu)));
btScalar initialFy(400e6);
btScalar fy(initialFy);
btScalar initialDamping(0.2);
btScalar damping(initialDamping);
btScalar initialFrequencyRatio(10);
btScalar frequencyRatio(initialFrequencyRatio);
bool initialLimitIfNeeded = true;
bool limitIfNeeded=initialLimitIfNeeded;
float energy = 0;
float maxEnergy;
btDynamicsWorld* dw;
btVector3 y_up(0.01, 1., 0.01);
btRigidBody *specimenBody, *hammerBody=0, *specimenBody2;
btHingeConstraint *hammerHinge;
btJointFeedback hammerHingeJointFeedback;
btAlignedObjectArray<btJointFeedback*> specimenJointFeedback;
btAlignedObjectArray<btHingeConstraint *>mode5Hinge;
btAlignedObjectArray<btScalar *>mode5HingeW1s;
btAlignedObjectArray<btPlasticHingeConstraint *>mode6Hinge;
btAlignedObjectArray<bt6DofElasticPlasticConstraint *>mode7c;
btAlignedObjectArray<bt6DofElasticPlastic2Constraint *>mode8c;
btScalar initialRestitution(0.);
btScalar restitution(initialRestitution);
btScalar initialMaxPlasticRotation(3.);
btScalar maxPlasticRotation(initialMaxPlasticRotation);
btAlignedObjectArray<btTypedConstraint*>tc; // points to specimen constraints
btScalar breakingImpulseThreshold = 0;
float maxForces[6];
FILE *fp;
bool openGraphFile = false;
char gfn[B_LEN];
FILE *rbd; // rigid body data
bool openRigidBodyDataFile = false;
char rbdfn[B_LEN];
int initialSolverType = 1;
int lemkeSolverType = 4;
int solverType = initialSolverType;
btClock rtClock;
btConstraintSolverType solverTypes[] = {
	BT_SEQUENTIAL_IMPULSE_SOLVER,
	BT_SEQUENTIAL_IMPULSE_SOLVER,
	BT_MLCP_SOLVER,
	BT_NNCG_SOLVER,
	BT_MLCP_SOLVER
};
const char * solverTypeNames[] = {
	"",
	"SI (Sequential Impulse)",
	"MLCP (Mixed Linear Complementarity Problem)",
	"NNCG (Nonlinear Nonsmooth Conjugate Gradient)",
	"MLCP (Lemke)"
};
list<PlasticityData> pData;
btConstraintSolver* getSolver(){
	btConstraintSolver* sol;
	switch (solverTypes[solverType]){
	case BT_SEQUENTIAL_IMPULSE_SOLVER:
		sol = new btSequentialImpulseConstraintSolver;
		break;
	case BT_MLCP_SOLVER:
	{
		btMLCPSolverInterface* mlcp = NULL;
		if (solverType == lemkeSolverType){
			mlcp = new btLemkeSolver;
		}
		else{
			mlcp=new btDantzigSolver;
		}
		sol = new btMLCPSolver(mlcp);
	}
	break;
	case BT_NNCG_SOLVER:
		sol = new btNNCGConstraintSolver();
		break;
	}
	return sol;
}


/*
Writes sgraph value using given format
*/
void wgfv(char*format, float f){
	fprintf(fp, format, f);
}
/*
Writes sgraph value using default format
*/
void wgv(float f){
	wgfv("%6.2f,", f);
}

/*
Writes rigid body data value using given format
*/
void wrfv(char*format, float f){
	fprintf(rbd, format, f);
}
/*
Writes rigid body data value using default format
*/
void wrv(const btVector3* v){
	fprintf(rbd, " % 6.4f,% 6.4f,% 6.4f, ", v->x(),v->y(),v->z());
}
void wrv(const btQuaternion* v){
	fprintf(rbd, " % 6.4f,% 6.4f,% 6.4f,% 6.4f, ", v->x(), v->y(), v->z(),v->w());
}


static GwenUserInterface* gui;
static Gwen::Controls::Canvas* canvas;
int gx = 10; // for labels
int gxi = 120; // for inputs elements
int wxi = 60; // width
int gy;
int gyInc=25;
bool restartRequested = false;
bool dropFocus = false;
void reinit();
btScalar minCurrentCollisionDistance(0.);
btScalar minCollisionDistance(0.);
btScalar maxImpact(0.);
btScalar maxSpeed2(0.);

/** Gets text value from TextBoxNumeric */
string getText(Gwen::Controls::Base* control){
	Gwen::Controls::TextBoxNumeric* box =
		static_cast<Gwen::Controls::TextBoxNumeric*>(control);
	if (!box)	{
		return "";
	}
	return Gwen::Utility::UnicodeToString(box->GetText());
}

void setScalar(Gwen::Controls::Base* control, btScalar * vp){
	string text = getText(control); 
	if (text.length()==0)	{
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
	string text = getText(control);
	if (text.length() == 0)	{
		return;
	}
	long fv = std::stol(text);
	*vp = fv;
}
void setInt(Gwen::Controls::Base* control, int * vp){
	string text = getText(control);
	if (text.length() == 0)	{
		return;
	}
	int fv = std::stoi(text);
	*vp = fv;
}

/**
http://www.bulletphysics.org/mediawiki-1.5.8/index.php?title=Anti_tunneling_by_Motion_Clamping
*/
void resetCcdMotionThreshHold()
{
	btScalar radius(ccdMotionThreshHold / 5.f);
	for (int i = dw->getNumCollisionObjects() - 1; i >= 0; i--)
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
	float e = 0.5*inertia*angularVelocity*angularVelocity;
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
void tuneRestitution(btRigidBody* rb){
	rb->setRestitution(restitution);
}

void tuneMode5(){
	// this is currently no-op if target speed is zero
	for (int i = 0; i < mode5Hinge.size(); i++){
		btScalar* w1 = mode5HingeW1s[i];
		mode5Hinge[i]->setMaxMotorImpulse((*w1)*timeStep);
	}
}

/**
* tunes hammerSpeed
* currently mostly no-op as moment is handled in btHingeConstraint.
*/
void tune(btPlasticHingeConstraint* mode6Hinge, int index){
	if (!mode6Hinge->isEnabled()){
		return;
	}
	if (!hammerHitsSpecimen){
		return;
	}
	btScalar applied = specimenJointFeedback[index]->m_appliedTorqueBodyA[1];
	btScalar capacity = mode6Hinge->getPlasticMoment();
	if (applied >= capacity){
		mode6Hinge->updateCurrentPlasticRotation();
	}
	return;
	btScalar energyLoss = mode6Hinge->getAbsorbedEnergy();
	if (energyLoss > 0 && hammerBody!=0){
		btRigidBody* ro = hammerBody;
		btScalar iM = ro->getInvMass();
		btVector3 v = ro->getLinearVelocity();
		btScalar m = 1 / iM;
		float linearEnergy = 0.5*m*v.length2();
		const btVector3 rv = ro->getAngularVelocity();
		const btVector3 iI = ro->getInvInertiaDiagLocal();
		float inertiaEnergy = 0;
		float rx = getRotationEnergy(iI.getX(), rv.getX());
		float ry = getRotationEnergy(iI.getY(), rv.getY());
		float rz = getRotationEnergy(iI.getZ(), rv.getZ());
		float currentEnergy = linearEnergy + rx + ry + rz;
		float velMultiplier = 0;
		if (currentEnergy > energyLoss){
			velMultiplier = btSqrt(1 - (energyLoss / currentEnergy));
		}
		ro->setLinearVelocity(velMultiplier*ro->getLinearVelocity());
		ro->setAngularVelocity(velMultiplier*ro->getAngularVelocity());
	}

}
void tuneMode6(){
	for (int i = 0; i < mode6Hinge.size(); i++){
		tune(mode6Hinge[i], i);
	}
}

/*
dw->setInternalTickCallback(mode#callback);
*/
void mode5callback(btDynamicsWorld *world, btScalar timeStep) {
	tuneMode5();
}
void mode6callback(btDynamicsWorld *world, btScalar timeStep) {
	tuneMode6();
}
void mode7callback(btDynamicsWorld *world, btScalar timeStep) {
	for (int i = 0; i < mode7c.size(); i++){
		if (mode7c[i]->isEnabled()){
			mode7c[i]->updatePlasticity(*specimenJointFeedback[i]);
		}
	}
}
void mode8callback(btDynamicsWorld *world, btScalar timeStep) {
	for (int i = 0; i < mode8c.size(); i++){
		if (mode8c[i]->isEnabled()){
			mode8c[i]->updatePlasticity(*specimenJointFeedback[i]);
		}
	}
}

class CharpyDemo : public Gwen::Event::Handler, public CommonRigidBodyBase
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
	int m_viewMode=1;
	int m_option;
	void showMessage();
	CommonWindowInterface* window;
public:
	int m_mode;
	CharpyDemo(CommonExampleOptions & options)
		:CommonRigidBodyBase(options.m_guiHelper), Gwen::Event::Handler()
	{
		m_option = options.m_option;
		initOptions();
		initParameterUi();
	}
	void initOptions(){
		int option = m_option;
		reinit();
		m_guiHelper->resetCamera(0.5, -70, 15,	0, 0.2, 0.0);
		m_mode = option % 100;
		option /= 100;
		if (option%100>0){
			sCount=(option%100);
		}
		while ((option /= 100) > 0){
			switch (option % 100){
			case 1:
				solverType = BT_MLCP_SOLVER;
				break;
			case 2:
				hammerThickness = btScalar(0);
				break;
			case 3: // slender
				l = btScalar(4.1);
				spaceBetweenAnvils = btScalar(4.0);
				break;
			case 4: // cutting
				hammerDraft = btScalar(0);
				fy = btScalar(250e6);
				spaceBetweenAnvils = btScalar(0.022);
				break;
			case 5: // sidestep
				hammerDraft = btScalar(0);
				setTimeStep = 0.017;
				variableTimeStep = false;
				break;
			case 6: // big and soft
				hammerDraft = btScalar(0);
				fy = btScalar(15e3);
				w = 0.2;
				h = 0.2;
				l = 0.4;
				hammerThickness = 0.2;
				spaceBetweenAnvils = btScalar(0.3);
				hammerDraft = btScalar(0);
				setTimeStep = 0.017;
				variableTimeStep = false;
				break;
			case 7: // Integration instability
				fy = btScalar(800e6);
			}
		}

	}
	~CharpyDemo()
	{
		clearParameterUi();
	}
	BT_DECLARE_ALIGNED_ALLOCATOR();
	virtual void	initPhysics();
	virtual void	exitPhysics();
	virtual void	stepSimulation(float deltaTime);
	virtual void	resetCamera();
	virtual bool	mouseMoveCallback(float x, float y){ return false; };
	virtual bool	mouseButtonCallback(int button, int state, float x, float y){
		return false;
	};
	virtual bool keyboardCallback(int key, int state);
	virtual bool ctrlKeyboardCallback(int key);
	virtual void setViewMode(int viewMode);
	virtual void updateView();
	virtual void renderScene();
	virtual void restart();
	btRigidBody* localCreateRigidBody(btScalar mass, const btTransform& startTransform,
		btCollisionShape* shape);
	/* 
	Limited formatter
	*/
#define UIF_SIZE 10
	std::string uif(btScalar value, const char* fmt="%.4f")
	{
		char buffer[UIF_SIZE];
		sprintf_s(buffer, UIF_SIZE, fmt, value);
		return std::string(buffer);
	}
	/** Gwen controls handling.
	Just flag changes to avoid need to specify all parent references
	if calls come from Gwen
	*/
	void restartHandler(Gwen::Controls::Base* control){
		restartRequested = true;
	}
	void resetHandler(Gwen::Controls::Base* control){
		reinit();
		restartHandler(control);
	}
	void setSCount(Gwen::Controls::Base* control){
		setInt(control, &sCount);
		restartHandler(control);
	}
	void setStartAngle(Gwen::Controls::Base* control){
		setScalar(control, &startAngle);
		restartHandler(control);
	}
	void setE(Gwen::Controls::Base* control){
		btScalar tv(E / 1e9);
		setScalar(control, &tv);
		E = tv*1e9;
		restartHandler(control);
	}
	void setFy(Gwen::Controls::Base* control){
		btScalar tv(fy / 1e6);
		setScalar(control, &tv);
		fy = tv*1e6;
restartHandler(control);
	}
	void setL(Gwen::Controls::Base* control){
		setScalar(control, &l);
		restartHandler(control);
	}
	void setW(Gwen::Controls::Base* control){
		setScalar(control, &w);
		restartHandler(control);
	}
	void setH(Gwen::Controls::Base* control){
		setScalar(control, &h);
		restartHandler(control);
	}
	void setSpaceBetweenAnvils(Gwen::Controls::Base* control){
		setScalar(control, &spaceBetweenAnvils);
		restartHandler(control);
	}
	void setHammerThickness(Gwen::Controls::Base* control){
		setScalar(control, &hammerThickness);
		restartHandler(control);
	}
	void setHammerDraft(Gwen::Controls::Base* control){
		setScalar(control, &hammerDraft);
		restartHandler(control);
	}
	void setRestitution(Gwen::Controls::Base* control){
		setScalar(control, &restitution);
		restartHandler(control);
	}
	void setDamping(Gwen::Controls::Base* control){
		setScalar(control, &damping);
		restartHandler(control);
	}
	void setFrequencyRatio(Gwen::Controls::Base* control){
		setScalar(control, &frequencyRatio);
		restartHandler(control);
	}
	void setLimitIfNeeded(Gwen::Controls::Base* control){
		Gwen::Controls::CheckBox* cb =
			static_cast<Gwen::Controls::CheckBox*>(control);
		limitIfNeeded = cb->IsChecked();
		restartHandler(control);
	}
	void setVariableTimeStep(Gwen::Controls::Base* control){
		Gwen::Controls::CheckBox* cb =
			static_cast<Gwen::Controls::CheckBox*>(control);
		variableTimeStep = cb->IsChecked();
		restartHandler(control);
	}
	void setUiTimeStep(Gwen::Controls::Base* control){
		btScalar tv(setTimeStep*1e3);
		setScalar(control, &tv);
		setTimeStep = tv / 1e3;
		dropFocus = true;
	}
	void setUiDisplayWait(Gwen::Controls::Base* control){
		setLong(control, &setDisplayWait);
		dropFocus = true;
	}
	void setUiNumIterations(Gwen::Controls::Base* control){
		setInt(control, &numIterations);
		if (NULL != dw){
			dw->getSolverInfo().m_numIterations = numIterations;
		}
		dropFocus = true;
	}

	Gwen::Controls::Base* pPage;
	Gwen::Controls::Label* addLabel(string txt){
		Gwen::Controls::Label* gc = new Gwen::Controls::Label(pPage);
		gc->SetText(txt);
		gc->SizeToContents();
		gc->SetPos(gx, gy);
		return gc;
	}
	Gwen::Controls::Button* pauseButton;
	void addPauseSimulationButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		pauseButton = gc;
		gc->SetText(L"Pause");
		gc->SetPos(gx, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &CharpyDemo::handlePauseSimulation);
	}
	void addRestartButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Restart");
		gc->SetPos(gx + wxi, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gc->onPress.Add(pPage, &CharpyDemo::restartHandler);
	}
	void addResetButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Reset");
		gc->SetPos(gx + 2 * wxi, gy);
		gc->SetSize(wxi - 4, gyInc - 4);
		gy += gyInc;
		gc->onPress.Add(pPage, &CharpyDemo::resetHandler);
	}
	void updatePauseButtonText(){
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
		pauseSimulation=!pauseSimulation;
		PlasticityExampleBrowser::setPauseSimulation(pauseSimulation);
	}

	void addSCount(){
		addLabel("parts in half");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(sCount);
		gc->SetText(text);
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setSCount);
	}
	void addStartAngle(){
		addLabel("startAngle +/-");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(startAngle,"%.2f");
		gc->SetText(text);
		gc->SetToolTip("In radians");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setStartAngle);
	}
	void addE(){
		addLabel("E [GPa]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(E / 1e9,"%.0f");
		gc->SetText(text);
		gc->SetToolTip("Young's modulus");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setE);
	}
	void addfy(){
		addLabel("fy [MPa]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(fy / 1e6,"%.3f");
		gc->SetText(text);
		gc->SetToolTip("Ultimate strength");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setFy);
	}
	void addL(){
		addLabel("length [m]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(l);
		gc->SetText(text);
		gc->SetToolTip("Length of specimen");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setL);
	}
	void addW(){
		addLabel("w [m]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(w);
		gc->SetText(text);
		gc->SetToolTip("Vertical width");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setW);
	}
	void addH(){
		addLabel("h [m]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(h);
		gc->SetText(text);
		gc->SetToolTip("Horizontal width");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setH);
	}
	void addSpaceBetweenAnvils(){
		addLabel("anvil distance [m]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(spaceBetweenAnvils);
		gc->SetText(text);
		gc->SetToolTip("Space between anvils");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setSpaceBetweenAnvils);
	}
	void addHammerThickness(){
		addLabel("hammer t[m]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(hammerThickness);
		gc->SetText(text);
		gc->SetToolTip("Thickness of hammer, use 0 to leave out");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setHammerThickness);
	}
	void addHammerDraft(){
		addLabel("hammer draft [m]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(hammerDraft);
		gc->SetText(text);
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setHammerDraft);
	}
	void addRestitution(){
		addLabel("restitution 0-1");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(restitution);
		gc->SetText(text);
		gc->SetToolTip("0 is plastic 1 elastic");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setRestitution);
	}
	void addDamping(){
		addLabel("damping 0-1");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(damping);
		gc->SetToolTip("0=no damping, 1=full damping");
		gc->SetText(text);
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setDamping);
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
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setFrequencyRatio);
	}
	void addLimitIfNeeded(){
		Gwen::Controls::Label* label=addLabel("limitIfNeeded");
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetToolTip("Limit stiffness to avoid blow ups");
		gc->SetPos(gxi, gy);
		gc->SetChecked(limitIfNeeded);
		gy += gyInc;
		gc->onCheckChanged.Add(pPage, &CharpyDemo::setLimitIfNeeded);
	}
	void addVariableTimeStep(){
		Gwen::Controls::Label* label = addLabel("variableTimeStep");
		Gwen::Controls::CheckBox* gc = new Gwen::Controls::CheckBox(pPage);
		gc->SetToolTip("Automatically tune time step");
		gc->SetPos(gxi, gy);
		gc->SetChecked(variableTimeStep);
		gy += gyInc;
		gc->onCheckChanged.Add(pPage, &CharpyDemo::setVariableTimeStep);
	}
	void addTimeStep(){
		addLabel("timeStep [ms]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = uif(setTimeStep*1000);
		gc->SetText(text);
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setUiTimeStep);
	}
	void addDisplayWait(){
		addLabel("displayWait [ms]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(displayWait);
		gc->SetToolTip("wait time after each simulation step");
		gc->SetText(text);
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setUiDisplayWait);
	}
	void addNumIterations(){
		addLabel("iteration count");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(numIterations);
		gc->SetText(text);
		gc->SetToolTip("Number of solver iterations");
		gc->SetPos(gxi, gy);
		gc->SetWidth(wxi);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setUiNumIterations);
	}
	bool isDampingUsed(){
		switch (m_mode){
		case 2:
		case 4:
		case 7:
		case 8:
			return true;
		}
		return false;
	}
	bool isFrequencyRatioUsed(){
		switch (m_mode){
		case 7:
			return true;
		}
		return false;
	}
	bool isLimitIfNeededUsed(){
		switch (m_mode){
		case 4:
		case 8:
			return true;
		}
		return false;
	}
	void initParameterUi(){
		gui = PlasticityExampleBrowser::getGui();
		window = PlasticityExampleBrowser::getWindow();
		canvas = gui->getInternalData()->pCanvas;

		pPage = gui->getInternalData()->m_demoPage->GetPage();
		gy = 5;
		addSCount();
		addStartAngle();
		addE();
		addfy();
		addL();
		addW();
		addH();
		addSpaceBetweenAnvils();
		addHammerThickness();
		addHammerDraft();
		addRestitution();
		if (isDampingUsed()){
			addDamping();
		}
		if (isFrequencyRatioUsed()){
			addFrequencyRatio();
		}
		if (isLimitIfNeededUsed()){
			addLimitIfNeeded();
		}
		addTimeStep();
		addVariableTimeStep();
		addDisplayWait();
		if (solverType != BT_MLCP_SOLVER){
			addNumIterations();
		}
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
	// set in initPhysics
	btVector3* basePoint;
	/**
	* return distance of specimen squared from base point
	*/
	btScalar getSpecimenDistance2(){
		btVector3 p1 = specimenBody->getCenterOfMassPosition();
		if (m_mode == 1){
			return (p1 - *basePoint).length2();
		}
		btVector3 p2 = specimenBody2->getCenterOfMassPosition();
		return (((p1 + p2) / 2) - *basePoint).length2();

	}
	/**
	Writes graph data if file can be opened
	*/
	void writeGraphData(){
		if (!openGraphFile || !fp){
			return;
		}
		wgfv("%8.5f,", currentTime);
		wgv(energy);
		wgv(maxEnergy - energy);
		if (m_mode>1){
			for (int j = 0; j < specimenJointFeedback.size(); j++){
				for (int i = 0; i < 3; i++){
					wgv(specimenJointFeedback[j]->m_appliedForceBodyA[i]);
				}
				for (int i = 0; i < 3; i++){
					wgv(specimenJointFeedback[j]->m_appliedTorqueBodyA[i]);
				}
			}
		}
		fprintf(fp, ";\n");
	}
	/**
	Writes rigid body data file
	*/
	void writeRigidBodyData(){
		if (!openRigidBodyDataFile || !rbd){
			return;
		}
		const btCollisionObjectArray objects = dw->getCollisionObjectArray();
		for (int i = 0; i<objects.capacity(); i++)
		{
			const btCollisionObject* o = objects[i];
			const btRigidBody* ro = btRigidBody::upcast(o);
			btScalar iM = ro->getInvMass();
			if (iM == 0){
				continue;
			}
			const btVector3 v = ro->getLinearVelocity();
			btScalar m = 1 / iM;
			const btVector3 com = ro->getCenterOfMassPosition();
			const btQuaternion rot = ro->getCenterOfMassTransform().getRotation();
			const btVector3 rv = ro->getAngularVelocity();
			fprintf(rbd,"%3d, %8.5f, %8.4f, %8.4f, ", i, currentTime,m,iM);
			wrv(&com);
			wrv(&rot);
			wrv(&v);
			wrv(&rv);
			fprintf(rbd, ";\n");
		}
	}
	btScalar getBodySpeed2(const btCollisionObject* o){
		return o->getInterpolationLinearVelocity().length2();
	}
	int centerParts; /* number of parts in center section */
	btScalar centerWidth; /* combined z length parts in center section */
	btScalar ht; /* z length of middle object */
	/**
	init parameters for object division
	Note that centerWidth may be wider than spaceBetweenAnvils
	See limit parameter below
	*/
	void initMeshParameters(){
		btScalar limit(1.3*spaceBetweenAnvils + 2 * anvilWidth);
		if (l < limit){
			centerParts = sCount;
			centerWidth = l;
		}
		else{
			centerWidth = btScalar(spaceBetweenAnvils);
			centerParts = (int)(sCount*spaceBetweenAnvils / l);
			if (centerParts < 1){
				centerParts = 1;
				centerWidth = l;
			}
		}
		if (hammerThickness > w && centerParts>1){
			ht = btScalar(hammerThickness);
		}
		else{
			ht = btScalar(l / sCount);
			if (ht > centerWidth){
				ht = centerWidth;
			}
		}
	}
	/**
	get half length for given part.
	If we have more than one part in each side
	use hammer width and space between anvils
	and for others split about evenly.
	If there is no hammer, center part length is l/sCount.
	*/
	btScalar getHalfLength(int partNumber){
		if (m_mode == 1){
			return btScalar(l / 2);
		}
		int centerDistance = getCenterDistance(partNumber);
		if (centerDistance<centerParts){
			switch (centerDistance){
			case 0:
				return ht / 4;
			default:
				return (centerWidth - ht) / ((centerParts - 1)*4);
			}
		}
		btScalar halfLength = btScalar((l - centerWidth) / 
			((sCount - centerParts) * 4));
		return halfLength;
	}
	/**
	Get length for constraint stiffness calculations.
	*/
	btScalar getLengthForStiffness(int firstPart){
		btScalar l1 = getHalfLength(firstPart);
		btScalar l2 = getHalfLength(firstPart + 1);
		return btScalar(l1 + l2);

	}
	/*
	start from negative z side	-l/2 and
	add twice other parts and half of current part

	(m_mode>1 ? (i == 0 ? -1 : 1)*halfLength : 0.f));

	*/
	btScalar getZPosition(int partNumber){
		if (m_mode == 1){
			return btScalar(0);
		}
		btScalar z = -l / 2;
		for (int i = 0; i < partNumber; i++){
			z += 2 * getHalfLength(i);
		}
		z += getHalfLength(partNumber);
		return z;
	}
	btVector3 getPosition(int partNumber){
		btScalar zPosition = getZPosition(partNumber);
		btVector3 pos(h / 2, 0.2 + w / 2, zPosition);
		return pos;
	}

	btScalar getManifoldSpeed2(btPersistentManifold* manifold){
		btScalar s0 = getBodySpeed2(manifold->getBody0());
		btScalar s1 = getBodySpeed2(manifold->getBody1());
		return s0>s1 ? s0 : s1;
	}
	btScalar getHammerAngle(){
		if (hammerBody == 0){
			return SIMD_PI;
		}
		return hammerBody->getCenterOfMassTransform().getRotation().getAngle();
	}
	void tuneDisplayWait(){
		if (timeStep<1e-3){
			displayWait = 2;
		}
		else{
			displayWait = setDisplayWait;
		}

	}
	void checkCollisions(){
		hammerHitsSpecimen = false;
		minCurrentCollisionDistance = btScalar(0);
		int numManifolds = dw->getDispatcher()->getNumManifolds();
		for (int i = 0; i<numManifolds; i++)
		{
			btPersistentManifold* contactManifold = dw->getDispatcher()->getManifoldByIndexInternal(i);
			const btCollisionObject* obA = contactManifold->getBody0();
			const btCollisionObject* obB = contactManifold->getBody1();

			int numContacts = contactManifold->getNumContacts();
			btScalar totalImpact(0);
			for (int j = 0; j<numContacts; j++)
			{
				btManifoldPoint& pt = contactManifold->getContactPoint(j);
				totalImpact += pt.m_appliedImpulse;
				btScalar distance(pt.getDistance());
				if (distance<minCollisionDistance){
					minCollisionDistance = distance;
				}
				if (distance<minCurrentCollisionDistance){
					minCurrentCollisionDistance = distance;
				}
			}
			if (totalImpact>maxImpact){
				maxImpact = totalImpact;
			}
			float maxManifoldSpeed2 = getManifoldSpeed2(contactManifold);
			if (maxManifoldSpeed2>maxSpeed2){
				maxSpeed2 = maxManifoldSpeed2;
			}
			if ((obA == hammerBody && obB == specimenBody) ||
				(obB == hammerBody && obA == specimenBody)
				){
				hammerHitsSpecimen = true;
			}
		}
		if (variableTimeStep){
			/**
			* tune timeStep so that simulation goes hm times
			* faster if hammer or specimen is outside impact area
			*/
			btScalar specimenDistance2 = getSpecimenDistance2();
			btScalar distance = btFabs(getHammerAngle());
			btScalar lowLimit = 0.05;
			btScalar highLimit = 0.2;
			btScalar specimenLimit2 = l*l; // compare ^2
			btScalar hm = defaultTimeStep / setTimeStep;
			if (specimenDistance2>specimenLimit2 || distance > highLimit){
				timeStep = setTimeStep * hm;
			}
			else if (distance<lowLimit){
				timeStep = setTimeStep;
			}
			else{
				btScalar m = (distance - lowLimit) / (highLimit - lowLimit);
				timeStep = (1 + m*(hm - 1))*setTimeStep;
			}
			tuneDisplayWait();
		}
		else{
			displayWait = setDisplayWait;
		}
	}
	/* Just for making breaking possible.
	This calculation has no known scientific background.
	Maximum impulse that can be generated is about 300 Ns
	(20 kg * 15 m/s)
	For basic case with 1.8 start angle 120 Ns (20 kg * 6 m/s)
	*/
	btScalar getBreakingImpulseThreshold(){
		btScalar b(w);
		btScalar h(w - notchSize); // notch is 2 mm
		btScalar w1(fy*b*h*h / 4);
		btScalar M(w1*maxPlasticRotation);
		btScalar v(60); // m/s but no known physical meaning
		breakingImpulseThreshold = M / v;
		return breakingImpulseThreshold;
	}
	void scalePlasticity(btScalar scale){
		if (m_mode == 1){
			return;
		}
		maxPlasticRotation *= scale;
		getBreakingImpulseThreshold();
		switch (m_mode){
		case 6:
			for (int i = 0; i < mode6Hinge.size(); i++){
				mode6Hinge[i]->setMaxPlasticRotation(maxPlasticRotation);
			}
			break;
		case 7:
			for (int i = 0; i < mode6Hinge.size(); i++){
				mode7c[i]->scalePlasticity(scale);
			}
			break;
		case 8:
			for (int i = 0; i < mode6Hinge.size(); i++){
				mode8c[i]->scalePlasticity(scale);
			}
			break;
		}
	}
	btTransform getTr(int i, btScalar scaleZ){
		btScalar halfLength = getHalfLength(i);
		btTransform ctr;
		ctr.setIdentity();
		btVector3 cpos(0, 0,scaleZ*halfLength);
		ctr.setOrigin(cpos);
		return ctr;
	}
	btTransform getTa(int i){
		return getTr(i, btScalar(1));
	}
	btTransform getTb(int i){
		return getTr(i + 1,btScalar(-1));
	}
	bool isNotch(int objectNumber){
		if (objectNumber == sCount - 1){
			return true;
		}
		return false;
	}
	btScalar getH(int i){
		btScalar localH;
		if (isNotch(i)){
			localH = btScalar(h - notchSize);
		}
		else{
			localH = h;
		}
		return localH;
	}
	/**
	mode 2
	*/
	void addSpringConstraint(btAlignedObjectArray<btRigidBody*> ha){
		int loopSize = ha.size() - 1;
		for (int i = 0; i < loopSize; i++){
			btScalar l4s = getLengthForStiffness(i);
			btGeneric6DofSpringConstraint *sc =
				new btGeneric6DofSpringConstraint(*ha[i], *ha[i+1],
				getTa(i), getTb(i), true);
			tc.push_back(sc);
			sc->setBreakingImpulseThreshold(getBreakingImpulseThreshold());
			btJointFeedback* jf = new btJointFeedback();
			specimenJointFeedback.push_back(jf);
			sc->setJointFeedback(jf);
			btScalar b(w);
			btScalar h=getH(i);
			btScalar I1(b*h*h*h / 12); // weaker
			btScalar I2(h*b*b*b / 12); // stronger
			btScalar k0(E*b*h / l4s / 2); // axial
			btScalar k1(48 * E*I1 / l4s / l4s / l4s);
			btScalar k2(48 * E*I2 / l4s / l4s / l4s);
			btScalar w1 = fy*b*h*h / 4;
			btScalar w2(fy*b*b*h / 4);
			sc->setStiffness(0, k1);
			sc->setStiffness(1, k2);
			sc->setStiffness(2, k0);
			sc->setStiffness(3, w2);
			sc->setStiffness(4, w1);
			sc->setStiffness(5, (w1 + w2) / 2);
			dw->addConstraint(sc, true);
			for (int i = 0; i < 6; i++)
			{
				sc->enableSpring(i, true);
			}
			for (int i = 0; i < 6; i++)
			{
				sc->setDamping(i, damping);
			}
			sc->setEquilibriumPoint();
		}
	}
	/**
	mode 3
	*/
	void addFixedConstraint(btAlignedObjectArray<btRigidBody*> ha){
		int loopSize = ha.size() - 1;
		for (int i = 0; i < loopSize; i++){
			btGeneric6DofConstraint *sc =
				new btGeneric6DofConstraint(*ha[i], *ha[i + 1],
				getTa(i), getTb(i), true);
			tc.push_back(sc);
			btJointFeedback* jf = new btJointFeedback();
			specimenJointFeedback.push_back(jf);
			sc->setJointFeedback(jf);
			sc->setBreakingImpulseThreshold(getBreakingImpulseThreshold());
			dw->addConstraint(sc, true);
			for (int i = 0; i < 6; i++){
				sc->setLimit(i, 0, 0); // make fixed
			}
		}
	}
	/**
	mode 4
	*/
	void addSpring2Constraint(btAlignedObjectArray<btRigidBody*> ha){
		int loopSize = ha.size() - 1;
		for (int i = 0; i < loopSize; i++){
			btScalar l4s = getLengthForStiffness(i);
			btGeneric6DofSpring2Constraint *sc =
				new btGeneric6DofSpring2Constraint(*ha[i], *ha[i+1],
				getTa(i), getTb(i));
			tc.push_back(sc);
			sc->setBreakingImpulseThreshold(getBreakingImpulseThreshold());
			btJointFeedback* jf = new btJointFeedback();
			specimenJointFeedback.push_back(jf);
			sc->setJointFeedback(jf);
			btScalar b(w);
			btScalar h = getH(i);
			btScalar I1(b*h*h*h / 12);
			btScalar I2(h*b*b*b / 12);
			btScalar k0(E*b*h / l4s / 2);
			btScalar k1(48 * E*I1 / l4s / l4s / l4s);
			btScalar k2(48 * E*I1 / l4s / l4s / l4s);
			btScalar w1 = fy*b*h*h / 4;
			btScalar w2(fy*b*b*h / 4);
			sc->setStiffness(0, k1,limitIfNeeded);
			sc->setLimit(0, 0, 0);
			sc->setStiffness(1, k2, limitIfNeeded);
			sc->setLimit(1, 0, 0);
			sc->setStiffness(2, k0, limitIfNeeded);
			sc->setLimit(2, 0, 0);
			sc->setStiffness(3, w2, limitIfNeeded);
			sc->setLimit(3, 0, 0);
			sc->setStiffness(4, w1, limitIfNeeded);
			sc->setStiffness(5, (w1 + w2) / 2, limitIfNeeded);
			sc->setLimit(5, 0, 0);
			dw->addConstraint(sc, true);
			for (int i = 0; i < 6; i++)
			{
				sc->enableSpring(i, true);
			}
			for (int i = 0; i < 6; i++)
			{
				sc->setDamping(i, damping);
			}
			sc->setEquilibriumPoint();
		}
	}

	/**
	mode 5
	*/
	void addHingeConstraint(btAlignedObjectArray<btRigidBody*> ha){
		int loopSize = ha.size() - 1;
		for (int i = 0; i < loopSize; i++){
			btScalar la = getHalfLength(i);
			btScalar lb = getHalfLength(i + 1);
			btVector3 pivotAxis(btZero, btScalar(1), btZero);
			btVector3 pivotInA(btZero, btZero, la);
			btVector3 pivotInB(btZero, btZero, -lb);
			btHingeConstraint *sc =
				new btHingeConstraint(*ha[i], *ha[i + 1],
				pivotInA, pivotInB, pivotAxis, pivotAxis, false);
			tc.push_back(sc);
			sc->setBreakingImpulseThreshold(getBreakingImpulseThreshold());
			mode5Hinge.push_back(sc);
			btScalar b(w);
			btScalar h=getH(i);
			btScalar w1 = fy*b*h*h / 4;
			mode5HingeW1s.push_back(new btScalar(w1));
			btJointFeedback* jf = new btJointFeedback();
			specimenJointFeedback.push_back(jf);
			sc->setJointFeedback(jf);
			sc->setLimit(-SIMD_PI, SIMD_PI); // until parts are overturn
			sc->enableAngularMotor(true, 0, w1*timeStep);
			dw->addConstraint(sc, true);
		}
	}

	/*
	mode 6
	*/
	void addPlasticHingeConstraint(btAlignedObjectArray<btRigidBody*> ha){
		int loopSize = ha.size() - 1;
		for (int i = 0; i < loopSize; i++){
			btScalar la = getHalfLength(i);
			btScalar lb = getHalfLength(i+1);
			btVector3 pivotAxis(btZero, btScalar(1), btZero);
			btVector3 pivotInA(btZero, btZero, la);
			btVector3 pivotInB(btZero, btZero, -lb);
			btPlasticHingeConstraint *sc =
				new btPlasticHingeConstraint(*ha[i], *ha[i + 1],
				pivotInA, pivotInB, pivotAxis, pivotAxis, false);
			tc.push_back(sc);
			mode6Hinge.push_back(sc);
			sc->setMaxPlasticRotation(maxPlasticRotation);
			btScalar b(w);
			btScalar h=getH(i);
			btScalar w1 = fy*b*h*h / 4;
			btJointFeedback* jf=new btJointFeedback();
			specimenJointFeedback.push_back(jf);
			sc->setJointFeedback(jf);
			sc->setLimit(-SIMD_HALF_PI, SIMD_HALF_PI); // until parts are overturn
			sc->setPlasticMoment(w1);
			sc->setLimit(0, 0);
			dw->addConstraint(sc, true);
		}
	}

	/*
	Mode 7
	*/
	void addElasticPlasticConstraint(btAlignedObjectArray<btRigidBody*> ha){
		int loopSize = ha.size() - 1;
		for (int i = 0; i < loopSize; i++){
			bt6DofElasticPlasticConstraint *sc =
				new bt6DofElasticPlasticConstraint(*ha[i], *ha[i + 1],
				getTa(i), getTb(i), true);
			btScalar l4s = getLengthForStiffness(i);
			tc.push_back(sc);
			mode7c.push_back(sc);
			sc->setMaxPlasticRotation(maxPlasticRotation);
			sc->setMaxPlasticStrain(w);
			btJointFeedback* jf = new btJointFeedback();
			specimenJointFeedback.push_back(jf);
			sc->setJointFeedback(jf);
			btScalar b(w);
			btScalar h=getH(i);
			btScalar I1(b*h*h*h / 12);
			btScalar I2(h*b*b*b / 12);
			btScalar It(0.14*b*h*h*h);
			btScalar k0(E*b*h / l4s / 2);
			btScalar k1(48 * E*I1 / l4s / l4s / l4s);
			btScalar k2(48 * E*I2 / l4s / l4s / l4s);
			btScalar w1 = fy*b*h*h / 4;
			btScalar w2(fy*b*b*h / 4);
			sc->setStiffness(2, k0);
			sc->setMaxForce(2, fy*b*h);
			sc->setStiffness(0, k1);
			sc->setMaxForce(0, fy*b*h / 2);
			sc->setStiffness(1, k2);
			sc->setMaxForce(1, fy*b*h / 2);
			sc->setStiffness(5, G*It / l4s); // not very exact
			sc->setMaxForce(5, fy / 2 * It / (h / 2)); // not very exact
			sc->setStiffness(4, 3 * E*I1 / l4s); // not very exact
			sc->setMaxForce(4, w1);
			sc->setStiffness(3, 3 * E*I2 / l4s); // not very exact
			sc->setMaxForce(3, w2);
			dw->addConstraint(sc, true);
			for (int i = 0; i < 6; i++)
			{
				sc->enableSpring(i, true);
			}
			for (int i = 0; i < 6; i++)
			{
				sc->setDamping(i, damping);
			}
			sc->setFrequencyRatio(frequencyRatio);
			sc->setEquilibriumPoint();
		}
	}

	/**
	mode 8
	*/
	void addElasticPlastic2Constraint(btAlignedObjectArray<btRigidBody*> ha){
		int loopSize = ha.size() - 1;
		for (int i = 0; i < loopSize; i++){
			bt6DofElasticPlastic2Constraint *sc =
				new bt6DofElasticPlastic2Constraint(*ha[i], *ha[i+1],
				getTa(i), getTb(i));
			btScalar l4s = getLengthForStiffness(i);
			tc.push_back(sc);
			mode8c.push_back(sc);
			sc->setMaxPlasticRotation(maxPlasticRotation);
			sc->setMaxPlasticStrain(w);
			btJointFeedback* jf = new btJointFeedback();
			specimenJointFeedback.push_back(jf);
			sc->setJointFeedback(jf);
			btScalar b(w);
			btScalar h = getH(i);
			btScalar I1(b*h*h*h / 12);
			btScalar I2(h*b*b*b / 12);
			btScalar It(0.14*b*h*h*h);
			btScalar k0(E*b*h / l4s / 2);
			btScalar k1(48 * E*I1 / l4s / l4s / l4s);
			btScalar k2(48 * E*I2 / l4s / l4s / l4s);
			btScalar w1 = fy*b*h*h / 4;
			btScalar w2(fy*b*b*h / 4);
			sc->setStiffness(2, k0, limitIfNeeded);
			sc->setMaxForce(2, fy*b*h);
			sc->setStiffness(0, k1, limitIfNeeded);
			sc->setMaxForce(0, fy*b*h / 2);
			sc->setStiffness(1, k2, limitIfNeeded);
			sc->setMaxForce(1, fy*b*h / 2);
			sc->setStiffness(5, G*It / l4s, limitIfNeeded); // not very exact
			sc->setMaxForce(5, fy / 2 * It / (h / 2)); // not very exact
			sc->setStiffness(4, 3 * E*I1 / l4s, limitIfNeeded); // not very exact
			sc->setMaxForce(4, w1);
			sc->setStiffness(3, 3 * E*I2 / l4s, limitIfNeeded); // not very exact
			sc->setMaxForce(3, w2);
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
	}
	int getObjectCount(){
		return (m_mode > 1 ? sCount * 2 : 1);
	}
	/**
	*/
	int getCenterDistance(int partNumber){
		if (m_mode == 1){
			return 0;
		}
		if (partNumber >= sCount){
			partNumber = 2 * sCount - partNumber - 1;
		}
		return sCount - partNumber - 1;
	}
	// support anvils leaving spaceBetweenAnvils  open space between them
	void addAnvilParts(){
		btScalar zHalf(anvilWidth/2); // for support shapes
		btScalar zTrans(spaceBetweenAnvils / 2 + zHalf);
		{ // lower part
			btBoxShape* shape =
				new btBoxShape(btVector3(0.05f + w, 0.1f, zHalf));
			m_collisionShapes.push_back(shape);
			// symmetrically around z=0 and x=0
			// top at y=0.2
			for (int i = 0; i<2; i++)
			{
				btTransform tr;
				tr.setIdentity();
				btVector3 pos(0, btScalar(0.1), btScalar((i == 0 ? -1 : 1)*zTrans));
				tr.setOrigin(pos);
				btRigidBody* rb = localCreateRigidBody(0.f, tr, shape);
				tuneRestitution(rb);
			}
		}
		{ // back support
			btBoxShape* shape =
				new btBoxShape(btVector3(0.025 + w / 2, 0.02 + w, zHalf));
			m_collisionShapes.push_back(shape);
			// symmetrically around z=0
			// bottom at y=0.2
			// frontsize at x=0
			for (int i = 0; i<2; i++)
			{
				btTransform tr;
				tr.setIdentity();
				btVector3 pos(btScalar(-0.025 - w / 2),
					btScalar(0.22 + w),
					btScalar((i == 0 ? -1 : 1)*zTrans));
				tr.setOrigin(pos);
				btRigidBody* rb = localCreateRigidBody(0.f, tr, shape);
				tuneRestitution(rb);
			}
		}
	}
	// charpy specimen using sCount parts, 
	// symmetrically around z=0
	// bottom at y=0.2
	// backside at x=0
	void addSpecimenParts(){
		btAlignedObjectArray<btRigidBody*> ha;
		btAlignedObjectArray<btTransform> ta;
		btVector3 localInertia(0, 0, 0);
		// Only one object in mode 1
		int objectCount = getObjectCount(); 
		for (int i = 0; i<objectCount; i++)
		{	
			btScalar halfLength = getHalfLength(i);
			btBoxShape* shape =
				new btBoxShape(btVector3(h / 2, w / 2, halfLength));
			btScalar sMass(h*w*halfLength*2.0f*7800.0f);
			m_collisionShapes.push_back(shape);
			shape->calculateLocalInertia(sMass, localInertia);
			btTransform tr;
			tr.setIdentity();
			btVector3 pos = getPosition(i);
			tr.setOrigin(pos);
			btDefaultMotionState* myMotionState
				= new btDefaultMotionState(tr);
			btRigidBody::btRigidBodyConstructionInfo
				rbInfo(sMass, myMotionState, shape, localInertia);
			btRigidBody* body =
				new btRigidBody(rbInfo);
			tuneRestitution(body);
			m_dynamicsWorld->addRigidBody(body);
			ha.push_back(body);
		}
		int middleIndex = (m_mode==1?0:sCount-1);
		specimenBody = ha[middleIndex];
		if (m_mode != 1){
			specimenBody2 = ha[middleIndex+1];
		}
		switch (m_mode) {
		case 2:
			addSpringConstraint(ha);
			break;
		case 3:
			addFixedConstraint(ha);
			break;
		case 4:
			addSpring2Constraint(ha);
			break;
		case 5:
			addHingeConstraint(ha);
			dw->setInternalTickCallback(mode5callback);
			break;
		case 6:
			addPlasticHingeConstraint(ha);
			dw->setInternalTickCallback(mode6callback);
			break;
		case 7:
			addElasticPlasticConstraint(ha);
			dw->setInternalTickCallback(mode7callback);
			break;
		case 8:
			addElasticPlastic2Constraint(ha);
			dw->setInternalTickCallback(mode8callback);
			break;
		}
	}
	/**
	* base has same corners as hammer
	* top is 2 mm wide
	*/
	btConvexHullShape* createDraft(){
		btScalar z = hammerThickness / 2;
		btScalar x = btZero;
		btScalar y = hammerHeight / 2;
		btConvexHullShape* draft = new btConvexHullShape();
		draft->addPoint(btVector3(x,-y,z ));
		draft->addPoint(btVector3(x, y, z));
		draft->addPoint(btVector3(x, y, -z));
		draft->addPoint(btVector3(x, -y, -z));
		x = -hammerDraft;
		z = btScalar(0.001);
		draft->addPoint(btVector3(x, -y, z));
		draft->addPoint(btVector3(x, y, z));
		draft->addPoint(btVector3(x, y, -z));
		draft->addPoint(btVector3(x, -y, -z));
		draft->setMargin(0.0001);
		return draft;
	}
	// hammer with arm and hinge
	// hammer is 0.5 m wide and 0.25 m high, thickness is 0.02
	// hammer and hinge are positioned so that impact is horizontal 
	// (global x-direction)
	// hammer should be able to provide impact of about 500 J
	// m*g*h=500
	// m~500/2/10~25 kg
	void addHammer(){
		if (hammerThickness <= 0){
			return;
		}
		btScalar zHalf = hammerThickness / 2;
		btScalar xHalf = hammerWidth / 2;
		btScalar yHalf = hammerHeight / 2;
		btCompoundShape* compound = new btCompoundShape();
		btCollisionShape* hammer =
			new btBoxShape(btVector3(xHalf, yHalf, zHalf));
		btScalar hMass = hammerWidth*hammerHeight*hammerThickness * 7800;
		btTransform hTr;
		hTr.setIdentity();
		// create hammer at y=0
		compound->addChildShape(hTr, hammer);
		// draft
		btTransform dTr;
		dTr.setIdentity();
		btVector3 dPos(-xHalf, btZero, btZero);
		dTr.setOrigin(dPos);
		compound->addChildShape(dTr, createDraft());
		btCollisionShape* arm =
			new btBoxShape(btVector3(0.02, 0.5, 0.02));
		btScalar aMass = 0.04*1.0*0.04 * 7800;
		btTransform aTr;
		aTr.setIdentity();
		// arm above hammer
		btVector3 aPos(btZero, btScalar(0.5 + 0.125), btZero);
		aTr.setOrigin(aPos);
		compound->addChildShape(aTr, arm);
		btTransform cTr;
		// move compound down so that axis-position
		// corresponding to armPivot is at y=0
		// rotate and then move back up and in
		// x-direction so that at impact time hammer is in down position
		// up so that center of hammer is about y=0.2
		const btVector3 armPivot(btZero,
			btScalar(1), btZero);
		btVector3 cPos(btZero, btScalar(1.2+w/2), btZero);
		btScalar xPos;
		// tune for cases where angle is negative and hammer hits from
		// opposite (negative) side
		if (startAngle > 0){
			xPos = btScalar(xHalf + h + hammerDraft);
		}
		else{
			xPos = btScalar(-xHalf);
		}
		btVector3 lPos(xPos, btZero, btZero);
		btTransform downTr;
		btTransform upTr;
		upTr.setIdentity();
		upTr.setOrigin(cPos);
		downTr.setIdentity();
		downTr.setOrigin(btScalar(-1)*armPivot);
		btQuaternion cRot;
		btVector3 axis(btZero, btZero, btScalar(1));
		cRot.setRotation(axis, btScalar(startAngle)); // pi means up
		btTransform axTr;
		axTr.setIdentity();
		axTr.setOrigin(cPos + lPos);
		//
		btTransform leftTr;
		leftTr.setIdentity();
		leftTr.setOrigin(lPos);
		// multiply transformations in reverse order
		cTr.setIdentity();
		cTr *= leftTr;
		cTr *= upTr;
		cTr *= btTransform(cRot);
		cTr *= downTr;
		btRigidBody *hBody = localCreateRigidBody(hMass + aMass, cTr, compound);
		hammerBody = hBody;
		tuneRestitution(hammerBody);
		btVector3 aDims(btScalar(0.01), btScalar(0.01), btScalar(0.05));
		btCollisionShape* axil =
			new btCylinderShapeZ(aDims);
		m_collisionShapes.push_back(axil);
		btRigidBody *axilBody = localCreateRigidBody(btZero, axTr, axil);
		const btVector3 axilPivot(btZero, btZero, btZero);
		btVector3 pivotAxis(btZero, btZero, btScalar(1));
		hammerHinge =
			new btHingeConstraint(*hBody, *axilBody,
			armPivot, axilPivot, pivotAxis, pivotAxis, false);
		hammerHinge->setJointFeedback(&hammerHingeJointFeedback);
		m_dynamicsWorld->addConstraint(hammerHinge, true);
	}
	bool isBroken(){
		int loopSize = tc.size();
		for (int i = 0; i < loopSize; i++){
			if (!tc[i]->isEnabled()){
				return true;
			}
		}
		return false;
	}
};
/* 
Adapted from ForkLiftDemo
Should not make big difference in this case but may be useful if larger adaptions are used

*/
void tuneSolver(){
	switch (solverTypes[solverType]){
	case BT_SEQUENTIAL_IMPULSE_SOLVER:
		dw->getSolverInfo().m_minimumSolverBatchSize = 128;
		break;
	case BT_MLCP_SOLVER:
		dw->getSolverInfo().m_minimumSolverBatchSize = 1;
		break;
	default:
		dw->getSolverInfo().m_minimumSolverBatchSize = 128;
		break;
	}
	// dw->getSolverInfo().m_erp2 = 0.95; // default is 0.8
	// dw->getSolverInfo().m_erp = 0.6; // default is 0.2
	// dw->getSolverInfo().m_splitImpulse = true; // default is true
	// dw->getSolverInfo().m_splitImpulsePenetrationThreshold = -0.002; // default is -0.04
	// Tuning of values above did not make big difference
	dw->getSolverInfo().m_numIterations = numIterations;
}



void toggleGraphFile(){
	openGraphFile = !openGraphFile;
	if (fp){
		fprintf(fp, "];\n");
		fclose(fp);
		fp = NULL;
	}else{
		sprintf_s(gfn,B_LEN, "d:/wrk/cgd.m");
		errno_t err = fopen_s(&fp,gfn, "w");
		if (!fp || err){
			strcpy_s(gfn, "");
			openGraphFile = false;
			return;
		}
		fprintf(fp,"cgd=[");
	}
}

void toggleLogPlasticityData(){
	bool currentValue = PlasticityData::getLogData();
	PlasticityData::setLogData(!currentValue);
}
CharpyDemo *charpyDemo = 0;
void toggleRigidBodyDataFile(){
	openRigidBodyDataFile = !openRigidBodyDataFile;
	if (rbd){
		fprintf(rbd, "];\n");
		fclose(rbd);
		rbd = NULL;
	}
	else{
		sprintf_s(rbdfn, B_LEN, "d:/wrk/rbd-%d.m",charpyDemo->m_mode);
		errno_t err = fopen_s(&rbd, rbdfn, "w");
		if (!rbd || err){
			strcpy_s(rbdfn, "");
			openRigidBodyDataFile = false;
			return;
		}
		fprintf(rbd, "rbd%d=[\n", charpyDemo->m_mode);
	}
}




btRigidBody* CharpyDemo::localCreateRigidBody(btScalar mass, const btTransform& startTransform, 
	btCollisionShape* shape)
{
	btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, myMotionState, shape, localInertia);
	btRigidBody* body = new btRigidBody(cInfo);
	m_dynamicsWorld->addRigidBody(body);
	return body;
}

/*
X axis is horizontal, positive to direction where hammer comes from (left)
Y axis is vertical, positive up
Z axis is horizontal and Z=0 is symmetry plane
*/
void	CharpyDemo::initPhysics()
{
	timeStep = setTimeStep;
	energy = 0;
	maxEnergy = energy;
	b3Printf("mode=%d startAngle=%f timeStep=%f", m_mode, startAngle, timeStep);
	basePoint = new btVector3(w / 2, 0.2 + w / 2, 0);
	m_collisionConfiguration = new btDefaultCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_broadphase = new btDbvtBroadphase();
	m_solver = getSolver();
	btDiscreteDynamicsWorld* btWorld =
		new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_dynamicsWorld = btWorld;
	dw=m_dynamicsWorld;
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);
	tuneSolver();
	// floor
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(50,floorHE,50));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-floorHE,0));
		btRigidBody* rb=localCreateRigidBody(0.f, groundTransform, groundShape);
		rb->setFriction(0.8); //
		rb->setRollingFriction(0.8); //
	}
	initMeshParameters();
	addAnvilParts();
	addSpecimenParts();
	addHammer();
	resetCcdMotionThreshHold();
	updateEnergy();
	currentTime=0;
	rtClock.reset();
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void CharpyDemo::resetCamera(){
	setViewMode(1);
}

void	CharpyDemo::restart()
{
	CharpyDemo::clearParameterUi();
	CharpyDemo::exitPhysics();
	PlasticityExampleBrowser::getRenderer()->removeAllInstances();
	CharpyDemo::initParameterUi();
	CharpyDemo::initPhysics();
}

void updateMaxForces(int baseIndex, const btVector3 &v){
	for (int i = 0; i < 3; i++){
		int j = baseIndex + i;
		float absValue = btFabs(v.m_floats[i]);
		if (absValue>maxForces[j]){
			maxForces[j] = absValue;
		}
	}
}

/** 
Get displacement of specimen part at center line
*/
const btVector3 getDisplacements(){
	btRigidBody *b = specimenBody;
	btTransform r = b->getCenterOfMassTransform();
	btBoxShape* box = (btBoxShape*)b->getCollisionShape();
	btVector3 he = box->getHalfExtentsWithMargin();
	btVector3 l = r*(he);
	const btVector3 v(l.x() - 2*he.x(), l.y() - 2*he.y()-0.2, l.z());
	return v;
}

void addDisplacements(char *buf, const btVector3 &v){
	sprintf_s(buf, B_LEN, "Displacements:X/Y/Z % 8.2g/% 8.2g/% 8.2g m",
		v.m_floats[0], v.m_floats[1], v.m_floats[2]);
}

void addForces(char *buf, const btVector3 &v){
	sprintf_s(buf,B_LEN, "Constraint forces:X/Y/Z % 8.2g/% 8.2g/% 8.2g N",
		v.m_floats[0], v.m_floats[1], v.m_floats[2]);
	updateMaxForces(0,v);
}

void addMoments(char *buf, const btVector3 &v){
	sprintf_s(buf,B_LEN, "Constraint moments:X/Y/Z  % 8.2g/% 8.2g/% 8.2g Nm",
		v.m_floats[0], v.m_floats[1], v.m_floats[2]);
	updateMaxForces(3, v);
}

void checkConstraints(){
}


const char * PROFILE_SLEEP = "CharpyDemo::Sleep";
void CharpyDemo::renderScene(){
	{
		BT_PROFILE("CharpyDemo::syncPhysicsToGraphics");
		m_guiHelper->syncPhysicsToGraphics(m_dynamicsWorld);
	}
	{
		BT_PROFILE("CharpyDemo::showMessage");
		showMessage();
	}
	{
		BT_PROFILE("CharpyDemo::render");
		m_guiHelper->render(m_dynamicsWorld);
	}
#ifdef _WIN32
	if (displayWait>0){
		BT_PROFILE(PROFILE_SLEEP);
		Sleep(displayWait);
	}
#endif
	updatePauseButtonText();
}

void CharpyDemo::stepSimulation(float deltaTime){
	if (restartRequested){
		restart();
		restartRequested = false;
	}
	if (m_dynamicsWorld)	{
		PlasticityData::setTime(currentTime);
		m_dynamicsWorld->stepSimulation(timeStep, 30, timeStep);
		{
			BT_PROFILE("CharpyDemo::stepStimulationExtras");
			checkCollisions();
			updateEnergy();
			checkConstraints();
			currentTime += timeStep;
			writeGraphData();
			writeRigidBodyData();
			updateView();
		}
	}
}

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
void CharpyDemo::showMessage()
{	
	if (!PlasticityData::getCollect()){
		return;
	}
	if (restartRequested){
		return;
	}
	pData.clear();
	int ci = sCount - 1; // constraint index for e.g. feedback in the middle
	char buf[B_LEN];
	sprintf_s(buf, B_LEN, "energy:max/current/loss %9.3g/%9.3g/%9.3g J", 
		maxEnergy,energy,maxEnergy-energy);
	infoMsg(buf);
	addDisplacements(buf, getDisplacements());
	infoMsg(buf);
	if (specimenJointFeedback.size()>ci){
		addForces(buf, specimenJointFeedback[ci]->m_appliedForceBodyA);
		infoMsg(buf);
		addMoments(buf, specimenJointFeedback[ci]->m_appliedTorqueBodyA);
		infoMsg(buf);
	}
	sprintf_s(buf,B_LEN, "minCollisionDistance: simulation/step % 1.3f/% 1.3f",
		minCollisionDistance, minCurrentCollisionDistance);
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "{/} to change displayWait(%3ld ms), now=%3ld ms", setDisplayWait,displayWait);
	infoMsg(buf);
	if (m_mode == 5){
		sprintf_s(buf,B_LEN, "hingeAngle=% 2.3f",
			btFabs(mode5Hinge[ci]->getHingeAngle()));
		infoMsg(buf);
	}
	switch (m_mode){
	case 1:
		break;
	case 6:
		sprintf_s(buf,B_LEN, "^a/^A to change mpr, "
			"mpr=%1.3f, cpr=%1.3f, ha=%1.3f",
			mode6Hinge[ci]->getMaxPlasticRotation(),
			mode6Hinge[ci]->getCurrentPlasticRotation(),
			btFabs(mode6Hinge[ci]->getHingeAngle()));
		infoMsg(buf);
		break;
	case 7:
		sprintf_s(buf,B_LEN, "^a/^A for mpr, "
			"mpr=%1.3f, cpr=%1.3f, mps=%1.3f, cps=%1.3f",
			mode7c[ci]->getMaxPlasticRotation(),
			mode7c[ci]->getCurrentPlasticRotation(),
			mode7c[ci]->getMaxPlasticStrain(),
			mode7c[ci]->getCurrentPlasticStrain()
			);
		infoMsg(buf);
		break;
	case 8:
		sprintf_s(buf, B_LEN, "^a/^A for mpr, "
			"mpr=%1.3f, cpr=%1.3f, mps=%1.3f, cps=%1.3f",
			mode8c[ci]->getMaxPlasticRotation(),
			mode8c[ci]->getCurrentPlasticRotation(),
			mode8c[ci]->getMaxPlasticStrain(),
			mode8c[ci]->getCurrentPlasticStrain()
			);
		infoMsg(buf);
		break;
	default:
		sprintf_s(buf,B_LEN, "^a/^A to change mpr, mpr=%1.3f, bith=%6.3f",
			maxPlasticRotation,
			breakingImpulseThreshold);
		infoMsg(buf);
		break;
	}
	switch (m_mode){
	case 1:
		break;
	default:
		sprintf_s(buf,B_LEN, "%sbroken",
			(isBroken()?"":"un"));
		infoMsg(buf);
		break;
	}
	sprintf_s(buf,B_LEN, "timeStep ./:/,/; now=%2.3f/%2.3f ms, auto(;)=%s",
	setTimeStep*1000,timeStep*1000,(variableTimeStep?"on":"off"));
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "^n/^N for numIterations, %d", numIterations);
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "k/K v/V to change width/length, now=%1.4f/%1.4f m", w, l);
	infoMsg(buf);
	sprintf_s(buf,B_LEN,"mode(F1-F8)=F%d: %s",m_mode,modes[m_mode]);
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "viewMode(<Shift>F1-F3)=F%d: %s", m_viewMode, viewModes[m_viewMode]);
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "solverType(<Ctrl>F1-F4)=F%d: %s", solverType, solverTypeNames[solverType]);
	infoMsg(buf);
	if (false){ // these do not currently seem interesting
		sprintf_s(buf,B_LEN, "</> to change ccdMotionThreshHold, now=%1.8f m",
			ccdMotionThreshHold);
		infoMsg(buf);
		sprintf_s(buf,B_LEN, "e/E to change margin, now=%1.8f m",margin);
		infoMsg(buf);
		sprintf_s(buf,B_LEN, "j/J to change floor half extents, now=%1.6f m",
			floorHE);
		infoMsg(buf);
	}
	if (openGraphFile){
		sprintf_s(buf,B_LEN,"Writing force data to %s, disable with ^d",gfn);
	}else{
		sprintf_s(buf,B_LEN, "Enable writing force data to file with ^d");
	}
	infoMsg(buf);
	if (openRigidBodyDataFile){
		sprintf_s(buf, B_LEN, "Writing rigid body data to %s, disable with ^g", rbdfn);
	}
	else{
		sprintf_s(buf, B_LEN, "Enable writing rigid body data to file with ^g");
	}
	infoMsg(buf);
	if (PlasticityData::getLogData()){
		sprintf_s(buf, B_LEN, "Writing plasticity log data to %s, disable with ^p", 
			PlasticityData::getLogDataFilename());
	}
	else{
		sprintf_s(buf, B_LEN, "Enable writing plasticity log data to file with ^p");
	}
	infoMsg(buf);
	sprintf_s(buf, B_LEN, "currentTime=%3.5f s, realTime=%3.1f currentAngle=%1.4f",
		currentTime, rtClock.getTimeSeconds(), getHammerAngle());
	infoMsg(buf);
	PlasticityData::setData(&pData);
}

void CharpyDemo::updateView(){
	btRigidBody* body;
	switch (m_viewMode){
	case 2:
		body = specimenBody;
		break;
	case 3:
		body = specimenBody2;
		break;
	default:
		return;
	}
	btVector3 comp=body->getCenterOfMassPosition();
	CommonCameraInterface* camera = PlasticityExampleBrowser::getRenderer()->getActiveCamera();
	camera->setCameraTargetPosition(comp.x(), comp.y(), comp.z());
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
	CommonCameraInterface* camera = 
		PlasticityExampleBrowser::getRenderer()->getActiveCamera();
	camera->setCameraDistance(btScalar(2.5));
	camera->setCameraUpVector(0,1,0);
	camera->setFrustumZNear(0.01);
	camera->setFrustumZFar(10);
	if (viewMode == 1){
		camera->setCameraTargetPosition(0, 0.2, 0);
	}
	else{
		updateView();
	}
}

void setSolverType(int val){
	solverType = val;
}


void initG(){
	G = E / (2 * (1 + nu));
}
/** 
Bring back to state right after starting
*/
void reinit(){
	startAngle = initialStartAngle;
	displayWait = initialDisplayWait;
	setDisplayWait = displayWait;
	timeStep = defaultTimeStep;
	setTimeStep = initialTimeStep;
	numIterations = initialNumIterations;
	variableTimeStep = true;
	restitution = initialRestitution;
	maxPlasticRotation = initialMaxPlasticRotation;
	solverType = initialSolverType;
	sCount = initialSCount;
	E = initialE;
	initG();
	fy = initialFy;
	l = initialL;
	w = initialW;
	h = initialH;
	spaceBetweenAnvils = initialSpaceBetweenAnvils;
	hammerThickness = initialHammerThickness;
	hammerDraft = initialHammerDraft;
	notchSize = initialNotchSize;
	frequencyRatio = initialFrequencyRatio;
	limitIfNeeded = initialLimitIfNeeded;
	damping = initialDamping;
	firstRun = true;
	if (openGraphFile){
		toggleGraphFile();
	}
	if (openRigidBodyDataFile){
		toggleRigidBodyDataFile();
	}
}


/*
CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;
if (window->isModifierKeyPressed(B3G_ALT))
if (window->isModifierKeyPressed(B3G_SHIFT))
if (window->isModifierKeyPressed(B3G_CONTROL))
*/

/*
handle control keys
@return true if scene should be reset i.e. simulation should be restarted
*/
bool CharpyDemo::ctrlKeyboardCallback(int key){
	bool shiftActive=false;
	CommonWindowInterface* window = m_guiHelper->getAppInterface()->m_window;
	if(window->isModifierKeyPressed(B3G_SHIFT)){
		shiftActive = true;
	}
	switch (key){
	case B3G_F1:
		setSolverType(1);
		return true;
	case B3G_F2:
		setSolverType(2);
		return true;
	case B3G_F3:
		setSolverType(3);
		return true;
	case B3G_F4:
		setSolverType(lemkeSolverType);
		return true;
	case 1: // a
		if (shiftActive){
			scalePlasticity(1.2);
		}
		else{
			scalePlasticity(0.8);
		}
		break;
	case 2: //b
		if (shiftActive){
			if (restitution < 1){
				restitution += 0.1;
				return true;
			}
		}
		else if (restitution>0.){
			restitution -= 0.1;
			return true;
		}
		break;
	case 3: //c
		if (shiftActive){
			fy *= 1.2;
		} else {
			fy /= 1.2;
		}
		return true;
	case 4: //d
		toggleGraphFile();
		return false;
	case 5: //e
		if (shiftActive){
			E *= 1.2;
		}
		else {
			E /= 1.2;
		}
		initG();
		return true;
	case 6: //f
		if (shiftActive){
			frequencyRatio *= 1.2;
		}
		else {
			frequencyRatio /= 1.2;
		}
		return true;
	case 7: //g
		toggleRigidBodyDataFile();
		return false;
	case 8: //h
		return false;
	case 9: //i
		return false;
	case 10: //j
		return false;
	case 11: //k
		return false;
	case 12: //l
		return false;
	case 13: //m
		return false;
	case 14: //n
		if (shiftActive){
			numIterations++;
		}
		else if (numIterations>1){
			numIterations--;
		}
		if (NULL!=dw){
			dw->getSolverInfo().m_numIterations = numIterations;
		}
		return false;
	case 15: //o
		return false;
	case 16: //p
		toggleLogPlasticityData();
		return false;
	case 17: //q
		return false;
	case 18: //r
		reinit();
		return true;
	case 19: //s
		return false;
	case 20: //t
		return false;
	case 21: //u
		return false;
	case 22: //v
		return false;
	case 23: //w
		return false;
	case 24: //x
		return false;
	case 25: //y
		return false;
	case 26: //z
		return false;
	case '{':
		if (setDisplayWait < 10 && setDisplayWait>0){
			setDisplayWait--;
		}
		else{
			setDisplayWait = (long)(setDisplayWait / 1.2);
		}
		displayWait = setDisplayWait;
		break;
	case '}':
		if (setDisplayWait < 10){
			setDisplayWait++;
		}
		else{
			setDisplayWait *= 1.2;
		}
		displayWait = setDisplayWait;
		break;
	}
	return false;
}
/** 
Handles keyboard activity.
<ul>
<li> drops keyboard focus if needed
<li> offers to gwen
<li> handles own keys
</ul>
*/
bool CharpyDemo::keyboardCallback(int key, int state){
	if (dropFocus){
		Gwen::KeyboardFocus = NULL;
		dropFocus = false;
	}
	bool bDown = (state==1?true:false);
	bool handled = BulletKeyToGwen::keyboardCallback(canvas, key, bDown);
	if (handled){
		return true;
	}
	/// no actions for modifiers
	switch (key){
	case B3G_SHIFT:
	case B3G_CONTROL :
	case B3G_ALT :
			return true;
	}
	if (window->isModifierKeyPressed(B3G_CONTROL)){
		if (ctrlKeyboardCallback(key)){
			restart();
		}
		return true;
	}
	bool resetScene = false;
	handled = true;
	switch (key)
	{
	case '+':
			startAngle+=0.1;
			restart();
			break;
	case '-':
			startAngle-=0.1;
			resetScene=true;
			break;
	case '(':
			if (damping>0.){
				damping -= 0.1;
				resetScene=true;
			}
			break;
	case ')':
			if (damping < 1){
				damping += 0.1;
				resetScene=true;
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
		if (window->isModifierKeyPressed(B3G_SHIFT)){
			margin /= 0.8;
		}
		else{
			margin *= 0.8;
		}
		resetCollisionMargin();
		break;
	case 'j':
		if (window->isModifierKeyPressed(B3G_SHIFT)){
			floorHE /= 0.8;
		}
		else{
			floorHE *= 0.8;
		}
	case 'k':
		if (window->isModifierKeyPressed(B3G_SHIFT)){
			w /= 0.8;
		}
		else{
			w *= 0.8;
		}
		resetScene=true;
		break;
	case 'v':
		if (window->isModifierKeyPressed(B3G_SHIFT)){
			l /= 0.8;
		}
		else{
			l *= 0.8;
		}
		resetScene=true;
		break;
	case ':':
		setTimeStep=btScalar(setTimeStep/0.8);
		break;
	case ' ':
		resetScene = true;
		break;
	case '.':
		setTimeStep = btScalar(setTimeStep*0.8);
		break;
	case ',':
		setTimeStep = initialTimeStep;
		break;
	case ';':
		variableTimeStep = !variableTimeStep;
		displayWait = setDisplayWait;
		break;
	case B3G_F1:
		if (window->isModifierKeyPressed(B3G_SHIFT)){
			setViewMode(1);
		}
		else{
			m_mode = 1;
			resetScene = true;
		}
		break;
	case B3G_F2:
		if (window->isModifierKeyPressed(B3G_SHIFT)){
			setViewMode(2);
		}
		else{
			m_mode = 2;
			resetScene = true;
		}
		break;
	case B3G_F3:
	 	if (window->isModifierKeyPressed(B3G_SHIFT)){
			setViewMode(3);
		}
		else{
			m_mode = 3;
			resetScene = true;
		}
		break;
	case B3G_F4:
		if (window->isModifierKeyPressed(B3G_CONTROL)){
		}
		else if (window->isModifierKeyPressed(B3G_SHIFT)){
		}
		else{
			m_mode = 4;
			resetScene = true;
		}
		break;
	case B3G_F5:
		if (window->isModifierKeyPressed(B3G_CONTROL)){
		}
		else if (window->isModifierKeyPressed(B3G_SHIFT)){
		}
		else{
			m_mode = 5;
			resetScene = true;
		}
		break;
	case B3G_F6:
		if (window->isModifierKeyPressed(B3G_CONTROL)){
		}
		else if (window->isModifierKeyPressed(B3G_SHIFT)){
		}
		else{
			m_mode = 6;
			resetScene = true;
		}
		break;
	case B3G_F7:
		if (window->isModifierKeyPressed(B3G_CONTROL)){
		}
		else if (window->isModifierKeyPressed(B3G_SHIFT)){
		}
		else{
			m_mode = 7;
			resetScene = true;
		}
		break;
	case B3G_F8:
		if (window->isModifierKeyPressed(B3G_CONTROL)){
		}
		else if (window->isModifierKeyPressed(B3G_SHIFT)){
		}
		else{
			m_mode = 8;
			resetScene = true;
		}
		break;
	default:
		handled = false;
	}
	if (resetScene)	{
		restart();
	}
	return handled;
}


void	CharpyDemo::exitPhysics()
{
	b3Printf("maxCollision was %f m",(float)(-1.*minCollisionDistance));
	b3Printf("maxImpact was %f Ns", maxImpact);
	b3Printf("maxSpeed was %f m/s, energyLoss was %f J", sqrtf(maxSpeed2),maxEnergy-energy);
	b3Printf("maximum constraint forces were: %6.2f %6.2f %6.2f %6.2f %6.2f %6.2f",
		maxForces[0], maxForces[1], maxForces[2], maxForces[3], maxForces[4], maxForces[5]);
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

	delete basePoint;
	basePoint = 0;
	for (int j = 0; j<specimenJointFeedback.size(); j++)
	{
		btJointFeedback* jf = specimenJointFeedback[j];
		delete jf;
	}
	specimenJointFeedback.clear();
	for (int j = 0; j<mode5HingeW1s.size(); j++)
	{
		btScalar* f = mode5HingeW1s[j];
		delete f;
	}
	hammerBody = 0;
	mode5HingeW1s.clear();
	mode5Hinge.clear();
	mode6Hinge.clear();
	mode7c.clear();
	mode8c.clear();
	tc.clear();
}
CommonExampleInterface*    CharpyDemoCreateFunc(CommonExampleOptions& options)
{
	charpyDemo=new CharpyDemo(options);
	return charpyDemo;
}
