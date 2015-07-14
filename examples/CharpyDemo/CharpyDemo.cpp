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
#include "../exampleBrowser/GwenGUISupport/gwenUserInterface.h"
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
btScalar initialL(0.055);
btScalar l(initialL);
btScalar initialW(0.01);
btScalar w(initialW);
btScalar initialE(200E9); // Steel
btScalar E(initialE);
btScalar nu(0.3); // Steel
btScalar G(initialE / (2 * (1 + nu)));
btScalar initialFu(400e6);
btScalar fu(initialFu);
btScalar initialDamping(0.2);
btScalar damping(initialDamping);
btScalar initialFrequencyRatio(10);
btScalar frequencyRatio(initialFrequencyRatio);
float energy = 0;
float maxEnergy;
btDynamicsWorld* dw;
btVector3 y_up(0.01, 1., 0.01);
btRigidBody *specimenBody, *hammerBody, *specimenBody2;
btHingeConstraint *hammerHinge;
btJointFeedback hammerHingeJointFeedback;
btJointFeedback specimenJointFeedback;
btHingeConstraint *mode5Hinge;
btPlasticHingeConstraint *mode6Hinge;
bt6DofElasticPlasticConstraint *mode7c;
bt6DofElasticPlastic2Constraint *mode8c;
btScalar initialRestitution(0.);
btScalar restitution(initialRestitution);
btScalar initialMaxPlasticRotation(3.);
btScalar maxPlasticRotation(initialMaxPlasticRotation);
btTypedConstraint *tc; // points to specimen constraint
btScalar breakingImpulseThreshold = 0;
btScalar w1;
float maxForces[6];
FILE *fp;
bool openGraphFile = false;
char gfn[B_LEN];
int initialSolverType = 1;
int solverType = initialSolverType;
btConstraintSolverType solverTypes[] = {
	BT_SEQUENTIAL_IMPULSE_SOLVER,
	BT_SEQUENTIAL_IMPULSE_SOLVER,
	BT_MLCP_SOLVER,
	BT_NNCG_SOLVER
};
const char * solverTypeNames[] = {
	"",
	"SI (Sequential Impulse)",
	"MLCP (Mixed Linear Complementarity Problem)",
	"NNCG (Nonlinear Nonsmooth Conjugate Gradient)"
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
		btDantzigSolver* mlcp = new btDantzigSolver();
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


static GwenUserInterface* gui;
static Gwen::Controls::Canvas* canvas;
int gx = 10;
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
	float fv = std::stof(text);
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

class CharpyDemo : public Gwen::Event::Handler, public CommonRigidBodyBase
{

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;
	int m_viewMode=1;
	int m_mode;
	void showMessage();

	CommonWindowInterface* window;
public:
	CharpyDemo(struct GUIHelperInterface* helper, int mode)
		:CommonRigidBodyBase(helper), Gwen::Event::Handler()
	{
		m_mode = mode;
		initParameterUi();
	}
	~CharpyDemo()
	{
		clearParameterUi();
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
	void setFu(Gwen::Controls::Base* control){
		btScalar tv(fu / 1e6);
		setScalar(control, &tv);
		fu = tv*1e6;
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
	void setUiTimeStep(Gwen::Controls::Base* control){
		btScalar tv(setTimeStep*1e3);
		setScalar(control, &tv);
		setTimeStep = tv/1e3;
		dropFocus = true;
	}
	void setUiDisplayWait(Gwen::Controls::Base* control){
		setLong(control, &setDisplayWait);
		dropFocus = true;
	}

	Gwen::Controls::Base* pPage;
	void addLabel(string txt){
		Gwen::Controls::Label* gc = new Gwen::Controls::Label(pPage);
		gc->SetText(txt);
		gc->SizeToContents();
		gc->SetPos(gx, gy);
		gy = gc->Bottom()+1;
	}
	void addRestartButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Restart");
		gc->SetPos(gx, gy);
		gy += gyInc;
		gc->onPress.Add(pPage, &CharpyDemo::restartHandler);
	}
	void addResetButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Reset");
		gc->SetPos(gx, gy);
		gy += gyInc;
		gc->onPress.Add(pPage, &CharpyDemo::resetHandler);
	}
	void handlePauseSimulation(Gwen::Controls::Base* control){
		Gwen::Controls::Button* gc =
			static_cast<Gwen::Controls::Button*>(control);
		bool pauseSimulation = PlasticityExampleBrowser::getPauseSimulation();
		pauseSimulation=!pauseSimulation;
		PlasticityExampleBrowser::setPauseSimulation(pauseSimulation);
		if (pauseSimulation){
			gc->SetText(L"Continue");
		}
		else{
			gc->SetText(L"Pause");
		}
	}

	void addPauseSimulationButton(){
		Gwen::Controls::Button* gc = new Gwen::Controls::Button(pPage);
		gc->SetText(L"Pause");
		gc->SetPos(gx, gy);
		gy += gyInc;
		gc->onPress.Add(pPage, &CharpyDemo::handlePauseSimulation);
	}
	void addStartAngle(){
		addLabel("startAngle +/-");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(startAngle);
		gc->SetText(text);
		gc->SetToolTip("In radians");
		gc->SetPos(gx, gy);
		gc->SetWidth(100);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setStartAngle);
	}
	void addE(){
		addLabel("E [GPa]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(E/1e9);
		gc->SetText(text);
		gc->SetToolTip("Young's modulus");
		gc->SetPos(gx, gy);
		gc->SetWidth(100);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setE);
	}
	void addFu(){
		addLabel("fu [MPa]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(fu / 1e6);
		gc->SetText(text);
		gc->SetToolTip("Ultimate strength");
		gc->SetPos(gx, gy);
		gc->SetWidth(100);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setFu);
	}
	void addL(){
		addLabel("length [m]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(l);
		gc->SetText(text);
		gc->SetToolTip("Length of specimen");
		gc->SetPos(gx, gy);
		gc->SetWidth(100);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setL);
	}
	void addW(){
		addLabel("width [m]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(w);
		gc->SetText(text);
		gc->SetToolTip("Width of specimen");
		gc->SetPos(gx, gy);
		gc->SetWidth(100);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setW);
	}
	void addRestitution(){
		addLabel("Restitution");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(restitution);
		gc->SetText(text);
		gc->SetPos(gx, gy);
		gc->SetWidth(100);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setRestitution);
	}
	void addDamping(){
		addLabel("Damping");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(damping);
		gc->SetToolTip("Damping in spring constraints");
		gc->SetText(text);
		gc->SetPos(gx, gy);
		gc->SetWidth(100);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setDamping);
	}
	void addFrequencyRatio(){
		addLabel("FrequencyRatio");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(frequencyRatio);
		gc->SetToolTip("How many simulation steps are required for spring period");
		gc->SetText(text);
		gc->SetPos(gx, gy);
		gc->SetWidth(100);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setFrequencyRatio);
	}
	void addTimeStep(){
		addLabel("TimeStep [ms]");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(setTimeStep*1000);
		gc->SetText(text);
		gc->SetPos(gx, gy);
		gc->SetWidth(100);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setUiTimeStep);
	}
	void addDisplayWait(){
		addLabel("DisplayWait");
		Gwen::Controls::TextBoxNumeric* gc = new Gwen::Controls::TextBoxNumeric(pPage);
		string text = std::to_string(displayWait);
		gc->SetText(text);
		gc->SetPos(gx, gy);
		gc->SetWidth(100);
		gy += gyInc;
		gc->onReturnPressed.Add(pPage, &CharpyDemo::setUiDisplayWait);
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
		addStartAngle();
		addE();
		addFu();
		addL();
		addW();
		addRestitution();
		if (isDampingUsed()){
			addDamping();
		}
		if (isFrequencyRatioUsed()){
			addFrequencyRatio();
		}
		addTimeStep();
		addDisplayWait();
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
		wgfv("%8.4f,", currentTime);
		wgv(energy);
		wgv(maxEnergy - energy);
		if (m_mode>1){
			for (int i = 0; i < 3; i++){
				wgv(specimenJointFeedback.m_appliedForceBodyA[i]);
			}
			for (int i = 0; i < 3; i++){
				wgv(specimenJointFeedback.m_appliedTorqueBodyA[i]);
			}
		}
		fprintf(fp, ";\n");
	}
	btScalar getBodySpeed2(const btCollisionObject* o){
		return o->getInterpolationLinearVelocity().length2();
	}

	btScalar getManifoldSpeed2(btPersistentManifold* manifold){
		btScalar s0 = getBodySpeed2(manifold->getBody0());
		btScalar s1 = getBodySpeed2(manifold->getBody1());
		return s0>s1 ? s0 : s1;
	}
	btScalar getHammerAngle(){
		return hammerBody->getCenterOfMassTransform().getRotation().getZ();
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
	}
	/* Just for making breaking possible.
	This calculation has no known scientific background.
	Maximum impulse that can be generated is about 300 Ns
	(20 kg * 15 m/s)
	For basic case with 1.8 start angle 120 Ns (20 kg * 6 m/s)
	*/
	btScalar getBreakingImpulseThreshold(){
		btScalar b(w);
		btScalar h(w - 0.002); // notch is 2 mm
		btScalar w1(fu*b*h*h / 4);
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
			mode6Hinge->setMaxPlasticRotation(maxPlasticRotation);
			break;
		case 7:
			mode7c->scalePlasticity(scale);
			break;
		case 8:
			mode8c->scalePlasticity(scale);
			break;
		}
	}
	void addSpringConstraint(btAlignedObjectArray<btRigidBody*> ha,
		btAlignedObjectArray<btTransform> ta){
		btGeneric6DofSpringConstraint *sc =
			new btGeneric6DofSpringConstraint(*ha[0], *ha[1],
			ta[0], ta[1], true);
		tc = sc;
		sc->setBreakingImpulseThreshold(getBreakingImpulseThreshold());
		sc->setJointFeedback(&specimenJointFeedback);
		btScalar b(w);
		btScalar h(w - 0.002); // notch is 2 mm
		btScalar I1(b*h*h*h / 12);
		btScalar I2(h*b*b*b / 12);
		btScalar k0(E*b*h / l / 2);
		btScalar k1(48 * E*I1 / l / l / l);
		btScalar k2(48 * E*I1 / l / l / l);
		w1 = fu*b*h*h / 4;
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

	void addSpring2Constraint(btAlignedObjectArray<btRigidBody*> ha,
		btAlignedObjectArray<btTransform> ta){
		btGeneric6DofSpring2Constraint *sc =
			new btGeneric6DofSpring2Constraint(*ha[0], *ha[1],
			ta[0], ta[1]);
		tc = sc;
		sc->setBreakingImpulseThreshold(getBreakingImpulseThreshold());
		sc->setJointFeedback(&specimenJointFeedback);
		btScalar b(w);
		btScalar h(w - 0.002); // notch is 2 mm
		btScalar I1(b*h*h*h / 12);
		btScalar I2(h*b*b*b / 12);
		btScalar k0(E*b*h / l / 2);
		btScalar k1(48 * E*I1 / l / l / l);
		btScalar k2(48 * E*I1 / l / l / l);
		w1 = fu*b*h*h / 4;
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


	void addFixedConstraint(btAlignedObjectArray<btRigidBody*> ha,
		btAlignedObjectArray<btTransform> ta){
		btGeneric6DofConstraint *sc =
			new btGeneric6DofConstraint(*ha[0], *ha[1],
			ta[0], ta[1], true);
		tc = sc;
		sc->setJointFeedback(&specimenJointFeedback);
		sc->setBreakingImpulseThreshold(getBreakingImpulseThreshold());
		dw->addConstraint(sc, true);
		for (int i = 0; i<6; i++){
			sc->setLimit(i, 0, 0); // make fixed
		}
	}

	void addHingeConstraint(btAlignedObjectArray<btRigidBody*> ha){
		btVector3 pivotAxis(btZero, btScalar(1), btZero);
		btVector3 pivotInA(btZero, btZero, l / 4);
		btVector3 pivotInB(btZero, btZero, -l / 4);
		btHingeConstraint *sc =
			new btHingeConstraint(*ha[0], *ha[1],
			pivotInA, pivotInB, pivotAxis, pivotAxis, false);
		tc = sc;
		sc->setBreakingImpulseThreshold(getBreakingImpulseThreshold());
		mode5Hinge = sc;
		tc = sc;
		btScalar b(w);
		btScalar h(w - 0.002); // notch is 2 mm
		w1 = fu*b*h*h / 4;
		sc->setJointFeedback(&specimenJointFeedback);
		sc->setLimit(-SIMD_PI, SIMD_PI); // until parts are overturn
		sc->enableAngularMotor(true, 0, w1*timeStep);
		dw->addConstraint(sc, true);
	}

	/*
	mode 6
	*/
	void addPlasticHingeConstraint(btAlignedObjectArray<btRigidBody*> ha){
		btVector3 pivotAxis(btZero, btScalar(1), btZero);
		btVector3 pivotInA(btZero, btZero, l / 4);
		btVector3 pivotInB(btZero, btZero, -l / 4);
		btPlasticHingeConstraint *sc =
			new btPlasticHingeConstraint(*ha[0], *ha[1],
			pivotInA, pivotInB, pivotAxis, pivotAxis, false);
		tc = sc;
		mode6Hinge = sc;
		mode6Hinge->setMaxPlasticRotation(maxPlasticRotation);
		btScalar b(w);
		btScalar h(w - 0.002); // notch is 2 mm
		w1 = fu*b*h*h / 4;
		sc->setJointFeedback(&specimenJointFeedback);
		sc->setLimit(-SIMD_HALF_PI, SIMD_HALF_PI); // until parts are overturn
		sc->setPlasticMoment(w1);
		sc->setLimit(0, 0);
		dw->addConstraint(sc, true);
	}

	/*
	Mode 7
	*/
	void addElasticPlasticConstraint(btAlignedObjectArray<btRigidBody*> ha,
		btAlignedObjectArray<btTransform> ta){
		bt6DofElasticPlasticConstraint *sc =
			new bt6DofElasticPlasticConstraint(*ha[0], *ha[1],
			ta[0], ta[1], true);
		tc = sc;
		mode7c = sc;
		mode7c->setMaxPlasticRotation(maxPlasticRotation);
		mode7c->setMaxPlasticStrain(w);
		sc->setJointFeedback(&specimenJointFeedback);
		btScalar b(w);
		btScalar h(w - 0.002); // notch is 2 mm
		btScalar I1(b*h*h*h / 12);
		btScalar I2(h*b*b*b / 12);
		btScalar It(0.14*b*h*h*h);
		btScalar k0(E*b*h / l / 2);
		btScalar k1(48 * E*I1 / l / l / l);
		btScalar k2(48 * E*I1 / l / l / l);
		w1 = fu*b*h*h / 4;
		btScalar w2(fu*b*b*h / 4);
		sc->setStiffness(0, k0);
		sc->setMaxForce(0, fu*b*h);
		sc->setStiffness(1, k1);
		sc->setMaxForce(1, fu*b*h / 2);
		sc->setStiffness(2, k2);
		sc->setMaxForce(2, fu*b*h / 2);
		sc->setStiffness(3, G*It / l); // not very exact
		sc->setMaxForce(3, fu / 2 * It / (h / 2)); // not very exact
		sc->setStiffness(4, 3 * E*I1 / l); // not very exact
		sc->setMaxForce(4, w2);
		sc->setStiffness(5, 3 * E*I2 / l); // not very exact
		sc->setMaxForce(5, w1);
		dw->addConstraint(sc, true);
		for (int i = 0; i<6; i++)
		{
			sc->enableSpring(i, true);
		}
		for (int i = 0; i<6; i++)
		{
			sc->setDamping(i, damping);
		}
		sc->setFrequencyRatio(frequencyRatio);
		sc->setEquilibriumPoint();
	}

	/**
	mode 8
	*/
	void addElasticPlastic2Constraint(btAlignedObjectArray<btRigidBody*> ha,
		btAlignedObjectArray<btTransform> ta){
		bt6DofElasticPlastic2Constraint *sc =
			new bt6DofElasticPlastic2Constraint(*ha[0], *ha[1],
			ta[0], ta[1]);
		tc = sc;
		tc = sc;
		mode8c = sc;
		mode8c->setMaxPlasticRotation(maxPlasticRotation);
		mode8c->setMaxPlasticStrain(w);
		sc->setJointFeedback(&specimenJointFeedback);
		btScalar b(w);
		btScalar h(w - 0.002); // notch is 2 mm
		btScalar I1(b*h*h*h / 12);
		btScalar I2(h*b*b*b / 12);
		btScalar It(0.14*b*h*h*h);
		btScalar k0(E*b*h / l / 2);
		btScalar k1(48 * E*I1 / l / l / l);
		btScalar k2(48 * E*I1 / l / l / l);
		w1 = fu*b*h*h / 4;
		btScalar w2(fu*b*b*h / 4);
		sc->setStiffness(0, k0);
		sc->setMaxForce(0, fu*b*h);
		sc->setStiffness(1, k1);
		sc->setMaxForce(1, fu*b*h / 2);
		sc->setStiffness(2, k2);
		sc->setMaxForce(2, fu*b*h / 2);
		sc->setStiffness(3, G*It / l); // not very exact
		sc->setMaxForce(3, fu / 2 * It / (h / 2)); // not very exact
		sc->setStiffness(4, 3 * E*I1 / l); // not very exact
		sc->setMaxForce(4, w2);
		sc->setStiffness(5, 3 * E*I2 / l); // not very exact
		sc->setMaxForce(5, w1);
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

void tuneRestitution(btRigidBody* rb){
	rb->setRestitution(restitution);
}

void tuneMode5(){
	// this is currently no-op if target speed is zero
	mode5Hinge->setMaxMotorImpulse(w1*timeStep);
}

/**
* tunes hammerSpeed
* currently mostly no-op as moment is handled in btHingeConstraint.
*/
void tuneMode6(){
	if (!mode6Hinge->isEnabled()){
		return;
	}
	if (!hammerHitsSpecimen){
		return;
	}
	btScalar applied = specimenJointFeedback.m_appliedTorqueBodyA[1];
	btScalar capacity = mode6Hinge->getPlasticMoment();
	if (applied >= capacity){
		mode6Hinge->updateCurrentPlasticRotation();
	}
	return;
	btScalar energyLoss = mode6Hinge->getAbsorbedEnergy();
	if (energyLoss > 0){
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
	if (!mode7c->isEnabled()){
		return;
	}
	mode7c->updatePlasticity(specimenJointFeedback);
}
void mode8callback(btDynamicsWorld *world, btScalar timeStep) {
	if (!mode8c->isEnabled()){
		return;
	}
	mode8c->updatePlasticity(specimenJointFeedback);
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
			btRigidBody* rb=localCreateRigidBody(0.f,tr,shape);
			tuneRestitution(rb);
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
			btRigidBody* rb=localCreateRigidBody(0.f, tr, shape);
			tuneRestitution(rb);
		}
	}

	// charpy specimen using two halfs, 
	// symmetrically around z=0
	// bottom at y=0.2
	// backside at x=0
	{
		btScalar halfLength=l/4;
		if(m_mode==1){
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
		for (int i=0;(m_mode>1?i<2:i<1);i++)
		{	
			btTransform tr;
			tr.setIdentity();
			btVector3 pos(w/2,0.2+w/2,
				(m_mode>1?(i==0?-1:1)*halfLength:0.f));
			tr.setOrigin(pos);
			btDefaultMotionState* myMotionState 
				= new btDefaultMotionState(tr);
			btRigidBody::btRigidBodyConstructionInfo 
				rbInfo(sMass,myMotionState,shape,localInertia);
			btRigidBody* body =
				new btRigidBody(rbInfo);
			tuneRestitution(body);
			m_dynamicsWorld->addRigidBody(body);
			ha.push_back(body);
			btTransform ctr;
			ctr.setIdentity();
			btVector3 cpos(0,0,
				(m_mode>1?(i==0?1:-1)*halfLength:0));
			ctr.setOrigin(cpos);
			ta.push_back(ctr);
		}
		specimenBody = ha[0];
		if (m_mode != 1){
			specimenBody2 = ha[1];
		}
		switch(m_mode) {
		case 2:
			addSpringConstraint(ha,ta);
			break;
		case 3:
			addFixedConstraint(ha,ta);
			break;
		case 4:
			addSpring2Constraint(ha, ta);
		case 5:
			addHingeConstraint(ha);
			dw->setInternalTickCallback(mode5callback);
			break;
		case 6:
			addPlasticHingeConstraint(ha);
			dw->setInternalTickCallback(mode6callback);
			break;
		case 7:
			addElasticPlasticConstraint(ha,ta);
			dw->setInternalTickCallback(mode7callback);
			break;
		case 8:
			addElasticPlastic2Constraint(ha,ta);
			dw->setInternalTickCallback(mode8callback);
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
			new btBoxShape(btVector3(0.25, 0.125, 0.01));
		btScalar hMass = 0.5*0.25*0.02 * 7800;
		btTransform hTr;
		hTr.setIdentity();
		// create hammer at y=0
		compound->addChildShape(hTr, hammer);
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
		btVector3 cPos(btZero, btScalar(1.2), btZero);
		btScalar xPos;
		// tune for cases where angle is negative and hammer hits from
		// opposite (negative) side
		if (startAngle > 0){
			xPos = btScalar(0.25 + w);
		}
		else{
			xPos = btScalar(-0.25);
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
	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
//	dw->setDebugDrawer(&gDebugDrawer);
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

void addForces(char *buf, const btVector3 &v){
	sprintf_s(buf,B_LEN, "Constraint forces:X/Y/Z %9.4f/%9.4f/%9.4f N",
		v.m_floats[0], v.m_floats[1], v.m_floats[2]);
	updateMaxForces(0,v);
}

void addMoments(char *buf, const btVector3 &v){
	sprintf_s(buf,B_LEN, "Constraint moments:X/Y/Z %9.4f/%9.4f/%9.4f Nm",
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
		BT_PROFILE("CharpyDemo::render");
		m_guiHelper->render(m_dynamicsWorld);
	}
#ifdef _WIN32
	if (displayWait>0){
		BT_PROFILE(PROFILE_SLEEP);
		Sleep(displayWait);
	}
#endif

}

void CharpyDemo::stepSimulation(float deltaTime){
	if (restartRequested){
		restart();
		restartRequested = false;
	}
	if (m_dynamicsWorld)	{
		m_dynamicsWorld->stepSimulation(timeStep, 30, timeStep);
		{
			BT_PROFILE("CharpyDemo::stepStimulationExtras");
			checkCollisions();
			updateEnergy();
			checkConstraints();
			currentTime += timeStep;
			writeGraphData();
			updateView();
			showMessage();
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
	pData.clear();
	char buf[B_LEN];
	sprintf_s(buf, B_LEN, "energy:max/current/loss %9.3g/%9.3g/%9.3g J", 
		maxEnergy,energy,maxEnergy-energy);
	infoMsg(buf);
	addForces(buf, specimenJointFeedback.m_appliedForceBodyA);
	infoMsg(buf);
	addMoments(buf, specimenJointFeedback.m_appliedTorqueBodyA);
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "minCollisionDistance: simulation/step % 1.3f/% 1.3f",
		minCollisionDistance, minCurrentCollisionDistance);
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "{/} to change displayWait, now=%3ld/%3ld ms", setDisplayWait,displayWait);
	infoMsg(buf);
	if (m_mode == 5){
		sprintf_s(buf,B_LEN, "hingeAngle=% 2.3f",
			btFabs(mode5Hinge->getHingeAngle()));
		infoMsg(buf);
	}
	switch (m_mode){
	case 1:
		break;
	case 6:
		sprintf_s(buf,B_LEN, "^a/^A to change mpr, "
			"mpr=%1.3f, cpr=%1.3f, ha=%1.3f",
			mode6Hinge->getMaxPlasticRotation(),
			mode6Hinge->getCurrentPlasticRotation(),
			btFabs(mode6Hinge->getHingeAngle()));
		infoMsg(buf);
		break;
	case 7:
		sprintf_s(buf,B_LEN, "^a/^A for mpr, "
			"mpr=%1.3f, cpr=%1.3f, mps=%1.3f, cps=%1.3f",
			mode7c->getMaxPlasticRotation(),
			mode7c->getCurrentPlasticRotation(),
			mode7c->getMaxPlasticStrain(),
			mode7c->getCurrentPlasticStrain()
			);
		infoMsg(buf);
		break;
	case 8:
		sprintf_s(buf, B_LEN, "^a/^A for mpr, "
			"mpr=%1.3f, cpr=%1.3f, mps=%1.3f, cps=%1.3f",
			mode8c->getMaxPlasticRotation(),
			mode8c->getCurrentPlasticRotation(),
			mode8c->getMaxPlasticStrain(),
			mode8c->getCurrentPlasticStrain()
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
			(tc->isEnabled()?"un":""));
		infoMsg(buf);
		break;
	}
	sprintf_s(buf,B_LEN, "timeStep ./:/,/; now=%2.3f/%2.3f ms, auto(;)=%s",
	setTimeStep*1000,timeStep*1000,(variableTimeStep?"on":"off"));
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "^n/^N for numIterations, %d", numIterations);
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "k/K v/V to change width/length, now=%1.6f/%1.6f m", w, l);
	infoMsg(buf);
	sprintf_s(buf,B_LEN,"mode(F1-F8)=F%d: %s",m_mode,modes[m_mode]);
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "viewMode(<Shift>F1-F3)=F%d: %s", m_viewMode, viewModes[m_viewMode]);
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "solverType(<Ctrl>F1-F3)=F%d: %s", solverType, solverTypeNames[solverType]);
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
		sprintf_s(buf,B_LEN,"Writing data to %s, disable with ^d",gfn);
	}else{
		sprintf_s(buf,B_LEN, "Enable writing data to file with ^d");
	}
	infoMsg(buf);
	sprintf_s(buf,B_LEN, "currentTime=%3.4f s, currentAngle=%1.4f",
		currentTime, getHammerAngle());
	infoMsg(buf);
	PlasticityData::setData(pData);
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
	E = initialE;
	initG();
	fu = initialFu;
	l = initialL;
	w = initialW;
	frequencyRatio = initialFrequencyRatio;
	damping = initialDamping;
	firstRun = true;
	if (openGraphFile){
		toggleGraphFile();
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
			fu *= 1.2;
		} else {
			fu /= 1.2;
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
		if (window->isModifierKeyPressed(B3G_CONTROL)){
			setSolverType(1);
			resetScene = true;
		}
		else if (window->isModifierKeyPressed(B3G_SHIFT)){
			setViewMode(1);
		}
		else{
			m_mode = 1;
			resetScene = true;
		}
		break;
	case B3G_F2:
		if (window->isModifierKeyPressed(B3G_CONTROL)){
				setSolverType(2);
				resetScene = true;
		}
		else if (window->isModifierKeyPressed(B3G_SHIFT)){
			setViewMode(2);
		}
		else{
			m_mode = 2;
			resetScene = true;
		}
		break;
	case B3G_F3:
		if (window->isModifierKeyPressed(B3G_CONTROL)){
			setSolverType(3);
			resetScene = true;
		}
		else 	if (window->isModifierKeyPressed(B3G_SHIFT)){
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
	b3Printf("maxImpact was %f J", maxImpact);
	b3Printf("maxSpeed was %f m/s", sqrtf(maxSpeed2));
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
}
CommonExampleInterface*    CharpyDemoF1CreateFunc(CommonExampleOptions& options)
{
	return new CharpyDemo(options.m_guiHelper,1);
}
CommonExampleInterface*    CharpyDemoF2CreateFunc(CommonExampleOptions& options)
{
	return new CharpyDemo(options.m_guiHelper, 2);
}
CommonExampleInterface*    CharpyDemoF3CreateFunc(CommonExampleOptions& options)
{
	return new CharpyDemo(options.m_guiHelper, 3);
}
CommonExampleInterface*    CharpyDemoF4CreateFunc(CommonExampleOptions& options)
{
	return new CharpyDemo(options.m_guiHelper, 4);
}
CommonExampleInterface*    CharpyDemoF5CreateFunc(CommonExampleOptions& options)
{
	return new CharpyDemo(options.m_guiHelper, 5);
}
CommonExampleInterface*    CharpyDemoF6CreateFunc(CommonExampleOptions& options)
{
	return new CharpyDemo(options.m_guiHelper, 6);
}
CommonExampleInterface*    CharpyDemoF7CreateFunc(CommonExampleOptions& options)
{
	return new CharpyDemo(options.m_guiHelper, 7);
}
CommonExampleInterface*    CharpyDemoF8CreateFunc(CommonExampleOptions& options)
{
	return new CharpyDemo(options.m_guiHelper, 8);
}
