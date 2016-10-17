#include "PlasticityExampleBrowser.h"
#include "LinearMath/btQuickprof.h"
#include "../OpenGLWindow/OpenGLInclude.h"
#include "../OpenGLWindow/SimpleOpenGL2App.h"
#ifndef NO_OPENGL3
#include "../OpenGLWindow/SimpleOpenGL3App.h"
#endif
#include "../CommonInterfaces/CommonRenderInterface.h"
#ifdef __APPLE__
#include "../OpenGLWindow/MacOpenGLWindow.h"
#else
#ifdef _WIN32
#include "../OpenGLWindow/Win32OpenGLWindow.h"
#else
//let's cross the fingers it is Linux/X11
#include "../OpenGLWindow/X11OpenGLWindow.h"
#endif //_WIN32
#endif//__APPLE__
#include "../ThirdPartyLibs/Gwen/Renderers/OpenGL_DebugFont.h"

#include "Bullet3Common/b3Vector3.h"
#include "assert.h"
#include <stdio.h>
#include "../ExampleBrowser/GwenGUISupport/gwenInternalData.h"
#include "../ExampleBrowser/GwenGUISupport/gwenUserInterface.h"
#include "../Utils/b3Clock.h"
#include "../ExampleBrowser/GwenGUISupport/GwenParameterInterface.h"
#include "../ExampleBrowser/GwenGUISupport/GwenProfileWindow.h"
#include "../ExampleBrowser/GwenGUISupport/GwenTextureWindow.h"
#include "../ExampleBrowser/GwenGUISupport/GraphingTexture.h"
#include "../CommonInterfaces/Common2dCanvasInterface.h"
#include "../CommonInterfaces/CommonExampleInterface.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "../OpenGLWindow/SimpleCamera.h"
#include "../OpenGLWindow/SimpleOpenGL2Renderer.h"
#include "PlasticityExampleEntries.h"
#include "../ExampleBrowser/OpenGLGuiHelper.h"
#include "Bullet3Common/b3FileUtils.h"
#include "PlasticityStatistics.h"
#include <time.h>
#include "LinearMath/btIDebugDraw.h"

static CommonGraphicsApp* s_app=0;
CommonGraphicsApp* PlasticityExampleBrowser::getApp(){
	return s_app;
}
static CommonWindowInterface* s_window = 0;
CommonWindowInterface* PlasticityExampleBrowser::getWindow(){
	return s_window;
}
static CommonParameterInterface*	s_parameterInterface = 0;
static CommonRenderInterface*	s_instancingRenderer=0;
CommonRenderInterface* PlasticityExampleBrowser::getRenderer(){
	return s_instancingRenderer;
}
static OpenGLGuiHelper*	s_guiHelper=0;
OpenGLGuiHelper* PlasticityExampleBrowser::getGuiHelper(){
	return s_guiHelper;
}
static MyProfileWindow* s_profWindow =0;
static PlasticityStatistics* s_pStatWindow = 0;

#define DEMO_SELECTION_COMBOBOX 13
const char* startFileName = "bulletDemo.txt";

static GwenUserInterface* gui  = 0;
GwenUserInterface* PlasticityExampleBrowser::getGui(){
	return gui;
}
static int sCurrentDemoIndex = -1;
static int sCurrentHightlighted = 0;
static CommonExampleInterface* sCurrentDemo = 0;
static b3AlignedObjectArray<const char*> allNames;

static class PlasticityExampleEntries* gAllExamples=0;
bool sUseOpenGL2 = false;
bool drawGUI=true;
#ifndef USE_OPENGL3
extern bool useShadowMap;
#endif

static bool visualWireframe=false;
static bool renderVisualGeometry=true;
static bool renderGrid = true;
int gDebugDrawFlags = 0;
static bool pauseSimulation=false;
bool PlasticityExampleBrowser::getPauseSimulation(){
	return pauseSimulation;
}
void PlasticityExampleBrowser::setPauseSimulation(bool pause){
	pauseSimulation=pause;
}
int midiBaseIndex = 176;
extern bool gDisableDeactivation;

///some quick test variable for the OpenCL examples

int gPreferredOpenCLDeviceIndex=-1;
int gPreferredOpenCLPlatformIndex=-1;
int gGpuArraySizeX=15;
int gGpuArraySizeY=15;
int gGpuArraySizeZ=15;

//#include <float.h>
//unsigned int fp_control_state = _controlfp(_EM_INEXACT, _MCW_EM);




void deleteDemo()
{
    if (sCurrentDemo)
	{
		sCurrentDemo->exitPhysics();
		s_instancingRenderer->removeAllInstances();
		delete sCurrentDemo;
		sCurrentDemo=0;
		delete s_guiHelper;
		s_guiHelper = 0;
	}
}


b3KeyboardCallback prevKeyboardCallback = 0;

void MyKeyboardCallback(int key, int state)
{

	//b3Printf("key=%d, state=%d", key, state);
	bool handled = false;
	
	if (gui && !handled )
	{
		handled = gui->keyboardCallback(key, state);
	}
	
	if (!handled && sCurrentDemo)
	{
		handled = sCurrentDemo->keyboardCallback(key,state);
	}

	
	

	if (handled)
		return;

	if (key=='a' && state)
	{
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawAabb;
	}
	if (key=='c' && state)
	{
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawConstraints;
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawContactPoints;
	}
	if (key == 'd' && state)
	{
		gDebugDrawFlags ^= btIDebugDraw::DBG_NoDeactivation;
		gDisableDeactivation = ((gDebugDrawFlags & btIDebugDraw::DBG_NoDeactivation) != 0);
	}
	if (key=='l' && state)
	{
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawConstraintLimits;
	}
	if (key=='w' && state)
	{
		visualWireframe=!visualWireframe;
		gDebugDrawFlags ^= btIDebugDraw::DBG_DrawWireframe;
	}


	if (key=='v' && state)
	{
		renderVisualGeometry = !renderVisualGeometry;
	}
	if (key=='g' && state)
	{
		renderGrid = !renderGrid;
	}


	if (key=='i' && state)
	{
		pauseSimulation = !pauseSimulation;
	}
#ifndef NO_OPENGL3
	if (key=='s' && state)
	{
		useShadowMap=!useShadowMap;
	}
#endif
	if (key==B3G_ESCAPE && s_window)
	{
        
		s_window->setRequestExit();
	}

	if (prevKeyboardCallback)
		prevKeyboardCallback(key,state);
}

b3MouseMoveCallback prevMouseMoveCallback = 0;
static void MyMouseMoveCallback( float x, float y)
{
	bool handled = false;
	if (sCurrentDemo)
		handled = sCurrentDemo->mouseMoveCallback(x,y);
	if (!handled && gui)
		handled = gui->mouseMoveCallback(x,y);
	if (!handled)
	{
		if (prevMouseMoveCallback)
			prevMouseMoveCallback(x,y);
	}
}

b3MouseButtonCallback prevMouseButtonCallback  = 0;

static void MyMouseButtonCallback(int button, int state, float x, float y)
{
	bool handled = false;
	//try picking first
	if (sCurrentDemo)
		handled = sCurrentDemo->mouseButtonCallback(button,state,x,y);

	if (!handled && gui)
		handled = gui->mouseButtonCallback(button,state,x,y);

	if (!handled)
	{
		if (prevMouseButtonCallback )
			prevMouseButtonCallback (button,state,x,y);
	}
	//	b3DefaultMouseButtonCallback(button,state,x,y);
}

#include <string.h>


void selectDemo(int demoIndex)
{
	bool resetCamera = (sCurrentDemoIndex != demoIndex);
	sCurrentDemoIndex = demoIndex;
	sCurrentHightlighted = demoIndex;
	int numDemos = gAllExamples->getNumRegisteredExamples();
	
	

	if (demoIndex>numDemos)
	{
		demoIndex = 0;
	}
	deleteDemo();
    
	CommonExampleInterface::CreateFunc* func = gAllExamples->getExampleCreateFunc(demoIndex);
	if (func)
	{
		s_parameterInterface->removeAllParameters();
		int option = gAllExamples->getExampleOption(demoIndex);
		s_guiHelper= new OpenGLGuiHelper(s_app, sUseOpenGL2);
		CommonExampleOptions options(s_guiHelper, option);
		sCurrentDemo = (*func)(options);
		if (sCurrentDemo)
		{
			if (gui)
			{
				bool isLeft = true;
				gui->setStatusBarMessage("Status: OK", false);
			}
			b3Printf("Selected demo: %s",gAllExamples->getExampleName(demoIndex));
			gui->setExampleDescription(gAllExamples->getExampleDescription(demoIndex));
			
			sCurrentDemo->initPhysics();
			if(resetCamera)
			{
				sCurrentDemo->resetCamera();
			}
		}
	}

}

#include <stdio.h>


static void saveCurrentDemoEntry(int currentEntry,const char* startFileName)
{
	FILE* f = NULL;
	errno_t err=fopen_s(&f,startFileName, "w");
	if (f && !err)
	{
		fprintf(f,"%d\n",currentEntry);
		fclose(f);
	}
};

static int loadCurrentDemoEntry(const char* startFileName)
{
	int currentEntry= 0;
	FILE* f=NULL;
	errno_t err=fopen_s(&f, startFileName, "r");
	if (f && !err)
	{
		int result;
		result = fscanf_s(f,"%d",&currentEntry);
		if (result)
		{
			return currentEntry;
		}
		fclose(f);
	}
	return 0;
};

void	MyComboBoxCallback(int comboId, const char* item)
{
	//printf("comboId = %d, item = %s\n",comboId, item);
	if (comboId==DEMO_SELECTION_COMBOBOX)
	{
		//find selected item
		for (int i=0;i<allNames.size();i++)
		{
			if (strcmp(item,allNames[i])==0)
			{
				selectDemo(i);
				saveCurrentDemoEntry(sCurrentDemoIndex,startFileName);
				break;
			}
		}
	}

}


void MyGuiPrintf(const char* msg)
{
	printf("b3Printf: %s\n",msg);
	if (gui)
	{
		gui->textOutput(msg);
		gui->forceUpdateScrollBars();
	}
}



void MyStatusBarPrintf(const char* msg)
{
	printf("b3Printf: %s\n", msg);
	if (gui)
	{
		bool isLeft = true;
		gui->setStatusBarMessage(msg,isLeft);
	}
}


void MyStatusBarError(const char* msg)
{
	printf("Warning: %s\n", msg);
	if (gui)
	{
		bool isLeft = false;
		gui->setStatusBarMessage(msg,isLeft);
		gui->textOutput(msg);
		gui->forceUpdateScrollBars();
	}
}

struct MyMenuItemHander :public Gwen::Event::Handler
{
	int					m_buttonId;

	MyMenuItemHander( int buttonId)
		:m_buttonId(buttonId)
	{
	}

	void onButtonA(Gwen::Controls::Base* pControl)
	{
		//const Gwen::String& name = pControl->GetName();
		Gwen::Controls::TreeNode* node = (Gwen::Controls::TreeNode*)pControl;
	//	Gwen::Controls::Label* l = node->GetButton();

		Gwen::UnicodeString la = node->GetButton()->GetText();// node->GetButton()->GetName();// GetText();
		Gwen::String laa = Gwen::Utility::UnicodeToString(la);
	//	const char* ha = laa.c_str();

		//printf("selected %s\n", ha);
		//int dep = but->IsDepressed();
		//int tog = but->GetToggleState();
//		if (m_data->m_toggleButtonCallback)
	//		(*m_data->m_toggleButtonCallback)(m_buttonId, tog);
	}
	void onButtonB(Gwen::Controls::Base* pControl)
	{
		Gwen::Controls::Label* label = (Gwen::Controls::Label*) pControl;
		Gwen::UnicodeString la = label->GetText();// node->GetButton()->GetName();// GetText();
		Gwen::String laa = Gwen::Utility::UnicodeToString(la);
		//const char* ha = laa.c_str();

		bool handled = false;
		
		selectDemo(sCurrentHightlighted);
		saveCurrentDemoEntry(sCurrentDemoIndex, startFileName);
	}
	void onButtonC(Gwen::Controls::Base* pControl)
	{
		/*Gwen::Controls::Label* label = (Gwen::Controls::Label*) pControl;
		Gwen::UnicodeString la = label->GetText();// node->GetButton()->GetName();// GetText();
		Gwen::String laa = Gwen::Utility::UnicodeToString(la);
		const char* ha = laa.c_str();


		printf("onButtonC ! %s\n", ha);
		*/
	}
	void onButtonD(Gwen::Controls::Base* pControl)
	{
/*		Gwen::Controls::Label* label = (Gwen::Controls::Label*) pControl;
		Gwen::UnicodeString la = label->GetText();// node->GetButton()->GetName();// GetText();
		Gwen::String laa = Gwen::Utility::UnicodeToString(la);
		const char* ha = laa.c_str();
		*/

	//	printf("onKeyReturn ! \n");
		selectDemo(sCurrentHightlighted);
		saveCurrentDemoEntry(sCurrentDemoIndex, startFileName);

	}

	void onButtonE(Gwen::Controls::Base* pControl)
	{
	//	printf("select %d\n",m_buttonId);
		sCurrentHightlighted = m_buttonId;
		gui->setExampleDescription(gAllExamples->getExampleDescription(sCurrentHightlighted));
	}

	void onButtonF(Gwen::Controls::Base* pControl)
	{
		//printf("selection changed!\n");
	}

	void onButtonG(Gwen::Controls::Base* pControl)
	{
		//printf("onButtonG !\n");
	}



};
#include "Bullet3Common/b3HashMap.h"

struct GL3TexLoader : public MyTextureLoader
{
	b3HashMap<b3HashString,GLint> m_hashMap;
	
	virtual void LoadTexture( Gwen::Texture* pTexture )
	{
		Gwen::String namestr = pTexture->name.Get();
		const char* n = namestr.c_str();
		GLint* texIdPtr = m_hashMap[n];
		if (texIdPtr)
		{
			pTexture->m_intData = *texIdPtr;
		}
	}
	virtual void FreeTexture( Gwen::Texture* pTexture )
	{
	}
};

void quitCallback()
{
   
    s_window->setRequestExit();
}

#define MAX_GRAPH_WINDOWS 15

struct GraphWindowInfo {
	const char * name;
	int x, y;
};

struct QuickCanvas : public Common2dCanvasInterface
{
	GL3TexLoader* m_myTexLoader;
	MyGraphWindow* m_gw[MAX_GRAPH_WINDOWS];
	GraphingTexture* m_gt[MAX_GRAPH_WINDOWS];
	GraphWindowInfo m_gwi[MAX_GRAPH_WINDOWS];
	int m_curNumGraphWindows;

	QuickCanvas(GL3TexLoader* myTexLoader)
		:m_myTexLoader(myTexLoader),
		m_curNumGraphWindows(0)
	{
		for (int i = 0; i<MAX_GRAPH_WINDOWS; i++)
		{
			m_gw[i] = 0;
			m_gt[i] = 0;
			m_gwi[i].name = "";
		}
	}
	virtual ~QuickCanvas() {}
	/**
	Use saved position if there is one with same name
	*/
	void findPosition(MyGraphInput* input){
		for (int i = 0; i<MAX_GRAPH_WINDOWS; i++)
		{
			GraphWindowInfo& gwi = m_gwi[i];
			if (strcmp(input->m_name, gwi.name) == 0){
				input->m_xPos = gwi.x;
				input->m_yPos = gwi.y;
				return;
			}
		}
		input->m_xPos = 10000;//GUI will clamp it to the right if too high (first)
		input->m_yPos = 10000;//GUI will clamp it to bottom
	}
	virtual int createCanvas(const char* canvasName, int width, int height)
	{
		if (m_curNumGraphWindows<MAX_GRAPH_WINDOWS)
		{
			//find a slot
			int slot = m_curNumGraphWindows;
			btAssert(slot<MAX_GRAPH_WINDOWS);
			if (slot >= MAX_GRAPH_WINDOWS)
				return 0;//don't crash

			m_curNumGraphWindows++;

			MyGraphInput input(gui->getInternalData());
			input.m_width = width;
			input.m_height = height;
			input.m_name = canvasName;
			input.m_texName = canvasName;
			findPosition(&input);
			m_gt[slot] = new GraphingTexture;
			m_gt[slot]->create(width, height);
			int texId = m_gt[slot]->getTextureId();
			m_myTexLoader->m_hashMap.insert(canvasName, texId);
			m_gw[slot] = setupTextureWindow(input);
			m_gwi[slot].name = canvasName;
			return slot;
		}
		return -1;
	}
	virtual void destroyCanvas(int canvasId)
	{
		btAssert(canvasId >= 0);
		delete m_gt[canvasId];
		m_gt[canvasId] = 0;
		Gwen::Controls::WindowControl* window = 
			(Gwen::Controls::WindowControl*)m_gw[canvasId];
		m_gwi[canvasId].x = window->X();
		m_gwi[canvasId].y = window->Y();
		destroyTextureWindow(m_gw[canvasId]);
		m_gw[canvasId] = 0;
		m_curNumGraphWindows--;
	}
	virtual void setPixel(int canvasId, int x, int y, unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha)
	{
		btAssert(canvasId >= 0);
		btAssert(canvasId<m_curNumGraphWindows);
		m_gt[canvasId]->setPixel(x, y, red, green, blue, alpha);
	}

	virtual void getPixel(int canvasId, int x, int y, unsigned char& red, unsigned char& green, unsigned char& blue, unsigned char& alpha)
	{
		btAssert(canvasId >= 0);
		btAssert(canvasId<m_curNumGraphWindows);
		m_gt[canvasId]->getPixel(x, y, red, green, blue, alpha);
	}

	virtual void refreshImageData(int canvasId)
	{
		m_gt[canvasId]->uploadImageData();
	}
};


PlasticityExampleBrowser::PlasticityExampleBrowser(class PlasticityExampleEntries* examples)
{
	gAllExamples = examples;
}

PlasticityExampleBrowser::~PlasticityExampleBrowser()
{
    deleteDemo();
	gAllExamples = 0;
	destroyPlasticityWindow(s_pStatWindow);
}

bool PlasticityExampleBrowser::init(int argc, char* argv[])
{
    b3CommandLineArgs args(argc,argv);
	
	int width = 1024;
    int height=768;
    SimpleOpenGL3App* simpleApp=0;
	sUseOpenGL2 =args.CheckCmdLineFlag("opengl2");
	const char* appTitle = "Bullet PlasticityExampleBrowser";
	time_t t = time(NULL);
	struct tm tm;
	errno_t err=localtime_s(&tm,&t);
#ifdef BT_USE_DOUBLE_PRECISION
	char * btScalarType = "double precision";
#else
	char * btScalarType = "single precision";
#endif
#define BLEN 256
	char buf[BLEN];
	sprintf_s(buf, BLEN, "0.9.3 - %d-%02d-%02d - %s",
		tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
		btScalarType);
#if defined (_DEBUG) || defined (DEBUG)
	const char* optMode = "Debug build (slow)";
#else
	const char* optMode = "Release build";
#endif

#ifndef NO_OPENGL3
    {
		char title[BLEN];
		sprintf_s(title,BLEN, "%s using OpenGL3+. %s %s", appTitle,optMode,buf);
        simpleApp = new SimpleOpenGL3App(title,width,height,true);
        s_app = simpleApp;
    }
#endif
    char* gVideoFileName = 0;
    args.GetCmdLineArgument("mp4",gVideoFileName);
   #ifndef NO_OPENGL3 
    if (gVideoFileName)
        simpleApp->dumpFramesToVideo(gVideoFileName);
   #endif 
   
    s_instancingRenderer = s_app->m_renderer;
	s_window  = s_app->m_window;
	prevMouseMoveCallback  = s_window->getMouseMoveCallback();
	s_window->setMouseMoveCallback(MyMouseMoveCallback);
	
	prevMouseButtonCallback = s_window->getMouseButtonCallback();
	s_window->setMouseButtonCallback(MyMouseButtonCallback);
	prevKeyboardCallback = s_window->getKeyboardCallback();
	s_window->setKeyboardCallback(MyKeyboardCallback);

	s_app->m_renderer->getActiveCamera()->setCameraDistance(13);
	s_app->m_renderer->getActiveCamera()->setCameraPitch(0);
	s_app->m_renderer->getActiveCamera()->setCameraTargetPosition(0,0,0);

	b3SetCustomWarningMessageFunc(MyGuiPrintf);
	b3SetCustomPrintfFunc(MyGuiPrintf);
	b3SetCustomErrorMessageFunc(MyStatusBarError);
	

    assert(glGetError()==GL_NO_ERROR);
	

	gui = new GwenUserInterface;
	GL3TexLoader* myTexLoader = new GL3TexLoader;
    
    Gwen::Renderer::Base* gwenRenderer = 0;
    if (sUseOpenGL2 )
    {
        gwenRenderer = new Gwen::Renderer::OpenGL_DebugFont();
    } 
#ifndef NO_OPENGL3
	else
    {
        sth_stash* fontstash=simpleApp->getFontStash();
        gwenRenderer = new GwenOpenGL3CoreRenderer(simpleApp->m_primRenderer,fontstash,width,height,s_window->getRetinaScale(),myTexLoader);
    }
#endif
	//

	gui->init(width,height,gwenRenderer,s_window->getRetinaScale());
	Gwen::Controls::TreeControl* tree = gui->getInternalData()->m_explorerTreeCtrl;
	s_pStatWindow = setupPlasticityWindow(gui->getInternalData());
	plasticityStatisticsWindowSetVisible(s_pStatWindow, true);
	gui->setFocus();

	s_parameterInterface  = s_app->m_parameterInterface = new GwenParameterInterface(gui->getInternalData());
	s_app->m_2dCanvasInterface = new QuickCanvas(myTexLoader);
	int numDemos = gAllExamples->getNumRegisteredExamples();
	int selectedDemo = loadCurrentDemoEntry(startFileName);
	Gwen::Controls::TreeNode* curNode = tree;
	MyMenuItemHander* handler2 = new MyMenuItemHander(-1);

	tree->onReturnKeyDown.Add(handler2, &MyMenuItemHander::onButtonD);
	int firstAvailableDemoIndex=-1;
	Gwen::Controls::TreeNode* firstNode=0;

	for (int d = 0; d<numDemos; d++)
	{
//		sprintf(nodeText, "Node %d", i);
		Gwen::UnicodeString nodeUText = Gwen::Utility::StringToUnicode(gAllExamples->getExampleName(d));
		if (gAllExamples->getExampleCreateFunc(d))//was test for gAllExamples[d].m_menuLevel==1
		{
			Gwen::Controls::TreeNode* pNode = curNode->AddNode(nodeUText);
			
			if (firstAvailableDemoIndex<0)
			{
				firstAvailableDemoIndex = d;
				firstNode = pNode;
			}
			
			if (d == selectedDemo)
			{
				firstAvailableDemoIndex = d;
				firstNode = pNode;
			}
			

			MyMenuItemHander* handler = new MyMenuItemHander(d);
			pNode->onNamePress.Add(handler, &MyMenuItemHander::onButtonA);
			pNode->GetButton()->onDoubleClick.Add(handler, &MyMenuItemHander::onButtonB);
			pNode->GetButton()->onDown.Add(handler, &MyMenuItemHander::onButtonC);
			pNode->onSelect.Add(handler, &MyMenuItemHander::onButtonE);
			pNode->onReturnKeyDown.Add(handler, &MyMenuItemHander::onButtonG);
			pNode->onSelectChange.Add(handler, &MyMenuItemHander::onButtonF);
		}
		 else
		 {
			 curNode = tree->AddNode(nodeUText);
		 }
	}

	if (sCurrentDemo==0)
	{
		if (firstAvailableDemoIndex>=0)
		{
			firstNode->SetSelected(true);
			while (firstNode != tree)
			{
				firstNode->ExpandAll();
				firstNode = (Gwen::Controls::TreeNode*)firstNode->GetParent();
			}
			
			selectDemo(firstAvailableDemoIndex);
		}

	}
	btAssert(sCurrentDemo!=0);
	if (sCurrentDemo==0)
	{
		printf("Error, no demo/example\n");
		exit(0);
	}
	gui->registerQuitCallback(quitCallback);
    
	return true;
}



CommonExampleInterface* PlasticityExampleBrowser::getCurrentExample()
{
	btAssert(sCurrentDemo);
	return sCurrentDemo;
}

bool PlasticityExampleBrowser::requestedExit()
{
	return s_window->requestedExit();
}

void PlasticityExampleBrowser::update(float deltaTime)
{
/*	if (sCurrentDemo)
	{
		sCurrentDemo->stepSimulation(deltaTime);
	}
	*/

			assert(glGetError()==GL_NO_ERROR);
		s_instancingRenderer->init();
        DrawGridData dg;
        dg.upAxis = s_app->getUpAxis();
        {
            BT_PROFILE("Update Camera and Light");
            s_instancingRenderer->updateCamera(dg.upAxis);
        }

		if (renderGrid)
        {
            BT_PROFILE("Draw Grid");
			glPolygonOffset(3.0, 3);
			glEnable(GL_POLYGON_OFFSET_FILL);
            s_app->drawGrid(dg);
        }
		static int frameCount = 0;
		frameCount++;

		if (0)
		{
            BT_PROFILE("Draw frame counter");
            char bla[BLEN];
            sprintf_s(bla,BLEN,"Frame %d", frameCount);
            s_app->drawText(bla,10,10);
		}

		
		if (sCurrentDemo)
		{
			if (!pauseSimulation)
			{
				sCurrentDemo->stepSimulation(deltaTime);//1./60.f);
			}
			else{
#ifdef _WIN32
				Sleep(50);
#endif
			}
			
			if (renderVisualGeometry && ((gDebugDrawFlags&btIDebugDraw::DBG_DrawWireframe)==0))
            {
				if (visualWireframe)
				{
					glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
				}
                BT_PROFILE("Render Scene");
                sCurrentDemo->renderScene();
            }
            {
				glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
				BT_PROFILE("physicsDebugDraw");
				sCurrentDemo->physicsDebugDraw(gDebugDrawFlags);
            }
		}

		{
			if (s_guiHelper && s_guiHelper->getRenderInterface() && 
				s_guiHelper->getRenderInterface()->getActiveCamera())
			{
				char msg[BLEN];
				CommonCameraInterface *cam = s_guiHelper->getRenderInterface()->getActiveCamera();
				float camDist = cam->getCameraDistance();
				float pitch = cam->getCameraPitch();
				float yaw = cam->getCameraYaw();
				float camTarget[3], camPosition[3];
				cam->getCameraTargetPosition(camTarget);
				cam->getCameraPosition(camPosition);
				sprintf_s(msg, BLEN,
					"d=%.1f, p=%.1f, y=%.1f, \
t=% .1f, % .2f, % .1f, \
p=% .1f, % .1f, % .1f", 
					camDist,pitch,yaw,
					camTarget[0],camTarget[1],camTarget[2],
					camPosition[0], camPosition[1], camPosition[2]
					);
				gui->setStatusBarMessage(msg, true);	
			}
			
		}

		{
			BT_PROFILE("processPlasticityData");
			processPlasticityData(s_pStatWindow, pauseSimulation);
		}
        if (sUseOpenGL2)
		{
					
			saveOpenGLState(s_instancingRenderer->getScreenWidth(),s_instancingRenderer->getScreenHeight());
		}
		{
			BT_PROFILE("Draw Gwen GUI");
			gui->draw(s_instancingRenderer->getScreenWidth(),s_instancingRenderer->getScreenHeight());
		}
        if (sUseOpenGL2)
        {
            restoreOpenGLState();
        }
        {
            BT_PROFILE("Sync Parameters");
            s_parameterInterface->syncParameters();
        }
        {
            BT_PROFILE("Swap Buffers");
            s_app->swapBuffer();
        }
	
		gui->forceUpdateScrollBars();

}
