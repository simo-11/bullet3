

#include "PlasticityExampleEntries.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "../CharpyDemo/CharpyDemo.h"
#include "../ExampleBrowser/ExampleEntries.h"

#ifdef ENABLE_LUA
#include "../LuaDemo/LuaPhysicsSetup.h"
#endif

struct ExampleEntry
{
	int									m_menuLevel;
	const char*							m_name;
	const char*							m_description;
	CommonExampleInterface::CreateFunc*		m_createFunc;
	int									m_option;

	ExampleEntry(int menuLevel, const char* name)
		:m_menuLevel(menuLevel), m_name(name), m_description(0), m_createFunc(0), m_option(0)
	{
	}

	ExampleEntry(int menuLevel, const char* name, const char* description, CommonExampleInterface::CreateFunc* createFunc, int option = 0)
		:m_menuLevel(menuLevel), m_name(name), m_description(description), m_createFunc(createFunc), m_option(option)
	{
	}
};

static ExampleEntry gDefaultExamples[]=
{
	ExampleEntry(0, "Plasticity"),
	ExampleEntry(1,"Charpy",
	"Create some rigid bodies using box collision shapes"
	, CharpyDemoCreateFunc),	
};

