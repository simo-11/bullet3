

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
	ExampleEntry(0, "Charpy Impact"),
	ExampleEntry(1,"F1",
	"single rigid object",
	CharpyDemoF1CreateFunc),
	ExampleEntry(1, "F2",
	"two objects and springConstraints",
	CharpyDemoF2CreateFunc),
	ExampleEntry(1, "F3",
	"two objects and constraints with zero limits",
	CharpyDemoF3CreateFunc),
	ExampleEntry(1, "F4",
	"two objects and spring2Constraints",
	CharpyDemoF4CreateFunc),
	ExampleEntry(1, "F5",
	"two objects and hingeConstraint",
	CharpyDemoF5CreateFunc),
	ExampleEntry(1, "F6",
	"two objects and plasticHingeConstraint",
	CharpyDemoF6CreateFunc),
	ExampleEntry(1, "F7",
	"two objects and 6DofElasticPlasticConstraint",
	CharpyDemoF7CreateFunc),
	ExampleEntry(1, "F8",
	"two objects and 6DofElasticPlastic2Constraint",
	CharpyDemoF8CreateFunc),

};
struct ExampleEntriesInternalData
{
	btAlignedObjectArray<ExampleEntry> m_allExamples;
};

PlasticityExampleEntries::PlasticityExampleEntries()
{
	m_data = new ExampleEntriesInternalData;
}

PlasticityExampleEntries::~PlasticityExampleEntries()
{
	delete m_data;
}


void PlasticityExampleEntries::initExampleEntries()
{
	m_data->m_allExamples.clear();
	int numDefaultEntries = sizeof(gDefaultExamples) / sizeof(ExampleEntry);
	for (int i = 0; i<numDefaultEntries; i++)
	{
		m_data->m_allExamples.push_back(gDefaultExamples[i]);
	}
}

void PlasticityExampleEntries::registerExampleEntry(int menuLevel, const char* name,
	const char* description, CommonExampleInterface::CreateFunc* createFunc, int option)
{
	ExampleEntry e(menuLevel, name, description, createFunc, option);
}

int PlasticityExampleEntries::getNumRegisteredExamples()
{
	return m_data->m_allExamples.size();
}

CommonExampleInterface::CreateFunc* PlasticityExampleEntries::getExampleCreateFunc(int index)
{
	return m_data->m_allExamples[index].m_createFunc;
}

int PlasticityExampleEntries::getExampleOption(int index)
{
	return m_data->m_allExamples[index].m_option;
}

const char* PlasticityExampleEntries::getExampleName(int index)
{
	return m_data->m_allExamples[index].m_name;
}

const char* PlasticityExampleEntries::getExampleDescription(int index)
{
	return m_data->m_allExamples[index].m_description;
}

