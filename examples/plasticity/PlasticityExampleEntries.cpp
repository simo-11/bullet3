
#define _CRT_SECURE_NO_WARNINGS
#include "PlasticityExampleEntries.h"

#include "LinearMath/btAlignedObjectArray.h"
#include "../CharpyDemo/CharpyDemo.h"
#include "../CharpyDemo/DemolisherDemo.h"
#include "../CharpyDemo/PlateDemo.h"
#include "../CharpyDemo/BlockDemo.h"
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

/**
option is decoded in constructors
*/
static ExampleEntry gDefaultExamples[]=
{
	ExampleEntry(0, "Tension"),
	ExampleEntry(1, "Tension:None",
	"no constraint",
	BlockDemoCreateFunc, 10),
	ExampleEntry(1, "Tension:Rigid",
	"rigid constraint using zero limits",
	BlockDemoCreateFunc, 11),
	ExampleEntry(1, "Tension:Spring",
	"springConstraint",
	BlockDemoCreateFunc, 12),
	ExampleEntry(1, "Tension:Impulse",
	"breakable constraint with zero limits",
	BlockDemoCreateFunc, 13),
	ExampleEntry(1, "Tension:Spring2",
	"spring2Constraint",
	BlockDemoCreateFunc, 14),
	ExampleEntry(1, "Tension:Slider",
	"SliderConstraint",
	BlockDemoCreateFunc, 15),
	ExampleEntry(1, "Tension:ElasticPlastic",
	"6DofElasticPlasticConstraint",
	BlockDemoCreateFunc, 16),
	ExampleEntry(2, "Tension:ElasticPlastic2",
	"6DofElasticPlastic2Constraint",
	BlockDemoCreateFunc, 17),
	ExampleEntry(0, "Bending"),
	ExampleEntry(1, "Bending:None",
	"no constraint",
	BlockDemoCreateFunc, 0),
	ExampleEntry(1, "Bending:Rigid",
	"rigid constraint using zero limits",
	BlockDemoCreateFunc, 1),
	ExampleEntry(1, "Bending:Spring",
	"springConstraint",
	BlockDemoCreateFunc, 2),
	ExampleEntry(1, "Bending:Impulse",
	"breakable constraint with zero limits",
	BlockDemoCreateFunc, 3),
	ExampleEntry(1, "Bending:Spring2",
	"spring2Constraint",
	BlockDemoCreateFunc, 4),
	ExampleEntry(1, "Bending:Hinge",
	"HingeConstraint",
	BlockDemoCreateFunc, 5),
	ExampleEntry(1, "Bending:ElasticPlastic",
	"6DofElasticPlasticConstraint",
	BlockDemoCreateFunc, 6),
	ExampleEntry(2, "Bending:ElasticPlastic2",
	"6DofElasticPlastic2Constraint",
	BlockDemoCreateFunc, 7),
	ExampleEntry(0, "Charpy Impact"),
	ExampleEntry(1, "Charpy:Rigid",
	"single rigid object",
	CharpyDemoCreateFunc, 1),
	ExampleEntry(1, "Charpy:Spring",
	"two objects and springConstraints",
	CharpyDemoCreateFunc, 2),
	ExampleEntry(1, "Charpy:Impulse",
	"two objects and constraints with zero limits",
	CharpyDemoCreateFunc, 3),
	ExampleEntry(1, "Charpy:Spring2",
	"two objects and spring2Constraints",
	CharpyDemoCreateFunc, 4),
	ExampleEntry(1, "Charpy:Hinge",
	"two objects and hingeConstraint",
	CharpyDemoCreateFunc, 5),
	ExampleEntry(1, "Charpy:PlasticHinge",
	"two objects and plasticHingeConstraint",
	CharpyDemoCreateFunc, 6),
	ExampleEntry(1, "Charpy:ElasticPlastic",
	"two objects and 6DofElasticPlasticConstraint",
	CharpyDemoCreateFunc, 7),
	ExampleEntry(2, "Charpy:ElasticPlastic2",
	"two objects and 6DofElasticPlastic2Constraint",
	CharpyDemoCreateFunc, 8),
	ExampleEntry(2, "elastic",
	"10 objects using 6DofElasticPlasticConstraint, \
SI solver, \
no hammer, \
elastic deflection, l=4 m",
	CharpyDemoCreateFunc, 2030507),
	ExampleEntry(2, "elastic2",
	"10 objects using 6DofElasticPlastic2Constraint, \
MLCP solver, \
no hammer, \
elastic deflection, l=4 m",
	CharpyDemoCreateFunc, 102030508),
	ExampleEntry(2, "cutting case",
	"4 objects using 6DofElasticPlastic2Constraint, \
cutting case",
	CharpyDemoCreateFunc, 40208),
	ExampleEntry(2, "sidestep",
	"single object does sidestep",
	CharpyDemoCreateFunc, 50001),
	ExampleEntry(2, "big and soft",
	"4 objects using 6DofElasticPlastic2Constraint, \
cutting case, large objects, very low fu",
	CharpyDemoCreateFunc, 60208),
	ExampleEntry(2, "integration instability",
	"2 objects using SpringConstraint, \
very stiff constraints cause instability",
	CharpyDemoCreateFunc, 70102),
	ExampleEntry(0, "Demolisher"),
	ExampleEntry(1, "Demolisher:None",
	"Only mass resistance",
	DemolisherDemoCreateFunc, 0),
	ExampleEntry(1, "Demolisher:Rigid",
	"Not moveable",
	DemolisherDemoCreateFunc, 1),
	ExampleEntry(1, "Demolisher:Impulse",
	"Constraints break based on impulse",
	DemolisherDemoCreateFunc, 2),
	ExampleEntry(1, "Demolisher:ElasticPlastic",
	"ElasticPlastic constraints which break based on work",
	DemolisherDemoCreateFunc, 3),
	ExampleEntry(1, "Demolisher:ElasticPlastic2",
	"ElasticPlastic2 constraints which break based on work",
	DemolisherDemoCreateFunc, 4),
	ExampleEntry(0, "Plate"),
	ExampleEntry(1, "Plate:None",
	"Only mass resistance",
	PlateDemoCreateFunc, 0),
	ExampleEntry(1, "Plate:Rigid",
	"Not moveable",
	PlateDemoCreateFunc, 1),
	ExampleEntry(1, "Plate:Impulse",
	"Constraints breaks based on impulse",
	PlateDemoCreateFunc, 2),
	ExampleEntry(1, "i-10-3-3-5",
	"Impulse constraint, \
10 mm thick, \
3 m width, \
3x3 objects, \
5 t load\
",
	PlateDemoCreateFunc, 50303102),
	ExampleEntry(1, "i-10-3-3-10",
	"Impulse constraint, \
10 mm thick, \
3 m width, \
3x3 objects, \
10 t load\
",
	PlateDemoCreateFunc, 100303102),
	ExampleEntry(1, "i-20-6-3-40",
	"Impulse constraint, \
20 mm thick, \
6 m width, \
3x3 objects, \
40 t load\
",
	PlateDemoCreateFunc, 400306202),
	ExampleEntry(1, "Plate:ElasticPlastic",
	"ElasticPlastic2 constraints which break based on work",
	PlateDemoCreateFunc, 3),
	ExampleEntry(1, "ep-10-3-3-5",
	"ElasticPlastic2 constraint, \
10 mm thick, \
3 m width, \
3x3 objects, \
5 t load\
",
	PlateDemoCreateFunc, 50303103),
	ExampleEntry(1, "ep-10-3-6-5",
	"ElasticPlastic2 constraint, \
10 mm thick, \
3 m width, \
6x6 objects, \
5 t load\
",
	PlateDemoCreateFunc, 50603103),
	ExampleEntry(1, "ep-10-3-3-10",
	"ElasticPlastic2 constraint, \
10 mm thick, \
3 m width, \
3x3 objects, \
10 t load\
",
	PlateDemoCreateFunc, 100303103),
	ExampleEntry(1, "ep-10-3-6-10",
	"ElasticPlastic2 constraint, \
10 mm thick, \
3 m width, \
6x6 objects, \
10 t load\
	",
	PlateDemoCreateFunc, 100603103),
	ExampleEntry(1, "ep-20-6-3-40",
	"ElasticPlastic2 constraint, \
20 mm thick, \
6 m width, \
3x3 objects, \
40 t load\
",
	PlateDemoCreateFunc, 400306203),
	ExampleEntry(1, "ep-20-6-6-40",
	"ElasticPlastic2 constraint, \
20 mm thick, \
6 m width, \
6x6 objects, \
40 t load\
",
	PlateDemoCreateFunc, 400606203),

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

