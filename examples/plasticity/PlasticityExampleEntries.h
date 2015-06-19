
#ifndef PLASTICITY_EXAMPLE_ENTRIES_H
#define PLASTICITY_EXAMPLE_ENTRIES_H

#include "../CommonInterfaces/CommonExampleInterface.h"




class PlasticityExampleEntries 
{
	struct ExampleEntriesInternalData* m_data;
public:
	PlasticityExampleEntries();
	virtual ~PlasticityExampleEntries();

	static void registerExampleEntry(int menuLevel, const char* name, 
		const char* description, CommonExampleInterface::CreateFunc* createFunc, int option = 0);

	void initExampleEntries();
	int getNumRegisteredExamples();
	CommonExampleInterface::CreateFunc* getExampleCreateFunc(int index);
	const char* getExampleName(int index);
	const char* getExampleDescription(int index);
	int	getExampleOption(int index);
};
#endif //PLASTICITY_EXAMPLE_ENTRIES_H
