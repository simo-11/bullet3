#ifndef PLASTICITY_BROWSER_GUI_H
#define PLASTICITY_BROWSER_GUI_H

#include "../ExampleBrowser/ExampleBrowserInterface.h"

class PlasticityExampleBrowser : public ExampleBrowserInterface
{
public:

	PlasticityExampleBrowser(class PlasticityExampleEntries* examples);
	virtual ~PlasticityExampleBrowser();
	
	virtual CommonExampleInterface* getCurrentExample();
	
	virtual bool init(int argc, char* argv[]);

	virtual void update(float deltaTime);

	virtual bool requestedExit();
	
};

#endif //PLASTICITY_BROWSER_GUI_H
