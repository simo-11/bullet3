#ifndef PLASTICITY_BROWSER_GUI_H
#define PLASTICITY_BROWSER_GUI_H

#include "../ExampleBrowser/ExampleBrowserInterface.h"
#include "../ExampleBrowser/GwenGUISupport/gwenUserInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"

class PlasticityExampleBrowser : public ExampleBrowserInterface
{
public:

	PlasticityExampleBrowser(class PlasticityExampleEntries* examples);
	virtual ~PlasticityExampleBrowser();
	
	virtual CommonExampleInterface* getCurrentExample();
	
	virtual bool init(int argc, char* argv[]);

	virtual void update(float deltaTime);

	virtual bool requestedExit();
	static CommonGraphicsApp* getApp();
	static CommonWindowInterface* getWindow();
	static CommonRenderInterface* getRenderer();
	static GwenUserInterface* getGui();

};

#endif //PLASTICITY_BROWSER_GUI_H
