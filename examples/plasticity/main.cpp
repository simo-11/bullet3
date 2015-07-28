/*
Simo Nikula 2015- based on bullet3 examples
*/
#ifdef _WIN32
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#endif
#include "Bullet3Common/b3CommandLineArgs.h"
#include "../Utils/b3Clock.h"

#include "PlasticityExampleEntries.h"
#include "PlasticityExampleBrowser.h"
#include "Bullet3Common/b3Logging.h"
int main(int argc, char* argv[])
{
	b3CommandLineArgs args(argc, argv);
	b3Clock clock;


	PlasticityExampleEntries examples;
	examples.initExampleEntries();

	ExampleBrowserInterface* exampleBrowser = new PlasticityExampleBrowser(&examples);
	bool init = exampleBrowser->init(argc, argv);
	clock.reset();
	if (init)
	{
		do
		{
			float deltaTimeInSeconds = clock.getTimeMicroseconds() / 1000000.f;
			clock.reset();
			exampleBrowser->update(deltaTimeInSeconds);

		} while (!exampleBrowser->requestedExit());
	}
	delete exampleBrowser;
#ifdef _WIN32
	_CrtDumpMemoryLeaks();
#endif
	return 0;
}

