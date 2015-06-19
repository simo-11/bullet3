/*
Simo Nikula 2015- based on bullet3 examples
*/
/*

int main(int argc,char** argv){
CharpyDemo ccdDemo;
ccdDemo.initPhysics();
time_t t = time(NULL);
struct tm tm = *localtime(&t);
#ifdef BT_USE_DOUBLE_PRECISION
char * btScalarType="double precision";
#else
char * btScalarType="single precision";
#endif
char buf[100];
sprintf(buf, "BulletCharpy 0.6.0-alpha - %d-%02d-%02d - %s",
tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
btScalarType);
return glutmain(argc, argv,640,480,buf,&ccdDemo);
}

*/
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


	return 0;
}

