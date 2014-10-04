/*
Simo Nikula 2014 based on bullet3 demos
*/

#include "CharpyDemo.h"
#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btHashMap.h"

	
int main(int argc,char** argv)
{

	
	CharpyDemo ccdDemo;
	ccdDemo.initPhysics();


#ifdef CHECK_MEMORY_LEAKS
	ccdDemo.exitPhysics();
#else
	return glutmain(argc, argv,640,480,"Bullet Physics Demo for plastic hinge in charpy v-test.",&ccdDemo);
#endif
	
	//default glut doesn't return from mainloop
	return 0;
}

