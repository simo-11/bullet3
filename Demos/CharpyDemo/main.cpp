/*
Simo Nikula 2014-2015 based on bullet3 demos
*/

#include "CharpyDemo.h"
#include "GlutStuff.h"
#include "btBulletDynamicsCommon.h"
#include "LinearMath/btHashMap.h"
#include <stdio.h> 
#include <time.h>
	
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
sprintf(buf, "BulletCharpy 0.5.0 - %d-%02d-%02d - %s",
	tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
	btScalarType);
	return glutmain(argc, argv,640,480,buf,&ccdDemo);
}

