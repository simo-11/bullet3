#ifdef _WINDOWS
/*
Simo Nikula 2014 based on bullet3 App_FractureDemo
*/

#include "CharpyDemo.h"

///The 'createDemo' function is called from Bullet/Demos/OpenGL/Win32AppMain.cpp to instantiate this particular demo
DemoApplication*	createDemo()
{
	return new CharpyDemo();
}

#endif
