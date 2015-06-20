/*
Simo Nikula 2014 based on bullet3 Demos
*/
#ifndef CHARPY_DEMO_H
#define CHARPY_DEMO_H

#include "../CommonInterfaces/CommonExampleInterface.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonWindowInterface.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "../CommonInterfaces/CommonRigidBodyBase.h"

#include "LinearMath/btAlignedObjectArray.h"

class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class CommonExampleInterface*    CharpyDemoCreateFunc(CommonExampleOptions& options);
#endif //CHARPY_DEMO_H

