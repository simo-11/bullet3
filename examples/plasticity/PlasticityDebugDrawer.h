#ifndef PLASTICITY_DEBUG_DRAWER_H
#define PLASTICITY_DEBUG_DRAWER

#include "../ExampleBrowser/OpenGLGuiHelper.h"
#include "btBulletDynamicsCommon.h"

// MyDebugDrawer has no own header
class MyDebugDrawer : public btIDebugDraw
{
public:
	MyDebugDrawer(CommonGraphicsApp* app);
	virtual void	drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
	virtual void	drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, 
		btScalar distance, int lifeTime, const btVector3& color);
	virtual void	reportErrorWarning(const char* warningString);
	virtual void	draw3dText(const btVector3& location, const char* textString);
	virtual void	setDebugMode(int debugMode);
	virtual int		getDebugMode() const;
};
class PlasticityDebugDrawer : public MyDebugDrawer
{
public:
	PlasticityDebugDrawer(CommonGraphicsApp* app);
	virtual ~PlasticityDebugDrawer(){};
	virtual void draw3dText(const btVector3& location, const char* textString);
};
#endif // PLASTICITY_DEBUG_DRAWER