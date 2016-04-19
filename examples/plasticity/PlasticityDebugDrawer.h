#ifndef PLASTICITY_DEBUG_DRAWER_H
#define PLASTICITY_DEBUG_DRAWER

#include "../ExampleBrowser/OpenGLGuiHelper.h"
#include "../CommonInterfaces/CommonGraphicsAppInterface.h"
#include "btBulletDynamicsCommon.h"

#define BT_LINE_BATCH_SIZE 512

ATTRIBUTE_ALIGNED16(struct) MyPDebugVec3
{
	BT_DECLARE_ALIGNED_ALLOCATOR();
	MyPDebugVec3(const btVector3& org)
		:x(org.x()),
		y(org.y()),
		z(org.z())
	{
	}

	float x;
	float y;
	float z;
};

ATTRIBUTE_ALIGNED16(class) PlasticityDebugDrawer : public btIDebugDraw
{
	CommonGraphicsApp* m_glApp;
	int m_debugMode;
	btAlignedObjectArray<MyPDebugVec3> m_linePoints;
	btAlignedObjectArray<unsigned int> m_lineIndices;
	btVector3 m_currentLineColor;
	DefaultColors m_ourColors;
	float textSize = 1;
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	void setTextSize(float v){ textSize = v; }
	float getTextSize(){ return textSize; }
	PlasticityDebugDrawer(CommonGraphicsApp* app);
	virtual void setDefaultColors(const DefaultColors& v){ m_ourColors = v; }
	virtual DefaultColors getDefaultColors(){ return m_ourColors; }
	virtual ~PlasticityDebugDrawer(){};
	virtual void	drawLine(const btVector3& from, const btVector3& to, const btVector3& color);
	virtual void	drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB,
		btScalar distance, int lifeTime, const btVector3& color);
	virtual void	reportErrorWarning(const char* warningString);
	virtual void	draw3dText(const btVector3& location, const char* textString);
	virtual void	setDebugMode(int debugMode);
	virtual void flushLines();
	virtual int		getDebugMode() const;
};
#endif // PLASTICITY_DEBUG_DRAWER