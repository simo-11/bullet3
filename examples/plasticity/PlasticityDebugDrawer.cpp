#include "../ExampleBrowser/OpenGLGuiHelper.h"
#include "btBulletDynamicsCommon.h"
#include "PlasticityDebugDrawer.h"

PlasticityDebugDrawer::PlasticityDebugDrawer(CommonGraphicsApp* app)
		: m_glApp(app)
		, m_debugMode(btIDebugDraw::DBG_DrawWireframe | btIDebugDraw::DBG_DrawAabb),
		m_currentLineColor(-1, -1, -1)
{
}

void PlasticityDebugDrawer::drawLine
	(const btVector3& from1, const btVector3& to1, const btVector3& color1)
{
	if (m_currentLineColor != color1 || m_linePoints.size() >= BT_LINE_BATCH_SIZE)
	{
		flushLines();
		m_currentLineColor = color1;
	}
	MyPDebugVec3 from(from1);
	MyPDebugVec3 to(to1);

	m_linePoints.push_back(from);
	m_linePoints.push_back(to);

	m_lineIndices.push_back(m_lineIndices.size());
	m_lineIndices.push_back(m_lineIndices.size());

}

void	PlasticityDebugDrawer::drawContactPoint
	(const btVector3& PointOnB, const btVector3& normalOnB, 
	btScalar distance, int lifeTime, const btVector3& color)
	{
		drawLine(PointOnB, PointOnB + normalOnB, color);
	}


void PlasticityDebugDrawer::reportErrorWarning(const char* warningString)
{
}

void PlasticityDebugDrawer::draw3dText(const btVector3& location, 
	const char* textString)
{
}

void PlasticityDebugDrawer::setDebugMode(int debugMode)
{
	m_debugMode = debugMode;
}

int	PlasticityDebugDrawer::getDebugMode() const
{
	return m_debugMode;
}

void PlasticityDebugDrawer::flushLines()
{
	int sz = m_linePoints.size();
	if (sz)
	{
		float debugColor[4];
		debugColor[0] = m_currentLineColor.x();
		debugColor[1] = m_currentLineColor.y();
		debugColor[2] = m_currentLineColor.z();
		debugColor[3] = 1.f;
		m_glApp->m_renderer->drawLines(&m_linePoints[0].x, debugColor,
			m_linePoints.size(), sizeof(MyPDebugVec3),
			&m_lineIndices[0],
			m_lineIndices.size(),
			1);
		m_linePoints.clear();
		m_lineIndices.clear();
	}
}
