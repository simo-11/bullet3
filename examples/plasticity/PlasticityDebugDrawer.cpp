#include "../ExampleBrowser/OpenGLGuiHelper.h"
#include "btBulletDynamicsCommon.h"
#include "PlasticityDebugDrawer.h"
#include "PlasticityData.h"

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

void PlasticityDebugDrawer::draw3dText(const btVector3& l, 
	const char* text)
{
	m_glApp->drawText3D(text, l.x(), l.y(), l.z(), textSize);
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

#define BLEN 6 
void PlasticityDebugDrawer::drawPlasticConstraint(btElasticPlasticConstraint* epc,
	btTypedConstraint* c, btIDebugDraw* debugDrawer){
	if (!c->isEnabled()){
		return;
	}
	char buffer[BLEN];
	sprintf_s(buffer, BLEN, "%d", epc->getId());
	btVector3 color(1, 0, 0);
	btRigidBody & rbA = c->getRigidBodyA(), &rbB = c->getRigidBodyB();
	btScalar dbgDrawSize = btScalar(0.3);
	bool drawTransforms = true;
	if (!rbA.isStaticObject()){
		btVector3 from = rbA.getCenterOfMassPosition();
		btTransform mtra = rbA.getCenterOfMassTransform();
		btVector3 to = mtra*epc->getFrameA().getOrigin();
		debugDrawer->draw3dText(to, buffer);
		debugDrawer->drawLine(from, to, color);
		if (drawTransforms){
			dbgDrawSize *= from.distance(to);
		}
	}
	if (!rbB.isStaticObject()){
		btVector3 from = rbB.getCenterOfMassPosition();
		btTransform mtrb = rbB.getCenterOfMassTransform();
		btVector3 to = mtrb*epc->getFrameB().getOrigin();
		debugDrawer->draw3dText(to, buffer);
		debugDrawer->drawLine(from, to, color);
		if (drawTransforms){
			dbgDrawSize *= from.distance(to);
		}
	}
	if (drawTransforms){
		btTransform cta = epc->getTransformA();
		debugDrawer->drawTransform(cta, dbgDrawSize);
		btTransform ctb = epc->getTransformB();
		debugDrawer->drawTransform(ctb, dbgDrawSize);
	}
}
void PlasticityDebugDrawer::drawPlasticConstraints(btDiscreteDynamicsWorld* dw){
	btIDebugDraw* debugDrawer = dw->getDebugDrawer();
	if (!debugDrawer)
	{
		return;
	}
	int mode = debugDrawer->getDebugMode();
	if (!(mode  & (btIDebugDraw::DBG_DrawConstraints | btIDebugDraw::DBG_DrawConstraintLimits)))
	{
		return;
	}
	for (int i = dw->getNumConstraints() - 1; i >= 0; i--)
	{
		btTypedConstraint* constraint = dw->getConstraint(i);
		btTypedConstraint* sc = dw->getConstraint(i);
		int type = sc->getUserConstraintType();
		if (type != BPT_EP2 && type != BPT_EP){
			continue;
		}
		btElasticPlasticConstraint *epc =
			dynamic_cast<btElasticPlasticConstraint*>(sc);
		drawPlasticConstraint(epc,constraint,debugDrawer);
	}
}
