/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
/*
Simo Nikula 2016- while studying plasticity
*/

#ifndef BT_ELASTIC_PLASTIC_PLATE_H
#define BT_ELASTIC_PLASTIC_PLATE_H

#include "LinearMath/btVector3.h"
#include "btBulletDynamicsCommon.h"
#include "bt6DofElasticPlastic2Constraint.h"
#include "btElasticPlasticMaterial.h"

/**
* Provides high level interface to plate that is internally subdivided
* so that subparts have constrains between them.
*/
ATTRIBUTE_ALIGNED16(class) btElasticPlasticPlate: public btActionInterface
{
protected:
	btBoxShape *m_mainShape;
	int m_lc=4; // how many objects for longest dimension
	int m_mc; // how many objects for middle dimension
	btScalar m_thickness;
	btBoxShape *m_subShape=0;
	btTransform m_mainTransform;
	/**
	* store all sequentially m-direction in inner loop
	*/
	btAlignedObjectArray<btRigidBody*> m_rb;
	btAlignedObjectArray<bt6DofElasticPlastic2Constraint*> m_constraints;
	btElasticPlasticMaterial* m_material;
	virtual void initSubShape();
	virtual void initRigidBodies(btDiscreteDynamicsWorld& dw);
	virtual void initConstraints(btDiscreteDynamicsWorld& dw);
	virtual const btTransform getConnectingFrame(btRigidBody& rbA, btRigidBody& rbB);
	virtual void updateConstraint(bt6DofElasticPlastic2Constraint &constraint);
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	btElasticPlasticPlate(){};
	~btElasticPlasticPlate();
	void setLongCount(int v){ m_lc = v; }
	void setMiddleCount(int v){ m_mc = v; }
	int getLongCount(){ return m_lc; }
	int getMiddleCount(){ return m_mc; }
	virtual void join(btDiscreteDynamicsWorld& dw);
	virtual btScalar getThickness(){ return m_thickness; }
	virtual void setMaterial(btElasticPlasticMaterial* material);
	btElasticPlasticMaterial* getMaterial(){ return m_material; }
	virtual void setTransform(btTransform& transform);
	btTransform getTransform(){ return m_mainTransform; }
	virtual void setShape(btBoxShape* shape);
	btBoxShape* getShape(){ return m_mainShape; }
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step);
	virtual void debugDraw(btIDebugDraw* debugDrawer);
};
#endif //BT_ELASTIC_PLASTIC_PLATE_H