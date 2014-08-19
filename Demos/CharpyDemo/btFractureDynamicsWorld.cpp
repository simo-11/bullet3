
#include "btFractureDynamicsWorld.h"
#include "btFractureBody.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"

#include "BulletCollision/CollisionDispatch/btUnionFind.h"

btFractureDynamicsWorld::btFractureDynamicsWorld ( btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration)
:btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration)
{

}



struct	btFracturePair
{
	btFractureBody* m_fracObj;
	btAlignedObjectArray<btPersistentManifold*>	m_contactManifolds;
};



void btFractureDynamicsWorld::solveConstraints(btContactSolverInfo& solverInfo)
{
	// todo: after fracture we should run the solver again for better realism
	// for example
	//	save all velocities and if one or more objects fracture:
	//	1) revert all velocties
	//	2) apply impulses for the fracture bodies at the contact locations
	//	3)and run the constaint solver again

	btDiscreteDynamicsWorld::solveConstraints(solverInfo);

	fractureCallback();
}

btFractureBody* btFractureDynamicsWorld::addNewBody(const btTransform& oldTransform,btScalar* masses, btCompoundShape* oldCompound)
{
	int i;

	btTransform shift;
	shift.setIdentity();
	btVector3 localInertia;
	btCompoundShape* newCompound = btFractureBody::shiftTransform(oldCompound,masses,shift,localInertia);
	btScalar totalMass = 0;
	for (i=0;i<newCompound->getNumChildShapes();i++)
		totalMass += masses[i];
	//newCompound->calculateLocalInertia(totalMass,localInertia);

	btFractureBody* newBody = new btFractureBody(totalMass,0,newCompound,localInertia, masses,newCompound->getNumChildShapes(), this);
	newBody->recomputeConnectivity(this);

	newBody->setCollisionFlags(newBody->getCollisionFlags()|btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	newBody->setWorldTransform(oldTransform*shift);
	addRigidBody(newBody);
	return newBody;
}

void btFractureDynamicsWorld::addRigidBody(btRigidBody* body)
{
	if (body->getInternalType() & CUSTOM_FRACTURE_TYPE)
	{
		btFractureBody* fbody = (btFractureBody*)body;
		m_fractureBodies.push_back(fbody);
	}
	btDiscreteDynamicsWorld::addRigidBody(body);
}

void	btFractureDynamicsWorld::removeRigidBody(btRigidBody* body)
{
	if (body->getInternalType() & CUSTOM_FRACTURE_TYPE)
	{
		btFractureBody* fbody = (btFractureBody*)body;
		btAlignedObjectArray<btTypedConstraint*> tmpConstraints;

		for (int i=0;i<fbody->getNumConstraintRefs();i++)
		{
			tmpConstraints.push_back(fbody->getConstraintRef(i));
		}

		//remove all constraints attached to this rigid body too		
		for (int i=0;i<tmpConstraints.size();i++)
			btDiscreteDynamicsWorld::removeConstraint(tmpConstraints[i]);

		m_fractureBodies.remove(fbody);
	}
	


	btDiscreteDynamicsWorld::removeRigidBody(body);
}

void	btFractureDynamicsWorld::breakDisconnectedParts( btFractureBody* fracObj)
{

	if (!fracObj->getCollisionShape()->isCompound())
		return;

	btCompoundShape* compound = (btCompoundShape*)fracObj->getCollisionShape();
	int numChildren = compound->getNumChildShapes();

	if (numChildren<=1)
		return;

	//compute connectivity
	btUnionFind unionFind;

	btAlignedObjectArray<int> tags;
	tags.resize(numChildren);
	int i, index = 0;
	for ( i=0;i<numChildren;i++)
	{
#ifdef STATIC_SIMULATION_ISLAND_OPTIMIZATION
		tags[i] = index++;
#else
		tags[i] = i;
		index=i+1;
#endif
	}

	unionFind.reset(index);
	int numElem = unionFind.getNumElements();
	for (i=0;i<fracObj->m_connections.size();i++)
	{
		btConnection& connection = fracObj->m_connections[i];
		if (connection.m_strength > 0.)
		{
			int tag0 = tags[connection.m_childIndex0];
			int tag1 = tags[connection.m_childIndex1];
			unionFind.unite(tag0, tag1);
		}
	}
	numElem = unionFind.getNumElements();

	index=0;
	for (int ai=0;ai<numChildren;ai++)
	{
		int tag = unionFind.find(index);
		tags[ai] = tag;
		//Set the correct object offset in Collision Object Array
#if STATIC_SIMULATION_ISLAND_OPTIMIZATION
		unionFind.getElement(index).m_sz = ai;
#endif //STATIC_SIMULATION_ISLAND_OPTIMIZATION
		index++;
	}
	unionFind.sortIslands();

	int endIslandIndex=1;
	int startIslandIndex;

	btAlignedObjectArray<btCollisionObject*> removedObjects;

	int numIslands = 0;

	for ( startIslandIndex=0;startIslandIndex<numElem;startIslandIndex = endIslandIndex)
	{
		int islandId = unionFind.getElement(startIslandIndex).m_id;
		for (endIslandIndex = startIslandIndex+1;(endIslandIndex<numElem) && (unionFind.getElement(endIslandIndex).m_id == islandId);endIslandIndex++)
		{
		}

	//	int fractureObjectIndex = -1;

		int numShapes=0;


		btCompoundShape* newCompound = new btCompoundShape();
		btAlignedObjectArray<btScalar> masses;

		int idx;
		for (idx=startIslandIndex;idx<endIslandIndex;idx++)
		{
			int i = unionFind.getElement(idx).m_sz;
	//		btCollisionShape* shape = compound->getChildShape(i);
			newCompound->addChildShape(compound->getChildTransform(i),compound->getChildShape(i));
			masses.push_back(fracObj->m_masses[i]);
			numShapes++;
		}
		if (numShapes)
		{
			btFractureBody* newBody = addNewBody(fracObj->getWorldTransform(),&masses[0],newCompound);
			newBody->setLinearVelocity(fracObj->getLinearVelocity());
			newBody->setAngularVelocity(fracObj->getAngularVelocity());

			numIslands++;
		}
	}





	removeRigidBody(fracObj);//should it also be removed from the array?


}

#include <stdio.h>


void btFractureDynamicsWorld::fractureCallback( )
{

	btAlignedObjectArray<btFracturePair> sFracturePairs;

	int numManifolds = getDispatcher()->getNumManifolds();

	sFracturePairs.clear();


	for (int i=0;i<numManifolds;i++)
	{
		btPersistentManifold* manifold = getDispatcher()->getManifoldByIndexInternal(i);
		if (!manifold->getNumContacts())
			continue;

		btScalar totalImpact = 0.f;
		for (int p=0;p<manifold->getNumContacts();p++)
		{
			totalImpact += manifold->getContactPoint(p).m_appliedImpulse;
		}

		

		//some threshold otherwise resting contact would break objects after a while
		if (totalImpact < 40.f)
			continue;

		//		printf("strong impact\n");


		//@todo: add better logic to decide what parts to fracture
		//For example use the idea from the SIGGRAPH talk about the fracture in the movie 2012:
		//
		//Breaking thresholds can be stored as connectivity information between child shapes in the fracture object
		//
		//You can calculate some "impact value" by simulating all the individual child shapes 
		//as rigid bodies, without constraints, running it in a separate simulation world 
		//(or by running the constraint solver without actually modifying the dynamics world)
		//Then measure some "impact value" using the offset and applied impulse for each child shape
		//weaken the connections based on this "impact value" and only break 
		//if this impact value exceeds the breaking threshold.
		//you can propagate the weakening and breaking of connections using the connectivity information

		int f0 = m_fractureBodies.findLinearSearch((btFractureBody*)manifold->getBody0());
		int f1 = m_fractureBodies.findLinearSearch((btFractureBody*)manifold->getBody1());

		if (f0 == f1 == m_fractureBodies.size())
			continue;


		if (f0<m_fractureBodies.size())
		{
			int j=f0;

			btCollisionObject* colOb = (btCollisionObject*)manifold->getBody1();
	//		btRigidBody* otherOb = btRigidBody::upcast(colOb);
			//	if (!otherOb->getInvMass())
			//		continue;

			int pi=-1;

			for (int p=0;p<sFracturePairs.size();p++)
			{
				if (sFracturePairs[p].m_fracObj == m_fractureBodies[j])
				{
					pi = p; break;
				}
			}

			if (pi<0)
			{
				btFracturePair p;
				p.m_fracObj = m_fractureBodies[j];
				p.m_contactManifolds.push_back(manifold);
				sFracturePairs.push_back(p);
			} else
			{
				btAssert(sFracturePairs[pi].m_contactManifolds.findLinearSearch(manifold)==sFracturePairs[pi].m_contactManifolds.size());
				sFracturePairs[pi].m_contactManifolds.push_back(manifold);
			}
		}


		if (f1 < m_fractureBodies.size())
		{
			int j=f1;
			{
				btCollisionObject* colOb = (btCollisionObject*)manifold->getBody0();
				btRigidBody* otherOb = btRigidBody::upcast(colOb);
				//	if (!otherOb->getInvMass())
				//		continue;


				int pi=-1;

				for (int p=0;p<sFracturePairs.size();p++)
				{
					if (sFracturePairs[p].m_fracObj == m_fractureBodies[j])
					{
						pi = p; break;
					}
				}
				if (pi<0)
				{
					btFracturePair p;
					p.m_fracObj = m_fractureBodies[j];
					p.m_contactManifolds.push_back( manifold);
					sFracturePairs.push_back(p);
				} else
				{
					btAssert(sFracturePairs[pi].m_contactManifolds.findLinearSearch(manifold)==sFracturePairs[pi].m_contactManifolds.size());
					sFracturePairs[pi].m_contactManifolds.push_back(manifold);
				}
			}
		}

		//
	}

	//printf("m_fractureBodies size=%d\n",m_fractureBodies.size());
	//printf("sFracturePairs size=%d\n",sFracturePairs.size());
	if (!sFracturePairs.size())
		return;


	{
		//		printf("fracturing\n");

		for (int i=0;i<sFracturePairs.size();i++)
		{
			//check impulse/displacement at impact

			//weaken/break connections (and propagate breaking)

			//compute connectivity of connected child shapes


			if (sFracturePairs[i].m_fracObj->getCollisionShape()->isCompound())
			{
				btTransform tr;
				tr.setIdentity();
				btCompoundShape* oldCompound = (btCompoundShape*)sFracturePairs[i].m_fracObj->getCollisionShape();
				if (oldCompound->getNumChildShapes()>1)
				{
					bool needsBreakingCheck = false;


					//weaken/break the connections

					//@todo: propagate along the connection graph
					for (int j=0;j<sFracturePairs[i].m_contactManifolds.size();j++)
					{
						btPersistentManifold* manifold = sFracturePairs[i].m_contactManifolds[j];
						for (int k=0;k<manifold->getNumContacts();k++)
						{
							btManifoldPoint& pt = manifold->getContactPoint(k);
							if (manifold->getBody0()==sFracturePairs[i].m_fracObj)
							{
								for (int f=0;f<sFracturePairs[i].m_fracObj->m_connections.size();f++)
								{
									btConnection& connection = sFracturePairs[i].m_fracObj->m_connections[f];
									if (	(connection.m_childIndex0 == pt.m_index0) ||
										(connection.m_childIndex1 == pt.m_index0)
										)
									{
										connection.m_strength -= pt.m_appliedImpulse;
										if (connection.m_strength<0)
										{
											//remove or set to zero
											connection.m_strength=0.f;
											needsBreakingCheck = true;
										}
									}
								}
							} else
							{
								for (int f=0;f<sFracturePairs[i].m_fracObj->m_connections.size();f++)
								{
									btConnection& connection = sFracturePairs[i].m_fracObj->m_connections[f];
									if (	(connection.m_childIndex0 == pt.m_index1) ||
										(connection.m_childIndex1 == pt.m_index1)
										)
									{
										connection.m_strength -= pt.m_appliedImpulse;
										if (connection.m_strength<0)
										{
											//remove or set to zero
											connection.m_strength=0.f;
											needsBreakingCheck = true;
										}
									}
								}
							}
						}
					}

					if (needsBreakingCheck)
					{
						breakDisconnectedParts(sFracturePairs[i].m_fracObj);
					}
				}

			}

		}
	}

	sFracturePairs.clear();

}

