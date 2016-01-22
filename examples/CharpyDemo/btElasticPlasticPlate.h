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
#include "bt6DofElasticPlastic2Constraint.h"
#include "btElasticPlasticMaterial.h"

ATTRIBUTE_ALIGNED16(class) btElasticPlasticPlate : public bt6DofElasticPlastic2Constraint
{
protected:
	btElasticPlasticMaterial* m_material;
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
    btElasticPlasticPlate(btRigidBody& rbA, btRigidBody& rbB);
	void updateConstraint();
	void setMaterial(btElasticPlasticMaterial* material);
	btElasticPlasticMaterial * getMaterial(){ return m_material; }
};
#endif //BT_ELASTIC_PLASTIC_PLATE_H