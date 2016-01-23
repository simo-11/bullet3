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

#ifndef BT_ELASTIC_PLASTIC_MATERIAL_H
#define BT_ELASTIC_PLASTIC_MATERIAL_H

#include "LinearMath/btVector3.h"

ATTRIBUTE_ALIGNED16(class) btElasticPlasticMaterial
{
protected:
	btScalar m_density = 7800;//
	btScalar m_E=21E10; // steel as default
	btScalar m_nu=0.3; // steel as default
	btScalar m_G=m_E/(2*(1+m_nu)); // isotropic as default
	btScalar m_fy=m_E/1000; // low carbon
	btScalar m_maxPlasticStrain=0.25; //
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
    btElasticPlasticMaterial();
	const btScalar getDensity() { return m_density; }
	const btScalar getE() { return m_E; }
	const btScalar getNu() { return m_nu; }
	const btScalar getG() { return m_G; }
	const btScalar getFy() { return m_fy; }
	const btScalar getMaxPlasticStrain() { return m_maxPlasticStrain; }
	void setDensity(btScalar v){ m_density = v; }
	void setE(btScalar v){ m_E = v; }
	void setNu(btScalar v){ m_nu = v; }
	void setG(btScalar v){ m_G = v; }
	void setFy(btScalar v){ m_fy = v; }
	void setMaxPlasticStrain(btScalar v){ m_maxPlasticStrain = v; }
};

#endif //BT_ELASTIC_PLASTIC_MATERIAL_H
