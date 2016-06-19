#ifndef AXIS_MAPPER_H
#define AXIS_MAPPER_H

#include "LinearMath/btVector3.h"
#include "btBulletDynamicsCommon.h"

/**
* Provides Helper for constraint generation.
Currently supports x and y axis alignment
*/
ATTRIBUTE_ALIGNED16(class) AxisMapper
{
protected:
	btScalar m_xlen, m_ylen, m_zlen;
	btScalar m_h, m_len, m_b;
	btVector3 m_cpos;
	int m_ami[6];
	void init();
	void initX();
	void initY();
public:
	BT_DECLARE_ALIGNED_ALLOCATOR();
	AxisMapper(btScalar xlen, btScalar ylen, btScalar zlen, btVector3& cpos) :
		m_xlen(xlen),
		m_ylen(ylen),
		m_zlen(zlen),
		m_cpos(cpos){
		init();
	};
	~AxisMapper(){};
	btScalar getH(){ return m_h; }
	btScalar getB(){ return m_b; }
	int* getIndexes(){ return m_ami; }
};
#endif //AXIS_MAPPER_H