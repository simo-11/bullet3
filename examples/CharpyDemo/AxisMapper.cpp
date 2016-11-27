#include "AxisMapper.h"
void AxisMapper::init(){
	if (m_cpos.getX()!=BT_ZERO){
		initX();
	}
	else if (m_cpos.getY() != BT_ZERO){
		initY();
	}
	else{
		btAssert(false);
	}
}

void AxisMapper::initX(){
	for (int i = 0; i < 6; i++){
		m_ami[i] = i;
	}
	m_h = m_ylen;
	m_b = m_zlen;
	m_len = m_xlen;
}

void AxisMapper::initY(){
	m_ami[0] = 1;
	m_ami[1] = 0;
	m_ami[2] = 2;
	m_ami[3] = 3;
	m_ami[4] = 4;
	m_ami[5] = 5;
	m_h = m_xlen;
	m_b = m_zlen;
	m_len = m_ylen;
}

/**
rough approximations
*/
void AxisMapper::initStiffness(){
	btScalar len = m_len / 2;
	btScalar h = m_h;
	btScalar b = m_b;
	btScalar k0(m_E*h*b / len);
	// I=bh^3/12, k for end moment with fixed end is EI/l
	btScalar im(m_E/ len / 12);
	btScalar k1(h*h*h*b*im);
	btScalar k2(h*b*b*b*im);
	m_stiffness[0] = k0;
	m_stiffness[1] = k0/2;
	m_stiffness[2] = k0/2;
	m_stiffness[3] = k0;
	m_stiffness[4] = k2;
	m_stiffness[5] = k1;
	stiffnessDone = true;
}
btScalar AxisMapper::getStiffness(int index){
	if (!stiffnessDone){
		initStiffness();
	}
	return m_stiffness[m_ami[index]];
}

void AxisMapper::initMaxForce(){
	btScalar len = m_len / 2;
	btScalar h = m_h;
	btScalar b = m_b;
	btScalar w0(m_fy*h*b);
	btScalar w1(m_fy*b*h*h / 4);
	btScalar w2(m_fy*b*b*h / 4);
	btScalar wr(0.19*m_fy*b*h*(b+h)/2);
	m_maxForce[0] = w0;
	m_maxForce[1] = w0/2;
	m_maxForce[2] = w0/2;
	m_maxForce[3] = wr;
	m_maxForce[4] = w2;
	m_maxForce[5] = w1;
	maxForceDone = true;
}

btScalar AxisMapper::getMaxForce(int index){
	if (!maxForceDone){
		initMaxForce();
	}
	int i = m_ami[index];
	btScalar value = m_maxForce[i];
	return value;
}

