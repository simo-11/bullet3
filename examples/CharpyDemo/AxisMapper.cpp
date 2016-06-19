#include "AxisMapper.h"
void AxisMapper::init(){
	if (m_cpos.getX() > 0){
		initX();
	}
	else if (m_cpos.getY()){
		initY();
	}
	else{
		btAssert(true);
	}
}

void AxisMapper::initX(){
	for (int i = 0; i < 6; i++){
		m_ami[i] = i;
	}
	m_h = m_ylen;
	m_b = m_zlen;
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
}