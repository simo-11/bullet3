#define _CRT_SECURE_NO_WARNINGS
#include "btElasticPlasticConstraint.h"

void btElasticPlasticConstraint::fillLimitReasons(char buff[], LimitReason reason[6]){
	for (int i = 0; i < 6; i++){
		char c;
		switch (reason[i]){
		case None:c = '-';break;
		case Force:c = 'F'; break;
		case Monitor:c = 'M'; break;
		case Stiffness:c = 'S'; break;
		case Damping:c = 'D'; break;
		default: c = 'U'; break; // Uknown
		}
		buff[i] = c;
	}
}
