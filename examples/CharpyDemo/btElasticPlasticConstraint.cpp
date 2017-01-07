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
/**
Count how many times next bit is not same as previous
*/
int btElasticPlasticConstraint::countChanges(velDirType byte)
{
	int count = 0;
	int bitCount = sizeof(velDirType) * 8 - 1;
	for (int i = 0; i < bitCount; i++){
		int last = byte & 1;
		byte >>= 1;
		int secondLast = byte & 1;
		if (last != secondLast){
			count++;
		}
	}
	return count;
}
