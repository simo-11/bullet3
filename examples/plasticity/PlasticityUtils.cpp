#include "PlasticityUtils.h"
#include <cmath>
#include <cstdio>
float PlasticityUtils::roundWithDigits(float val, int decimals){
	const int blen = 10;
	char format[blen],buff[blen];
	sprintf_s(format, blen, "%%%d.%dg", decimals+4,decimals);
	sprintf_s(buff, blen, format, val);
	float f=atof(buff);
	return f;
}
/**
Count how many times next bit is not same as previous
*/
int PlasticityUtils::countChanges(velDirType byte)
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
