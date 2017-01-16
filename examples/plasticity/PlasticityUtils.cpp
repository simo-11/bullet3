#include "PlasticityUtils.h"
#include <cmath>
#include <cstdio>
/**
non-optimal implementation
e.g. 10.1 to 11 or 0.00101 to 0.0011 takes 5 iterations.
*/
float PlasticityUtils::roundUpWithDigits(float in, int decimals){
	const int blen = 10;
	char format[blen],buff[blen];
	float m = pow(10, -decimals);
	float val = in;
	sprintf_s(format, blen, "%%%d.%dg", decimals + 4, decimals);
	for (int count = 1; count < 100;count++){
		sprintf_s(buff, blen, format, val);
		val = (float)atof(buff);
		if (val >= in){
			return val;
		}
		val = (1+count*m)*in;
	}
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
