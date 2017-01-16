/**
Common static low level routines that are tested with gtest
*/
#ifndef _PLASTICITY_UTILS_H
#define _PLASTICITY_UTILS_H
typedef  unsigned short int velDirType;

class PlasticityUtils
{
public:
	static int countChanges(velDirType val);
	/** For x-y-plot scaling */
	static float roundUpWithDigits(float val, int decimals);
};

#endif // PLASTICITY_UTILS_H