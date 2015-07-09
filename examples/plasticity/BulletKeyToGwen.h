#ifndef BULLET_KEY_TO_GWEN_H
#define BULLET_KEY_TO_GWEN_H
#include "../ExampleBrowser/GwenGUISupport/gwenInternalData.h"

class BulletKeyToGwen
{
public:
	static bool keyboardCallback(Gwen::Controls::Canvas* canvas, int key, bool bDown);
};

#endif 
