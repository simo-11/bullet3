#include "btBulletDynamicsCommon.h"
#include "BulletKeyToGwen.h"
#include "../exampleBrowser/GwenGUISupport/gwenUserInterface.h"
#include "../ExampleBrowser/GwenGUISupport/gwenInternalData.h"
#include "../CommonInterfaces/CommonWindowInterface.h"

bool BulletKeyToGwen::keyboardCallback(Gwen::Controls::Canvas* canvas, int key, bool bDown){
	Gwen::UnicodeChar chr = 0;
	switch (key){
	case B3G_LEFT_ARROW:
		chr = Gwen::Key::Left;
		break;
	case B3G_RIGHT_ARROW:
		chr = Gwen::Key::Right;
		break;
	case B3G_UP_ARROW:
		chr = Gwen::Key::Up;
		break;
	case B3G_DOWN_ARROW:
		chr = Gwen::Key::Down;
		break;
	case B3G_END:
		chr = Gwen::Key::End;
		break;
	case B3G_HOME:
		chr = Gwen::Key::Home;
		break;
	case B3G_DELETE:
		chr = Gwen::Key::Delete;
		break;
	case B3G_BACKSPACE:
		chr = Gwen::Key::Backspace;
		break;
	case B3G_RETURN:
		chr = Gwen::Key::Return;
		break;
	}
	if (chr){
		return canvas->InputKey(chr, bDown);
	}
	else{
		chr = (Gwen::UnicodeChar) key;
		return canvas->InputCharacter(chr);
	}
}

