#include "PlasticityStatistics.h"
#include "PlasticityData.h"
#include "../exampleBrowser/GwenGUISupport/gwenUserInterface.h"
#include "../exampleBrowser/GwenGUISupport/gwenInternalData.h"
#include <list>
using namespace std;
class PlasticityStatistics : public Gwen::Controls::WindowControl
{
protected:
	Gwen::Controls::WindowControl *textArea;
	void onButtonA(Gwen::Controls::Base* pControl)
	{
        //		OpenTissue::glut::toggleIdle();
	}
	
	void SliderMoved(Gwen::Controls::Base* pControl )
	{
	//	Gwen::Controls::Slider* pSlider = (Gwen::Controls::Slider*)pControl;
		//this->m_app->scaleYoungModulus(pSlider->GetValue());
		//	printf("Slider Value: %.2f", pSlider->GetValue() );
	}
	
	
	void	OnCheckChangedStiffnessWarping (Gwen::Controls::Base* pControl)
	{
	//	Gwen::Controls::CheckBox* labeled = (Gwen::Controls::CheckBox* )pControl;
//		bool checked = labeled->IsChecked();
		//m_app->m_stiffness_warp_on  = checked;
	}
public:
	
	PlasticityStatistics (	Gwen::Controls::Base* pParent)
    : Gwen::Controls::WindowControl( pParent )
	{
		SetTitle( L"Plasticity Statistics" );
		SetSize( 450, 450 );
		this->SetPos(10,400);
		textArea = new Gwen::Controls::WindowControl(this);
		textArea->SetSize(430,430);
		textArea->SetPos(20, 20);
	}
	void dumpData(PlasticityStatistics* pStat)
	{  
		list<PlasticityData>::iterator psIterator;
		string txt;
		list<PlasticityData> pData = PlasticityData::getData();
		int y = 30;
		for (psIterator = pData.begin(); psIterator != pData.end(); psIterator++)
		{	
			txt = psIterator->getValue();
			Gwen::Controls::Label* label = new Gwen::Controls::Label(textArea);
			label->SetText(txt);
			label->SizeToContents();
			label->SetPos(20, y);
			y += 20;
		}
	}
	
	void	UpdateText(PlasticityStatistics* pStat, bool idle)
	{		
		if (!idle)
		{
		}
		pStat->textArea->RemoveAllChildren();
        dumpData(pStat);
		pStat->GetCanvas()->RenderCanvas();
	}
	
	void Render( Gwen::Skin::Base* skin )
	{
		Gwen::Controls::WindowControl::Render( skin );		
	}
	
	
};

class MyMenuItems :  public Gwen::Controls::Base
{
	
public:
	
	class PlasticityStatistics* m_profWindow;
    MyMenuItems() :Gwen::Controls::Base(0)
    {
    }
   
    void MenuItemSelect(Gwen::Controls::Base* pControl)
    {
		if (m_profWindow->Hidden())
		{
			m_profWindow->SetHidden(false);
		} else
		{
			m_profWindow->SetHidden(true);
		}
		
    }
};


PlasticityStatistics* setupPlasticityWindow(GwenInternalData* data)
{
	MyMenuItems* menuItems = new MyMenuItems;
	PlasticityStatistics* psWindow = new PlasticityStatistics(data->pCanvas);
	data->m_viewMenu->GetMenu()->AddItem( L"Plasticity Statistics", 
		menuItems,(Gwen::Event::Handler::Function)&MyMenuItems::MenuItemSelect);
	menuItems->m_profWindow = psWindow;
	return psWindow;
}


void	processPlasticityData(PlasticityStatistics* pStat, bool idle)
{
	if (pStat)
	{	
		pStat->UpdateText(pStat, idle);
	}	
}

void plasticityStatisticsWindowSetVisible(PlasticityStatistics* window, bool visible)
{
	window->SetHidden(!visible);
}
void destroyPlasticityWindow(PlasticityStatistics* window)
{
	delete window;
}
