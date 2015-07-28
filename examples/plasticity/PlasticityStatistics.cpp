#include "PlasticityStatistics.h"
#include "PlasticityData.h"
#include "../exampleBrowser/GwenGUISupport/gwenUserInterface.h"
#include "../exampleBrowser/GwenGUISupport/gwenInternalData.h"
#include <list>
using namespace std;
class PlasticityStatistics : public Gwen::Controls::WindowControl
{
protected:
	Gwen::Controls::ResizableControl *textArea;

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
		int w = 550;
		int h = 450;
		SetTitle( L"Plasticity Statistics" );
		SetSize( w, h );
		this->SetPos(10,400);
		textArea = new Gwen::Controls::ResizableControl(this);
		textArea->SetSize(w,h);
		textArea->SetPos(0, 0);
	}
	PlasticityStatistics::~PlasticityStatistics()
	{
		delete textArea;
	}
	void dumpData(PlasticityStatistics* pStat)
	{  
		list<PlasticityData>::iterator psIterator;
		string txt;
		list<PlasticityData> *pData = PlasticityData::getData();
		int y = 10;
		for (psIterator = pData->begin(); psIterator != pData->end(); psIterator++)
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
		if (!Visible()){
			return;
		}
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
	void PlasticityStatistics::SetHidden(bool hidden)
	{
		BaseClass::SetHidden(hidden);
		PlasticityData::setCollect(!hidden);
	}

	
};

class MyMenuItems :  public Gwen::Controls::Base
{
	
public:
	
	class PlasticityStatistics* m_pWindow;
    MyMenuItems() :Gwen::Controls::Base(0)
    {
    }
   
    void MenuItemSelect(Gwen::Controls::Base* pControl)
    {
		if (m_pWindow->Hidden())
		{
			m_pWindow->SetHidden(false);
		} else
		{
			m_pWindow->SetHidden(true);
		}
		
    }
};


PlasticityStatistics* setupPlasticityWindow(GwenInternalData* data)
{
	MyMenuItems* menuItems = new MyMenuItems;
	PlasticityStatistics* psWindow = new PlasticityStatistics(data->pCanvas);
	data->m_viewMenu->GetMenu()->AddItem( L"Plasticity Statistics", 
		menuItems,(Gwen::Event::Handler::Function)&MyMenuItems::MenuItemSelect);
	menuItems->m_pWindow = psWindow;
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
