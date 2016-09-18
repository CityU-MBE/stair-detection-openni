#include <wx/wx.h>
#include "UIMyFrame1.h"
class MyApp: public wxApp {
        public:
                MyApp() {}
                virtual ~MyApp() {}
                virtual bool OnInit();
                virtual int OnExit() { return 0; }
};
IMPLEMENT_APP (MyApp);
inline bool MyApp::OnInit() {
        //Init main Frame
        wxFrame* mainFrame = new UIMyFrame1(NULL);
        mainFrame->Show(true);
        SetTopWindow(mainFrame);
        return true;
}

