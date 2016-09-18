#include "UIMyFrame1.h"
#include <iostream>
using namespace std;

UIMyFrame1::UIMyFrame1( wxWindow* parent )
        : MyFrame1( parent )
{

}

void UIMyFrame1::ugv_mode( wxCommandEvent& event )
{
        // TODO: Implement ugv_mode
        cout << "Change to UGV mode" << endl;
        system("./test j");
}

void UIMyFrame1::walk_mode( wxCommandEvent& event )
{
        // TODO: Implement walk_mode
        cout << "Change to WALK mode" << endl;
        system("./test g");

}

void UIMyFrame1::stair_mode( wxCommandEvent& event )
{
        cout << "Change to STAIR mode" << endl;
        system("./test i");
        // TODO: Implement stair_mode
}

void UIMyFrame1::table_mode( wxCommandEvent& event )
{
        cout << "Change to TABLE mode" << endl;
        // TODO: Implement table_mode
        system("./test h");
}
