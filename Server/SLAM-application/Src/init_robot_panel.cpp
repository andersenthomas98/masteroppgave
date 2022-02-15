#include "init_robot_panel.h"
#include "window.h"
#include "imgui.h"

//Not in use atm
namespace NTNU::gui::panel
{
	init_robot_panel::init_robot_panel() :
		init_pose_({0,0,0})
	{
		set_fun([&]() {
			static int manual_input[3] = { 0, 0, 0 };
			if (ImGui::InputInt3("Init pose:", manual_input))
				init_pose_ = { manual_input[0], manual_input[1] };
			});
	}

}