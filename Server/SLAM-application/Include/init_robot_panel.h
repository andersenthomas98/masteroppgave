#pragma once

#include "panel.h"
#include <stdbool.h>

namespace NTNU::gui::panel {

class init_robot_panel :
	public panel
{
public:
	init_robot_panel();
	~init_robot_panel() {};


private:
	  std::vector<int> init_pose_;
};

}