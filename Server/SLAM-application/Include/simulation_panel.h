#pragma once
#include "panel.h"
#include <stdbool.h>
#include <array>

namespace NTNU::gui::panel {

class simulation_panel :
	public panel
{
public:
	simulation_panel();
	~simulation_panel() {};

	void set_robot_sim_enable(bool robot_sim);
	bool get_robot_sim_enable();

private:
	bool robot_sim_;
};

}
