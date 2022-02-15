#include "simulation_panel.h"
#include "window.h"
#include "imgui.h"
#include <boost/thread.hpp>

namespace NTNU::gui::panel 
{
	boost::mutex sim_mutex;

	simulation_panel::simulation_panel() :
		robot_sim_(false)
	{
		set_fun([&]() {
			boost::lock_guard<boost::mutex> lock{ sim_mutex };
			ImGui::Checkbox("Simulation On", &robot_sim_);

		});

		

	}
	
	void simulation_panel::set_robot_sim_enable(bool robot_sim) {
		boost::lock_guard<boost::mutex> lock{ sim_mutex };
		robot_sim_ = robot_sim;
	}
	bool simulation_panel::get_robot_sim_enable() {
		boost::lock_guard<boost::mutex> lock{ sim_mutex };
		return robot_sim_;
	}
}

