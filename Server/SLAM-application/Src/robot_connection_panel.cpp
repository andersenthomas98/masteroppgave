#include "robot_connection_panel.h"
#include "window.h"
#include "imgui.h"
#include <array>

namespace NTNU::gui::panel
{

	robot_connection_panel::robot_connection_panel() :
		init_pose_(std::array<int16_t, 3>{0, 0, 0}),
		ready_(false)
	{
		set_fun([&]() {
			const ImVec2 btn_size{ 100, 0 };

			static int init[3] = { 0, 0, 0 };
			if (ImGui::InputInt3("Initial pose (x, y, angle)", init))
				init_pose_ = { (int16_t)init[0], (int16_t)init[1], (int16_t)init[2] };

			ImGui::Spacing();

			if (ImGui::Button("Done", btn_size))
				ready_ = true;
			});

	}

	bool robot_connection_panel::ready() const {
		return ready_;
	}

	std::array<int16_t, 3> robot_connection_panel::get_init_pose() const {
		return init_pose_;
	}
	
}
