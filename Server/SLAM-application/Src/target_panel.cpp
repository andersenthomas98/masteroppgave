#include "imgui.h"
#include "target_panel.h"
#include "window.h"

namespace NTNU::gui::panel
{
	target_panel::target_panel() :
		manual_target_({ 0, 0 }),
		manual_robot_drive_(false)
	{
				
		set_fun([&]() {
			static bool bottom_left_corner_is_start = true;
			static int offset = 0;
			const ImVec2 btn_size{ 200, 0 };
			static int square_size_double{ 500 };
			static int square_size{ 250 };

			ImGui::Checkbox("Enable Manual Drive", &manual_robot_drive_);
			ImGui::Checkbox("Bottom-Left Corner Is Starting Point", &bottom_left_corner_is_start);

			offset = bottom_left_corner_is_start ? square_size : 0;

			if (ImGui::InputInt("Square Size", &square_size_double, 2))
				square_size = square_size_double / 2;

			ImGui::Spacing();

			if (ImGui::Button("Top-Left Corner", btn_size))
				manual_target_ = { -square_size + offset, square_size + offset };
			ImGui::SameLine();
			ImGui::Text("X [%d], Y [%d]", -square_size + offset, square_size + offset);

			if (ImGui::Button("Top-Right Corner", btn_size))
				manual_target_ = { square_size + offset, square_size + offset };
			ImGui::SameLine();
			ImGui::Text("X [%d], Y [%d]", square_size + offset, square_size + offset);

			if (ImGui::Button("Bottom-Left Corner", btn_size))
				manual_target_ = { -square_size + offset, -square_size + offset };
			ImGui::SameLine();
			ImGui::Text("X [%d], Y [%d]", -square_size + offset, -square_size + offset);

			if (ImGui::Button("Bottom-Right Corner", btn_size))
				manual_target_ = { square_size + offset, -square_size + offset };
			ImGui::SameLine();
			ImGui::Text("X [%d], Y [%d]", square_size + offset, -square_size + offset);

			ImGui::Spacing();

			static int manual_input[2] = { 0, 0 };
			if (ImGui::InputInt2("Manual Input", manual_input))
				manual_target_ = { manual_input[0], manual_input[1] };

			ImGui::Spacing();

			ImGui::Text("Current target is: X [%d], Y [%d]", manual_target_.first, manual_target_.second);
			});
	}

	ipair target_panel::get_manual_target(){
		return manual_target_;
	}
	void target_panel::set_manual_target(ipair manual_target) {
		manual_target_ = manual_target;
	}
	bool target_panel::get_manual_robot_drive() {
		return manual_robot_drive_;
	}
	void target_panel::set_manual_robot_drive(bool manual_robot_drive) {
		manual_robot_drive_ = manual_robot_drive;
	}
}