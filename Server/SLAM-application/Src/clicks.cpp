#include "clicks.h"
#include "imgui.h"

namespace NTNU::gui::panel
{
clicks::clicks() :
	left_click_(clicks_choices::nothing),
	right_click_(clicks_choices::nothing)
{
	set_fun([&]() {
		bool left_changed = false;
		bool right_changed = false;

		//ImGui::Text("Left Click:");
		//left_changed |= ImGui::RadioButton("Nothing##1", (int *)(&left_click_), clicks_choices::nothing);
		//ImGui::SameLine();
		//left_changed |= ImGui::RadioButton("Set Target##1", (int *)&left_click_, clicks_choices::set_target);
		//ImGui::SameLine();
		//left_changed |= ImGui::RadioButton("Obstruct##1", (int *)&left_click_, clicks_choices::obstruct);

		//ImGui::Separator();

		ImGui::Text("Right Click:");
		right_changed |= ImGui::RadioButton("Nothing##2", (int *)&right_click_, clicks_choices::nothing);
		ImGui::SameLine();
		right_changed |= ImGui::RadioButton("Set Target##2", (int *)&right_click_, clicks_choices::set_target);
		ImGui::SameLine();
		right_changed |= ImGui::RadioButton("Obstruct##2", (int *)&right_click_, clicks_choices::obstruct);

		if (left_changed)
			call_callback(clicks_events::left_click_mode_changed, left_click_);

		if (right_changed)
			call_callback(clicks_events::right_click_mode_changed, right_click_);

	});
}

bool clicks::left_is(clicks_choices choice) const
{
	return left_click_ == choice;
}

bool clicks::right_is(clicks_choices choice) const
{
	return right_click_ == choice;
}

clicks::~clicks(){}

}