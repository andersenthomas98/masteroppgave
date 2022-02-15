#include "mqtt_panel.h"
#include "imgui.h"
#include "slam_message.h"
#include "slam_utility.h"
#include <array>
#include "log.h"
#include <iostream>
#include <sstream>
#include <ios>			// std::hex
#include <iomanip>		// std::setw

namespace NTNU::gui::panel
{

mqtt_panel::mqtt_panel(const std::string& id) :
	id_(id),
	log_max_entries_(25)
{
	set_fun([&]() {
		ImGuiTabBarFlags tab_bar_flags = ImGuiTabBarFlags_None;
		if (ImGui::BeginTabBar("MQTT Tab Bar", tab_bar_flags))
		{
			if (ImGui::BeginTabItem("Main"))
			{
				// ID
				std::string info{ "Client id: '" };
				info += id_ + "'";
				ImGui::Text(info.c_str());

				// STATUS
				// todo

				ImGui::EndTabItem();
			}

			if (ImGui::BeginTabItem("Subscribe"))
			{
				for (const auto& sub : mqtt_subs_)
				{
					ImGui::Text(sub.c_str());
				}
				ImGui::Separator();

				static std::array<char, 256> bufsub{ 0 };
				ImGui::InputTextWithHint("Topic", "Subscribe to...", bufsub.data(), bufsub.size());
				if (ImGui::Button("Subscribe")) {
					std::string topic{ bufsub.data() };
					// Duplicate subscribes are allowed technically but unwanted.
					if (topic.size() > 0 && std::find(mqtt_subs_.begin(), mqtt_subs_.end(), topic) == mqtt_subs_.end())
						call_callback(mqtt_panel_events::SUBSCRIBE_REQUEST, topic);
				}

				ImGui::EndTabItem();
			}
			if (ImGui::BeginTabItem("Publish"))
			{
				static std::array<char, 256> buftopic{ 0 };
				ImGui::InputTextWithHint("Topic", "Topic...", buftopic.data(), buftopic.size());

				static int pos[3]{ 0, 0, 0 };
				static int obs[2]{ 0, 0 };
				ImGui::SliderInt2("Position", pos, -1000, 1000);
				ImGui::SliderInt2("Obstruction", obs, -1000, 1000);

				static bool include_obstruction = false;
				ImGui::Checkbox("Include Obstruction", &include_obstruction);

				if (ImGui::Button("Publish")) {
					std::string topic{ buftopic.data() };

					if (topic.size() > 0) {
						//std::cout << "Topic: " << topic << '\n';
						LOG_INFO("Topic: {}", topic);
						NTNU::application::SLAM::message msg(topic);

						if (include_obstruction)
						{
							NTNU::application::SLAM::message::pose robot_pos{ pos[0], pos[1], pos[2] };
							NTNU::application::SLAM::message::position robot_obs{ obs[0], obs[1] };
							msg.set_payload(robot_pos, { robot_obs }, (int8_t)8 );
						}
						else
						{
							NTNU::application::SLAM::message::pose robot_pos{ pos[0], pos[1], pos[2] };
							//Does not support custom messages to simulator
							msg.set_payload(robot_pos, false);
						}
						call_callback(mqtt_panel_events::PUBLISH_REQUEST, msg);
					}
				}

				ImGui::EndTabItem();
			}
			if (ImGui::BeginTabItem("Messages"))
			{
				static int child_1;
				static int child_2;
				static bool autoscroll = true;
				static bool as_raw = true;
				ImGui::Checkbox("Autoscroll", &autoscroll);
				ImGui::Checkbox("Raw Data", &as_raw);

				ImGui::BeginGroup();

				ImGui::Text("Incoming Messages");
				ImGui::BeginChild(ImGui::GetID(&child_1), ImVec2(ImGui::GetWindowWidth() * 0.45f, 0.0f), true, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_HorizontalScrollbar);
				if (autoscroll)
					ImGui::SetScrollFromPosY(ImGui::GetScrollMaxY());

				{
					// CRITICAL SECTION
					std::lock_guard<std::mutex> lock(rw_mutex_);
					for (const auto& mqttm : mqtt_msg_in_)
					{
						std::stringstream ss;
						ss << "[" << mqttm.first << "] ";
						if (as_raw) {
							ss << std::hex << std::uppercase;
							for (const auto& c : mqttm.second) {
								ss << "0x" << std::setw(2) << std::setfill('0') << (static_cast<int>(c) & 0xFF) << " ";
							}
							// restore previous stream state
							ss << std::dec << std::nouppercase;
						}
						else {
							ss << mqttm.second;
						}
						ImGui::Text(ss.str().c_str());
					}
				}

				ImGui::EndChild();
				ImGui::EndGroup();

				ImGui::SameLine();

				ImGui::BeginGroup();
				ImGui::Text("Outgoing Messages");
				ImGui::BeginChild(ImGui::GetID(&child_2), ImVec2(ImGui::GetWindowWidth() * 0.45f, 0.0f), true, ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_HorizontalScrollbar);

				if (autoscroll)
					ImGui::SetScrollFromPosY(ImGui::GetScrollMaxY());

				{
					std::lock_guard<std::mutex> lock(rw_mutex_);
					for (const auto& mqttm : mqtt_msg_out_)
					{
						std::stringstream ss;
						ss << "[" << mqttm.first << "] ";
						if (as_raw) {
							ss << std::hex << std::uppercase;
							for (const auto& c : mqttm.second) {
								ss << "0x" << std::setw(2) << std::setfill('0') << (static_cast<int>(c) & 0xFF) << " ";
							}
							// restore previous stream state
							ss << std::dec << std::nouppercase;
						}
						else {
							ss << mqttm.second;
						}
						ImGui::Text(ss.str().c_str());
					}
				}

				ImGui::EndChild();
				ImGui::EndGroup();

				ImGui::EndTabItem();
			}
		}
		ImGui::EndTabBar();
	});
}

void mqtt_panel::set_client_id(const std::string & id)
{
	id_ = id;
}

void mqtt_panel::add_sub(const std::string & sub)
{
	std::lock_guard<std::mutex> lock(rw_mutex_);
	mqtt_subs_.push_back(sub);
}

void mqtt_panel::add_msg_in(const std::string & topic, const std::string & msg)
{
	std::lock_guard<std::mutex> lock(rw_mutex_);
	mqtt_msg_in_.emplace_back(std::make_pair(topic, msg));
	if (mqtt_msg_in_.size() > log_max_entries_)
		mqtt_msg_in_.pop_front();
}

void mqtt_panel::add_msg_out(const std::string & topic, const std::string & msg)
{
	std::lock_guard<std::mutex> lock(rw_mutex_);
	mqtt_msg_out_.emplace_back(std::make_pair(topic, msg));
	if (mqtt_msg_out_.size() > log_max_entries_)
		mqtt_msg_out_.pop_front();
}

}