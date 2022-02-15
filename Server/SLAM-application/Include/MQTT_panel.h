#pragma once
#include "panel.h"
#include "callbacks.h"
#include <mutex>

namespace NTNU::gui::panel
{

enum class mqtt_panel_events
{
	SUBSCRIBE_REQUEST,
	PUBLISH_REQUEST,
};

class mqtt_panel :
	public panel,
	public NTNU::utility::callbacks<mqtt_panel_events>
{
public:
	mqtt_panel(const std::string& mqtt_client_name = "");
	~mqtt_panel() {};

	void set_client_id(const std::string& id);

	void add_sub(const std::string& sub);
	void add_msg_in(const std::string& topic, const std::string& msg);
	void add_msg_out(const std::string& topic, const std::string& msg);

private:
	std::string id_;

	std::mutex rw_mutex_;

	std::vector<std::string> mqtt_subs_;

	// Keep a log of messages in pairs of topic and payload.
	// This way the topic and/or messages can be further used in separation.
	int log_max_entries_;
	std::list<std::pair<std::string, std::string>> mqtt_msg_in_;
	std::list<std::pair<std::string, std::string>> mqtt_msg_out_;

};

}
