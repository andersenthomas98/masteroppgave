#pragma once

#include <string>
#include <optional>
#include <iostream>

namespace NTNU::networking::protocols::MQTT::utility
{

struct topic {
	std::string version;
	std::string sender;
	std::string id;
	std::string command;
};

std::optional<topic> parse_topic(const std::string& topic);
std::ostream& operator<<(std::ostream& out, const topic& topic);

}
