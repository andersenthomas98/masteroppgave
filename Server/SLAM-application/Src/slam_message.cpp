#include "slam_message.h"
#include "slam_utility.h"
#include "log.h"
#include <iostream>
#include <bitset>

namespace NTNU::application::SLAM
{

using position = message::position;
using pose = message::pose;
using line = message::line;

message::message(std::string sender) :
	valid_(false),
	sender_(sender),
	robot_pose_(),
	target_(),
	obstacles_(),
	object_(0),
	is_pose_(false),
	is_object_(),
	is_sim_object_(),
	msg_type_in_(),
	msg_type_out_()
{
}

void message::set_payload(std::string payload)
{
	std::vector<std::byte> raw;

	for (const auto& ch : payload) {
		auto byte = std::byte(ch);
		raw.push_back(byte);
	}

	auto it = raw.begin();

	msg_type_in_ = (msg_type_in)utility::from_byte_to_int8_ptr(&(*it));
	it += 1;

	if (!sanity_check(msg_type_in_, payload.size()))
	{
		valid_ = false;
		//std::cout << "Invalid size of payload!";
		LOG_WARN("Invalid size of payload!");
		return;
	}

	valid_ = true;

	if (msg_type_in_ == msg_type_in::UPDATE) {
		robot_pose_.x = utility::from_byte_ptr(&(*it));
		it += 2;

		robot_pose_.y = utility::from_byte_ptr(&(*it));
		it += 2;

		robot_pose_.theta = utility::from_byte_ptr(&(*it));
		it += 2;	

		for (it; it < (raw.end() - 2);)
		{
			position obstacle;

			obstacle.x = utility::from_byte_ptr(&(*it));
			it += 2;
			obstacle.y = utility::from_byte_ptr(&(*it));
			it += 2;

			obstacles_.push_back(obstacle);
		}

		int8_t obs_status = utility::from_byte_to_int8_ptr(&(*it));
		it += 1;

		std::bitset<8> status_bits(obs_status);

		for (auto i = 3; i > -1; i--) {
			if (status_bits[i] == 1)
				is_object_.push_back(true);
			else
				is_object_.push_back(false);
		}
	}
	else if (msg_type_in_ == msg_type_in::SIM_INIT) {
		robot_pose_.x = utility::from_byte_ptr(&(*it));
		it += 2;

		robot_pose_.y = utility::from_byte_ptr(&(*it));
		it += 2;

		robot_pose_.theta = utility::from_byte_ptr(&(*it));
		it += 2;
	}
	else if (msg_type_in_ == msg_type_in::SIM_TARGET) {
		target_.x = utility::from_byte_ptr(&(*it));
		it += 2;

		target_.y = utility::from_byte_ptr(&(*it));
		it += 2;
	}
	else if (msg_type_in_ == msg_type_in::LINE) {
		// LOG_INFO("Received line");
		robot_pose_.x = utility::from_byte_ptr(&(*it));
		it += 2;

		robot_pose_.y = utility::from_byte_ptr(&(*it));
		it += 2;

		robot_pose_.theta = utility::from_byte_ptr(&(*it));
		it += 2;

		line_.startPoint.x = utility::from_byte_ptr(&(*it));
		it += 2;

		line_.startPoint.y = utility::from_byte_ptr(&(*it));
		it += 2;

		line_.endPoint.x = utility::from_byte_ptr(&(*it));
		it += 2;

		line_.endPoint.y = utility::from_byte_ptr(&(*it));
		it += 2;

		//LOG_INFO("({},{}) --- ({},{})", line_.startPoint.x, line_.startPoint.y, line_.endPoint.x, line_.endPoint.y);
	}
}

void message::set_payload(pose pos, bool sim)
{
	robot_pose_ = pos;
	is_pose_ = true;
	valid_ = true;
	if (sim)
		msg_type_in_ = msg_type_in::SIM_INIT;
	else
		msg_type_out_ = msg_type_out::INIT;
}


void message::set_payload(position pos, bool sim)
{
	target_ = pos;
	valid_ = true;
	if (sim)
		msg_type_in_ = msg_type_in::SIM_TARGET;
	else
		msg_type_out_ = msg_type_out::TARGET;
}

void message::set_payload(pose pos, std::vector<position> obstacles, int8_t is_object)
{
	robot_pose_ = pos;
	is_pose_ = true;
	obstacles_ = obstacles;
	is_sim_object_ = true;
	object_ = is_object;
	valid_ = true;
	msg_type_in_ = msg_type_in::UPDATE;
}


void message::set_scan() 
{
	msg_type_in_ = msg_type_in::SCAN_BORDER;
}

void message::set_collision()
{
	msg_type_in_ = msg_type_in::COL_DET;
}

bool message::is_valid() const
{
	return valid_;
}

std::vector<std::byte> message::serialize() const
{
	std::vector<std::byte> raw;

	auto serialize_int16 = [&](int16_t val) {
		auto data_ptr = reinterpret_cast<const std::byte *> (&val);
		raw.push_back(data_ptr[0]);
		raw.push_back(data_ptr[1]);
	};
	auto serialize_int8 = [&](int8_t val) {
		auto data_ptr = reinterpret_cast<const std::byte*> (&val);
		raw.push_back(data_ptr[0]);
	};

	/*
	If the message is a scan border or collision detected, only serialize the message type. (Output from simulator to server)
	If the message is an incoming message, don't serialize msg_type_out, except simulated msgs (Output from simulator to server).
	If the message is an outgoing message, don't serialize msg_type_in. (Output from server to robots)
	*/

	if (msg_type_in_ == msg_type_in::SCAN_BORDER || msg_type_in_ == msg_type_in::COL_DET){
		serialize_int8((int8_t)msg_type_in_);
		return raw;
	}
	else if (msg_type_in_ == msg_type_in::UPDATE || msg_type_in_ == msg_type_in::SIM_INIT || msg_type_in_ == msg_type_in::SIM_TARGET)
		serialize_int8((int8_t)msg_type_in_);
	else
		serialize_int8((int8_t)msg_type_out_);

	if (is_pose_) {
		serialize_int16(robot_pose_.x);
		serialize_int16(robot_pose_.y);
		serialize_int16(robot_pose_.theta);
	}
	else {
		serialize_int16(target_.x);
		serialize_int16(target_.y);
	}

	//For simulation:
	for (const auto& obs : obstacles_) {
		serialize_int16(obs.x);
		serialize_int16(obs.y);
	}
	//For simulation:
	if (is_sim_object_)
		serialize_int8(object_);

	return raw;
}

std::string message::sender() const
{
	return sender_;
}

pose message::robot_pos() const
{
	return robot_pose_;
}

position message::target() const
{
	return target_;
}

line message::get_line() const
{
	return line_;
}

std::vector<bool> message::is_object() const
{
	return is_object_;
}

std::vector<position> message::obstacles() const
{
	return obstacles_;
}

/*For simulation and after fixed message from robot*/

message::msg_type_in message::type() const
{
	return msg_type_in_;
}

bool message::sanity_check(msg_type_in msg_type, int size)
{
	constexpr auto pose_size = 3 * sizeof(int16_t);
	constexpr auto position_size = 2 * sizeof(int16_t);
	constexpr auto obs_size = 8 * sizeof(int16_t);
	constexpr auto identifier_size = sizeof(int8_t);

	//Message type SCAN_BORDER and COL_DET:
	//Size of 1 int8_t, msg_type

	//Message type UPDATE:
	//Size of two int8_t's, msg_type identifier and object/non-object identifier +
	//Size of 3 int16_t's, pose +
	//Size of 8 int16_t's, 4 "object" positions

	//Message type SIM_INIT:
	//Size of 1 int8_t, msg_type +
	//Size of 3 int16_t's, pose

	//Message type SIM_TARGET:
	//Size of 1 int8_t, msg_type +
	//Size of 2 int16_t's, position

	if (msg_type == msg_type_in::SCAN_BORDER || msg_type == msg_type_in::COL_DET)
		return (size == identifier_size);
	else if (msg_type == msg_type_in::UPDATE)
		return (size == (2 * identifier_size + pose_size + obs_size));
	else if (msg_type == msg_type_in::SIM_INIT)
		return (size == (identifier_size + pose_size));
	else if (msg_type == msg_type_in::SIM_TARGET)
		return (size == (identifier_size + position_size));
	else if (msg_type == msg_type_in::LINE)
		return (size == (identifier_size + pose_size + 2 * position_size));
	else
		return false;
}

}
