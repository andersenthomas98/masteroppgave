#pragma once

#include <string>
#include <vector>
#include <optional>
#include <variant>

namespace NTNU::application::SLAM
{

class message
{
public:
	//Add message types here if more is needed.
	//Remember to update the sanity_check if so.
	//Types are initialised to the first alternative, so this is NOT_xx

	//Revise names, especially init (set?)
	enum class msg_type_in : int8_t {
		NOT_IN,
		SCAN_BORDER,
		UPDATE,
		COL_DET,
		SIM_INIT = 8,
		SIM_TARGET = 9
	};

	enum class msg_type_out : int8_t {
		NOT_OUT,
		INIT,
		TARGET
	};

	struct position {
		int16_t x;
		int16_t y;
	};

	struct pose {
		int16_t x;
		int16_t y;
		int16_t theta;
	};

	message(std::string sender = "");

	void set_payload(std::string payload);
	void set_payload(pose pos, bool sim);
	void set_payload(pose pos, std::vector<position> obstacles, int8_t is_object);
	void set_payload(position pos, bool sim);
	void set_scan();
	void set_collision();
	bool is_valid() const;

	std::vector<std::byte> serialize() const;

	std::string sender() const;
	std::vector<bool> is_object() const;
	pose robot_pos() const;
	position target() const;
	msg_type_in type() const;
	std::vector<position> obstacles() const;

	virtual ~message() {};

private:
	bool valid_;
	bool is_pose_;
	std::string sender_;
	std::vector<bool> is_object_;
	bool is_sim_object_;
	int8_t object_;
	pose robot_pose_;
	position target_;
	std::vector<position> obstacles_;
	msg_type_in msg_type_in_;
	msg_type_out msg_type_out_;

	bool sanity_check(msg_type_in msg_type, int size);
};

}
