#pragma once
#include "panel.h"
#include "callbacks.h"

namespace NTNU::gui::panel
{

enum class clicks_events
{
	left_click_mode_changed,
	right_click_mode_changed,
};

enum clicks_choices : int
{
	nothing = 0,
	set_target = 1,
	obstruct = 2,
};

class clicks : public panel, public NTNU::utility::callbacks<clicks_events, clicks_choices>
{
public:
	clicks();

	bool left_is(clicks_choices choice) const;
	bool right_is(clicks_choices choice) const;

	~clicks();

private:
	clicks_choices left_click_;
	clicks_choices right_click_;

};

}
