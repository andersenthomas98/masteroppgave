#pragma once
#include "panel.h"
#include <optional>
#include <stdbool.h>

using ipair = std::pair<int, int>;

namespace NTNU::gui::panel 
{
	class target_panel :
		public panel
	{
	public:
		target_panel();
		~target_panel() {};

		ipair get_manual_target();
		void set_manual_target(ipair manual_target);

		bool get_manual_robot_drive();
		void set_manual_robot_drive(bool manual_robot_drive);
	private:
		ipair manual_target_;
		bool manual_robot_drive_;
	};
}


