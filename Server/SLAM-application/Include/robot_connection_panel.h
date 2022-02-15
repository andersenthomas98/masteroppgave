#pragma once

#include "panel.h"
#include <stdbool.h>
#include <array>

namespace NTNU::gui::panel {

	class robot_connection_panel :
		public panel
	{
	public:
		robot_connection_panel();
		~robot_connection_panel() {};

		bool ready() const;
		std::array<int16_t, 3> get_init_pose() const;

	private:
		std::array<int16_t, 3> init_pose_;
		bool ready_;
	};

}
