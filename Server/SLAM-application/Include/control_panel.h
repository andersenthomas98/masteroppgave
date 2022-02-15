#pragma once
#include "panel.h"
#include "callbacks.h"

namespace NTNU::gui::panel
{
enum class control_panel_event
{
	QUIT
};

class control_panel :
	public panel,
	public NTNU::utility::callbacks<enum control_panel_event>
{
public:
	control_panel();
	void embed_panel(const NTNU::gui::panel::panel* panel, const std::string& title);
	void remove_panel();

	~control_panel() {};

private:
	std::vector<std::pair<const NTNU::gui::panel::panel*, std::string>> panels_;
};

}
