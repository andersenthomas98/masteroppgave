#pragma once

#include <functional>
#include <any>

#include "panel_base.h"

namespace NTNU::gui::panel
{

class panel
{
	using fun = std::function<void()>;

public:
	// If a title is given, the panel will have its own window.
	// Else, it is expected to be embedded in another window.
	panel(std::optional<std::string> title = std::nullopt);

	virtual ~panel() {};

	void set_fun(fun f);
	void show() const;

private:
	fun fun_;
	std::optional<std::string> title_;
};

}

