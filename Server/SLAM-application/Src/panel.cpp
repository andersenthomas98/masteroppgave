#include "panel.h"
#include "imgui.h"

namespace NTNU::gui::panel
{

panel::panel(std::optional<std::string> title) :
	fun_([]() {; }),
	title_(title)
{
}

void panel::set_fun(fun f)
{
	fun_ = f;
}

void panel::show() const
{
	if (title_)
	{
		ImGui::Begin(title_.value().c_str());
		fun_();
		ImGui::End();
	}
	else
	{
		fun_();
	}
}

}
