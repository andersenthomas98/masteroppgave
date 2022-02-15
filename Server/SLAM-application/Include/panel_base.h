#pragma once

#include <utility>
#include <optional>

namespace gui::base
{

using fpair = std::pair<float, float>;
using ofpair = std::optional<fpair>;

class panel_base
{
public:

	panel_base(ofpair size, ofpair pos);
	virtual ~panel_base() = default;

	virtual bool show() = 0;

	void setSize(fpair size);
	ofpair getSize() const;

	void setPos(fpair pos);
	ofpair getPos() const;

protected:
	ofpair size_;
	ofpair pos_;
};

}

