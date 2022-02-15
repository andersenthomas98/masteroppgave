#include "panel_base.h"

namespace gui::base
{

panel_base::panel_base(ofpair size, ofpair pos) :
	size_(size),
	pos_(pos)
{
}

void panel_base::setSize(fpair size)
{
	size_ = size;
}

ofpair panel_base::getSize() const
{
	return size_;
}

void panel_base::setPos(fpair pos)
{
	pos_ = pos;
}

ofpair panel_base::getPos() const
{
	return pos_;
}

}
