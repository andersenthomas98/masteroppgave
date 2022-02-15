#pragma once

#include "grid_common.h"

namespace NTNU::graph::base
{

class grid
{
public:
	grid(int16_t rows, int16_t cols);
	virtual ~grid() {};

	void reset_to(int16_t rows, int16_t cols);

	int16_t rows() const;
	int16_t columns() const;
protected:
	std::shared_ptr<NTNU::graph::grid::bgrid> grid_;//check:unique
	
};

}
