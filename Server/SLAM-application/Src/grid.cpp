#include "grid.h"

#include <iostream>

using namespace boost;

namespace NTNU::graph::base
{

grid::grid(int16_t rows, int16_t cols) :
	grid_()
{
	reset_to(rows, cols);
}

void grid::reset_to(int16_t rows, int16_t cols)
{
	grid_ = std::make_unique<NTNU::graph::grid::bgrid>(array<int16_t, 2>({ int16_t(rows), int16_t(cols) }));
}

int16_t grid::rows() const
{
	return grid_->length(0);
}

int16_t grid::columns() const
{
	return grid_->length(1);
}

}
