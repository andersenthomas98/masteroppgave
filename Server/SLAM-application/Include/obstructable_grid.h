#pragma once

#include <memory>
#include <map>
//#include <boost/unordered_set.hpp>
#include "grid_common.h"
#include "grid.h"

using NTNU::graph::base::grid;

namespace NTNU::graph::grid {

class obstructable_grid : public grid
{
public:
	obstructable_grid(int16_t rows, int16_t cols);

	virtual void obstruct(int16_t row, int16_t col);
	virtual void semi_obstruct(int16_t, int16_t col);
	virtual void unobstruct(int16_t row, int16_t col);
	virtual void single_reset(int16_t row, int16_t col);
	bool is_obstructed(int16_t row, int16_t col) const;
	bool is_unobstructed(int16_t row, int16_t col) const;
	bool is_semi_obstructed(int16_t row, int16_t col) const;

	void reset_to(int16_t rows, int16_t cols);

	filtered_grid get_filtered_grid() const;

	virtual ~obstructable_grid() {};

private:
	vertex_set obstructed_;
	//vertex_set unobstructed_;
	boost::unordered_set<std::pair<int16_t, int16_t>> unobstructed_; //Can probably be vertex_set if this works
	vertex_set total_obstructed_;
	std::shared_ptr<filtered_grid> fgrid_; //check:unique
};

}