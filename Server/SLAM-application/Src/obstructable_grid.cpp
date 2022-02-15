#include "grid_common.h"
#include "obstructable_grid.h"
#include "grid_util.h"

using NTNU::graph::grid::utility::convert;


//---------------------- Grid to manage obstructions and available points ----------------- //


namespace NTNU::graph::grid {
obstructable_grid::obstructable_grid(int16_t rows, int16_t cols) :
	NTNU::graph::base::grid(rows, cols)
{
	reset_to(rows, cols);
}
void obstructable_grid::obstruct(int16_t row, int16_t col)
{
	if (is_unobstructed(row, col))
		unobstructed_.erase({ row, col });

	obstructed_.insert(convert(row, col));
	total_obstructed_.insert(convert(row, col));
}
void obstructable_grid::unobstruct(int16_t row, int16_t col)
{
	if (is_obstructed(row, col)) {
		//test
		return;

		obstructed_.erase(convert(row, col));
		//total_obstructed_.erase(convert(row, col));
	}
	
	unobstructed_.insert({ {row, col}, {row, col} });
}

void obstructable_grid::semi_obstruct(int16_t row, int16_t col)
{
	//if (is_unobstructed(row, col))
	//	unobstructed_.erase(convert(row, col));

	total_obstructed_.insert(convert(row, col));
}

void obstructable_grid::single_reset(int16_t row, int16_t col)
{
	if (is_obstructed(row, col)) 
		obstructed_.erase(convert(row, col));
	else if (is_unobstructed(row, col)) 
		unobstructed_.erase({ row, col });
	
	if (is_semi_obstructed(row, col)) 
		total_obstructed_.erase(convert(row, col));
}
bool obstructable_grid::is_obstructed(int16_t row, int16_t col) const
{
	return obstructed_.find(convert(row, col)) != obstructed_.end();
}

bool obstructable_grid::is_unobstructed(int16_t row, int16_t col) const
{
	return unobstructed_.find({ row, col }) != unobstructed_.end();
}

bool obstructable_grid::is_semi_obstructed(int16_t row, int16_t col) const
{
	return total_obstructed_.find(convert(row, col)) != total_obstructed_.end();
}

void obstructable_grid::reset_to(int16_t rows, int16_t cols)
{
	grid::reset_to(rows, cols);
	obstructed_.clear();
	unobstructed_.clear();
	total_obstructed_.clear();
	fgrid_ = std::make_unique<filtered_grid>(boost::make_vertex_subset_complement_filter(*grid_.get(), total_obstructed_));
}

filtered_grid obstructable_grid::get_filtered_grid() const
{
	return *fgrid_.get();
}
}
