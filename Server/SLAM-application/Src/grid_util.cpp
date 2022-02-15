#include "grid_util.h"
#include <boost/graph/graphviz.hpp>
#include <iostream>

using vertex_descriptor = boost::graph_traits<boost::grid_graph<2, int16_t>>::vertex_descriptor;

namespace NTNU::graph::grid::utility
{

vertex_descriptor convert(int16_t row, int16_t col)
{
	return { static_cast<int16_t>(row), static_cast<int16_t>(col) };
}

}

