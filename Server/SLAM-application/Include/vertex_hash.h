#pragma once

#include <boost/graph/properties.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/container_hash/hash.hpp>

namespace NTNU::graph::grid
{

// Uses both indices ({x, y}) as seed for hash.
struct vertex_hash
{
	std::size_t operator()(boost::graph_traits<boost::grid_graph<2, int16_t>>::vertex_descriptor const& u) const {
		std::size_t seed = 0;
		boost::hash_combine(seed, u[0]);
		boost::hash_combine(seed, u[1]);
		return seed;
	}

	std::size_t operator()(std::pair<std::pair<boost::graph_traits<boost::grid_graph<2, int16_t>>::vertex_descriptor, boost::graph_traits<boost::grid_graph<2, int16_t>>::vertex_descriptor>, int> const& u) const {
		std::size_t seed = 0;
		boost::hash_combine(seed, u.first.first[0]);
		boost::hash_combine(seed, u.first.first[1]);
		boost::hash_combine(seed, u.first.second[0]);
		boost::hash_combine(seed, u.first.second[1]);
		return seed;
	}
};

}
