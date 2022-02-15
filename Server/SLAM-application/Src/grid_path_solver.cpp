#include "grid_common.h"
#include "vertex_hash.h"
#include "grid_path_solver.h"
#include <boost/graph/astar_search.hpp>
#include <boost/unordered_map.hpp>
#include <algorithm>
#include <cmath>

using vertex_hash = NTNU::graph::grid::vertex_hash;
using grid = boost::grid_graph<2, int16_t>;
using vertex_descriptor = boost::graph_traits<grid>::vertex_descriptor;

namespace NTNU::graph::pathfinding
{

// Helper class; determines the heuristic for what constitues a "good" path
class euclidean_heuristic :
	public boost::astar_heuristic<filtered_grid, double>
{
public:
	euclidean_heuristic(vertex_descriptor goal) :
		goal_(goal) {};

	double operator()(vertex_descriptor v) {
		return sqrt(pow(double(goal_[0] - v[0]), 2) + pow(double(goal_[1] - v[1]), 2));
	}

private:
	vertex_descriptor goal_;
};

struct found_goal {};

// Visitor that terminates when we find the goal vertex
struct astar_goal_visitor : public boost::default_astar_visitor {
	astar_goal_visitor(vertex_descriptor goal) :
		goal_(goal) {};

	void examine_vertex(vertex_descriptor u, const filtered_grid&) {
		if (u == goal_)
			throw found_goal();
	}

private:
	vertex_descriptor goal_;
};

std::optional<std::vector<point>> solve(const filtered_grid& grid, point from, point to)
{
	typedef boost::unordered_map<vertex_descriptor, vertex_descriptor, vertex_hash> pred_map;
	typedef boost::unordered_map<vertex_descriptor, double, vertex_hash> dist_map;

	boost::static_property_map<double> weight(1);

	// The predecessor map is a vertex-to-vertex mapping.
	pred_map predecessor;
	boost::associative_property_map<pred_map> pred_pmap(predecessor);
	// The  map is a vertex-to-double mapping.
	dist_map distance;
	boost::associative_property_map<dist_map> dist_pmap(distance);

	vertex_descriptor s = { {static_cast<int16_t>(from.first), static_cast<int16_t>(from.second)} };
	vertex_descriptor g = { {static_cast<int16_t>(to.first), static_cast<int16_t>(to.second)} };

	euclidean_heuristic heuristic(g);
	astar_goal_visitor visitor(g);

	try {
		astar_search(grid, s, heuristic,
			boost::weight_map(weight).
			predecessor_map(pred_pmap).
			distance_map(dist_pmap).
			visitor(visitor));
	}
	catch (found_goal) {
		std::vector<point> solution;
		// Walk backwards from the goal through the predecessor chain adding
		// vertices to the solution path.
		for (vertex_descriptor u = g; u != s; u = predecessor[u])
			solution.push_back({ u[0], u[1] });

		solution.push_back({ s[0], s[1] });

		// Reverse the vector since we now have a to-->from solution instead of a from-->to.
		std::reverse(solution.begin(), solution.end());
		return solution;
	}

	return std::nullopt;
}

std::vector<point> reduce(const std::vector<point> path)
{
	if (path.size() == 0)
		return {};

	std::vector<point> reduction{path.at(0)};
	
	for (auto it = path.cbegin() + 1; it != path.cend(); it++)
	{
		const auto& [row, col] = reduction.back();
		const auto& [next_row, next_col] = *it;

		auto distance = std::sqrt(std::pow(next_row - row, 2) + std::pow(next_col - col, 2));

		if ((row != next_row && col != next_col) || distance >= 40 )
		{
			// We changed rows and cols, line cannot be straight.
			// In order to keep a straight line we need to store the previous point.
			auto prev = --it;
			reduction.push_back(*prev);
		}
	}

	reduction.push_back(path.back());
	return reduction;
}

}
