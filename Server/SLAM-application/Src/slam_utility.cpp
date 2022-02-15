#define _USE_MATH_DEFINES

#include "slam_utility.h"
#include <cmath>
#include <iostream>
#include <random>
#include "log.h"

namespace NTNU::application::SLAM::utility
{

std::optional<std::pair<float, float>> row_col_to_coord(std::array<int16_t, 2> size, int16_t separation, int16_t row, int16_t col)
{
	if (row < 0 || col < 0)
	{
		//std::cout << "Can't have negative row/col";
		LOG_ERROR("Can't have negative row");
		return std::nullopt;
	}
	if (row > size[0] || col > size[1])
	{
		LOG_ERROR("Row or col out of range");
		//std::cout << "Row or col out of range" << std::endl;
		return std::nullopt;
	}

	const float row_offset = size[0] * separation / 2.0f;
	const float col_offset = size[1] * separation / 2.0f;

	float y = -row_offset;
	float x = -col_offset;

	y += row * separation;
	x += col * separation;

	return std::make_pair(x, y);
}

std::optional<std::pair<int16_t, int16_t>> coord_to_row_col(std::array<int16_t,2> size, int16_t separation , float x, float y)
{
	// Coordinates are signed,
	// row, col are unsigned.
	// Therefore we must offset the result such as that the
	// most negative possible signed result becomes row col {0, 0}.
	const float row_offset = size[0]*separation / 2.0f;
	const float col_offset = size[1]*separation / 2.0f;

	y += row_offset;
	x += col_offset;

	if (x < 0 || y < 0)
	{
		//std::cout << "Coord out of bounds (less than zero)" << std::endl;
		return std::nullopt;
	}

	auto row_f = y / separation;
	auto col_f = x / separation;


	auto row = static_cast<int16_t>(row_f);
	auto col = static_cast<int16_t>(col_f);

	if (row > size[0] || col > size[0])
	{
		//std::cout << "Coord out of bounds: " << x << ", " << y << ", Row/col: " << row << ", " << col << std::endl;
		return std::nullopt;
	}

	return std::make_pair(row, col);
}

std::vector<std::pair<float, float>> points_to_coords(const NTNU::application::SLAM::slam_grid & grid, const std::vector<std::pair<int16_t, int16_t>>& points)
{
	std::vector<std::pair<float, float>> coords;

	std::array<int16_t, 2> size = { grid.rows(), grid.cols() };

	for (const auto&[row, col] : points)
	{

		auto result = row_col_to_coord(size, grid.separation(), (int16_t)row, (int16_t)col);
		if (result)
		{
			auto[x, y] = result.value();

			auto sep = grid.separation();
			
			x += sep / 2;
			y += sep / 2;

			coords.emplace_back(x, y);
		}
	}

	return coords;
}

double utility::get_random(double mean, double std)
{
	static std::random_device rd;
	static std::mt19937 gen(rd());

	if (std < 0.001)
		std = 0.001;

	std::normal_distribution<double> dis(mean, std);

	return dis(gen);
}

std::vector<std::pair<int16_t, int16_t>> bresenham(std::pair<int16_t,int16_t> start, std::pair<int16_t,int16_t> end) {
	std::vector<std::pair<int16_t, int16_t>> ray;

	int16_t startX = start.second;
	int16_t startY = start.first;
	int16_t endX = end.second;
	int16_t endY = end.first;

	ray.push_back({ startY , startX });

	int16_t dx = endX - startX;
	int16_t dy = endY - startY;
	int16_t D = 2 * dy - dx;
	int16_t y = startY;
	if (D > 0) {
		y = y + 1;
		D = D - (2 * dx);
	}

	for (int16_t x = startX + 1; x < endX; x++) {
		ray.push_back({y,x});

		D = D + (2 * dy);
		if (D > 0) {
			y = y + 1;
			D = D - (2 * dx);
		}

	}
	return ray;
}

std::pair<int16_t,int16_t> switchToOctantZeroFrom(int16_t octant, std::pair<int16_t,int16_t> location) {
	std::pair<int16_t, int16_t> newLocation = { 0 , 0 };
	switch (octant) {
	case 0:
		newLocation.second = location.second;
		newLocation.first = location.first;
		return newLocation;
	case 1:
		newLocation.second = location.first;
		newLocation.first = location.second;
		return newLocation;
	case 2:
		newLocation.second = location.first;
		newLocation.first = -location.second;
		return newLocation;
	case 3:
		newLocation.second = -location.second;
		newLocation.first = location.first;
		return newLocation;
	case 4:
		newLocation.second = -location.second;
		newLocation.first = -location.first;
		return newLocation;
	case 5:
		newLocation.second = -location.first;
		newLocation.first = -location.second;
		return newLocation;
	case 6:
		newLocation.second = -location.first;
		newLocation.first = location.second;
		return newLocation;
	case 7:
		newLocation.second = location.second;
		newLocation.first = -location.first;
		return newLocation;
	default:
		//std::cout << "ERROR INVALID OCTANT: " << octant << std::endl;
		LOG_ERROR("Invalid Octant: {}", octant);
		return newLocation;
	}
}

std::pair<int16_t,int16_t> switchFromOctantZeroTo(int16_t octant, std::pair<int16_t,int16_t> location) {
	std::pair<int16_t, int16_t> newLocation = { 0 , 0 };
	switch (octant) {
	case 0:
		newLocation.second = location.second;
		newLocation.first = location.first;
		return newLocation;
	case 1:
		newLocation.second = location.first;
		newLocation.first = location.second;
		return newLocation;
	case 2:
		newLocation.second = -location.first;
		newLocation.first = location.second;
		return newLocation;
	case 3:
		newLocation.second = -location.second;
		newLocation.first = location.first;
		return newLocation;
	case 4:
		newLocation.second = -location.second;
		newLocation.first = -location.first;
		return newLocation;
	case 5:
		newLocation.second = -location.first;
		newLocation.first = -location.second;
		return newLocation;
	case 6:
		newLocation.second = location.first;
		newLocation.first = -location.second;
		return newLocation;
	case 7:
		newLocation.second = location.second;
		newLocation.first = -location.first;
		return newLocation;
	default:
		return newLocation;
	}
}

int16_t getOctant(double angle) {
	if (angle > 0 && angle <= 45) {
		return 0;
	} else if (angle > 45 && angle <= 90) {
		return 1;
	} else if (angle > 90 && angle <= 135) {
		return 2;
	} else if (angle > 135 && angle <= 180) {
		return 3;
	} else if (angle > 180 && angle <= 225) {
		return 4;
	} else if (angle > 225 && angle <= 270) {
		return 5;
	} else if (angle > 270 && angle <= 315) {
		return 6;
	} else if ((angle > 315 && angle < 360) || angle == 0) {
		return 7;
	} else {
		//	std::cout << "ERROR -- INVALID ANGLE: " << angle << std::endl;
		LOG_ERROR("Invalid angle: {}", angle);
		return -1;
	}
}

std::pair< int16_t, std::vector<std::pair<int16_t,int16_t>>> get_line_between_pts(std::pair<int16_t, int16_t> p1, std::pair<int16_t, int16_t> p2) { 
	auto dx = p2.second - p1.second;
	auto dy = p2.first - p1.first;
	auto angle = atan2(dy, dx) * (180 / M_PI); // Degrees
	if (angle < 0) {
		angle += 360;
	}

	auto oct = getOctant(angle);
	auto locOct = switchToOctantZeroFrom(oct, { dy, dx });
	auto lineOct = bresenham({ 0 , 0 }, locOct);

	auto size = lineOct.size();
	auto size_test = (int)size;

	std::vector<std::pair<int16_t, int16_t>> line;
	//std::pair<int, int> * line;
	//line = new std::pair<int,int>[size + 1];

	for (auto i = 0; i < size; i++) {
		auto loc = switchFromOctantZeroTo(oct, lineOct[i]);
		//line[i] = std::make_pair(p1.first + loc.first, p1.second + loc.second);
		line.push_back(std::make_pair((int16_t)(p1.first + loc.first), (int16_t)(p1.second + loc.second)));
	}

	//To ensure endpoint is included
	//line[size] = std::make_pair(p2.first, p2.second);
	line.push_back(std::make_pair((int16_t)p2.first, (int16_t)p2.second));

	return std::make_pair(size + 1, line);
}

sf::Color get_random_color(int16_t alpha)
{
	return sf::Color(get_random(0, 255), get_random(0, 255), get_random(0, 255), alpha);
}

int16_t from_byte_ptr(const std::byte * pbyte)
{
	return *reinterpret_cast<const int16_t *>(pbyte);
}

int8_t from_byte_to_int8_ptr(const std::byte* pbyte)
{
	return *reinterpret_cast<const int8_t*>(pbyte);
}

int16_t from_bytes(std::byte lsb, std::byte msb)
{
	std::vector<std::byte> v{ lsb, msb };
	return from_byte_ptr(v.data());
}

}
