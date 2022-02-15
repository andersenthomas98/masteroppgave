    #pragma once
#include <SFML/Graphics.hpp>
#include <optional>
#include <stddef.h>
#include <vector>
#include "slam_grid.h"

namespace NTNU::application::SLAM::utility
{

std::optional<std::pair<float, float>> row_col_to_coord(std::array<int16_t, 2> size, int16_t separation, int16_t row, int16_t col);
std::optional<std::pair<int16_t, int16_t>> coord_to_row_col(std::array<int16_t,2> size, int16_t separation, float x, float y);

std::vector<std::pair<float, float>> points_to_coords(const NTNU::application::SLAM::slam_grid& grid, const std::vector<std::pair<int16_t, int16_t>>& points);

std::pair< int16_t, std::vector<std::pair<int16_t, int16_t>> > get_line_between_pts(std::pair<int16_t, int16_t> p1, std::pair<int16_t, int16_t> p2);
double get_random(double mean, double std);

sf::Color get_random_color(int16_t alpha = 255);

int16_t from_byte_ptr(const std::byte* pbyte);
int8_t from_byte_to_int8_ptr(const std::byte* pbyte);
int16_t from_bytes(std::byte lsb, std::byte msb);

}
