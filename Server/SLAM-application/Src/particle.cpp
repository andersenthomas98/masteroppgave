#define _USE_MATH_DEFINES

#include "particle.h"
#include "slam_utility.h"
#include "eigenmvn.h"
#include "log.h"
#if defined __GNUC__ || defined __APPLE__
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif
#include <boost/chrono.hpp>
#include <cmath>
#include <random>
#include <iostream>
#include <numeric>
#include <string>

using NTNU::graph::grid::obstructable_grid;
using NTNU::application::SLAM::utility::coord_to_row_col;

template <typename T>
bool IsInBounds(const T& value, const T& low, const T& high) {
    return !(value < low) && (value < high);
}


namespace NTNU::application::SLAM
{

Particle::Particle(int16_t id_p, pose_t pose_p, double weight_p, obstructable_grid map_p) :
    //Particle constructor
    id(id_p),
    pose(pose_p),
    weight(weight_p),
    map(map_p)
{
}

Particle::Particle(int16_t id_p, pose_t pose_p, double weight_p) :
    //Particle constructor
    id(id_p),
    pose(pose_p),
    weight(weight_p)
{
}

    //"Settings" as a tuple with 5 arbitrary is not very clean. Revise
particleFilter::particleFilter(std::string id, pose_t init_pose, std::array<double,4> a, std::array<int16_t,4> settings):
    robot_id_(id),
    pose_(init_pose),
    //a is an array containing factors to estimate noise in odometry readings
    a_(a),
	weight_(),
    size_({ settings[1], settings[0] }),  //rows, cols
    separation_(settings[2]),
	noiseObs_(),
    numParticles_(settings[3]),
    particles_(),
    sample_std_(5),
    best_(0)
{
    //Initialise particles

    for (auto i = 0; i < numParticles_; i++) {
        pose_t pose = { init_pose.x , init_pose.y , init_pose.theta };

        obstructable_grid map_p = obstructable_grid(size_[0], size_[1]);
        Particle p(i, pose, 1.0, map_p);

        particles_.push_back(std::make_shared<Particle>(p));
    }
}

//Check only last 3 points
bool collinearityCheck(std::vector<message::position>& points) {
    auto tolerance = 5.0;
    auto last = points.size() - 1;

    auto diff = (std::abs((points[last - 2].y - points[last - 1].y) * (points[last - 2].x - points[last].x) - (points[last - 2].y - points[last].y) * (points[last - 2].x - points[last - 1].x)) <= tolerance);
    return diff;
}


//Check first line and last line
std::optional<std::array<cell_t, 2>> collinearityCheck(std::vector<cell_t>& cells) {
    auto tolerance = 5.0;
    auto last = cells.size() - 1;

    //Compute vector of direction for both lines
    cell_t r = { cells[1].row - cells[0].row, cells[1].col - cells[0].col };
    cell_t s = { cells[last].row - cells[last - 1].row, cells[last].col - cells[last - 1].col };

    cell_t diff = { cells[last - 1].row - cells[0].row, cells[last - 1].col - cells[0].col };


    if (std::abs((double)r.row * (double)s.col - (double)r.col * (double)s.row) <= tolerance && std::abs((double)diff.row * (double)r.col - (double)diff.col * (double)r.row) <= tolerance)
    {
        //If collinearity, check if they intersect or almost intersect (no space between)

        auto t0 = ((double)diff.row * (double)r.row + (double)diff.col * (double)r.col) / ((double)r.row * (double)r.row + (double)r.col * (double)r.col);
        auto t1 = t0 + ((double)s.row * (double)r.row + (double)s.col * (double)r.col) / ((double)r.row * (double)r.row + (double)r.col * (double)r.col);

        auto buff = 5 / (std::sqrt(std::pow(r.row, 2) + std::pow(r.col, 2)));

        if ((double)s.row * (double)r.row + (double)s.col * (double)r.col < 0)
        {
            if (t0 < 0 - buff || t1 > 1 + buff)                                                                                                                                                                                                                                                                                                                                                                                                                
                return std::nullopt;
            else if (t0 < 1 && t1 > 0)
                return std::array<cell_t, 2> {cells[0], cells[1]};
            else if (t0 >= 1 && t1 <= 0 )
                return std::array<cell_t, 2> {cells[last - 1], cells[last]};
            else if (t1 <= 0 && t0 <= 1 )
                return std::array<cell_t, 2> {cells[last], cells[1]};
            else if (t1 >= 0 && t0 >= 1 )
                return std::array<cell_t, 2> {cells[0], cells[last-1]};
            else {
                //std::cout << "Missing line case detected (Opposite directions). t0 = " << t0 << ", t1 = " << t1 << ", buff = " << buff << std::endl;
                //std::cout << "Lines are R: " << r.row << ", " << r.col << ", S: " << s.row << ", " << s.col << ", diff: " << diff.row << ", " << diff.col << std::endl;
                LOG_WARN("Missing line case detected (Opposite directions). t0: {}, t1: {}, buff: {}", t0, t1, buff);
                LOG_WARN("Lines are R: {},{}   S: {},{}   diff: {},{}", r.row, r.col, s.row, s.col, diff.row, diff.col);
                return std::nullopt;
            }
        }
        else {

            if (t1 < 0 - buff || t0 > 1 + buff)
                return std::nullopt;
            else if (t1 < 1 && t0 > 0 )
                return std::array<cell_t, 2> {cells[0], cells[1]};
            else if (t1 >= 1 && t0 <= 0)
                return std::array<cell_t, 2> {cells[last - 1], cells[last]};
            else if (t0 <= 0 && t1 <= 1)
                return std::array<cell_t, 2> {cells[last - 1], cells[1]};
            else if (t0 >= 0 && t1 >= 1)
                return std::array<cell_t, 2> {cells[0], cells[last]};
            else {
                //std::cout << "Missing line case detected. t0 = " << t0 << ", t1 = " << t1 << ", buff = " << buff << std::endl;
                //std::cout << "Lines are R: " << r.row << ", " << r.col << ", S: " << s.row << ", " << s.col << ", diff: " << diff.row << ", " << diff.col << std::endl;
                LOG_WARN("Missing line case detected. t0: {}, t1: {}, buff: {}", t0, t1, buff);
                LOG_WARN("Lines are R: {},{}   S: {},{}   diff: {},{}", r.row, r.col, s.row, s.col, diff.row, diff.col);

                return std::nullopt;
            }
        }
    }
    else
        return std::nullopt;
}

//Extract lines from scan, store in vector
std::vector<std::pair<cell_t, int>> buildRaster(std::vector<message::position>& points, std::vector<bool>& is_object) {
    std::vector<message::position> collinear_points;
    std::vector<std::pair<cell_t, int>> local_raster_array;

    for (auto k = 0; k < 4; k++)
    {
        for (auto j = k; j < points.size(); j += 4)
        {
            if (is_object[j]) {
                //Check if points are collinear
                collinear_points.push_back(points[j]);

                //Need 3 points to check colinearity
                if (collinear_points.size() < 3)
                    continue;

                if (collinearityCheck(std::ref(collinear_points)) && j + 4 < points.size())
                    continue;
            }

            //If non-collinear or non-obstructed point is detected
            if (collinear_points.size() > 3) {
                auto last = collinear_points.size() - 2;

                //ROW IS LOCAL Y COORDINATE AND COL IS LOCAL X COORDINATE
                std::pair new_start = { cell_t({collinear_points[0].y , collinear_points[0].x}), collinear_points.size() };
                local_raster_array.push_back(new_start);
                std::pair new_end = { cell_t({collinear_points[last].y , collinear_points[last].x}), collinear_points.size() };
                local_raster_array.push_back(new_end);

                collinear_points.clear();
                collinear_points.push_back(points[j]);
            }
            else if (!collinear_points.empty())
                collinear_points.erase(collinear_points.begin());
        }
    }

    /* Print resulting raster array */
    //std::cout << "Current local lines: " << std::endl;
    //for (auto i = 0; i < local_raster_array.size(); i = i + 2)
    //    std::cout << "Start: " << local_raster_array[i].first.row << ", " << local_raster_array[i].first.col << ", End: " << local_raster_array[i + 1].first.row << ", " << local_raster_array[i + 1].first.col << std::endl;

    return local_raster_array;
}

void particleFilter::search_for_transform(Particle& p, std::vector<message::position>& points, pose_t& pose_guess, std::vector<bool>& is_object, std::array<int16_t, 2>& size, int16_t& separation, std::vector<std::pair<cell_t, int>>& local_raster_array, std::vector<std::pair<pose_t, int>>& best_trans)
{

    for (auto rot = -10; rot < 11; rot += 2) 
    {
        for (auto tx = -60; tx < 61; tx += 20)
        {
            for (auto ty = -60; ty < 61; ty += 20)
            {
                auto w = 1.0;

                for (auto k = 0; k < local_raster_array.size(); k += 2)
                {
                    //Transform to global coordinates and add search transform
                    auto transf_x_start = pose_guess.x + local_raster_array[k].first.col * cos((pose_guess.theta + rot) * M_PI / 180) - local_raster_array[k].first.row * sin((pose_guess.theta + rot) * M_PI / 180) + tx;
                    auto transf_y_start = pose_guess.y + local_raster_array[k].first.col * sin((pose_guess.theta + rot) * M_PI / 180) + local_raster_array[k].first.row * cos((pose_guess.theta + rot) * M_PI / 180) + ty;
                    auto transf_x_end = pose_guess.x + local_raster_array[k + 1].first.col * cos((pose_guess.theta + rot) * M_PI / 180) - local_raster_array[k + 1].first.row * sin((pose_guess.theta + rot) * M_PI / 180) + tx;
                    auto transf_y_end = pose_guess.y + local_raster_array[k + 1].first.col * sin((pose_guess.theta + rot) * M_PI / 180) + local_raster_array[k + 1].first.row * cos((pose_guess.theta + rot) * M_PI / 180) + ty;

                    //Transform to row/col of grid
                    auto obs_start_grid = coord_to_row_col(size, separation, transf_x_start, transf_y_start);
                    auto obs_end_grid = coord_to_row_col(size, separation, transf_x_end, transf_y_end);

                    if (obs_start_grid && obs_end_grid)
                    {
                        auto [row_start, col_start] = obs_start_grid.value();
                        auto [row_end, col_end] = obs_end_grid.value();

                        auto start = cell_t{ row_start, col_start };
                        auto end = cell_t{ row_end, col_end };

                        if (start == end) continue;

                        std::vector<cell_t> reference_check;

                        reference_check.push_back(start);
                        reference_check.push_back(end);

                        for (auto j = 0; j < p.raster_array.size(); j = j + 2)
                        {
                            reference_check.push_back(p.raster_array[j].first);
                            reference_check.push_back(p.raster_array[j + 1].first);

                            auto collinear = collinearityCheck(std::ref(reference_check));
                            if (!collinear)
                                continue;

                            //If lines are collinear and intersect, increase weight
                            w += sqrt(local_raster_array[k].second * p.raster_array[j].second); //Could also be mean weight
                            break;
                        }

                    }
                }
                

                /*  Novel Point to point  */
                /*
                pose_t transformed_pose;
                transformed_pose.x = pose_guess.x + tx * cos((pose_guess.theta + rot) * M_PI / 180) - ty * sin((pose_guess.theta + rot) * M_PI / 180);
                transformed_pose.y = pose_guess.y + tx * sin((pose_guess.theta + rot) * M_PI / 180) + ty * cos((pose_guess.theta + rot) * M_PI / 180);
                transformed_pose.theta = pose_guess.theta + rot;

                w = correlation_model(p, transformed_pose, points, is_object);
                */

                if (best_trans.empty() || best_trans[0].second < (int)w) {
                    best_trans.clear();
                    best_trans.push_back(std::make_pair(pose_t({ (double)tx, (double)ty, (double)rot }), (int)w));
                }
                else if (best_trans[0].second == (int)w) {
                    best_trans.push_back(std::make_pair(pose_t({ (double)tx, (double)ty, (double)rot }), (int)w));
                }
            }
        }
    }

    return;

}

void particleFilter::estimate_pose(Particle& p , pose_t& odom)
{
    //calculate the movements between previous and present instances
    auto rot_1 = atan2(odom.y, odom.x) - p.pose.theta * M_PI / 180;
    auto trans = sqrt(pow(odom.x,2) + pow(odom.y,2));
    auto rot_2 = odom.theta * M_PI / 180 - rot_1;

    //calculate correct noise stds according to the motion model
    auto rot_1_std = sqrt(a_[0] * abs(rot_1) + a_[1] * abs(trans));
    auto trans_std = sqrt(a_[2] * abs(trans) + a_[3] * (abs(rot_1) + abs(rot_2)));
    auto rot_2_std = sqrt(a_[0] * abs(rot_2) + a_[1] * abs(trans));

    //add the gaussian distributed noise
    auto rot_1_sampled = get_random(rot_1, rot_1_std);
    auto trans_sampled = get_random(trans, trans_std);
    auto rot_2_sampled = get_random(rot_2, rot_2_std);

    /* Evaluate the computations */
    /*
    std::cout << "rot 1" << rot_1 << std::endl;

    std::cout << "trans" << trans << std::endl;

    std::cout << "rot 2" << rot_2 << std::endl;

    std::cout << "rot 1 std" << rot_1_std << std::endl;
    std::cout << "trans std" << trans_std << std::endl;
    std::cout << "rot 2 std" << rot_2_std << std::endl;

    std::cout << "rot 1 sampled" << rot_1_sampled << std::endl;
    std::cout << "trans sampled" << trans_sampled << std::endl;
    std::cout << "rot 2 sampled" << rot_2_sampled << std::endl;
    */

    //update the pose
    p.pose.x = trans_sampled * cos(p.pose.theta * M_PI / 180.0 + rot_1_sampled) + p.pose.x;
    p.pose.y = trans_sampled * sin(p.pose.theta * M_PI / 180.0 + rot_1_sampled) + p.pose.y;
    p.pose.theta = rot_1_sampled * 180.0 / M_PI + rot_2_sampled * 180.0 / M_PI + p.pose.theta;

    /* For testing of scan matcher */
    /*
    auto prev = p.pose;

    p.pose.x = p.pose.x + odom.x;
    p.pose.y = p.pose.y + odom.y;
    p.pose.theta = p.pose.theta + odom.theta;

    if (p.pose.x == prev.x && p.pose.y == prev.y) {
        p.pose.x += 1;
    }
    */

}

double particleFilter::correlation_model(Particle& p, std::vector<message::position>& points, std::vector<bool>& is_object) {

    auto w = 1.0;
    //auto count = 0;
    //for (auto& obs : points) {
    std::vector<message::position> collinear_points;
    for (auto k = 0; k < 4; k++)
    {
        for (auto j = k; j < points.size(); j += 4) // j = k , j = j + 4
        {
            auto obs_x = p.pose.x + points[j].x * cos(p.pose.theta * M_PI / 180) - points[j].y * sin(p.pose.theta * M_PI / 180);
            auto obs_y = p.pose.y + points[j].x * sin(p.pose.theta * M_PI / 180) + points[j].y * cos(p.pose.theta * M_PI / 180);

            auto pos_grid = coord_to_row_col(size_, separation_, p.pose.x, p.pose.y);
            auto obs_grid = coord_to_row_col(size_, separation_, obs_x, obs_y);

            //Unobstruct line between robot pos and observation
            
            if (pos_grid && obs_grid) {
                auto [pos_row, pos_col] = pos_grid.value();
                auto [obs_row, obs_col] = obs_grid.value();
                

                if (is_object[j] && p.map.is_obstructed(obs_row, obs_col))
                    w++;

            }
           
            /* Add points to raster, does not affect particle weight. Comment out if testing RBPF without improved distribution */
            
            //If observation in point
            if (is_object[j])
            {
                //Check if points are collinear
                collinear_points.push_back(points[j]);

                //Need 3 points to check colinearity
                if (collinear_points.size() < 3)
                    continue;

                if (collinearityCheck(std::ref(collinear_points)) && j + 4 < points.size())
                    continue;
            }

            if (collinear_points.size() > 3) {
                //Last collinear point is the second last point in the vector
                auto last = collinear_points.size() - 2;
                    
                //Transform points to global coordinates
                auto obs_x_start = p.pose.x + collinear_points[0].x * cos(p.pose.theta * M_PI / 180) - collinear_points[0].y * sin(p.pose.theta * M_PI / 180);
                auto obs_y_start = p.pose.y + collinear_points[0].x * sin(p.pose.theta * M_PI / 180) + collinear_points[0].y * cos(p.pose.theta * M_PI / 180);
                auto obs_x_end = p.pose.x + collinear_points[last].x * cos(p.pose.theta * M_PI / 180) - collinear_points[last].y * sin(p.pose.theta * M_PI / 180);
                auto obs_y_end = p.pose.y + collinear_points[last].x * sin(p.pose.theta * M_PI / 180) + collinear_points[last].y * cos(p.pose.theta * M_PI / 180);

                auto obs_start_grid = coord_to_row_col(size_, separation_, obs_x_start, obs_y_start);
                auto obs_end_grid = coord_to_row_col(size_, separation_, obs_x_end , obs_y_end);

                if (obs_start_grid && obs_end_grid)
                {
                    auto [row_start, col_start] = obs_start_grid.value();
                    auto [row_end, col_end] = obs_end_grid.value();
                    auto [pos_row, pos_col] = pos_grid.value();

                    auto start = cell_t{ row_start, col_start };
                    auto end = cell_t{ row_end, col_end };

                    if (start == end) continue;
                    
                    std::pair new_cell_start = { cell_t({row_start,col_start}), collinear_points.size() };
                    p.raster_array.push_back(new_cell_start);

                    std::pair new_cell_end = { cell_t({row_end,col_end}), collinear_points.size() };
                    p.raster_array.push_back(new_cell_end);

                }

                //Clear the collinear points and add back the non-collinear point
                collinear_points.clear();
                collinear_points.push_back(points[j]);
            }
            else if (!collinear_points.empty())
                collinear_points.erase(collinear_points.begin());
            
            
        }
    }
    //std::cout << "w: " << w << std::endl;

    return w;

}

double particleFilter::correlation_model(Particle& p, pose_t& pose, std::vector<message::position>& points, std::vector<bool>& is_object) {

    auto w = 1.0;

    std::vector<message::position> collinear_points;
    for (auto k = 0; k < 4; k++)
    {
        for (auto j = k; j < points.size(); j += 4)
        {
            auto obs_x = pose.x + points[j].x * cos(pose.theta * M_PI / 180) - points[j].y * sin(pose.theta * M_PI / 180);
            auto obs_y = pose.y + points[j].x * sin(pose.theta * M_PI / 180) + points[j].y * cos(pose.theta * M_PI / 180);

            auto pos_grid = coord_to_row_col(size_, separation_, pose.x, pose.y);
            auto obs_grid = coord_to_row_col(size_, separation_, obs_x, obs_y);

            //Unobstruct line between robot pos and observation

            if (pos_grid && obs_grid) {
                auto [pos_row, pos_col] = pos_grid.value();
                auto [obs_row, obs_col] = obs_grid.value();

                /* More complex weighting */
                /*
                for (auto i = 0; i < size-1; i++) {
                    if (p.map.is_unobstructed(line[i].first, line[i].second))
                        w = w + 1 / size;
                    else if (p.map.is_obstructed(line[i].first, line[i].second)) {
                        w = w - 1 / size;
                        break;
                    }

                }
                */

                if (is_object[j] && p.map.is_obstructed(obs_row, obs_col))
                    w++;

            }
        }
    }

    auto correlation_score = w;
    return correlation_score;

}

//Input a single particle, or update all inside this function?
void particleFilter::update_particle(pose_t odom, std::vector<message::position> obstacles, std::vector<bool> is_object)
{
    //std::cout << "Odom: " << odom.x << ", " << odom.y << ", " << odom.theta << std::endl;
    //std::co"   " << odom.x << ", " << odom.y << ", " << odom.theta << std::endl;
    LOG_INFO("Odom        : x: {:>10}, y: {:>10}, theta: {:>10}", odom.x, odom.y, odom.theta);
    //auto t1 = boost::chrono::high_resolution_clock::now();
    
    /* Extract lines from new scan */
    auto local_raster_array = buildRaster(obstacles, is_object);
    
    for (auto& p : particles_ ) {
        auto probab = p->weight;

        pose_t pose_guess;
        pose_guess.x = p->pose.x + odom.x; // * cos/sin(DEG2RAD(odom.theta)) if not in global coordinates?
        pose_guess.y = p->pose.y + odom.y;
        pose_guess.theta = p->pose.theta + odom.theta;

        //std::cout << "Guess: " << pose_guess.x << ", " << pose_guess.y << ", " << pose_guess.theta << std::endl;

        //RBPF with improved distribution
        auto pose_est = scan_match(*p, obstacles, pose_guess, is_object, local_raster_array);

        //RBPF without improved distribution
        //auto pose_est = std::optional<std::vector<pose_t>>();

        if (!pose_est || odom == pose_t{0,0,0}) {
            estimate_pose(*p, odom);
            probab *= correlation_model(*p, obstacles, is_object);  //based on prediction.
            //std::cout << "New pose: " << p->pose.x << ", " << p->pose.y << ", " << p->pose.theta << ", Weight: " << probab << std::endl;
            LOG_FILE("New position: x: {: 10.4g}, y: {: 10.4g}, theta: {: 10.4g} weight: {: 3g}", p->pose.x, p->pose.y, p->pose.theta, probab);
            LOG_INFO("New position: x: {: 10.4g}, y: {: 10.4g}, theta: {: 10.4g} weight: {: 3g}", p->pose.x, p->pose.y, p->pose.theta, probab);

        }
        else {
            //auto [transform, w] = pose_est.value();
            auto transforms = pose_est.value();

            std::vector<pose_t> chosen_trans = transforms;

            //Sample 100 indexes between 0 and number of transforms - 1
            std::vector<int> trans_idx;

            for (auto t = 0; t < 100; t++)
                trans_idx.push_back((int)(rand() % chosen_trans.size()));

            std::vector<pose_t> pose_samples;

            //Sample around mode
            for (auto& i : trans_idx)
                pose_samples.push_back(pose_t{ get_random(pose_guess.x + chosen_trans[i].x, sample_std_), get_random(pose_guess.y + chosen_trans[i].y, sample_std_), get_random(pose_guess.theta + chosen_trans[i].theta, sample_std_/ 10) });

            pose_t cov_col_1{ 0.0,0.0,0.0 }, cov_col_2{ 0.0,0.0,0.0 }, cov_col_3{ 0.0,0.0,0.0 }, mu {0.0,0.0,0.0};
            double norm = 0.0;
            std::vector<double> corr_samples;

            //Compute mean and correlation coefficient
            for (auto& s : pose_samples) {
                auto corr = correlation_model(*p, s, obstacles, is_object);
                mu = mu + (s * corr);
                norm += corr;
                corr_samples.push_back(corr);
            }
            //Normalise mean
            mu = mu / norm;

            Eigen::Vector3d mean;
            Eigen::Matrix3d covar;
            mean << mu.x, mu.y, mu.theta;

            //Compute covariance
            auto cntr = 0;
            for (auto& s : pose_samples) {
                cov_col_1 = cov_col_1 + (s - mu) * (s.x - mu.x) * corr_samples[cntr];
                cov_col_2 = cov_col_2 + (s - mu) * (s.y - mu.y) * corr_samples[cntr];
                cov_col_3 = cov_col_3 + (s - mu) * (s.theta - mu.theta) * corr_samples[cntr++];
            }
            
            cov_col_1 = cov_col_1 / norm;
            cov_col_2 = cov_col_2 / norm;
            cov_col_3 = cov_col_3 / norm;

            covar << cov_col_1.x, cov_col_2.x, cov_col_3.x, cov_col_1.y, cov_col_2.y, cov_col_3.y, cov_col_1.theta, cov_col_2.theta, cov_col_3.theta;

            //Create multivariance normal distribution
            Eigen::EigenMultivariateNormal<double> normX_cholesk(mean, covar, true);
            Eigen::MatrixXd new_pose = normX_cholesk.samples(1).transpose();

            p->pose.x = new_pose(0, 0);
            p->pose.y = new_pose(0, 1);
            p->pose.theta = new_pose(0, 2);
            

            probab *= norm;

            //Add local raster array in particle raster array

            for (auto j = 0; j < local_raster_array.size(); j += 2)
            {
                //Transform to global coordinates and add search transform
                auto transf_x_start = p->pose.x + local_raster_array[j].first.col * cos(p->pose.theta * M_PI / 180) - local_raster_array[j].first.row * sin(p->pose.theta * M_PI / 180);
                auto transf_y_start = p->pose.y + local_raster_array[j].first.col * sin(p->pose.theta * M_PI / 180) + local_raster_array[j].first.row * cos(p->pose.theta * M_PI / 180);
                auto transf_x_end = p->pose.x + local_raster_array[j + 1].first.col * cos(p->pose.theta * M_PI / 180) - local_raster_array[j + 1].first.row * sin(p->pose.theta * M_PI / 180);
                auto transf_y_end = p->pose.y + local_raster_array[j + 1].first.col * sin(p->pose.theta * M_PI / 180) + local_raster_array[j + 1].first.row * cos(p->pose.theta  * M_PI / 180);

                //Transform to row/col of grid
                auto obs_start_grid = coord_to_row_col(size_, separation_, transf_x_start, transf_y_start);
                auto obs_end_grid = coord_to_row_col(size_, separation_, transf_x_end, transf_y_end);

                if (obs_start_grid && obs_end_grid) {
                    p->raster_array.push_back(std::pair<cell_t, int>({ cell_t({obs_start_grid.value().first, obs_start_grid.value().second}), local_raster_array[j].second }));
                    p->raster_array.push_back(std::pair<cell_t, int>({ cell_t({obs_end_grid.value().first, obs_end_grid.value().second}), local_raster_array[j].second }));
                }
            }

            //std::cout << "New pose: " << p->pose.x << ", " << p->pose.y << ", " << p->pose.theta << ", Weight: " << probab << std::endl;
        }

        //update map
        integrate_scan(*p, obstacles, is_object);
        p->weight = probab;

        //auto t2 = boost::chrono::high_resolution_clock::now();
        //auto duration = boost::chrono::duration_cast<boost::chrono::microseconds>(t2 - t1).count();

        //std::cout << "Duration until particle " << p->id << " completed: " << duration << std::endl;
    }

    resample_particles();
    
}


std::optional<std::vector<pose_t>> particleFilter::scan_match(Particle& p, std::vector<message::position>& points, pose_t& pose_guess,
                                                                    std::vector<bool>& is_object, std::vector<std::pair<cell_t, int>>& local_raster_array)
{
    //Can't match scans if nothing to compare with.
    if (p.raster_array.empty())
        return std::nullopt;

    std::vector<std::pair<pose_t, int>> best_trans;

    std::vector<pose_t> chosen_trans;

    search_for_transform(p, points, pose_guess, is_object, size_, separation_, local_raster_array, best_trans);
    
    auto obstructed_points = 0;

    for (auto i = 0; i < is_object.size(); i++)
        if (is_object[i]) obstructed_points++;

    //std::cout << "Obstructed points: " << obstructed_points << std::endl;

    if (best_trans[0].second < obstructed_points / 4) {  //This threshold should be tuned
        return std::nullopt;
    }

    for (auto& trans : best_trans)
        chosen_trans.push_back(trans.first);

   
    return chosen_trans;
}



void particleFilter::integrate_scan(Particle& p, std::vector<message::position>& obstacles, std::vector<bool>& is_object) {
    
    auto count = 0;
    for (const auto& obstacle : obstacles)
    {

        auto obs_x = p.pose.x + obstacle.x * cos(p.pose.theta * M_PI / 180) - obstacle.y * sin(p.pose.theta * M_PI / 180);
        auto obs_y = p.pose.y + obstacle.x * sin(p.pose.theta * M_PI / 180) + obstacle.y * cos(p.pose.theta * M_PI / 180);

        /*  May be necessary for real robot.
        Depends on what point the robots sends when point is unobstructed */
        /*
        if (!is_object[count]) {
            obs_x = p.pose.x + 0.8*(obstacle.x * cos(p.pose.theta * M_PI / 180) - obstacle.y * sin(p.pose.theta * M_PI / 180));
            obs_y = p.pose.y + 0.8*(obstacle.x * sin(p.pose.theta * M_PI / 180) + obstacle.y * cos(p.pose.theta * M_PI / 180));
        }
        */

        auto pos_grid = coord_to_row_col(size_, separation_, p.pose.x, p.pose.y);
        auto obs_grid = coord_to_row_col(size_, separation_, obs_x , obs_y);

        //Unobstruct line between robot pos and observation
        if (pos_grid && obs_grid) {
            auto [pos_row, pos_col] = pos_grid.value();
            auto [obs_row, obs_col] = obs_grid.value();

            auto [size, line] = NTNU::application::SLAM::utility::get_line_between_pts(std::make_pair(pos_row, pos_col), std::make_pair(obs_row, obs_col));

            for (auto i = 0; i < size; i++) {
                p.map.unobstruct(line[i].first, line[i].second);
            }

            /* Growing objects */
            //if (is_object[count]) {
                //for (auto i = -5; i < 6 ; i++)
                //    for (auto j = -5; j < 6; j++) {
                //        if (i != 0 || j != 0)
                //            p.map.semi_obstruct(obs_row + i, obs_col + j);
                //        else
                //            p.map.obstruct(obs_row, obs_col);
                //    }
            //}
            //else if (!is_object[count])
            //    p.map.unobstruct(obs_row, obs_col);
            

            /* Visualise points from observations */
            if (is_object[count])
                p.map.obstruct(obs_row, obs_col);

            if (!is_object[count])
                p.map.unobstruct(obs_row, obs_col);

        }

        count++;

    }
    

    /* Visualise lines of raster array */
    /*
    for (auto i = 0; i < p.raster_array.size() ; i = i + 2)
    {
        auto [size, line] = NTNU::application::SLAM::utility::get_line_between_pts(std::make_pair(p.raster_array[i].first.row, p.raster_array[i].first.col), 
                                                                                        std::make_pair(p.raster_array[i+1].first.row, p.raster_array[i+1].first.col));

        for (auto i = 0; i < size; i++) {
            p.map.obstruct(line[i].first, line[i].second);
        }
    }
    */
}

void particleFilter::resample_particles()
{
    //Resample particles with weight as probability of being picked.
    std::vector<double> weights;
    auto best_weight = 0.0;

    // Create weight array
    // Simulateously find best weighted particle
    for (auto& p : particles_) {
        weights.push_back(p->weight);

        if (p->weight > best_weight) {
            best_weight = p->weight;
            best_ = p->id;
        }

    }

    auto sum_weights = std::accumulate(weights.begin(), weights.end(), 0.0);

    //Normalise weights
    for (auto& w : weights)
        w /= sum_weights;

    //Compute number of effective particles
    auto sumSqWeights = 0.0;

    for (auto& w : weights) 
        sumSqWeights += pow(w, 2);
    
    auto n_eff = 1.0 / sumSqWeights;

    //std::cout << "Neff: " << n_eff << std::endl;

    const auto threshold = numParticles_ / 2;

    //If number of effective particles is numParticles/2, resample particles.
    if (n_eff < threshold) {
        auto idx = systematic_resample(weights);
        
        std::vector<std::shared_ptr<Particle> > resampled_particles;
        std::shared_ptr<Particle> p (NULL);
        
        auto new_id = 0;

        for (auto& i : idx) {
            p = particles_[i];

            p->id = new_id++;
            p->weight = 1;

            resampled_particles.push_back(std::make_shared<Particle>(*p));

        }

        particles_.clear();

        for (auto& p : resampled_particles) {
            particles_.push_back(std::move(p));
        }

        resampled_particles.clear();
    }
}


std::vector<int16_t> particleFilter::systematic_resample(std::vector<double>& weights) {
    // make N subdivisions of equal size, with a random offset,
    //std::cout << "<------------------------Resampling------------------------>" << std::endl;
    LOG_INFO("{:-^100}", "Resampling");
    
    auto rand_offset = get_random(0.5, 0.11);

    std::vector<double> positions;

    for (auto n = 0; n < numParticles_; n++) {
        positions.push_back(((double)n + rand_offset)/static_cast<double>(numParticles_));
    }

    std::vector<double> cumsum ( weights.size() );
    std::partial_sum(weights.begin(), weights.end(), cumsum.begin());
    std::vector<int16_t> indexes;
    int16_t i = 0;
    int16_t j = 0;

    while (i < numParticles_) {
        if (positions[i] < cumsum[j]) {
            indexes.push_back(j);
            i++;
        }
        else
            j++;
    }
    
    return indexes;
}

pose_t particleFilter::getPose() {
    //return average pose of particles, i.e. estimated pose.
    pose_t sum_pose = { 0.0, 0.0, 0.0 };

    for (auto& p : particles_) {
        sum_pose.x += p->pose.x;
        sum_pose.y += p->pose.y;
        sum_pose.theta += p->pose.theta;
    }

    pose_.x = sum_pose.x / numParticles_;
    pose_.y = sum_pose.y / numParticles_;
    pose_.theta = sum_pose.theta / numParticles_;
    
    return pose_;
}

obstructable_grid particleFilter::getBestMap() {
    return particles_[best_]->map;
}

void particleFilter::cleanRaster() {
    //Merge lines which can be merged in all particles
    //Want to merge lines which are collinear and intersect
    auto total_particle_size = 0;
    
    for (auto& p : particles_) {
        for (auto i = 0; i < p->raster_array.size(); i += 2) {

            //Check if the line is only a single cell
            if (p->raster_array[i].first == p->raster_array[i + 1].first)
            {
                //Remove the line
                for (auto d = 0; d < 2; d++)
                    p->raster_array.erase(p->raster_array.begin() + i);

                //Start over with new line
                i -= 2;
                continue;
            }

            std::vector<cell_t> reference_check;
            reference_check.push_back(p->raster_array[i].first);
            reference_check.push_back(p->raster_array[i + 1].first);

            

            for (auto j = i + 2; j < p->raster_array.size(); j += 2) {

                reference_check.push_back(p->raster_array[j].first);
                reference_check.push_back(p->raster_array[j + 1].first);

                auto collinear = collinearityCheck(std::ref(reference_check));
                if (!collinear)
                    continue;
                
                /* Print lines about to be merged */
                //std::cout << "Merging lines " << reference_check[0].row << ", " << reference_check[0].col << " -> " << reference_check[1].row << ", " << reference_check[1].col << std::endl;
                //std::cout << "and " << p.raster_array[j].first.row << ", " << p.raster_array[j].first.col << " -> " << p.raster_array[j + 1].first.row << ", " << p.raster_array[j + 1].first.col << std::endl;
                
                //If lines are collinear and intersect, merge
                //and check if existing lines are collinear
                //and intersect with new line
                auto [new_start, new_end] = collinear.value();
                p->raster_array[i].first = new_start;
                p->raster_array[i + 1].first = new_end;

                //Add weight
                p->raster_array[i].second += p->raster_array[j].second;
                p->raster_array[i + 1].second += p->raster_array[i + 1].second;

                //Remove the old line
                for (auto d = 0; d < 2; d++)
                    p->raster_array.erase(p->raster_array.begin() + j);

                //Start over with new line
                i -= 2;
                break;
            }

        }

        //After 20 lines are detected, start clearing 
        //low weighted lines ( <= 10 ) from the first half of the vector
        if (p->raster_array.size() > 40) {
            for (auto i = 0; i < p->raster_array.size() / 2; i += 2) {
                if (p->raster_array[i].second <= 10) {
                    for (auto d = 0; d < 2; d++)
                        p->raster_array.erase(p->raster_array.begin() + i);
                    i -= 2;
                }
            }
        }
        
        
        total_particle_size += p->raster_array.size();
        /* Print individual array sizes */
        //std::cout << "Raster array size: " << p->raster_array.size() << std::endl;

        /* Print info on individual lines */
        //std::cout << "Current lines: " << std::endl;
        //for (auto i = 0; i < p.raster_array.size(); i = i + 2)
        //    std::cout << "Start: " << p.raster_array[i].first.row << ", " << p.raster_array[i].first.col << ", End: " << p.raster_array[i + 1].first.row << ", " << p.raster_array[i + 1].first.col << ", Weight: " << p.raster_array[i].second << std::endl;

    }
    //std::cout << "Avg raster array size: " << total_particle_size/100 << std::endl;
    LOG_INFO("Average raster array size: {}", total_particle_size / 100);
    
}

double particleFilter::get_random(double mean, double std)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());

    if (std < 0.001)
        std = 0.001;

    std::normal_distribution<double> dis(mean, std);

    return dis(gen);
}

}