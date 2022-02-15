#define _USE_MATH_DEFINES

#include "particle.h"
#include "slam_utility.h"
#include <boost/chrono.hpp>
#include <cmath>
#include <random>
#include <iostream>
#include <numeric>

using NTNU::graph::grid::obstructable_grid;

boost::mutex b_lock;  //Mutex to prevent race condition on best_raster_array and best_trans

namespace NTNU::application::SLAM
{

Particle::Particle(int id_p, pose_t pose_p, double weight_p, obstructable_grid map_p) :
    //Particle constructor
    id(id_p),
    pose(pose_p),
    weight(weight_p),
    map(map_p)
{
}

Particle::Particle(int id_p, pose_t pose_p, double weight_p) :
    //Particle constructor
    id(id_p),
    pose(pose_p),
    weight(weight_p)
{
}

    //"Settings" as a tuple with 5 arbitrary is not very clean. Revise
particleFilter::particleFilter(std::string id, pose_t init_pose, std::array<double,4> a, std::array<int,4> settings):
    robot_id_(id),
    pose_(init_pose),
    //a is an array containing factors to estimate noise in odometry readings
    a_(a),
	weight_(),
    size_({ settings[1], settings[0] }),  //rows, cols
    separation_(settings[2]),
	noiseObs_(),
	path_(),
    numParticles_(settings[3]),
    particles_(),
    scan_weight_punisher_(0.25)
{
    //Initialise particles
    for (auto i = 0; i < numParticles_; i++) {
        pose_t pose = { get_random(init_pose.x, 5) ,get_random(init_pose.y, 5) ,get_random(init_pose.theta, 5) };

        obstructable_grid map_p = obstructable_grid(size_[0], size_[1]);
        Particle p(i, pose, 1.0, map_p);

        particles_.push_back(p);
    }
}

void search_for_transform(Particle& p, std::vector<message::position> points, pose_t pose_guess, std::vector<bool> is_object, std::array<int,2> size, int separation, std::vector<std::pair<pose_t, int>> &best_trans, std::vector<std::pair<cell_t, int>> &best_raster_array, int i)// , // , boost::shared_ptr<std::pair<pose_t, int>>& best_trans_weight, boost::shared_ptr<std::vector<std::pair<cell_t, int>>>& best_raster_array, int i) {
{  
    //auto cntr = 0;
    for (auto tx = -100; tx < 100; tx = tx + 20) {
        for (auto ty = -100; ty < 100; ty = ty + 20) {
            std::vector<std::pair<cell_t, int>> local_raster_array;
            std::vector<message::position> collinear_points;
            for (auto k = 0; k < 4; k++)
            {
                for (auto j = k; j < points.size(); j = j + 4)
                {

                    if (is_object[j]) {
                        //Check if points are collinear
                        collinear_points.push_back(points[j]);

                        //Need 3 points to check colinearity
                        if (collinear_points.size() < 3)
                            continue;

                        if (collinearityCheck(collinear_points))
                            continue;

                        if (collinear_points.size() > 3) {
                            //Last collinear point is the second last point in the vector
                            auto last = collinear_points.size() - 2;

                            //Transform to global coordinates
                            auto transf_x_start = pose_guess.x + collinear_points[0].x * cos((-pose_guess.theta + i) * M_PI / 180) + collinear_points[0].y * sin((-pose_guess.theta + i) * M_PI / 180) + tx;
                            auto transf_y_start = pose_guess.y + -collinear_points[0].x * sin((-pose_guess.theta + i) * M_PI / 180) + collinear_points[0].y * cos((-pose_guess.theta + i) * M_PI / 180) + ty;
                            auto transf_x_end = pose_guess.x + collinear_points[last].x * cos((-pose_guess.theta + i) * M_PI / 180) + collinear_points[last].y * sin((-pose_guess.theta + i) * M_PI / 180) + tx;
                            auto transf_y_end = pose_guess.y + -collinear_points[last].x * sin((-pose_guess.theta + i) * M_PI / 180) + collinear_points[last].y * cos((-pose_guess.theta + i) * M_PI / 180) + ty;

                            auto obs_start_grid = NTNU::application::SLAM::utility::coord_to_row_col(size, separation, transf_x_start + pose_guess.x, transf_y_start + pose_guess.y);
                            auto obs_end_grid = NTNU::application::SLAM::utility::coord_to_row_col(size, separation, transf_x_end + pose_guess.x, transf_y_end + pose_guess.y);

                            auto found = false;
                            if (obs_start_grid && obs_end_grid)
                            {
                                auto [row_start, col_start] = obs_start_grid.value();
                                auto [row_end, col_end] = obs_end_grid.value();

                                auto start = cell_t{ row_start, col_start };
                                auto end = cell_t{ row_end, col_end };

                                std::vector<cell_t> reference_check;
                                reference_check.push_back(start);
                                reference_check.push_back(end);

                                //Check if new line exists or intersects with existing in raster array
                                if (!local_raster_array.empty())
                                {
                                    for (auto l = 0; l < local_raster_array.size(); l = l + 2)
                                    {
                                        reference_check.push_back(local_raster_array[l].first);
                                        reference_check.push_back(local_raster_array[l + 1].first);

                                        auto collinear = collinearityCheck(reference_check);
                                        if (!collinear)
                                            continue;

                                        //If lines are collinear and intersect, merge lines
                                        found = true;
                                        auto [new_start, new_end] = collinear.value();
                                        local_raster_array[l].first = new_start;
                                        local_raster_array[l + 1].first = new_end;
                                        local_raster_array[l].second++;
                                        local_raster_array[l + 1].second++;
                                        break;
                                    }
                                }
                                if (!found) {
                                    std::pair new_cell_start = { cell_t({row_start,col_start}), 1 };
                                    local_raster_array.push_back(new_cell_start);
                                    std::pair new_cell_end = { cell_t({row_end,col_end}), 1 };
                                    local_raster_array.push_back(new_cell_end);
                                }
                                else
                                    found = false;

                                /*
                                for (auto& r : p.raster_array) {
                                    if (r.first.row == row && r.first.col == col) {
                                        found = true;
                                        r.second++;
                                        break;
                                    }
                                }
                                if (!found) {
                                    std::pair new_cell = { cell_t({row,col}), 1 };
                                    p.raster_array.push_back(new_cell);
                                }
                                else
                                    found = false;
                                    */



                            }
                        }
                        else
                            collinear_points.erase(collinear_points.begin());
                    }
                    //cntr++;
                }
            }

            /*
            for (auto& obs : points) {
                if (is_object[cntr]) {
                    //First rotate points:
                    //auto transf_x = (obs.x + pose_guess.x) * cos((pose_guess.theta + i) * M_PI / 180) + (obs.y + pose_guess.y) * sin((pose_guess.theta + i) * M_PI / 180) + tx;
                    //auto transf_y = -(obs.x + pose_guess.x) * sin((pose_guess.theta + i) * M_PI / 180) + (obs.y + pose_guess.y) * cos((pose_guess.theta + i) * M_PI / 180) + ty;
                    
                    auto transf_x = pose_guess.x + obs.x * cos((-pose_guess.theta + i) * M_PI / 180) + obs.y * sin((-pose_guess.theta + i) * M_PI / 180) + tx;
                    auto transf_y = pose_guess.y + -obs.x * sin((-pose_guess.theta + i) * M_PI / 180) + obs.y * cos((-pose_guess.theta + i) * M_PI / 180) + ty;

                    //std::cout << "Transformed: " << transf_x << ", " << transf_y << std::endl;

                    //Create sparse array for local points
                    auto obs_grid = NTNU::application::SLAM::utility::coord_to_row_col(size, separation, transf_x, transf_y);
                    
                    if (obs_grid)
                    {
                        auto found = false;
                        auto [row, col] = obs_grid.value();
                        for (auto& l : local_raster_array) {
                            if (l.first.row == row && l.first.col == col) {
                                found = true;
                                l.second++;
                                break;
                            }
                        }
                        if (!found) {
                            std::pair new_cell = { cell_t({row,col}), 1 };
                            local_raster_array.push_back(new_cell);
                        }
                    }
                    
                }
                cntr++;
                
            }
            */
            //cntr = 0;
            auto w = 0.0;

            

            for (auto k = 0; k < local_raster_array.size(); k = k + 2)
            {
                std::vector<cell_t> reference_check;
                reference_check.push_back(local_raster_array[k].first);
                reference_check.push_back(local_raster_array[k+1].first);

                for (auto j = 0; j < p.raster_array.size(); j = j + 2)
                {
                    reference_check.push_back(p.raster_array[j].first);
                    reference_check.push_back(p.raster_array[j + 1].first);
                    auto collinear = collinearityCheck(reference_check);
                    if (!collinear)
                        continue;

                    //If lines are collinear and intersect, increase weight
                    w = w + sqrt(local_raster_array[k].second * p.raster_array[j].second); //Could also be mean weight
                    break;
                }
            }
            /*
            //auto t1 = boost::chrono::high_resolution_clock::now();
            for (auto& l : local_raster_array) {
                for (auto & r : p.raster_array) {
                    //std::cout << "Rows: " << r.first.row << ", " << l.first.row << ", Cols: " << r.first.col << ", " << l.first.row << ", " << std::endl;
                    //std::cout << "Weights: " << r.second << ", " << l.second << std::endl;
                    //Compare sparse arrays of local and global points
                    if (r.first == l.first) {
                        w = w + sqrt(r.second * l.second); //Could also be mean weight
                        //std::cout << "Found common point" << std::endl;
                        break;
                    }
                }
            }
            */
            //auto t2 = boost::chrono::high_resolution_clock::now();

            //auto duration = boost::chrono::duration_cast<boost::chrono::microseconds>(t2 - t1).count();

            //std::cout << "Duration of thread " << i << " for loop: " << duration << std::endl;
            

            //Store the overall best weighted transform

            //Mutex to prevent race condition since many threads are running this concurrently
            b_lock.lock();

            if (best_trans.empty() || best_trans[0].second < w) {
                auto new_best_array = local_raster_array;
                best_raster_array = new_best_array;
                best_trans.clear();
                best_trans.push_back(std::make_pair(pose_t({ (double)tx, (double)ty, (double)i }), (int)w));
            }
            else if (best_trans[0].second == w) {
                //std::cout << "Chosen best: " << best_trans[0].first.x << ", " << best_trans[0].first.y << ", " << best_trans[0].first.theta << std::endl;
                //std::cout << "Equal to best: " << tx << ", " << ty << ", " << i << ", Weights: " << w << std::endl;
                best_trans.push_back(std::make_pair(pose_t({ (double)tx, (double)ty, (double)i }), (int)w));
            }

            //else if (w == trans_weight_array[0].second)             //Use if equal weights turn out to be a problem. Need to return vector in that case
            //    trans_weight_array.push_back(std::make_pair(pose_t({ tx,ty,i }), w));

            //Store all weights of transforms for evaluation. Maybe use if PDF is necessary
            //trans_weight_array.push_back(std::make_pair(pose_t({ tx,ty,i }), w ));
            b_lock.unlock();
        }
    }
    
    

    return;

}

void particleFilter::estimate_pose(Particle& p , pose_t odom)
{
    //calculate the movements between previous and present instances
    auto rot_1 = atan2(odom.y, odom.x) - p.pose.theta * M_PI / 180;
    auto trans = sqrt(pow(odom.x,2) + pow(odom.y,2));
    auto rot_2 = odom.theta * M_PI / 180 - rot_1;

    //calculate correct noise stds according to the motion model
    auto rot_1_std = a_[0] * abs(rot_1) + a_[1] * trans;
    auto trans_std = a_[2] * trans + a_[3] * (abs(rot_1) + abs(rot_2));
    auto rot_2_std = a_[0] * abs(rot_2) + a_[1] * trans;

    //add the gaussian distributed noise
    auto rot_1_sampled = get_random(rot_1, rot_1_std);
    auto trans_sampled = get_random(trans, trans_std);
    auto rot_2_sampled = get_random(rot_2, rot_2_std);

    //update the pose
    p.pose.x = trans_sampled * cos(p.pose.theta * M_PI / 180.0 + rot_1_sampled) + p.pose.x;
    p.pose.y = trans_sampled * sin(p.pose.theta * M_PI / 180.0 + rot_1_sampled) + p.pose.y;
    p.pose.theta = rot_1_sampled * 180.0 / M_PI + rot_2_sampled * 180.0 / M_PI + p.pose.theta;

    //path_.append(pose) Probably need this later

    //std::cout << "Given estimated theta: " << p.pose.theta << std::endl;
}

double particleFilter::correlation_model(Particle& p, std::vector<message::position> points, std::vector<bool> is_object) {

    auto w = 0.0;
    auto count = 0;
    for (auto& obs : points) {

        auto obs_x = p.pose.x + obs.x * cos(-p.pose.theta * M_PI / 180) + obs.y * sin(-p.pose.theta * M_PI / 180);
        auto obs_y = p.pose.y - obs.x * sin(-p.pose.theta * M_PI / 180) + obs.y * cos(-p.pose.theta * M_PI / 180);

        auto pos_grid = NTNU::application::SLAM::utility::coord_to_row_col(size_, separation_, p.pose.x, p.pose.y);
        auto obs_grid = NTNU::application::SLAM::utility::coord_to_row_col(size_, separation_, obs_x, obs_y);

        //Unobstruct line between robot pos and observation
        
        if (pos_grid && obs_grid) {
            auto [pos_row, pos_col] = pos_grid.value();
            auto [obs_row, obs_col] = obs_grid.value();

            auto [size, line] = NTNU::application::SLAM::utility::get_line_between_pts(std::make_pair(pos_row, pos_col), std::make_pair(obs_row, obs_col));

            for (auto i = 0; i < size; i++) {
                if (p.map.is_unobstructed(line[i].first, line[i].second))
                    w = w + 1/size;
                else if (p.map.is_obstructed(line[i].first, line[i].second))
                    w--;
            }

        }
        
        auto found = false;

        if (obs_grid)
        {
            auto [row, col] = obs_grid.value();
            //If observation in point
            if (is_object[count])
            {
                //Increase or add raster cell weight
                for (auto& r : p.raster_array) {
                    if (r.first.row == row && r.first.col == col) {
                        found = true;
                        r.second++;
                        //std::cout << "Increase cell weight to: " << r.second << std::endl;
                        break;
                    }
                }
                if (!found) {
                    std::pair new_cell = { cell_t({row,col}), 1 };
                    p.raster_array.push_back(new_cell);
                }


                if (p.map.is_obstructed(row, col))
                    w++;
                else if (p.map.is_unobstructed(row, col))
                    w--;
            }
            //If no observation in point
            else if (!is_object[count]) {
                if (p.map.is_obstructed(row, col))
                    w--;
                else if (p.map.is_unobstructed(row, col) && !is_object[count])
                    w++;
            }
        }
        count++;

    }

    //std::cout << "w: " << w << std::endl;

    auto correlation_score = exp(w / count); //w * convergence, convergence = ?
    return correlation_score;


    /*
    if (is_object[cntr]) {
                auto obs_grid = NTNU::application::SLAM::utility::coord_to_row_col(size_, separation_, obs.x + pose_guess.x, obs.y + pose_guess.y);
                auto found = false;
                if (obs_grid)
                {
                    auto [row, col] = obs_grid.value();
                    for (auto& r : p->raster_array) {
                        if (r.first.row == row && r.first.col == col) {
                            found = true;
                            r.second++;
                            break;
                        }
                    }
                    if (!found) {
                        std::pair new_cell = { cell_t({row,col}), 1 };
                        p->raster_array.push_back(new_cell);
                    }
                    else
                        found = false;
                }
            }
            cntr++;*/
}

//Input a single particle, or update all inside this function?
void particleFilter::update_particle(pose_t odom, std::vector<message::position> obstacles, std::vector<bool> is_object)
{

    for (auto& p : particles_) {
        auto probab = p.weight;

        if (odom >= pose_t({ 100, 100, 100 })) {

            //scan-matching

            //std::cout << "Init: " << p.pose.x << ", " << p.pose.y << ", " << p.pose.theta << std::endl;
            //std::cout << "Odom: " << odom.x << ", " << odom.y << ", " << odom.theta << std::endl;

            pose_t pose_guess;
            pose_guess.x = p.pose.x + odom.x; // * cos/sin(DEG2RAD(odom.theta)) if not in global coordinates?
            pose_guess.y = p.pose.y + odom.y;
            pose_guess.theta = p.pose.theta + odom.theta;

            //std::cout << "Guess: " << pose_guess.x << ", " << pose_guess.y << ", " << pose_guess.theta << std::endl;

            //auto obstructed_points = 0;

            //for (auto i = 0; i < is_object.size(); i++)
            //    if (is_object[i]) obstructed_points++;

            //Challenge the scan matcher more to see if it actually works
            auto pose_est = scan_match(p, obstacles, pose_guess, is_object);

            if (!pose_est) {
                std::cout << "Scan match failed." << std::endl;
                estimate_pose(p, odom);
                probab *= correlation_model(p, obstacles, is_object);  //based on prediction.
                std::cout << "New pose: " << p.pose.x << ", " << p.pose.y << ", " << p.pose.theta << ", Weight: " << probab << std::endl;
            }
            else {
                auto [transform, w] = pose_est.value();

                p.pose.x = get_random(pose_guess.x + ( transform.x * cos(-transform.theta * M_PI / 180) + transform.y * sin(-transform.theta * M_PI / 180)), obstacles.size() / w); //the nominator of std must be tuned
                p.pose.y = get_random(pose_guess.y + ( -transform.x * sin(-transform.theta * M_PI / 180) + transform.y * cos(-transform.theta * M_PI / 180)), obstacles.size() / w); //the nominator of std must be tuned
                p.pose.theta = get_random(pose_guess.theta - transform.theta, obstacles.size() / w); //the nominator of std must be tuned

                
                //std::cout << "Given scan_matched theta: " << p.pose.theta << std::endl;

                probab = probab * w / (obstacles.size() * scan_weight_punisher_) ;  //The denominator should be tuned

                //std::cout << "Weight: " << probab << std::endl;
                std::cout << "New pose: " << p.pose.x << ", " << p.pose.y << ", " << p.pose.theta << ", Weight: " << probab << std::endl;
            }
        }
        else if (p.raster_array.empty())
            scan_match(p, obstacles, p.pose, is_object);

        //update map
        integrate_scan(p, obstacles, is_object);
        p.weight = probab;
    }

    resample_particles();

}


std::optional<std::pair<pose_t, int>> particleFilter::scan_match(Particle& p, std::vector<message::position> points, pose_t pose_guess, std::vector<bool> is_object) {
    //Novel point-to-point scan matching algorithm
    //auto cntr = 0;
    std::vector<message::position> collinear_points;
    //Can't match scans if nothing to compare with.
    //Add points automatically.
    if (p.raster_array.empty()) {
        //All points from one sensor must be processed before the next sensor
        for (auto k = 0; k < 4; k++) 
        {
            for (auto j = k; j < points.size() ; j = j + 4)
            {
                //for (auto& obs : points) {
                if (is_object[j]) {   //Commented method: cntr
                    //Check if points are collinear
                    collinear_points.push_back(points[j]);

                    //Need 3 points to check colinearity
                    if (collinear_points.size() < 3)
                        continue;

                    if (collinearityCheck(collinear_points))
                        continue;

                    if (collinear_points.size() > 3) {
                        //Last collinear point is the second last point in the vector
                        auto last = collinear_points.size() - 2;
                        auto obs_start_grid = NTNU::application::SLAM::utility::coord_to_row_col(size_, separation_, collinear_points[0].x + pose_guess.x, collinear_points[0].y + pose_guess.y);
                        auto obs_end_grid = NTNU::application::SLAM::utility::coord_to_row_col(size_, separation_, collinear_points[last].x + pose_guess.x, collinear_points[last].x + pose_guess.y);

                        if (obs_start_grid && obs_end_grid)
                        {
                            auto [row_start, col_start] = obs_start_grid.value();
                            auto [row_end, col_end] = obs_end_grid.value();

                            auto start = cell_t{ row_start, col_start };
                            auto end = cell_t{ row_end, col_end };

                            std::vector<cell_t> reference_check;
                            reference_check.push_back(start);
                            reference_check.push_back(end);

                            //Check if new line exists or intersects with existing in raster array
                            auto found = false;
                            if (!p.raster_array.empty())
                            {
                                for (auto i = 0; i < p.raster_array.size(); i = i + 2)
                                {
                                    reference_check.push_back(p.raster_array[i].first);
                                    reference_check.push_back(p.raster_array[i + 1].first);

                                    auto collinear = collinearityCheck(reference_check);
                                    if (!collinear)
                                        continue;

                                    //If lines are collinear and intersect
                                    found = true;
                                    auto [new_start, new_end] = collinear.value();
                                    p.raster_array[i].first = new_start;
                                    p.raster_array[i + 1].first = new_end;
                                    p.raster_array[i].second++;
                                    p.raster_array[i + 1].second++;
                                    break;
                                }
                            }
                            if (!found) {
                                std::pair new_cell_start = { cell_t({row_start,col_start}), 1 };
                                p.raster_array.push_back(new_cell_start);
                                std::pair new_cell_end = { cell_t({row_end,col_end}), 1 };
                                p.raster_array.push_back(new_cell_end);
                            }

                            /*
                            for (auto& r : p.raster_array) {
                                if (r.first.row == row && r.first.col == col) {
                                    found = true;
                                    r.second++;
                                    break;
                                }
                            }
                            if (!found) {
                                std::pair new_cell = { cell_t({row,col}), 1 };
                                p.raster_array.push_back(new_cell);
                            }
                            else
                                found = false;
                                */



                        }
                    }
                    else
                        collinear_points.erase(collinear_points.begin());
                }
            //cntr++;
            }
        }
        
        return std::nullopt;
    }

    std::vector<std::pair<pose_t, int>> best_trans;

    std::pair<pose_t, int> chosen_trans;
    chosen_trans.second = 0;
    std::vector<std::pair<cell_t, int>> best_raster_array;

    //Transform of points need to happen here, so that the maps and raster weights are correct.
    //Finds the best translation for each rotation angle.

    boost::thread_group search_threads;

    for (auto i = -10; i < 10; i = i + 2) {
        //Create separate threads to compute best translation for each rotation angle concurrently
        //auto t1 = boost::chrono::high_resolution_clock::now();
        search_threads.add_thread(new boost::thread(search_for_transform, p, points, pose_guess, is_object, size_, separation_, boost::ref(best_trans), boost::ref(best_raster_array), i));
        //auto t2 = boost::chrono::high_resolution_clock::now();
        //search_threads.create_thread(boost::bind(search_for_transform, p, points, pose_guess, is_object, size_, separation_, boost::ref(best_trans), boost::ref(best_raster_array), i));
        //auto duration = boost::chrono::duration_cast<boost::chrono::microseconds>(t2 - t1).count();

        //std::cout << "Duration of thread " << i << " creation: " << duration << std::endl;
    }

    //Wait until all computations are finished
    search_threads.join_all();

    //std::cout << "Best transform: " << best_trans.first.x << ", " << best_trans.first.y << ", " << best_trans.first.theta << ", Weight: " << best_trans.second << std::endl;

    //If more transforms have the same best weight, choose random
    if (best_trans.size() > 1) {
        std::vector<std::pair<pose_t, int>> temp_chosen;
        std::sample(
            best_trans.begin(),
            best_trans.end(),
            std::back_inserter(temp_chosen),
            1,
            std::mt19937{ std::random_device{}() }
        );
        chosen_trans = temp_chosen[0];
        std::cout << "Multiple bests" << std::endl;
    }
    else {
        chosen_trans = best_trans[0];
        std::cout << "Single best" << std::endl;
    }

    std::cout << "Chosen transform: " << chosen_trans.first.x << ", " << chosen_trans.first.y << ", " << chosen_trans.first.theta << ", Weight: " << chosen_trans.second << std::endl;
    std::cout << "Raster array size: " << p.raster_array.size() << std::endl;
    //Only return match if it has sufficiently high score. Must be tuned.
    if (chosen_trans.second < points.size() / 2) {
        return std::nullopt;
    }

    //Merge local raster array with particle raster array
    for (auto j = 0; j < best_raster_array.size(); j = j + 2)
    {
        std::vector<cell_t> reference_check;
        reference_check.push_back(best_raster_array[j].first);
        reference_check.push_back(best_raster_array[j + 1].first);

        //Check if new line exists or intersects with existing in raster array
        auto found = false;
        for (auto i = 0; i < p.raster_array.size(); i = i + 2)
        {
        
            reference_check.push_back(p.raster_array[i].first);
            reference_check.push_back(p.raster_array[i + 1].first);

            auto collinear = collinearityCheck(reference_check);
            if (!collinear)
                continue;

            //If lines are collinear and intersect
            found = true;
            auto [new_start, new_end] = collinear.value();
            p.raster_array[i].first = new_start;
            p.raster_array[i + 1].first = new_end;
            p.raster_array[i].second += best_raster_array[j].second;
            p.raster_array[i + 1].second += best_raster_array[j + 1].second;  //May not be necessary
            break;
        }
        
        if (!found) {
            //std::pair new_cell_start = { cell_t({best_raster_array[j].first.,col_start}), 1 };
            p.raster_array.push_back(best_raster_array[j]);
            //std::pair new_cell_end = { cell_t({row_end,col_end}), 1 };
            p.raster_array.push_back(best_raster_array[j+1]);
        }
    }

    /*for (auto& l : best_raster_array) {
        auto found = false;
        for (auto& r :p.raster_array) {
            //Compare sparse arrays of local and global points
            //Add if not in raster_array
            if (r.first == l.first) {
                found = true;
                r.second += l.second;
                break;
            }
            
        }
        if (!found) {
            p.raster_array.push_back(l);
        }
    }
    */
    //for (auto& q : p->raster_array)
    //    std::cout << "Row: " << q.first.row << ", Col: " << q.first.col << ", Weight: " << q.second << std::endl;


    return chosen_trans;

    //std::cout << "w: " << w << std::endl;
}



void particleFilter::integrate_scan(Particle& p, std::vector<message::position> obstacles, std::vector<bool> is_object) {
    auto count = 0;
    for (const auto& obstacle : obstacles)
    {

        auto obs_x = p.pose.x + obstacle.x * cos(-p.pose.theta * M_PI / 180) + obstacle.y * sin(-p.pose.theta * M_PI / 180);
        auto obs_y = p.pose.y - obstacle.x * sin(-p.pose.theta * M_PI / 180) + obstacle.y * cos(-p.pose.theta * M_PI / 180);

        auto pos_grid = NTNU::application::SLAM::utility::coord_to_row_col(size_, separation_, p.pose.x, p.pose.y);
        auto obs_grid = NTNU::application::SLAM::utility::coord_to_row_col(size_, separation_, obs_x , obs_y);

        //Unobstruct line between robot pos and observation
        if (pos_grid && obs_grid) {
            auto [pos_row, pos_col] = pos_grid.value();
            auto [obs_row, obs_col] = obs_grid.value();

            auto [size, line] = NTNU::application::SLAM::utility::get_line_between_pts(std::make_pair(pos_row, pos_col), std::make_pair(obs_row, obs_col));

            for (auto i = 0; i < size; i++) {
                p.map.unobstruct(line[i].first, line[i].second);
            }

        }

        if (obs_grid)
        {
            auto [row, col] = obs_grid.value();
            if (is_object[count]) {
                p.map.obstruct(row, col);
            }
            else if (!is_object[count])
                p.map.unobstruct(row, col);
        }
        count++;

    }
}

void particleFilter::resample_particles()
{
    //Resample particles with weight as probability of being picked.
    std::vector<double> weights;

    for (auto &p : particles_)
        weights.push_back(p.weight);

    auto sum_weights = std::accumulate(weights.begin(), weights.end(), 0.0);

    //Normalise weights
    for (auto& w : weights)
        w = w / sum_weights;

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

        std::vector<Particle> resampled_particles;
        auto new_id = 0;
        for (auto& i : idx) {
            particles_[i].weight = 1;
            particles_[i].id = new_id++;
            resampled_particles.push_back(particles_[i]);
        }
        particles_.clear();
        particles_.insert(particles_.begin(), resampled_particles.begin(), resampled_particles.end());
    }
}


std::vector<int> particleFilter::systematic_resample(std::vector<double> weights) {
    // make N subdivisions of equal size, with a random offset,
    std::cout << "<------------------------Resampling------------------------>" << std::endl;
    auto rand_offset = get_random(0.5, 0.12);

    std::vector<double> positions;

    for (auto n = 0.0; n < numParticles_; n++) {
        positions.push_back((n + rand_offset)/static_cast<double>(numParticles_));
    }

    std::vector<double> cumsum ( weights.size() );
    std::partial_sum(weights.begin(), weights.end(), cumsum.begin());
    std::vector<int> indexes;
    auto i = 0;
    auto j = 0;

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
        sum_pose.x += p.pose.x;
        sum_pose.y += p.pose.y;
        sum_pose.theta += p.pose.theta;
    }

    pose_.x = sum_pose.x / numParticles_;
    pose_.y = sum_pose.y / numParticles_;
    pose_.theta = sum_pose.theta / numParticles_;
    
    return pose_;
}

obstructable_grid particleFilter::getBestMap() {
    auto best_weight = 0.0;
    auto best_idx = 0;
    
    for (auto& p : particles_) {
        if (p.weight > best_weight) {
            best_idx = p.id;
            best_weight = p.weight;
        }
    }
    //std::cout << "Best: " << best_idx << std::endl;
    return particles_[best_idx].map;
}

void particleFilter::CleanUpRaster() {
    
}

//Check only last 3 points
bool collinearityCheck(std::vector<message::position> points) {
    auto tolerance = 20.0;
    auto last = points.size()-1;

    return (std::abs((points[last-2].y - points[last-1].y) * (points[last-2].x - points[last].x) - (points[last-2].y - points[last].y) * (points[last-2].x - points[last-1].x)) <= tolerance);
}

//Check first line and last line
std::optional<std::array<cell_t, 2>> collinearityCheck(std::vector<cell_t> cells) {
    auto tolerance = 2.0;
    auto last = cells.size()-1;

    //Compute vector of direction
    cell_t r = { cells[1].col - cells[0].col, cells[1].row - cells[0].row };
    cell_t s = { cells[last].col - cells[last-1].col, cells[last].row - cells[last-1].row };

    cell_t diff = { cells[last - 1].col - cells[0].col, cells[last - 1].row - cells[0].row };



    if ((r.col * s.row - r.row * s.col) <= tolerance && (diff.col * r.row - diff.row * r.col) <= tolerance)
    {
        //If collinearity, check if they intersect
        auto t0 = (diff.col * r.col + diff.row * r.row) / (r.col * r.col + r.row * r.row);
        auto t1 = t0 + (s.col * r.col + s.row * r.row) / (r.col * r.col + r.row * r.row);

        if (s.col * r.col + s.row * r.row < 0)
        {
            if (t0 < 0 || t1 > 1)
                return std::nullopt;
            else if (t0 < 1 && t1 > 0)
                return std::array<cell_t, 2> {cells[0], cells[1]};
            else if (t0 > 1 && t1 < 0)
                return std::array<cell_t, 2> {cells[last - 1], cells[last]};
            else if (t1 < 0 && t0 < 1)
                return std::array<cell_t, 2> {cells[last - 1], cells[1]};
            else if (t1 > 0 && t0 > 1)
                return std::array<cell_t, 2> {cells[last - 1], cells[1]};
        }
        else {
            if (t1 < 0 || t0 > 1)
                return std::nullopt;
            else if (t1 < 1 && t0 > 0)
                return std::array<cell_t, 2> {cells[0], cells[1]};
            else if (t1 > 1 && t0 < 0)
                return std::array<cell_t, 2> {cells[last-1], cells[last]};
            else if (t0 < 0 && t1 < 1)
                return std::array<cell_t, 2> {cells[last-1], cells[1]};
            else if (t0 > 0 && t1 > 1)
                return std::array<cell_t, 2> {cells[last-1], cells[1]};
        }
    }
    else
        return std::nullopt;
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