#include "Planners/MPPIPlanner.hpp"
#include <SceneAndSimulation/Robot.hpp>
#include <iomanip>

namespace Antipatrea {
    bool MPPIPlanner::Solve(const int nrIters, double dt, bool &canBeSolved) {
        geometry_msgs::Twist cmd_vel;

        robot = GetSimulator()->GetRobot();

        parent = {0, 0, 0, robot->getPoseState().velocity_, robot->getPoseState().angular_velocity_, true};
        parent_odom = robot->getPoseState();

        std::pair<std::vector<PoseState>, bool> best_traj;
        best_traj.first.reserve(nr_steps_);

        commonParameters(*robot);

        switch (robot->getRobotState()) {
            case Robot_config::NO_MAP_PLANNING:
                return handleNoMapPlanning(cmd_vel);

            case Robot_config::NORMAL_PLANNING:
                return handleNormalSpeedPlanning(cmd_vel, best_traj, dt);

            case Robot_config::LOW_SPEED_PLANNING:
                return handleLowSpeedPlanning(cmd_vel, best_traj, dt);

            default:
                return handleAbnormalPlaning(cmd_vel, best_traj, dt);
        }
    }

    bool MPPIPlanner::handleNoMapPlanning(geometry_msgs::Twist &cmd_vel) {
        if (robot->setting(Robot_config::NO_ANY_RECEIVED, 2) == false)
            return false;

        normalParameters(*robot);

        const double angle_to_goal = calculateTheta(parent, &robot->getGlobalGoalCfg()[0]);

        double angular = std::clamp(angle_to_goal, -1.0, 1.0);
        angular = (angular > 0) ? std::max(angular, 0.1) : std::min(angular, -0.1);

        publishCommand(cmd_vel, GetSimulator()->GetMaxVelocity(), angular);
        return true;
    }

    bool MPPIPlanner::handleNormalSpeedPlanning(geometry_msgs::Twist &cmd_vel,
                                                   std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {

        if (robot->setting(Robot_config::ONLY_LASER_RECEIVED, 2) == false)
            return false;

        normalParameters(*robot);

        auto result = mppi_planning(parent, parent_odom, best_traj, dt);

        robot->viewTrajectories(best_traj.first, nr_steps_, 0.0, timeInterval);

        // if (result == false) {
        //     robot->setRobotState(Robot_config::BRAKE_PLANNING);
        //     publishCommand(cmd_vel, robot->getPoseState().velocity_, robot->getPoseState().angular_velocity_);
        // } else {
        //     publishCommand(cmd_vel, best_traj.first.front().velocity_, best_traj.first.front().angular_velocity_);
        // }

        publishCommand(cmd_vel, best_traj.first.front().velocity_, best_traj.first.front().angular_velocity_);

        return true;
    }

    bool MPPIPlanner::handleLowSpeedPlanning(geometry_msgs::Twist &cmd_vel,
                                                std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {

        if (!robot->setting(Robot_config::ONLY_LASER_RECEIVED, 1))
            return false;

        lowSpeedParameters(*robot);

        auto result = mppi_planning(parent, parent_odom, best_traj, dt);

        robot->viewTrajectories(best_traj.first, nr_steps_, 0.0, timeInterval);

        if (!result) {
            robot->setRobotState(Robot_config::BRAKE_PLANNING);
            publishCommand(cmd_vel, 0, 0);
        } else
            publishCommand(cmd_vel, best_traj.first.front().velocity_, best_traj.first.front().angular_velocity_);

        return true;
    }

    bool MPPIPlanner::handleAbnormalPlaning(geometry_msgs::Twist &cmd_vel,
                                               std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {

        if (robot->getRobotState() == Robot_config::BRAKE_PLANNING) {
            double vel = robot->getPoseState().velocity_;
            if (vel > 0.01)
                publishCommand(cmd_vel, -0.1, 0.0);
            else {
                publishCommand(cmd_vel, 0.0, 0.0);
                robot->setRobotState(Robot_config::RECOVERY);
            }

            return true;
        }

        if (robot->getRobotState() == Robot_config::ROTATE_PLANNING) {
            if (robot->setting(Robot_config::ONLY_LASER_RECEIVED, 2) == false)
                return false;

            double angle = normalizeAngle(robot->rotating_angle - robot->getPoseState().theta_);

            if (fabs(angle) <= 0.10) {
                robot->setRobotState(Robot_config::NORMAL_PLANNING);
                return true;
            }

            double z = angle > 0 ? std::min(angle, 1.0) : std::max(angle, -1.0);
            z = z > 0 ? std::max(z, 0.5) : std::min(z, -0.5);
            publishCommand(cmd_vel, 0.0, z);
            return true;
        }

        if (robot->getRobotState() == Robot_config::RECOVERY) {
            bool results;

            if (robot->front_obs <= 0.10) {
                robot->setRobotState(Robot_config::BACKWARD);
                return true;
            }

            if (robot->setting(Robot_config::ONLY_COSTMAP_RECEIVED, 2) == false)
                return false;

            recoverParameters(*robot);

            auto best_theta = recover(parent, parent_odom, best_traj, results);

            if (results == false)
                return false;

            robot->rotating_angle = normalizeAngle(robot->getPoseState().theta_ + best_theta);

            robot->viewTrajectories(best_traj.first, nr_steps_, best_theta, timeInterval);
            robot->setRobotState(Robot_config::ROTATE_PLANNING);
        }

        if (robot->getRobotState() == Robot_config::BACKWARD) {
            if (robot->setting(Robot_config::ONLY_LASER_RECEIVED, 2) == false)
                return false;

            frontBackParameters(*robot);

            if (robot->front_obs >= 0.10) {
                robot->setRobotState(Robot_config::RECOVERY);
                return true;
            }

            publishCommand(cmd_vel, -0.3, 0);
        }

        return true;
    }


    void MPPIPlanner::publishCommand(geometry_msgs::Twist &cmd_vel, double linear, double angular) {
        cmd_vel.linear.x = linear;
        cmd_vel.angular.z = angular;
        robot->Control().publish(cmd_vel);
    }

    double MPPIPlanner::recover(
        PoseState &state, PoseState &state_odom,
        std::pair<std::vector<PoseState>, bool> &best_traj, bool &results) {
        const double angularVelocity_resolution =
                std::max(2 * M_PI / (w_steps_ - 1),
                         DBL_EPSILON);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        std::vector<Cost> costs;
        std::vector<double> theta_set;

        double _ = -2;
        std::vector<double> tmp_;

        std::vector<std::pair<std::vector<PoseState>, bool> > trajectories;
        int available_traj_count = 0;

        for (int i = 0; i < w_steps_; ++i) {
            std::pair<std::vector<PoseState>, bool> traj;
            traj.first.reserve(nr_steps_);

            const double w = -M_PI + angularVelocity_resolution * i;
            state_.theta_ = normalizeAngle(state.theta_ + w);
            state_odom_.theta_ = normalizeAngle(state_odom.theta_ + w);

            theta_set.push_back(state_.theta_);

            auto result = mppi_planning(state_, state_odom_, traj, dt);

            if (result == false)
                continue;

            robot->viewTrajectories(traj.first, nr_steps_, state_.theta_, timeInterval);

            const Cost cost = evaluate_trajectory(traj.first, _, tmp_);
            costs.push_back(cost);
            trajectories.push_back(traj);

            if (cost.obs_cost_ != 1e6 && cost.path_cost_ != 1e6)
                available_traj_count++;
        }

        double best_theta = 0.0;

        Cost min_cost(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e6);

        if (available_traj_count == 0) {
            ROS_ERROR_THROTTLE(1.0, "When a collision occurs, the robot cannot find any path during rotation");
            //best_traj.first = generateTrajectory(state, state_odom, 0.0, 0.0).first;
            results = false;
            return best_theta;
        }

        //Logger::m_out << "available trajectory " << available_traj_count << std::endl;
        normalize_costs(costs);

        for (int i = 0; i < costs.size(); ++i) {
            if (costs[i].total_cost_ < min_cost.total_cost_) {
                min_cost = costs[i];
                best_traj.first = trajectories[i].first;
            }
        }

        results = true;

        return best_theta;
    }

    bool MPPIPlanner::mppi_planning(PoseState &state, PoseState &state_odom,
                                       std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {
        Timer::Clock d_t;
        Timer::Start(d_t);

        Cost min_cost(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e6);

        Window dw = calc_dynamic_window(state, dt);

        std::vector<std::pair<double, double> > pairs;

        for (int i = 0; i < nr_pairs_; ++i) {
            double linear_velocity = RandomUniformReal(dw.min_velocity_, dw.max_velocity_);
            double angular_velocity = RandomUniformReal(dw.min_angular_velocity_, dw.max_angular_velocity_);
            pairs.emplace_back(linear_velocity, angular_velocity);
        }

        best_traj.first.reserve(nr_steps_);

        num_threads = 16;

        std::vector<std::thread> threads;
        threads.reserve(num_threads);

        int task_per_thread = nr_pairs_ / num_threads;

        std::vector<std::vector<Cost> > thread_costs(num_threads);
        std::vector<std::vector<std::pair<std::vector<PoseState>, std::vector<
            PoseState> > > > thread_trajectories(num_threads);

        for (int i = 0; i < num_threads; ++i) {
            int start = i * task_per_thread;
            int end = (i == num_threads - 1) ? nr_pairs_ : (start + task_per_thread);

            thread_costs[i].reserve(end - start);
            thread_trajectories[i].reserve(end - start);

            threads.emplace_back(
                [this, i, start, end, &state, &state_odom, &dw, &pairs,
                    &thread_costs, &thread_trajectories]() {
                    this->process_segment(i, start, end, state, state_odom, dw, pairs,
                                          thread_costs[i], thread_trajectories[i]);
                });
        }

        for (auto &thread: threads) {
            thread.join();
        }

        std::vector<Cost> costs;
        std::vector<std::pair<std::vector<PoseState>, std::vector<PoseState> > >
                trajectories;

        for (int i = 0; i < num_threads; ++i) {
            costs.insert(costs.end(), thread_costs[i].begin(), thread_costs[i].end());
            trajectories.insert(trajectories.end(), thread_trajectories[i].begin(), thread_trajectories[i].end());
        }

        auto cost_it = costs.begin();
        auto traj_it = trajectories.begin();
        auto pairs_it = pairs.begin();

        // robot->viewTrajectories(trajectories[0].first, nr_steps_, 0.0);

        while (cost_it != costs.end() && traj_it != trajectories.end() && pairs_it != pairs.end()) {
            if (cost_it->obs_cost_ == 1e6 || cost_it->path_cost_ == 1e6) {
                cost_it = costs.erase(cost_it);
                traj_it = trajectories.erase(traj_it);
                pairs_it = pairs.erase(pairs_it);
            } else {
                ++cost_it;
                ++traj_it;
                ++pairs_it;
            }
        }

        if (costs.empty()) {
            ROS_ERROR_THROTTLE(1.0, "No available trajectory after cleaning.");
            best_traj.second = false;
            return false;
        }

        //Logger::m_out << "available trajectory " << available_traj_count << std::endl;
        normalize_costs(costs);

        const size_t max_elements = 10;
        const size_t top_n = std::min(max_elements, costs.size());

        std::vector<size_t> indices(costs.size());
        std::iota(indices.begin(), indices.end(), 0);

        std::sort(indices.begin(), indices.end(), [&](size_t i, size_t j) {
            return costs[i].total_cost_ < costs[j].total_cost_;
        });

        double J_min = costs[indices[0]].total_cost_;
        std::vector<double> costs_weights(top_n, 0.0);
        const double lambda = 1.0;
        double weight_sum = 0.0;

        for (size_t i = 0; i < costs.size(); ++i) {
            if (costs[i].total_cost_ < min_cost.total_cost_) {
                min_cost = costs[i];
                best_traj.first = trajectories[i].first;
            }
        }

        for (size_t k = 0; k < top_n && k < indices.size(); ++k) {
            size_t idx = indices[k];
            costs_weights[k] = std::exp(-(costs[idx].total_cost_ - J_min) / lambda);
            weight_sum += costs_weights[k];
        }

        if (weight_sum > 1e-6) {
            for (size_t k = 0; k < top_n && k < indices.size(); ++k) {
                costs_weights[k] /= weight_sum;
            }
        } else {
            ROS_ERROR("Weight sum is zero. Check cost calculation.");
            return false;
        }

        double delta_v_sum = 0.0;
        double delta_w_sum = 0.0;

        for (size_t k = 0; k < top_n && k < indices.size(); ++k) {
            size_t idx = indices[k];
            delta_v_sum += costs_weights[k] * pairs[idx].first;
            delta_w_sum += costs_weights[k] * pairs[idx].second;
        }

        best_traj.first = generateTrajectory(state, state_odom, delta_v_sum, delta_w_sum).first;

        best_traj.second = true;

        return true;
    }

    void MPPIPlanner::process_segment(int thread_id, int start, int end, PoseState &state,
                                         PoseState &state_odom, Window &dw,
                                         std::vector<std::pair<double, double> > &pairs,
                                         std::vector<Cost> &thread_costs,
                                         std::vector<std::pair<std::vector<PoseState>, std::vector<
                                             PoseState> > > &thread_trajectories) {
        Timer::Clock d_t;
        Timer::Start(d_t);

        for (int i = start; i < end; ++i) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::normal_distribution<double> linear_dist(0.0, linear_stddev);
            std::normal_distribution<double> angular_dist(0.0, angular_stddev);
            std::vector<std::pair<double, double> > perturbations(nr_steps_);
            double dist = -1;
            std::vector<double> last_position;

            for (int j = 0; j < nr_steps_; ++j) {
                double delta_v = linear_dist(gen);
                double delta_w = angular_dist(gen);

                double sampled_v = pairs[i].first + delta_v;
                double sampled_w = pairs[i].second + delta_w;
                // double sampled_v = 1;
                // double sampled_w = -0.7;

                perturbations[j] = {sampled_v, sampled_w};
            }

            std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectories;
            trajectories = generateTrajectory(state, state_odom, perturbations);
            getTrajBySavitzkyGolayFilter(trajectories);
            const Cost cost = evaluate_trajectory(trajectories, dist, last_position);

            thread_costs.emplace_back(cost);
            thread_trajectories.emplace_back(trajectories);
        }
    }

    bool MPPIPlanner::hasRotateFirst(PoseState &state, PoseState &state_odom,
                                        double angle_to_goal) {
        if (fabs(angle_to_goal) < angle_to_goal_)
            return false;

        const double angular_velocity = std::min(std::max(angle_to_goal, GetSimulator()->GetMinAngularVelocity()),
                                                 GetSimulator()->GetMaxAngularVelocity());

        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory = generateTrajectory(
            state, state_odom, angular_velocity);

        if (collisionCheck(trajectory.first))
            return true;
        else
            return false;
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    MPPIPlanner::generateTrajectory(PoseState &state, PoseState &state_odom,
                                       double angular_velocity) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        n = 0.0;
        for (int i = 0; i < nr_steps_; ++i) {
            motion(state_, 0.0000001, angular_velocity, dt);
            trajectory.first[i] = state_;
            motion(state_odom, 0.0000001, angular_velocity, dt);
            trajectory.second[i] = state_odom_;
            //n++;
        }

        return trajectory;
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    MPPIPlanner::generateTrajectory(PoseState &state, PoseState &state_odom,
                                       std::vector<std::pair<double, double> > &perturbations) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        for (int i = 0; i < nr_steps_; i++) {
            motion(state_, perturbations[i].first, perturbations[i].second, dt);
            trajectory.first[i] = state_;
            motion(state_odom_, perturbations[i].first, perturbations[i].second, dt);
            trajectory.second[i] = state_odom_;
        }

        return trajectory;
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    MPPIPlanner::generateTrajectory(PoseState &state, PoseState &state_odom, const double v,
                                       const double w) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        for (int i = 0; i < nr_steps_; i++) {
            motion(state_, v + 0.00001, w, dt);
            trajectory.first[i] = state_;
            motion(state_odom_, v + 0.00001, w, dt);
            trajectory.second[i] = state_odom_;
        }

        return trajectory;
    }

    double MPPIPlanner::updateVelocity(double current, double target, double maxAccel, double minAccel, double t) {
        if (current < target) {
            return std::min(current + maxAccel * t, target);
        } else {
            return std::max(current + minAccel * t, target);
        }
    }

    bool MPPIPlanner::invertMatrix(std::vector<std::vector<double> > &mat) {
        int n = mat.size();
        std::vector<std::vector<double> > identity(n, std::vector<double>(n, 0.0));
        for (int i = 0; i < n; ++i) identity[i][i] = 1.0;

        for (int i = 0; i < n; ++i) {
            double diag = mat[i][i];
            if (std::abs(diag) < 1e-8) return false; // 检查是否可逆

            for (int j = 0; j < n; ++j) {
                mat[i][j] /= diag;
                identity[i][j] /= diag;
            }

            for (int k = 0; k < n; ++k) {
                if (k == i) continue;
                double factor = mat[k][i];
                for (int j = 0; j < n; ++j) {
                    mat[k][j] -= factor * mat[i][j];
                    identity[k][j] -= factor * identity[i][j];
                }
            }
        }
        mat = identity;
        return true;
    }

    std::vector<double> MPPIPlanner::calculateSGCoefficients(int window_size, int poly_order) {
        if (window_size % 2 == 0 || poly_order >= window_size) {
            throw std::invalid_argument("Window size must be odd and greater than polynomial order.");
        }

        int half_window = window_size / 2;


        std::vector<std::vector<double> > A(window_size, std::vector<double>(poly_order + 1, 0.0));
        for (int i = -half_window; i <= half_window; ++i) {
            for (int j = 0; j <= poly_order; ++j) {
                A[i + half_window][j] = pow(i, j);
            }
        }

        std::vector<std::vector<double> > ATA(poly_order + 1, std::vector<double>(poly_order + 1, 0.0));
        for (int i = 0; i <= poly_order; ++i) {
            for (int j = 0; j <= poly_order; ++j) {
                for (int k = 0; k < window_size; ++k) {
                    ATA[i][j] += A[k][i] * A[k][j];
                }
            }
        }

        if (!invertMatrix(ATA)) {
            throw std::runtime_error("Matrix inversion failed. Check input parameters.");
        }

        std::vector<double> coefficients(window_size, 0.0);
        for (int i = 0; i < window_size; ++i) {
            for (int j = 0; j <= poly_order; ++j) {
                coefficients[i] += ATA[j][0] * A[i][j];
            }
        }

        return coefficients;
    }


    std::vector<double> MPPIPlanner::savitzkyGolayFilter(const std::vector<double> &data, int window_size,
                                                            int poly_order) {
        std::vector<double> coefficients = {-0.0952, 0.1429, 0.2857, 0.3333, 0.2857, 0.1429, -0.0952};
        //std::vector<double> coefficients = calculateSGCoefficients(window_size, poly_order);
        int half_window = window_size / 2;

        std::vector<double> smoothed_data(data.size(), 0.0);
        for (size_t i = 0; i < data.size(); ++i) {
            double smoothed_value = 0.0;

            for (int j = -half_window; j <= half_window; ++j) {
                int idx = std::min(std::max(static_cast<int>(i) + j, 0), static_cast<int>(data.size()) - 1);
                smoothed_value += coefficients[j + half_window] * data[idx];
            }

            smoothed_data[i] = smoothed_value;
        }

        return smoothed_data;
    }

    void MPPIPlanner::motion(PoseState &state, const double velocity, const double angular_velocity, double t) {
        double v = updateVelocity(state.velocity_, velocity, maxAccelerSpeed, minAccelerSpeed, t);
        double w = updateVelocity(state.angular_velocity_, angular_velocity, maxAngularAccelerSpeed,
                                  minAngularAccelerSpeed, t);

        state.theta_ += w * t;
        state.x_ += v * cos(state.theta_) * t;
        state.y_ += v * sin(state.theta_) * t;
        state.velocity_ = v;
        state.angular_velocity_ = w;

        state.theta_ = normalizeAngle(state.theta_);
    }

    void MPPIPlanner::normalize_costs(std::vector<MPPIPlanner::Cost> &costs) {
        Cost min_cost(1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6), max_cost;

        for (const auto &cost: costs) {
            if (cost.obs_cost_ != 1e6) {
                min_cost.obs_cost_ = std::min(min_cost.obs_cost_, cost.obs_cost_);
                max_cost.obs_cost_ = std::max(max_cost.obs_cost_, cost.obs_cost_);
                if (use_goal_cost_) {
                    min_cost.to_goal_cost_ = std::min(min_cost.to_goal_cost_, cost.to_goal_cost_);
                    max_cost.to_goal_cost_ = std::max(max_cost.to_goal_cost_, cost.to_goal_cost_);
                }
                if (use_ori_cost_) {
                    min_cost.ori_cost_ = std::min(min_cost.ori_cost_, cost.ori_cost_);
                    max_cost.ori_cost_ = std::max(max_cost.ori_cost_, cost.ori_cost_);
                }
                if (use_speed_cost_) {
                    min_cost.speed_cost_ = std::min(min_cost.speed_cost_, cost.speed_cost_);
                    max_cost.speed_cost_ = std::max(max_cost.speed_cost_, cost.speed_cost_);
                }
                if (use_path_cost_) {
                    min_cost.path_cost_ = std::min(min_cost.path_cost_, cost.path_cost_);
                    max_cost.path_cost_ = std::max(max_cost.path_cost_, cost.path_cost_);
                }
                if (use_angular_cost_) {
                    min_cost.aw_cost_ = std::min(min_cost.aw_cost_, cost.aw_cost_);
                    max_cost.aw_cost_ = std::max(max_cost.aw_cost_, cost.aw_cost_);
                }
            }
        }

        for (auto &cost: costs) {
            if (cost.obs_cost_ != 1e6) {
                cost.obs_cost_ =
                        (cost.obs_cost_ - min_cost.obs_cost_) / (max_cost.obs_cost_ - min_cost.obs_cost_ + DBL_EPSILON);
                if (use_goal_cost_) {
                    cost.to_goal_cost_ = (cost.to_goal_cost_ - min_cost.to_goal_cost_) /
                                         (max_cost.to_goal_cost_ - min_cost.to_goal_cost_ + DBL_EPSILON);
                }
                if (use_ori_cost_)
                    cost.ori_cost_ =
                            (cost.ori_cost_ - min_cost.ori_cost_) /
                            (max_cost.ori_cost_ - min_cost.ori_cost_ + DBL_EPSILON);
                if (use_speed_cost_)
                    cost.speed_cost_ =
                            (cost.speed_cost_ - min_cost.speed_cost_) /
                            (max_cost.speed_cost_ - min_cost.speed_cost_ + DBL_EPSILON);
                if (use_path_cost_)
                    cost.path_cost_ =
                            (cost.path_cost_ - min_cost.path_cost_) /
                            (max_cost.path_cost_ - min_cost.path_cost_ + DBL_EPSILON);
                if (use_angular_cost_)
                    cost.aw_cost_ =
                            (cost.aw_cost_ - min_cost.aw_cost_) /
                            (max_cost.aw_cost_ - min_cost.aw_cost_ + DBL_EPSILON);

                cost.to_goal_cost_ *= to_goal_cost_gain_;
                cost.obs_cost_ *= obs_cost_gain_;
                cost.speed_cost_ *= speed_cost_gain_;
                cost.path_cost_ *= path_cost_gain_;
                cost.ori_cost_ *= ori_cost_gain_;
                cost.aw_cost_ *= aw_cost_gain_;
                cost.calc_total_cost();
            }
        }
    }

    double MPPIPlanner::calculateTheta(const PoseState &state, const double *y) {
        double deltaX = y[0] - state.x_;
        double deltaY = y[1] - state.y_;
        double theta = atan2(deltaY, deltaX);

        double normalizedTheta = normalizeAngle(state.theta_);

        return normalizeAngle(theta - normalizedTheta);
    }

    double MPPIPlanner::normalizeAngle(double a) {
        a = fmod(a + M_PI, 2 * M_PI);
        if (a <= 0)
            a += 2 * M_PI;

        return a - M_PI;
    }

    bool MPPIPlanner::collisionCheck(std::vector<PoseState> &traj) {
        auto obss = robot->getDataMap();
        std::vector<double> info = robot->getSize();
        for (size_t i = 0; i < traj.size() - 1; ++i) {
            const auto &state1 = traj[i];
            const auto &state2 = traj[i + 1];
            RobotBox moving_box = calculateMovingBoundingBox(state1, state2, info[0], info[1]);

            for (const auto &obs: obss) {
                RobotBox expanded_box = moving_box;
                expanded_box.x_min -= robot_radius_;
                expanded_box.x_max += robot_radius_;
                expanded_box.y_min -= robot_radius_;
                expanded_box.y_max += robot_radius_;

                if (isBoxIntersectingBox(expanded_box, obs)) {
                    return false;
                }
            }
        }
        return true;
    }

    double MPPIPlanner::calc_path_cost(const std::vector<PoseState> &traj) {
        if (!use_path_cost_)
            return 0.0;

        double d = 0;
        for (int i = 0; i < traj.size() - 2; i++)
            d += Algebra::PointDistance(2, &traj[i].pose()[0], &traj[i + 1].pose()[0]);

        if (d <= distance)
            return 1e6;

        std::vector<std::vector<double>> local_path = robot->local_paths;
        if (local_path.empty()) {
            // std::cerr << "Local path is empty!" << std::endl;
            return 0;
        }

        int i = 0;
        for (const auto& state : traj) {
            double min_distance = std::numeric_limits<double>::max();
            for (const auto& point : local_path) {
                if (point.size() < 2) continue; // Ensure the point has at least x and y coordinates
                double dx = state.x_ - point[0];
                double dy = state.y_ - point[1];
                double distance = std::sqrt(dx * dx + dy * dy);
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
            d += min_distance; // Add the minimum distance to total
            i++;
        }

        return d;
    }

    double MPPIPlanner::calc_ori_cost(const std::vector<PoseState> &traj) {
        if (!use_ori_cost_)
            return 0.0;

        double theta = calculateTheta(traj[traj.size() - 1], &local_goal[0]);

        return fabs(theta);
    }

    double MPPIPlanner::calc_angular_velocity(const std::vector<PoseState> &traj) {
        if (use_angular_cost_) {
            double angular_velocity = std::abs(traj.front().angular_velocity_);
            double angular_velocity_cost = angular_velocity * angular_velocity;
            return angular_velocity_cost;
        }

        return 0.0;
    }


    double MPPIPlanner::calc_to_goal_cost(const std::vector<PoseState> &traj) {
        if (use_goal_cost_ == false)
            return 0.0;

        // double d = 0;
        // for (int i = 0; i < traj.size() - 1; i++) {
        //     d += Algebra::PointDistance(2, &traj[i].pose()[0], &local_goal[0]);
        // }
        // return d / traj.size();

        return Algebra::PointDistance(2, &traj[traj.size() - 1].pose()[0], &local_goal[0]);

    }

    double MPPIPlanner::calc_dist_to_path(const std::vector<double> &state) {
        auto edge_point1 = local_paths.front();
        auto edge_point2 = local_paths.back();

        const double a = edge_point2[STATE_Y] - edge_point1[STATE_Y];
        const double b = -(edge_point2[STATE_X] - edge_point1[STATE_X]);
        const double c = -a * edge_point1[STATE_Y] - b * edge_point1[STATE_Y];

        return std::round(fabs(a * state[STATE_X] + b * state[STATE_Y] + c) / (hypot(a, b) + DBL_EPSILON) * 1000) /
               1000;
    }

    double MPPIPlanner::calc_obs_cost(const std::vector<PoseState> &traj) {
        auto obss = robot->getDataMap();
        auto distances = robot->laserDataDistance;
        bool flag = (distances.size() == obss.size());
        std::vector<double> info = robot->getSize();
        double v = info[3];

        double halfLength = info[0] / 2.0;
        double halfWidth = info[1] / 2.0;

        double min_dist = obs_range_;

        for (size_t i = 0; i < traj.size() - 1; ++i) {
            double cosTheta = std::cos(-traj[i].theta_);
            double sinTheta = std::sin(-traj[i].theta_);

            for (size_t j = 0; j < obss.size(); ++j) {
                double dist;

                double d = std::hypot(traj[i].x_ - obss[j][STATE_X], traj[i].y_ - obss[j][STATE_Y]);

                if (flag && d >= v)
                    dist = d - 0.33;
                else
                    dist = calculateDistanceToCarEdge(traj[i].x_, traj[i].y_, cosTheta, sinTheta, halfLength, halfWidth, obss[j]) - 0.05;

                if (dist < DBL_EPSILON) {
                    return 1e6;
                }

                min_dist = std::min(min_dist, dist);
            }
        }

        double cost;
        if (min_dist < 0.1) {
            cost = 1.0 / std::pow(min_dist + 1e-6, 2);

            if (cost >= 1e6)
                return 1e6;

        }else if (min_dist >= 0.1 && min_dist < 0.5) {
            cost = obs_range_ - min_dist + 1 / min_dist;
        }else
            cost = 0;

        return cost;
    }

    double MPPIPlanner::calculateDistanceToCarEdge(
        double carX, double carY, double cosTheta, double sinTheta,
        double halfLength, double halfWidth, const std::vector<double>& obs) {

        double relX = obs[0] - carX;
        double relY = obs[1] - carY;

        double localX = relX * cosTheta - relY * sinTheta;
        double localY = relX * sinTheta + relY * cosTheta;

        double dx = std::max(std::abs(localX) - halfLength, 0.0);
        double dy = std::max(std::abs(localY) - halfWidth, 0.0);

        return std::sqrt(dx * dx + dy * dy);
    }

    double MPPIPlanner::calc_speed_cost(const std::vector<PoseState> &traj) {
        if (!use_speed_cost_)
            return 0.0;

        const Window dw = calc_dynamic_window(parent, dt);

        return dw.max_velocity_ - traj.front().velocity_;
    }

    MPPIPlanner::RobotBox MPPIPlanner::calculateMovingBoundingBox(const PoseState &state1,
                                                                        const PoseState &state2, double robot_width,
                                                                        double robot_length) {
        RobotBox bbox;

        double dx = state2.x_ - state1.x_;
        double dy = state2.y_ - state1.y_;
        double angle = std::atan2(dy, dx);

        double half_width = robot_width / 2.0;
        double half_length = robot_length / 2.0;

        std::vector<std::pair<double, double> > corners = {

            {
                state1.x_ - half_length * std::cos(angle) + half_width * std::sin(angle),
                state1.y_ - half_length * std::sin(angle) - half_width * std::cos(angle)
            },

            {
                state1.x_ - half_length * std::cos(angle) - half_width * std::sin(angle),
                state1.y_ - half_length * std::sin(angle) + half_width * std::cos(angle)
            },

            {
                state2.x_ + half_length * std::cos(angle) - half_width * std::sin(angle),
                state2.y_ + half_length * std::sin(angle) + half_width * std::cos(angle)
            },

            {
                state2.x_ + half_length * std::cos(angle) + half_width * std::sin(angle),
                state2.y_ + half_length * std::sin(angle) - half_width * std::cos(angle)
            }
        };

        bbox.x_min = std::min({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        bbox.x_max = std::max({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        bbox.y_min = std::min({corners[0].second, corners[1].second, corners[2].second, corners[3].second});
        bbox.y_max = std::max({corners[0].second, corners[1].second, corners[2].second, corners[3].second});

        return bbox;
    }

    MPPIPlanner::RobotBox::RobotBox() : x_max(0.0), x_min(0.0), y_max(0.0), y_min(0.0) {
    }

    MPPIPlanner::RobotBox::RobotBox(double x_min_, double x_max_, double y_min_, double y_max_)
        : x_max(x_max_), x_min(x_min_), y_min(y_min_), y_max(y_max_) {
    }

    MPPIPlanner::Cost MPPIPlanner::evaluate_trajectory(
        std::pair<std::vector<PoseState>, std::vector<PoseState> > &trajectory,
        double &dist, std::vector<double> &last_position) {
        Cost cost;

        cost.to_goal_cost_ = calc_to_goal_cost(trajectory.first);
        cost.obs_cost_ = calc_obs_cost(trajectory.first);
        cost.speed_cost_ = calc_speed_cost(trajectory.first);
        cost.path_cost_ = calc_path_cost(trajectory.first);
        cost.ori_cost_ = calc_ori_cost(trajectory.first);
        cost.aw_cost_ = calc_angular_velocity(trajectory.first);
        cost.calc_total_cost();
        return cost;
    }

    MPPIPlanner::Cost MPPIPlanner::evaluate_trajectory(std::vector<PoseState> &trajectory,
                                                             double &dist, std::vector<double> &last_position) {
        Cost cost;
        cost.to_goal_cost_ = calc_to_goal_cost(trajectory);
        cost.obs_cost_ = calc_obs_cost(trajectory);
        cost.speed_cost_ = calc_speed_cost(trajectory);
        cost.path_cost_ = calc_path_cost(trajectory);
        cost.ori_cost_ = calc_ori_cost(trajectory);
        cost.aw_cost_ = calc_angular_velocity(trajectory);
        cost.calc_total_cost();
        return cost;
    }

    MPPIPlanner::Cost::Cost() : obs_cost_(0.0), to_goal_cost_(0.0), speed_cost_(0.0), path_cost_(0.0),
                                   ori_cost_(0.0), aw_cost_(0.0), total_cost_(0.0) {
    }

    MPPIPlanner::Cost::Cost(
        const double obs_cost, const double to_goal_cost, const double speed_cost, const double path_cost,
        const double ori_cost, const double aw_cost, const double total_cost)
        : obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), speed_cost_(speed_cost), path_cost_(path_cost),
          ori_cost_(ori_cost), aw_cost_(aw_cost), total_cost_(total_cost) {
    }

    void MPPIPlanner::Cost::show() const {
        ROS_INFO_STREAM("Cost: " << total_cost_);
        ROS_INFO_STREAM("\tObs cost: " << obs_cost_);
        ROS_INFO_STREAM("\tGoal cost: " << to_goal_cost_);
        ROS_INFO_STREAM("\tSpeed cost: " << speed_cost_);
        ROS_INFO_STREAM("\tPath cost: " << path_cost_);
        ROS_INFO_STREAM("\tOri cost: " << ori_cost_);
    }

    void MPPIPlanner::Cost::calc_total_cost() {
        total_cost_ = obs_cost_ + to_goal_cost_ + speed_cost_ + path_cost_ + ori_cost_;
    }

    void MPPIPlanner::Window::show() const {
        ROS_INFO_STREAM("Window:");
        ROS_INFO_STREAM("\tVelocity:");
        ROS_INFO_STREAM("\t\tmax: " << max_velocity_);
        ROS_INFO_STREAM("\t\tmin: " << min_velocity_);
        ROS_INFO_STREAM("\tYawrate:");
        ROS_INFO_STREAM("\t\tmax: " << max_angular_velocity_);
        ROS_INFO_STREAM("\t\tmin: " << min_angular_velocity_);
    }

    MPPIPlanner::Window::Window() : min_velocity_(0.0), max_velocity_(0.0), min_angular_velocity_(0.0),
                                       max_angular_velocity_(0.0) {
    }

    MPPIPlanner::Window MPPIPlanner::calc_dynamic_window(PoseState &state, double dt) {
        Window window;
        std::vector<double> info = robot->getSize();

        window.min_velocity_ = std::max((state.velocity_ + minAccelerSpeed * dt),
                                        info[2]);
        window.max_velocity_ = std::min((state.velocity_ + maxAccelerSpeed * dt),
                                        info[3]);

        window.min_angular_velocity_ = std::max(
            (state.angular_velocity_ + minAngularAccelerSpeed * dt), info[4]);
        window.max_angular_velocity_ = std::min(
            (state.angular_velocity_ + maxAngularAccelerSpeed * dt), info[5]);

        return window;
    }
}
