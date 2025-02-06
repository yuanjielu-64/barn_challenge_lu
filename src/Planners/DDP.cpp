#include "Planners/DDP.hpp"
#include <SceneAndSimulation/Robot.hpp>
#include <iomanip>

namespace Antipatrea {
    bool DDP::Solve(const int nrIters, double dt, bool &canBeSolved) {
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

    bool DDP::handleNoMapPlanning(geometry_msgs::Twist &cmd_vel) {
        if (robot->setting(Robot_config::NO_ANY_RECEIVED, 2) == false)
            return false;

        normalParameters(*robot);

        const double angle_to_goal = calculateTheta(parent, &robot->getGlobalGoalCfg()[0]);

        double angular = std::clamp(angle_to_goal, -1.0, 1.0);
        angular = (angular > 0) ? std::max(angular, 0.1) : std::min(angular, -0.1);

        publishCommand(cmd_vel, GetSimulator()->GetMaxVelocity(), angular);
        return true;
    }

    bool DDP::handleNormalSpeedPlanning(geometry_msgs::Twist &cmd_vel,
                                        std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {
        if (robot->setting(Robot_config::ONLY_LASER_RECEIVED, 2) == false)
            return false;

        normalParameters(*robot);

        auto result = mppi_planning(parent, parent_odom, best_traj, dt);

        robot->viewTrajectories(best_traj.first, nr_steps_, 0.0, timeInterval);
        if (result == false) {
            robot->setRobotState(Robot_config::LOW_SPEED_PLANNING);
            publishCommand(cmd_vel, best_traj.first[3].velocity_ / 2, best_traj.first[3].angular_velocity_ / 2);
        } else
            publishCommand(cmd_vel, best_traj.first[3].velocity_, best_traj.first[3].angular_velocity_);

        return true;
    }

    bool DDP::handleLowSpeedPlanning(geometry_msgs::Twist &cmd_vel,
                                     std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {
        if (!robot->setting(Robot_config::ONLY_LASER_RECEIVED, 1))
            return false;

        lowSpeedParameters(*robot);

        auto result = mppi_planning(parent, parent_odom, best_traj, dt);

        robot->viewTrajectories(best_traj.first, nr_steps_, 0.0, timeInterval);
        if (!result) {
            publishCommand(cmd_vel, 0, 0);
        } else
            publishCommand(cmd_vel, best_traj.first[3].velocity_, best_traj.first[3].angular_velocity_);

        return true;
    }

    bool DDP::handleAbnormalPlaning(geometry_msgs::Twist &cmd_vel,
                                    std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {
        if (robot->getRobotState() == Robot_config::BRAKE_PLANNING) {
            double vel = robot->getPoseState().velocity_;

            if (vel >= 0.5)
                publishCommand(cmd_vel, -0.5, 0.0);
            else if (vel >= 0.1 && vel <= 0.5)
                publishCommand(cmd_vel, -0.2, 0.0);
            else {
                publishCommand(cmd_vel, -0.1, 0.0);
                robot->setRobotState(Robot_config::RECOVERY);
            }

            return true;
        }

        if (robot->getRobotState() == Robot_config::RECOVERY) {
            if (robot->front_obs <= 0.1) {
                robot->setRobotState(Robot_config::BACKWARD);
                return true;
            }

            if (robot->setting(Robot_config::ONLY_LASER_RECEIVED, 2) == false)
                return false;

            recoverParameters(*robot);
            bool result = false;

            result = mppi_planning(parent, parent_odom, best_traj, dt);
            robot->viewTrajectories(best_traj.first, nr_steps_, 0.0, timeInterval);

            if (result) {
                publishCommand(cmd_vel, best_traj.first[3].velocity_, best_traj.first[3].angular_velocity_);

                robot->recover_times ++;

                if (robot->recover_times >= robot->re * 5) {
                    robot->setRobotState(Robot_config::LOW_SPEED_PLANNING);
                    robot->re += 1;
                }

                return true;
            }

            robot->recover_times = 0;
            std::vector<std::vector<double> > local_paths = robot->local_paths_odom;

            double sum_angle = 0.0;
            int count = 0;
            for (const auto &point: local_paths) {
                double dx = point[0] - parent_odom.x_;
                double dy = point[1] - parent_odom.y_;

                double angle = atan2(dy, dx);
                sum_angle += angle;
                count++;
            }

            double rotate_angle = sum_angle / count - parent_odom.theta_;
            rotate_angle = normalizeAngle(rotate_angle);

            if (fabs(rotate_angle) <= 0.2) {
                robot->setRobotState(Robot_config::LOW_SPEED_PLANNING);
                return true;
            }

            double z = rotate_angle > 0 ? std::min(rotate_angle, 1.0) : std::max(rotate_angle, -1.0);
            z = z > 0 ? std::max(z, 0.5) : std::min(z, -0.5);
            publishCommand(cmd_vel, 0.0, z);

            return true;
        }

        if (robot->getRobotState() == Robot_config::ROTATE_PLANNING) {
            return true;
        }

        if (robot->getRobotState() == Robot_config::BACKWARD) {
            if (robot->setting(Robot_config::ONLY_LASER_RECEIVED, 2) == false)
                return false;

            frontBackParameters(*robot);

            if (robot->front_obs >= 0.1) {
                robot->setRobotState(Robot_config::RECOVERY);
                return true;
            }

            publishCommand(cmd_vel, -0.1, 0);
        }

        return true;
    }

    void DDP::publishCommand(geometry_msgs::Twist &cmd_vel, double linear, double angular) {
        cmd_vel.linear.x = linear;
        cmd_vel.angular.z = angular;
        robot->Control().publish(cmd_vel);
    }

    bool DDP::mppi_planning(PoseState &state, PoseState &state_odom,
                            std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {
        Timer::Clock d_t;
        Timer::Start(d_t);

        double total_explore_time = 2; // 1 second
        timeInterval.clear();

        // double p = 1.7;
        // double alpha = 2;
        //
        // double previous_time = 0.0;
        //
        // for (int i = 1; i <= nr_steps_; ++i) {
        //     double normalized_step = static_cast<double>(i) / nr_steps_;
        //     double current_time = pow(normalized_step, p) * total_explore_time;
        //     double interval = current_time - previous_time;
        //     interval = std::round(interval * 10000.0) / 10000.0;
        //     timeInterval.push_back(interval);
        //     previous_time = current_time;
        //
        //     double weight = std::exp(-alpha * interval);
        //     // double weight = xx + i * 0.2;
        //     weights.push_back(weight);
        // }

        timeInterval = {
            0.0123, 0.0276, 0.0396, 0.0501, 0.0598, 0.0688, 0.0774, 0.0855,
            0.0934, 0.1009, 0.1083, 0.1154, 0.1223, 0.1291, 0.1357, 0.1422,
            0.1486, 0.1548, 0.1610, 0.1670
        };

        best_traj.first.reserve(nr_steps_);

        Cost min_cost(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e6);

        Window dw = calc_dynamic_window(state, dt);

        unsigned int max_threads = std::thread::hardware_concurrency();
        num_threads = std::min(16, static_cast<int>(max_threads));

        std::vector<std::vector<Cost> > thread_costs(num_threads);
        std::vector<std::vector<std::pair<std::vector<PoseState>, std::vector<PoseState> > > > thread_trajectories(
            num_threads);
        std::vector<std::vector<std::vector<std::pair<double, double> > > > thread_pairs(num_threads);

        int task_per_thread = nr_pairs_ / num_threads;
        nr_pairs_ = task_per_thread * num_threads;

        std::vector<std::pair<double, double> > pairs;
        pairs.reserve(nr_pairs_);
        double linear_velocity;
        double angular_velocity;

        if (robot->getRobotState() == Robot_config::LOW_SPEED_PLANNING) {
            for (int i = 0; i < nr_pairs_ - 100; ++i) {
                if (RandomUniformReal(0, 1) < 0.05 && delta_v_sum != FLT_MIN && delta_w_sum != FLT_MAX) {
                    linear_velocity = delta_v_sum;
                    angular_velocity = delta_w_sum;
                } else {
                    linear_velocity = RandomUniformReal(dw.min_velocity_, dw.max_velocity_);
                    angular_velocity = RandomUniformReal(dw.min_angular_velocity_, dw.max_angular_velocity_);
                }
                pairs.emplace_back(linear_velocity, angular_velocity);
            }

            for (int i = 0; i < 100; ++i) {
                pairs.emplace_back(0.01, 0.0);
            }
        } else {
            for (int i = 0; i < nr_pairs_; ++i) {
                if (RandomUniformReal(0, 1) < 0.05 && delta_v_sum != FLT_MIN && delta_w_sum != FLT_MAX) {
                    linear_velocity = delta_v_sum;
                    angular_velocity = delta_w_sum;
                } else {
                    linear_velocity = RandomUniformReal(dw.min_velocity_, dw.max_velocity_);
                    angular_velocity = RandomUniformReal(dw.min_angular_velocity_, dw.max_angular_velocity_);
                }
                pairs.emplace_back(linear_velocity, angular_velocity);
            }
        }

        std::vector<std::thread> threads;
        threads.reserve(num_threads);

        for (int i = 0; i < num_threads; ++i) {
            int start = i * task_per_thread;
            int end = (i == num_threads - 1) ? nr_pairs_ : (start + task_per_thread);

            thread_costs[i].reserve(end - start);
            thread_trajectories[i].reserve(end - start);
            thread_pairs[i].reserve(end - start);

            threads.emplace_back(
                [this, i, start, end, &state, &state_odom, &dw, &pairs,
                    &thread_costs, &thread_trajectories, &thread_pairs]() {
                    this->process_segment(i, start, end, state, state_odom, dw, pairs,
                                          thread_costs[i], thread_trajectories[i], thread_pairs[i]);
                });
        }

        for (auto &thread: threads) {
            thread.join();
        }

        // Logger::m_out << "multi_thread_1 " << Timer::Elapsed(d_t) << std::endl;

        std::vector<Cost> costs;
        std::vector<std::pair<std::vector<PoseState>, std::vector<PoseState> > >
                trajectories;
        std::vector<std::vector<std::pair<double, double> > > pairs_set;

        for (int i = 0; i < num_threads; ++i) {
            costs.insert(costs.end(), thread_costs[i].begin(), thread_costs[i].end());
            trajectories.insert(trajectories.end(), thread_trajectories[i].begin(), thread_trajectories[i].end());
            pairs_set.insert(pairs_set.end(), thread_pairs[i].begin(), thread_pairs[i].end());
        }
        // Logger::m_out << "multi_thread_2 " << Timer::Elapsed(d_t) << std::endl;

        if (costs.empty()) {
            ROS_ERROR_THROTTLE(1.0, "No available trajectory after cleaning.");
            best_traj.second = false;
            return false;
        }

        // Logger::m_out << "multi_thread_2 " << Timer::Elapsed(d_t) << std::endl;

        normalize_costs(costs);

        const size_t max_elements = 10;
        const size_t top_n = std::min(max_elements, costs.size());

        std::vector<size_t> indices(costs.size());
        std::iota(indices.begin(), indices.end(), 0);

        std::sort(indices.begin(), indices.end(), [&](size_t i, size_t j) {
            return costs[i].total_cost_ < costs[j].total_cost_;
        });

        robot->viewTrajectories(trajectories[indices[0]].second, nr_steps_, timeInterval);

        // double dist = -1;
        // std::vector<double> last_position;
        // const Cost test = evaluate_trajectory(trajectories[indices[0]].first, dist, last_position);

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

        double p = 2;
        for (size_t k = 0; k < top_n && k < indices.size(); ++k) {
            double normalized_step = static_cast<double>(top_n - k) / static_cast<double>(top_n);
            double ws = 0.0 + 1.0 * pow(normalized_step, p);

            size_t idx = indices[k];
            costs_weights[k] = std::exp(-(costs[idx].total_cost_ - J_min) / lambda) * ws;
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

        delta_v_sum = 0.0;
        delta_w_sum = 0.0;

        for (size_t k = 0; k < top_n && k < indices.size(); ++k) {
            size_t idx = indices[k];
            for (size_t j = 0; j < nr_steps_; ++j) {
                delta_v_sum += costs_weights[k] * pairs_set[idx][j].first * timeInterval[j] / 2.0;
                delta_w_sum += costs_weights[k] * pairs_set[idx][j].second * timeInterval[j] / 2.0;
            }
        }

        delta_v_sum = std::clamp(delta_v_sum, robot->getSize()[2], robot->getSize()[3]);
        delta_w_sum = std::clamp(delta_w_sum, robot->getSize()[4], robot->getSize()[5]);

        best_traj.first = generateTrajectory(state, state_odom, delta_v_sum, delta_w_sum).first;

        best_traj.second = true;

        // Logger::m_out << "multi_thread_3 " << Timer::Elapsed(d_t) << std::endl;

        return true;
    }

    void DDP::process_segment(int thread_id, int start, int end, PoseState &state,
                              PoseState &state_odom, Window &dw,
                              std::vector<std::pair<double, double> > &pairs,
                              std::vector<Cost> &thread_costs,
                              std::vector<std::pair<std::vector<PoseState>, std::vector<
                                  PoseState> > > &thread_trajectories,
                              std::vector<std::vector<std::pair<double, double> > > &thread_pairs) {
        auto start_time = std::chrono::high_resolution_clock::now();

        static thread_local std::mt19937 gen(std::random_device{}());
        std::normal_distribution<double> linear_dist(0.0, linear_stddev);
        std::normal_distribution<double> angular_dist(0.0, angular_stddev);

        for (int i = start; i < end; ++i) {
            if (timeout_flag.load()) break;

            auto elapsed_time = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_time).
                    count();

            if (elapsed_time >= 0.040) {
                timeout_flag.store(true);
                break;
            }

            std::vector<std::pair<double, double> > perturbations(nr_steps_);
            double dist = -1;
            std::vector<double> last_position;

            for (int j = 0; j < nr_steps_; ++j) {
                double delta_v = linear_dist(gen);
                double delta_w = angular_dist(gen);

                double sampled_v = pairs[i].first + delta_v;
                double sampled_w = pairs[i].second + delta_w;

                sampled_v = std::clamp(sampled_v, robot->getSize()[2], robot->getSize()[3]);
                sampled_w = std::clamp(sampled_w, robot->getSize()[4], robot->getSize()[5]);

                perturbations[j] = {sampled_v, sampled_w};
            }

            std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectories;
            trajectories = generateTrajectory(state, state_odom, perturbations);
            // robot->viewTrajectories(trajectories.first, nr_steps_, 0.0, timeInterval);

            const Cost cost = evaluate_trajectory(trajectories, dist, last_position);

            if (cost.obs_cost_ != 1e6 && cost.path_cost_ != 1e6 && cost.ori_cost_ != 1e6) {
                thread_pairs.emplace_back(perturbations);
                thread_costs.emplace_back(cost);
                thread_trajectories.emplace_back(trajectories);
            }
        }
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    DDP::generateTrajectory(PoseState &state, PoseState &state_odom,
                            double angular_velocity) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        for (int i = 0; i < nr_steps_; ++i) {
            motion(state_, 0.0000001, angular_velocity, timeInterval[i]);
            trajectory.first[i] = state_;
            motion(state_odom, 0.0000001, angular_velocity, timeInterval[i]);
            trajectory.second[i] = state_odom_;
        }

        return trajectory;
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    DDP::generateTrajectory(PoseState &state, PoseState &state_odom,
                            std::vector<std::pair<double, double> > &perturbations) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        for (int i = 0; i < nr_steps_; i++) {
            motion(state_, perturbations[i].first, perturbations[i].second, timeInterval[i]);
            trajectory.first[i] = state_;
            motion(state_odom_, perturbations[i].first, perturbations[i].second, timeInterval[i]);
            trajectory.second[i] = state_odom_;
        }

        return trajectory;
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    DDP::generateTrajectory(PoseState &state, PoseState &state_odom, const double v,
                            const double w) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        for (int i = 0; i < nr_steps_; i++) {
            motion(state_, v + 0.00001, w, timeInterval[i]);
            trajectory.first[i] = state_;
            motion(state_odom_, v + 0.00001, w, timeInterval[i]);
            trajectory.second[i] = state_odom_;
        }

        return trajectory;
    }

    double DDP::updateVelocity(double current, double target, double maxAccel, double minAccel, double t) {
        if (current < target) {
            return std::min(current + maxAccel * t, target);
        }

        return std::max(current + minAccel * t, target);

    }

    void DDP::motion(PoseState &state, const double velocity, const double angular_velocity, double t) {
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

    void DDP::normalize_costs(std::vector<DDP::Cost> &costs) {
        Cost min_cost(1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6, 1e6), max_cost;

        for (const auto &cost: costs) {
            if (cost.obs_cost_ != 1e6 && cost.path_cost_ != 1e6 && cost.ori_cost_ != 1e6) {
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

                if (use_space_cost_) {
                    min_cost.space_cost_ = std::min(min_cost.space_cost_, cost.space_cost_);
                    max_cost.space_cost_ = std::max(max_cost.space_cost_, cost.space_cost_);
                }
            }
        }

        for (auto &cost: costs) {
            if (cost.obs_cost_ != 1e6 && cost.path_cost_ != 1e6 && cost.ori_cost_ != 1e6) {
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

                if (use_space_cost_)
                    cost.space_cost_ =
                            (cost.space_cost_ - min_cost.space_cost_) /
                            (max_cost.space_cost_ - min_cost.space_cost_ + DBL_EPSILON);

                cost.to_goal_cost_ *= to_goal_cost_gain_;
                cost.obs_cost_ *= obs_cost_gain_;
                cost.speed_cost_ *= speed_cost_gain_;
                cost.path_cost_ *= path_cost_gain_;
                cost.ori_cost_ *= ori_cost_gain_;
                cost.aw_cost_ *= aw_cost_gain_;
                cost.space_cost_ *= space_cost_gain_;
                cost.calc_total_cost();
            }
        }
    }

    double DDP::calculateTheta(const PoseState &state, const double *y) {
        double deltaX = y[0] - state.x_;
        double deltaY = y[1] - state.y_;
        double theta = atan2(deltaY, deltaX);

        double normalizedTheta = normalizeAngle(state.theta_);

        return normalizeAngle(theta - normalizedTheta);
    }

    double DDP::normalizeAngle(double a) {
        a = fmod(a + M_PI, 2 * M_PI);
        if (a <= 0)
            a += 2 * M_PI;

        return a - M_PI;
    }

    double DDP::calc_path_cost(const std::vector<PoseState> &traj) {
        double d = 0;
        for (int i = 0; i < traj.size() - 2; i++)
            d += Algebra::PointDistance(2, &traj[i].pose()[0], &traj[i + 1].pose()[0]);

        if (d <= distance)
            return 1e6;

        if (!use_path_cost_)
            return 0.0;

        std::vector<std::vector<double> > local_path = robot->local_paths;
        if (local_path.empty()) {
            return 0.0;
        }

        int i = 0;
        for (const auto &state: traj) {
            double min_distance = std::numeric_limits<double>::max();
            for (const auto &point: local_path) {
                if (point.size() < 2) continue; // Ensure the point has at least x and y coordinates
                double dx = state.x_ - point[0];
                double dy = state.y_ - point[1];
                double distance = std::sqrt(dx * dx + dy * dy);
                if (distance < min_distance) {
                    min_distance = distance;
                }
            }
            d += min_distance;
            i++;
        }

        return d / (double) traj.size();
    }

    double DDP::calc_ori_cost(const std::vector<PoseState> &traj) {
        if (!use_ori_cost_)
            return 0.0;

        double theta = 0;

        for (int i = int(3 * traj.size() / 4); i < traj.size() - 1; i++) {
            theta += fabs(calculateTheta(traj[i], &local_goal[0]));
        }

        return theta / int(1 * traj.size() / 4);;
    }

    double DDP::calc_angular_velocity(const std::vector<PoseState> &traj) {
        if (use_angular_cost_) {
            double angular_velocity = std::abs(traj.front().angular_velocity_);
            double angular_velocity_cost = angular_velocity * angular_velocity;
            return angular_velocity_cost;
        }

        return 0.0;
    }

    double DDP::calc_to_goal_cost(const std::vector<PoseState> &traj) {
        if (use_goal_cost_ == false)
            return 0.0;

        double d = 0;
        for (int i = int(3 * traj.size() / 4); i < traj.size() - 1; i++) {
            d += Algebra::PointDistance(2, &traj[i].pose()[0], &local_goal[0]);
        }

        return d / int(1 * traj.size() / 4);

    }

    double DDP::pointToSegmentDistance(const PoseState &p1, const PoseState &p2, const std::vector<double> &o) {
        double dx = p2.x_ - p1.x_;
        double dy = p2.y_ - p1.y_;

        std::vector<double> info = robot->getSize();
        double half_length = info[0] / 2.0;
        double half_width = info[1] / 2.0;

        double t = ((o[0] - p1.x_) * dx + (o[1] - p1.y_) * dy) / (dx * dx + dy * dy);

        double theta = atan2(dy, dx);
        double cosTheta = cos(theta);
        double sinTheta = sin(theta);

        double closestX, closestY;
        if (t < 0) {
            closestX = p1.x_;
            closestY = p1.y_;
        } else if (t > 1) {
            closestX = p2.x_;
            closestY = p2.y_;
        } else {
            closestX = p1.x_ + t * dx;
            closestY = p1.y_ + t * dy;
        }

        return calculateDistanceToCarEdge(closestX, closestY, cosTheta, sinTheta, half_length, half_width, o);
    }

    double DDP::calc_obs_cost(const std::vector<PoseState> &traj, double &t) {
        auto obss = robot->getDataMap();
        auto distances = robot->laserDataDistance;
        bool flag = (distances.size() == obss.size());
        std::vector<double> info = robot->getSize();

        double radius = robot_radius_;
        double cost = 0.0;
        double space_cost = 0.0;

        const double angle_threshold = M_PI / 4;

        for (size_t i = 0; i < traj.size() - 1; ++i) {
            double min_dist = FLT_MAX;
            double min_front_dist = 3;

            for (auto & obs : obss) {
                double dist;
                double dx = obs[STATE_X] - traj[i].x_;
                double dy = obs[STATE_Y] - traj[i].y_;
                double d = std::hypot(dx, dy);

                if (flag && d >= 1)
                    dist = d - 0.33;
                else
                    dist = pointToSegmentDistance(traj[i], traj[i + 1], obs) - radius;

                if (dist < DBL_EPSILON) {
                    return 1e6;
                }

                if (min_dist > dist) min_dist = dist;

                if (use_space_cost_) {
                    if (i > int(3 * traj.size() / 4)) {
                        double traj_dx = traj[i + 1].x_ - traj[i].x_;
                        double traj_dy = traj[i + 1].y_ - traj[i].y_;
                        double traj_angle = std::atan2(traj_dy, traj_dx);
                        double obs_angle = std::atan2(dy, dx);
                        double angle_diff = std::fabs(traj_angle - obs_angle);

                        if (angle_diff > M_PI) angle_diff = 2 * M_PI - angle_diff;

                        if (angle_diff < angle_threshold) {
                            if (dist < min_front_dist) {
                                min_front_dist = dist;
                            }
                        }
                    } else {
                        min_front_dist = 0;
                    }
                } else {
                    min_front_dist = 0;
                }
            }

            if (min_dist < 0.05) {
                cost += 300;
            } else if (min_dist >= 0.05 && min_dist < 0.1) {
                cost += 150;
            } else if (min_dist >= 0.1 && min_dist < 1) {
                cost += obs_range_ - min_dist + 1 / min_dist;
            } else
                cost += 0;

            if (use_space_cost_) {
                if (i > int(3 * traj.size() / 4)) {
                    space_cost += std::max(obs_range_ - min_front_dist, 0.0);
                } else
                    space_cost += 0;
            } else {
                space_cost += 0;
            }
        }

        t = space_cost;

        return cost;
    }

    double DDP::calc_obs_cost(const std::vector<PoseState> &traj) {
        auto obss = robot->getDataMap();
        auto distances = robot->laserDataDistance;
        bool flag = (distances.size() == obss.size());
        std::vector<double> info = robot->getSize();
        double v = info[3];

        double halfLength = info[0] / 2.0;
        double halfWidth = info[1] / 2.0;

        double min_dist = obs_range_;
        double radius = robot_radius_;

        for (size_t i = 0; i < traj.size() - 1; ++i) {
            double cosTheta = std::cos(-traj[i].theta_);
            double sinTheta = std::sin(-traj[i].theta_);

            for (auto & obs : obss) {
                double dist;

                double d = std::hypot(traj[i].x_ - obs[STATE_X], traj[i].y_ - obs[STATE_Y]);

                if (flag && d >= v / 2)
                    dist = d - 0.33;
                else
                    dist = calculateDistanceToCarEdge(traj[i].x_, traj[i].y_, cosTheta, sinTheta, halfLength, halfWidth,
                                                      obs) - radius;

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
        } else if (min_dist >= 0.1 && min_dist < 1) {
            cost = obs_range_ - min_dist + 3 / min_dist;
        } else
            cost = 0;

        return cost;
    }

    double DDP::calculateDistanceToCarEdge(
        double carX, double carY, double cosTheta, double sinTheta,
        double halfLength, double halfWidth, const std::vector<double> &obs) {
        double relX = obs[0] - carX;
        double relY = obs[1] - carY;

        double localX = relX * cosTheta + relY * sinTheta;
        double localY = -relX * sinTheta + relY * cosTheta;

        double dx = std::max(std::abs(localX) - halfLength, 0.0);
        double dy = std::max(std::abs(localY) - halfWidth, 0.0);

        return std::sqrt(dx * dx + dy * dy);
    }

    double DDP::calc_speed_cost(const std::vector<PoseState> &traj) {
        if (!use_speed_cost_)
            return 0.0;

        const Window dw = calc_dynamic_window(parent, dt);

        return dw.max_velocity_ - traj.front().velocity_;
    }

    DDP::RobotBox::RobotBox() : x_max(0.0), x_min(0.0), y_max(0.0), y_min(0.0) {
    }

    DDP::RobotBox::RobotBox(double x_min_, double x_max_, double y_min_, double y_max_)
        : x_max(x_max_), x_min(x_min_), y_min(y_min_), y_max(y_max_) {
    }

    DDP::Cost DDP::evaluate_trajectory(
        std::pair<std::vector<PoseState>, std::vector<PoseState> > &trajectory,
        double &dist, std::vector<double> &last_position) {
        Cost cost;
        double t = 0.0;
        cost.to_goal_cost_ = calc_to_goal_cost(trajectory.first);
        cost.obs_cost_ = calc_obs_cost(trajectory.first, t);
        cost.space_cost_ = t;
        cost.speed_cost_ = calc_speed_cost(trajectory.first);
        cost.path_cost_ = calc_path_cost(trajectory.first);
        cost.ori_cost_ = calc_ori_cost(trajectory.first);
        cost.aw_cost_ = calc_angular_velocity(trajectory.first);

        cost.calc_total_cost();
        return cost;
    }

    DDP::Cost DDP::evaluate_trajectory(std::vector<PoseState> &trajectory,
                                       double &dist, std::vector<double> &last_position) {
        Cost cost;
        double t = 0.0;

        cost.to_goal_cost_ = calc_to_goal_cost(trajectory);
        cost.obs_cost_ = calc_obs_cost(trajectory, t);
        cost.space_cost_ = t;
        cost.speed_cost_ = calc_speed_cost(trajectory);
        cost.path_cost_ = calc_path_cost(trajectory);
        cost.ori_cost_ = calc_ori_cost(trajectory);
        cost.aw_cost_ = calc_angular_velocity(trajectory);
        cost.calc_total_cost();
        return cost;
    }

    DDP::Cost::Cost() : obs_cost_(0.0), to_goal_cost_(0.0), speed_cost_(0.0), path_cost_(0.0),
                        ori_cost_(0.0), aw_cost_(0.0), space_cost_(0.0), total_cost_(0.0) {
    }

    DDP::Cost::Cost(
        const double obs_cost, const double to_goal_cost, const double speed_cost, const double path_cost,
        const double ori_cost, const double aw_cost, const double space_cost, const double total_cost)
        : obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), speed_cost_(speed_cost), path_cost_(path_cost),
          ori_cost_(ori_cost), aw_cost_(aw_cost), space_cost_(space_cost), total_cost_(total_cost) {
    }

    void DDP::Cost::show() const {
        ROS_INFO_STREAM("Cost: " << total_cost_);
        ROS_INFO_STREAM("\tObs cost: " << obs_cost_);
        ROS_INFO_STREAM("\tGoal cost: " << to_goal_cost_);
        ROS_INFO_STREAM("\tSpeed cost: " << speed_cost_);
        ROS_INFO_STREAM("\tPath cost: " << path_cost_);
        ROS_INFO_STREAM("\tOri cost: " << ori_cost_);
        ROS_INFO_STREAM("\tSpace cost: " << space_cost_);
    }

    void DDP::Cost::calc_total_cost() {
        total_cost_ = obs_cost_ + to_goal_cost_ + speed_cost_ + path_cost_ + ori_cost_ + space_cost_;
    }

    void DDP::Window::show() const {
        ROS_INFO_STREAM("Window:");
        ROS_INFO_STREAM("\tVelocity:");
        ROS_INFO_STREAM("\t\tmax: " << max_velocity_);
        ROS_INFO_STREAM("\t\tmin: " << min_velocity_);
        ROS_INFO_STREAM("\tYawrate:");
        ROS_INFO_STREAM("\t\tmax: " << max_angular_velocity_);
        ROS_INFO_STREAM("\t\tmin: " << min_angular_velocity_);
    }

    DDP::Window::Window() : min_velocity_(0.0), max_velocity_(0.0), min_angular_velocity_(0.0),
                            max_angular_velocity_(0.0) {
    }

    DDP::Window DDP::calc_dynamic_window(PoseState &state, double dt) {
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
