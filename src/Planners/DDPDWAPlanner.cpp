#include "Planners/DDPDWAPlanner.hpp"
#include <SceneAndSimulation/Robot.hpp>
#include <iomanip>
#include <opencv2/core/mat.hpp>

namespace Antipatrea {
    bool DDPDWAPlanner::Solve(const int nrIters, double dt, bool &canBeSolved) {
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

    bool DDPDWAPlanner::handleNoMapPlanning(geometry_msgs::Twist &cmd_vel) {
        if (robot->setting(Robot_config::NO_ANY_RECEIVED, 2) == false)
            return false;

        normalParameters(*robot);

        const double angle_to_goal = calculateTheta(parent, &robot->getGlobalGoalCfg()[0]);

        double angular = std::clamp(angle_to_goal, -1.0, 1.0);
        angular = (angular > 0) ? std::max(angular, 0.1) : std::min(angular, -0.1);

        publishCommand(cmd_vel, GetSimulator()->GetMaxVelocity(), angular);
        return true;
    }

    bool DDPDWAPlanner::handleNormalSpeedPlanning(geometry_msgs::Twist &cmd_vel,
                                                  std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {

        if (robot->setting(Robot_config::ONLY_LASER_RECEIVED, 2) == false)
            return false;

        normalParameters(*robot);

        auto result = dwa_planning(parent, parent_odom, best_traj, dt);

        robot->viewTrajectories(best_traj.first, nr_steps_, 0.0, timeInterval);

        if (result == false) {
            publishCommand(cmd_vel, 0.5 , 0);
        } else {
            publishCommand(cmd_vel, best_traj.first[1].velocity_ , best_traj.first[1].angular_velocity_);
        }
        // if (result == false) {
        //     robot->setRobotState(Robot_config::BRAKE_PLANNING);
        //     publishCommand(cmd_vel, robot->getPoseState().velocity_, robot->getPoseState().angular_velocity_);
        // } else {
        //     auto output_command = cal_weight_output_commands(best_traj.first);
        //     publishCommand(cmd_vel, output_command[0], output_command[1]);
        // }

        return true;
    }

    bool DDPDWAPlanner::handleLowSpeedPlanning(geometry_msgs::Twist &cmd_vel,
                                               std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {

        if (!robot->setting(Robot_config::ONLY_LASER_RECEIVED, 1))
            return false;

        lowSpeedParameters(*robot);

        auto result = dwa_planning(parent, parent_odom, best_traj, dt);

        robot->viewTrajectories(best_traj.first, nr_steps_, 0.0, timeInterval);

        if (!result) {
            robot->setRobotState(Robot_config::BRAKE_PLANNING);
            publishCommand(cmd_vel, 0, 0);
        } else {
            auto output_command = cal_weight_output_commands(best_traj.first);
            publishCommand(cmd_vel, output_command[0], output_command[1]);
        }
        return true;
    }

    bool DDPDWAPlanner::handleAbnormalPlaning(geometry_msgs::Twist &cmd_vel,
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

        // if (robot->getRobotState() == Robot_config::FORWARD) {
        //     if (robot->setting(Robot_config::ONLY_COSTMAP_RECEIVED, 2) == false)
        //         return false;
        //
        //     frontBackParameters(*robot);
        //
        //     if (robot->latter_obs >= 0.15) {
        //         robot->setRobotState(Robot_config::RECOVERY);
        //         return true;
        //     }
        //
        //     auto result = dwa_planning(parent, parent_odom, best_traj, dt);
        //
        //     robot->viewTrajectories(best_traj.first, nr_steps_, 0.0);
        //
        //     if (result == false)
        //         publishCommand(cmd_vel, 0.1, 0.0);
        //     else
        //         publishCommand(cmd_vel, best_traj.first.front().velocity_, best_traj.first.front().angular_velocity_);
        // }

        return true;
    }


    void DDPDWAPlanner::publishCommand(geometry_msgs::Twist &cmd_vel, double linear, double angular) {
        cmd_vel.linear.x = linear;
        cmd_vel.angular.z = angular;
        robot->Control().publish(cmd_vel);
    }

    std::vector<double> DDPDWAPlanner::cal_weight_output_commands(std::vector<PoseState> &traj) {
        std::vector<double> output_commands;

        std::vector<double> times;
        times.reserve(traj.size());
        double t = 0;
        for (int i = 0; i < traj.size(); i++) {
            t += timeInterval[i];
            times.push_back(t);
        }

        int closestIndex = -1;
        double minDifference = std::numeric_limits<double>::max();

        for (int i = 0; i < times.size(); ++i) {
            double difference = std::abs(times[i] - dt);
            if (difference < minDifference) {
                minDifference = difference;
                closestIndex = i;
            }
        }
        output_commands.push_back(traj[closestIndex].velocity_);
        output_commands.push_back(traj[closestIndex].angular_velocity_);

        return output_commands;
    }

    double DDPDWAPlanner::recover(
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

            auto result = dwa_planning(state_, state_odom_, traj, dt);

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
            best_traj.first = generateTrajectory(state, state_odom, 0.0, 0.0).first;
            results = false;
            return best_theta;
        } else {
            //Logger::m_out << "available trajectory " << available_traj_count << std::endl;
            normalize_costs(costs);
            for (int i = 0; i < costs.size(); ++i) {
                if (costs[i].obs_cost_ != 1e6 && costs[i].path_cost_ != 1e6) {
                    if (costs[i].total_cost_ < min_cost.total_cost_) {
                        min_cost = costs[i];
                        best_traj.first = trajectories[i].first;
                        best_theta = theta_set[i];
                    }
                }
            }

            results = true;
        }

        return best_theta;
    }

    bool DDPDWAPlanner::dwa_planning(PoseState &state, PoseState &state_odom,
                                     std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {
        Timer::Clock d_t;
        Timer::Start(d_t);

        double total_explore_time = 1.8;  // 1 second
        timeInterval.clear();
        timeInterval.reserve(nr_steps_);
        weights.clear();
        weights.reserve(nr_steps_);

        // double p = 1.3;
        // double alpha = 2;
        //
        // double previous_time = 0.0;
        // for (int i = 1; i <= nr_steps_; ++i) {
        //     double normalized_step = static_cast<double>(i) / nr_steps_;
        //     double current_time = pow(normalized_step, p) * total_explore_time;
        //     double interval = current_time - previous_time;
        //     interval = std::round(interval * 10000.0) / 10000.0;
        //     timeInterval.push_back(interval);
        //     previous_time = current_time;
        //
        //     double weight = std::exp(-alpha * interval);
        //     weights.push_back(weight);
        // }

         timeInterval = {
             0.0366, 0.0536, 0.0626, 0.0693, 0.0748, 0.0794, 0.0835, 0.0872,
             0.0905, 0.0936, 0.0964, 0.0991, 0.1016, 0.1040, 0.1062, 0.1084,
             0.1104, 0.1124, 0.1143, 0.1161
         };

        weights = {
            0.7833, 0.6998, 0.6586, 0.6300, 0.6074, 0.5891, 0.5730, 0.5595,
            0.5468, 0.5358, 0.5259, 0.5165, 0.5079, 0.5001, 0.4926, 0.4856,
            0.4789, 0.4727, 0.4667, 0.4612
        };

        Cost min_cost(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e6);

        Window dw = calc_dynamic_window(state, dt);

        best_traj.first.reserve(nr_steps_);

        const double velocity_resolution =
                std::max((dw.max_velocity_ - dw.min_velocity_) / (v_steps_ - 1),
                         DBL_EPSILON);
        const double angularVelocity_resolution =
                std::max(
                    (dw.max_angular_velocity_ - dw.min_angular_velocity_) / (w_steps_ - 1),
                    DBL_EPSILON);

        //Logger::m_out << "dw.max_angular_velocity_ " << dw.max_angular_velocity_ << "  dw.min_angular_velocity_ "  << dw.min_angular_velocity_ << std::endl;

        num_threads = 18;

        std::vector<std::thread> threads;
        threads.reserve(num_threads);

        int task_per_thread = w_steps_ / num_threads;

        std::vector<std::vector<Cost> > thread_costs(num_threads);
        std::vector<std::vector<std::pair<std::vector<PoseState>, std::vector<
            PoseState> > > > thread_trajectories(num_threads);

        for (int i = 0; i < num_threads; ++i) {
            int start = i * task_per_thread;
            int end = (i == num_threads - 1) ? w_steps_ : (start + task_per_thread);

            thread_costs[i].reserve((end - start) * (v_steps_ + 1));
            thread_trajectories[i].reserve((end - start) * (v_steps_ + 1));

            threads.emplace_back(
                [this, i, start, end, &state, &state_odom, velocity_resolution, angularVelocity_resolution, &dw, &
                    thread_costs, &
                    thread_trajectories]() {
                    this->process_segment(i, start, end, state, state_odom, velocity_resolution,
                                          angularVelocity_resolution, dw,
                                          thread_costs[i], thread_trajectories[i]);
                });
        }

        for (auto &thread: threads) {
            thread.join();
        }

        //Logger::m_out << "multi_thread_1 " << Timer::Elapsed(d_t) << std::endl;

        std::vector<Cost> costs;
        std::vector<std::pair<std::vector<PoseState>, std::vector<PoseState> > >
                trajectories;

        for (int i = 0; i < num_threads; ++i) {
            costs.insert(costs.end(), thread_costs[i].begin(), thread_costs[i].end());
            trajectories.insert(trajectories.end(), thread_trajectories[i].begin(), thread_trajectories[i].end());
        }

        //Logger::m_out << "multi_thread_2 " << Timer::Elapsed(d_t) << std::endl;

        auto cost_it = costs.begin();
        auto traj_it = trajectories.begin();
        while (cost_it != costs.end() && traj_it != trajectories.end()) {
            if (cost_it->obs_cost_ == 1e6 || cost_it->path_cost_ == 1e6) {
                cost_it = costs.erase(cost_it);
                traj_it = trajectories.erase(traj_it);
            } else {
                ++cost_it;
                ++traj_it;
            }
        }

        if (costs.empty()) {
            ROS_ERROR_THROTTLE(1.0, "No available trajectory after cleaning.");
            best_traj.second = false;
            return false;
        }

        //Logger::m_out << "available trajectory " << available_traj_count << std::endl;
        normalize_costs(costs);

        for (int i = 0; i < costs.size(); ++i) {
            if (costs[i].total_cost_ < min_cost.total_cost_) {
                min_cost = costs[i];
                best_traj.first = trajectories[i].first;
            }
        }

        best_traj.second = true;


        //Logger::m_out << "best_traj velocity  " << best_traj.first.front().velocity_ << " best_traj anglar velocity "  << best_traj.first.front().angular_velocity_ << std::endl;

        //Logger::m_out << "multi_thread_3 " << Timer::Elapsed(d_t) << std::endl;
        return true;
    }

    void DDPDWAPlanner::process_segment(int thread_id, int start, int end, PoseState &state,
                                        PoseState &state_odom, double velocity_resolution,
                                        double angularVelocity_resolution, Window &dw,
                                        std::vector<Cost> &thread_costs,
                                        std::vector<std::pair<std::vector<PoseState>, std::vector<
                                            PoseState> > > &thread_trajectories) {
        Timer::Clock d_t;
        Timer::Start(d_t);

        for (int i = start; i < end; i++) {
            const double w = dw.min_angular_velocity_ + angularVelocity_resolution * i;

            double dist = -1;
            std::vector<double> last_position;
            for (int j = v_steps_; j > 0; j--) {
                std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectories;

                const double v = dw.min_velocity_ + velocity_resolution * j;

                // v = 1.05  w = -1.35
                trajectories = generateTrajectory(state, state_odom, v, w);

                // robot->viewTrajectories(trajectories.first, nr_steps_, 0.0, timeInterval);

                const Cost cost = evaluate_trajectory(trajectories, dist, last_position);

                thread_costs.emplace_back(cost);
                thread_trajectories.emplace_back(trajectories);
            }

            if (dw.min_angular_velocity_ < 0.0 && 0.0 < dw.max_angular_velocity_) {
                dist = -1;
                last_position.clear();
                std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectories;
                const double v = dw.min_velocity_ + velocity_resolution * i;

                trajectories = generateTrajectory(state, state_odom, v, 0.0);

                const Cost cost = evaluate_trajectory(trajectories, dist, last_position);

                thread_costs.emplace_back(cost);
                thread_trajectories.emplace_back(trajectories);
            }
        }

        //Logger::m_out << "thread ID " << thread_id << " time " << Timer::Elapsed(d_t) << std::endl;
    }


    bool DDPDWAPlanner::hasRotateFirst(PoseState &state, PoseState &state_odom,
                                       double angle_to_goal) {
        if (fabs(angle_to_goal) < angle_to_goal_)
            return false;

        const double angular_velocity = std::min(std::max(angle_to_goal, GetSimulator()->GetMinAngularVelocity()),
                                                 GetSimulator()->GetMaxAngularVelocity());

        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory =
                generateTrajectory(state, state_odom, angular_velocity);

        if (collisionCheck(trajectory.first))
            return true;
        else
            return false;
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    DDPDWAPlanner::generateTrajectory(PoseState &state, PoseState &state_odom,
                                      double angular_velocity) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        n = 0.0;
        for (int i = 0; i < nr_steps_; ++i) {
            motion(state_, 0.0000001, angular_velocity, timeInterval[i]);
            trajectory.first[i] = state_;
            motion(state_odom, 0.0000001, angular_velocity, timeInterval[i]);
            trajectory.second[i] = state_odom_;
            //n++;
        }

        return trajectory;
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    DDPDWAPlanner::generateTrajectory(PoseState &state, PoseState &state_odom, const double v,
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

    double DDPDWAPlanner::updateVelocity(double current, double target, double maxAccel, double minAccel, double t) {
        if (current < target) {
            return std::min(current + maxAccel * t, target);
        } else {
            return std::max(current + minAccel * t, target);
        }
    }

    void DDPDWAPlanner::motion(PoseState &state, const double velocity, const double angular_velocity, double t) {
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

    void DDPDWAPlanner::normalize_costs(std::vector<DDPDWAPlanner::Cost> &costs) {
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

    double DDPDWAPlanner::calculateTheta(const PoseState &state, const double *y) {
        double deltaX = y[0] - state.x_;
        double deltaY = y[1] - state.y_;
        double theta = atan2(deltaY, deltaX);

        double normalizedTheta = normalizeAngle(state.theta_);

        return normalizeAngle(theta - normalizedTheta);
    }

    double DDPDWAPlanner::normalizeAngle(double a) {
        a = fmod(a + M_PI, 2 * M_PI);
        if (a <= 0)
            a += 2 * M_PI;

        return a - M_PI;
    }

    bool DDPDWAPlanner::collisionCheck(std::vector<PoseState> &traj) {
        auto obss = robot->getDataMap();
        std::vector<double> info = robot->getSize();
        for (size_t i = 0; i < traj.size(); ++i) {
            const auto &state = traj[i];
            RobotBox moving_box = calculateMovingBoundingBox(state, info[0], info[1]);

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

    double DDPDWAPlanner::calc_path_cost(const std::vector<PoseState> &traj) {
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
            d += min_distance * weights[i]; // Add the minimum distance to total
            i++;
        }

        return d;
    }

    double DDPDWAPlanner::calc_ori_cost(const std::vector<PoseState> &traj) {
        if (!use_ori_cost_)
            return 0.0;

        double theta = calculateTheta(traj[traj.size() - 1], &local_goal[0]);

        return fabs(theta);
    }

    double DDPDWAPlanner::calc_angular_velocity(const std::vector<PoseState> &traj) {
        if (use_angular_cost_) {
            double angular_velocity = std::abs(traj.front().angular_velocity_);
            double angular_velocity_cost = angular_velocity * angular_velocity;
            return angular_velocity_cost;
        }

        return 0.0;
    }

    double DDPDWAPlanner::calc_to_goal_cost(const std::vector<PoseState> &traj) {
        if (use_goal_cost_ == false)
            return 0.0;

        double d = 0;
        for (int i = 10; i < traj.size() - 1; i++) {
            d += Algebra::PointDistance(2, &traj[i].pose()[0], &local_goal[0]) * weights[i];
        }

        return d / 10;
    }

    double DDPDWAPlanner::calc_dist_to_path(const std::vector<double> &state) {
        auto edge_point1 = local_paths.front();
        auto edge_point2 = local_paths.back();

        const double a = edge_point2[STATE_Y] - edge_point1[STATE_Y];
        const double b = -(edge_point2[STATE_X] - edge_point1[STATE_X]);
        const double c = -a * edge_point1[STATE_Y] - b * edge_point1[STATE_Y];

        return std::round(fabs(a * state[STATE_X] + b * state[STATE_Y] + c) / (hypot(a, b) + DBL_EPSILON) * 1000) /
               1000;
    }

    double DDPDWAPlanner::calc_obs_cost(const std::vector<PoseState> &traj) {
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
                    dist = calculateDistanceToCarEdge(traj[i].x_, traj[i].y_, cosTheta, sinTheta, halfLength, halfWidth, obss[j]) - 0.01;

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

        }else
            cost = obs_range_ - min_dist + 4 / min_dist;

        return cost;
    }

    double DDPDWAPlanner::calculateDistanceToCarEdge(
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

    double DDPDWAPlanner::calc_speed_cost(const std::vector<PoseState> &traj) {
        if (!use_speed_cost_)
            return 0.0;

        const Window dw = calc_dynamic_window(parent, dt);

        return dw.max_velocity_ - traj.front().velocity_;
    }

    DDPDWAPlanner::RobotBox DDPDWAPlanner::calculateMovingBoundingBox(const PoseState &state,
                                                                      double robot_width,
                                                                      double robot_length) {
        RobotBox bbox;

        double x = state.x_;
        double y = state.y_;
        double angle = state.theta_; // Direction of the robot

        // Calculate half dimensions
        double half_width = robot_width / 2.0;
        double half_length = robot_length / 2.0;

        // Compute the four corners of the bounding box
        std::vector<std::pair<double, double> > corners = {
            {
                x - half_length * std::cos(angle) + half_width * std::sin(angle),
                y - half_length * std::sin(angle) - half_width * std::cos(angle)
            },

            {
                x - half_length * std::cos(angle) - half_width * std::sin(angle),
                y - half_length * std::sin(angle) + half_width * std::cos(angle)
            },

            {
                x + half_length * std::cos(angle) - half_width * std::sin(angle),
                y + half_length * std::sin(angle) + half_width * std::cos(angle)
            },

            {
                x + half_length * std::cos(angle) + half_width * std::sin(angle),
                y + half_length * std::sin(angle) - half_width * std::cos(angle)
            }
        };

        // Determine bounding box limits
        bbox.x_min = std::min({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        bbox.x_max = std::max({corners[0].first, corners[1].first, corners[2].first, corners[3].first});
        bbox.y_min = std::min({corners[0].second, corners[1].second, corners[2].second, corners[3].second});
        bbox.y_max = std::max({corners[0].second, corners[1].second, corners[2].second, corners[3].second});

        return bbox;
    }

    DDPDWAPlanner::RobotBox::RobotBox() : x_max(0.0), x_min(0.0), y_max(0.0), y_min(0.0) {
    }

    DDPDWAPlanner::RobotBox::RobotBox(double x_min_, double x_max_, double y_min_, double y_max_)
        : x_max(x_max_), x_min(x_min_), y_min(y_min_), y_max(y_max_) {
    }

    DDPDWAPlanner::Cost DDPDWAPlanner::evaluate_trajectory(
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

    DDPDWAPlanner::Cost DDPDWAPlanner::evaluate_trajectory(std::vector<PoseState> &trajectory,
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

    DDPDWAPlanner::Cost::Cost() : obs_cost_(0.0), to_goal_cost_(0.0), speed_cost_(0.0), path_cost_(0.0),
                                  ori_cost_(0.0), aw_cost_(0.0), total_cost_(0.0) {
    }

    DDPDWAPlanner::Cost::Cost(
        const double obs_cost, const double to_goal_cost, const double speed_cost, const double path_cost,
        const double ori_cost, const double aw_cost, const double total_cost)
        : obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), speed_cost_(speed_cost), path_cost_(path_cost),
          ori_cost_(ori_cost), aw_cost_(aw_cost), total_cost_(total_cost) {
    }

    void DDPDWAPlanner::Cost::show() const {
        ROS_INFO_STREAM("Cost: " << total_cost_);
        ROS_INFO_STREAM("\tObs cost: " << obs_cost_);
        ROS_INFO_STREAM("\tGoal cost: " << to_goal_cost_);
        ROS_INFO_STREAM("\tSpeed cost: " << speed_cost_);
        ROS_INFO_STREAM("\tPath cost: " << path_cost_);
        ROS_INFO_STREAM("\tOri cost: " << ori_cost_);
    }

    void DDPDWAPlanner::Cost::calc_total_cost() {
        total_cost_ = obs_cost_ + to_goal_cost_ + speed_cost_ + path_cost_ + ori_cost_;
    }

    void DDPDWAPlanner::Window::show() const {
        ROS_INFO_STREAM("Window:");
        ROS_INFO_STREAM("\tVelocity:");
        ROS_INFO_STREAM("\t\tmax: " << max_velocity_);
        ROS_INFO_STREAM("\t\tmin: " << min_velocity_);
        ROS_INFO_STREAM("\tYawrate:");
        ROS_INFO_STREAM("\t\tmax: " << max_angular_velocity_);
        ROS_INFO_STREAM("\t\tmin: " << min_angular_velocity_);
    }

    DDPDWAPlanner::Window::Window() : min_velocity_(0.0), max_velocity_(0.0), min_angular_velocity_(0.0),
                                      max_angular_velocity_(0.0) {
    }

    DDPDWAPlanner::Window DDPDWAPlanner::calc_dynamic_window(PoseState &state, double dt) {
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
