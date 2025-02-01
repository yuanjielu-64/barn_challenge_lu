#include "Planners/DWAPlanner.hpp"

#include <iomanip>
#include <SceneAndSimulation/Robot.hpp>

namespace Antipatrea {
    bool DWAPlanner::Solve(const int nrIters, double dt, bool &canBeSolved) {

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

    bool DWAPlanner::handleNoMapPlanning(geometry_msgs::Twist &cmd_vel) {

        if (robot->setting(Robot_config::NO_ANY_RECEIVED, 2) == false)
          return false;

        normalParameters(*robot);

        const double angle_to_goal = calculateTheta(parent, &robot->getGlobalGoalCfg()[0]);

        double angular = std::clamp(angle_to_goal, -1.0, 1.0);
        angular = (angular > 0) ? std::max(angular, 0.1) : std::min(angular, -0.1);

        publishCommand(cmd_vel, GetSimulator()->GetMaxVelocity(), angular);
        return true;
    }

    bool DWAPlanner::handleNormalSpeedPlanning(geometry_msgs::Twist &cmd_vel,
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

//        if (result == false) {
//            robot->setRobotState(Robot_config::BRAKE_PLANNING);
//            publishCommand(cmd_vel, robot->getPoseState().velocity_, robot->getPoseState().angular_velocity_);
//        } else
//            publishCommand(cmd_vel, best_traj.first.front().velocity_, best_traj.first.front().angular_velocity_);

        return true;
    }

    bool DWAPlanner::handleLowSpeedPlanning(geometry_msgs::Twist &cmd_vel,
                                            std::pair<std::vector<PoseState>, bool> &best_traj, double dt) {

        if (!robot->setting(Robot_config::ONLY_LASER_RECEIVED, 1))
            return false;

        lowSpeedParameters(*robot);

        auto result = dwa_planning(parent, parent_odom, best_traj, dt);

        robot->viewTrajectories(best_traj.first, nr_steps_, 0.0, timeInterval);

        if (!result) {
            robot->setRobotState(Robot_config::BRAKE_PLANNING);
            publishCommand(cmd_vel, 0, 0);
        } else
            publishCommand(cmd_vel, best_traj.first.front().velocity_, best_traj.first.front().angular_velocity_);

        return true;
    }

    bool DWAPlanner::handleAbnormalPlaning(geometry_msgs::Twist &cmd_vel,
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

    void DWAPlanner::publishCommand(geometry_msgs::Twist &cmd_vel, double linear, double angular) {
        cmd_vel.linear.x = linear;
        cmd_vel.angular.z = angular;
        robot->Control().publish(cmd_vel);
    }

    double DWAPlanner::recover(
        PoseState &state, PoseState &state_odom,
        std::pair<std::vector<PoseState>, bool> &best_traj, bool &results) {
        const double angularVelocity_resolution = std::max(2 * M_PI / (w_steps_ - 1), DBL_EPSILON);

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
        }

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


        return best_theta;
    }

    bool DWAPlanner::dwa_planning(PoseState &state, PoseState &state_odom,
                                  std::pair<std::vector<PoseState>, bool> &best_traj, double time) {
        Timer::Clock d_t;
        Timer::Start(d_t);

        Cost min_cost(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1e6);

        Window dw = calc_dynamic_window(state);

        best_traj.first.reserve(nr_steps_);

        const double velocity_resolution =
                std::max((dw.max_velocity_ - dw.min_velocity_) / (v_steps_ - 1),
                         DBL_EPSILON);
        const double angularVelocity_resolution =
                std::max(
                    (dw.max_angular_velocity_ - dw.min_angular_velocity_) / (w_steps_ - 1),
                    DBL_EPSILON);

        //Logger::m_out << "dw.max_angular_velocity_ " << dw.max_angular_velocity_ << "  dw.min_angular_velocity_ "  << dw.min_angular_velocity_ << std::endl;

        int num_threads = 18;

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
        //Logger::m_out << "multi_thread_2 " << Timer::Elapsed(d_t) << std::endl;

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

    void DWAPlanner::process_segment(int thread_id, int start, int end, PoseState &state,
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

                trajectories = generateTrajectory(state, state_odom, v, w);

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


    bool DWAPlanner::hasRotateFirst(PoseState &state, PoseState &state_odom,
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
    DWAPlanner::generateTrajectory(PoseState &state, PoseState &state_odom,
                                   double angular_velocity) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;

        n = 0.0;
        for (int i = 0; i < nr_steps_; ++i) {
            motion(state_, 0.0000001, angular_velocity);
            trajectory.first[i] = state_;
            motion(state_odom, 0.0000001, angular_velocity);
            trajectory.second[i] = state_odom_;
            //n++;
        }

        return trajectory;
    }

    std::pair<std::vector<PoseState>, std::vector<PoseState> >
    DWAPlanner::generateTrajectory(PoseState &state, PoseState &state_odom, const double v,
                                   const double w) {
        std::pair<std::vector<PoseState>, std::vector<PoseState> > trajectory;
        trajectory.first.resize(nr_steps_);
        trajectory.second.resize(nr_steps_);
        PoseState state_ = state;
        PoseState state_odom_ = state_odom;
        n = 0.0;
        for (int i = 0; i < nr_steps_; i++) {
            motion(state_, v + 0.00001, w);
            trajectory.first[i] = state_;
            motion(state_odom_, v + 0.00001, w);
            trajectory.second[i] = state_odom_;
            //n++;
        }

        return trajectory;
    }

    void DWAPlanner::motion(PoseState &state, const double velocity, const double angular_velocity) {
        double t = dt * pow(2.0, n / 2);

        state.theta_ += angular_velocity * t;
        state.x_ += velocity * cos(state.theta_) * t;
        state.y_ += velocity * sin(state.theta_) * t;
        state.velocity_ = velocity;
        state.angular_velocity_ = angular_velocity;

        state.theta_ = normalizeAngle(state.theta_);
    }

    void DWAPlanner::normalize_costs(std::vector<DWAPlanner::Cost> &costs) {
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

    DWAPlanner::Cost DWAPlanner::evaluate_trajectory(
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

    DWAPlanner::Cost DWAPlanner::evaluate_trajectory(std::vector<PoseState> &trajectory,
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

    double DWAPlanner::calculateTheta(const PoseState &state, const double *y) {
        double deltaX = y[0] - state.x_;
        double deltaY = y[1] - state.y_;
        double theta = atan2(deltaY, deltaX);

        double normalizedTheta = normalizeAngle(state.theta_);

        return normalizeAngle(theta - normalizedTheta);
    }

    double DWAPlanner::normalizeAngle(double a) {
        a = fmod(a + M_PI, 2 * M_PI);
        if (a <= 0)
            a += 2 * M_PI;

        return a - M_PI;
    }

    double DWAPlanner::calc_dist_to_path(const std::vector<double> &state) {
        auto edge_point1 = local_paths.front();
        auto edge_point2 = local_paths.back();

        const double a = edge_point2[STATE_Y] - edge_point1[STATE_Y];
        const double b = -(edge_point2[STATE_X] - edge_point1[STATE_X]);
        const double c = -a * edge_point1[STATE_Y] - b * edge_point1[STATE_Y];

        return std::round(fabs(a * state[STATE_X] + b * state[STATE_Y] + c) / (hypot(a, b) + DBL_EPSILON) * 1000) /
               1000;
    }

    bool DWAPlanner::collisionCheck(std::vector<PoseState> &traj) {
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

    double DWAPlanner::calc_obs_cost(const std::vector<PoseState> &traj) {
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

    double DWAPlanner::calculateDistanceToCarEdge(
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

    double DWAPlanner::calc_speed_cost(const std::vector<PoseState> &traj) {
        if (!use_speed_cost_)
            return 0.0;

        const Window dw = calc_dynamic_window(parent);

        return dw.max_velocity_ - traj.front().velocity_;
    }

    DWAPlanner::RobotBox DWAPlanner::calculateMovingBoundingBox(const PoseState &state1,
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

    DWAPlanner::RobotBox::RobotBox() : x_max(0.0), x_min(0.0), y_max(0.0), y_min(0.0) {
    }

    DWAPlanner::RobotBox::RobotBox(double x_min_, double x_max_, double y_min_, double y_max_)
        : x_max(x_max_), x_min(x_min_), y_min(y_min_), y_max(y_max_) {
    }

    DWAPlanner::Cost::Cost() : obs_cost_(0.0), to_goal_cost_(0.0), speed_cost_(0.0), path_cost_(0.0),
                               ori_cost_(0.0), aw_cost_(0.0), total_cost_(0.0) {
    }

    DWAPlanner::Cost::Cost(
        const double obs_cost, const double to_goal_cost, const double speed_cost, const double path_cost,
        const double ori_cost, const double aw_cost, const double total_cost)
        : obs_cost_(obs_cost), to_goal_cost_(to_goal_cost), speed_cost_(speed_cost), path_cost_(path_cost),
          ori_cost_(ori_cost), aw_cost_(aw_cost), total_cost_(total_cost) {
    }

    void DWAPlanner::Cost::show() const {
        ROS_INFO_STREAM("Cost: " << total_cost_);
        ROS_INFO_STREAM("\tObs cost: " << obs_cost_);
        ROS_INFO_STREAM("\tGoal cost: " << to_goal_cost_);
        ROS_INFO_STREAM("\tSpeed cost: " << speed_cost_);
        ROS_INFO_STREAM("\tPath cost: " << path_cost_);
        ROS_INFO_STREAM("\tOri cost: " << ori_cost_);
    }

    void DWAPlanner::Cost::calc_total_cost() {
        total_cost_ = obs_cost_ + to_goal_cost_ + speed_cost_ + path_cost_ + ori_cost_;
    }

    void DWAPlanner::Window::show() const {
        ROS_INFO_STREAM("Window:");
        ROS_INFO_STREAM("\tVelocity:");
        ROS_INFO_STREAM("\t\tmax: " << max_velocity_);
        ROS_INFO_STREAM("\t\tmin: " << min_velocity_);
        ROS_INFO_STREAM("\tYawrate:");
        ROS_INFO_STREAM("\t\tmax: " << max_angular_velocity_);
        ROS_INFO_STREAM("\t\tmin: " << min_angular_velocity_);
    }

    DWAPlanner::Window::Window() : min_velocity_(0.0), max_velocity_(0.0), min_angular_velocity_(0.0),
                                   max_angular_velocity_(0.0) {
    }

    DWAPlanner::Window DWAPlanner::calc_dynamic_window(PoseState &state) {
        auto sim = GetSimulator();
        Window window;
        double dt = robot->dt;
        std::vector<double> info = robot->getSize();

        window.min_velocity_ = std::max((parent.velocity_ + sim->GetMinAcceleration() * dt),
                                        info[2]);
        window.max_velocity_ = std::min((parent.velocity_ + sim->GetMaxAcceleration() * dt),
                                        info[3]);

        window.min_angular_velocity_ = std::max(
            (parent.angular_velocity_ + sim->GetMinAngularAcceleration() * dt), info[4]);
        window.max_angular_velocity_ = std::min(
            (parent.angular_velocity_ + sim->GetMaxAngularAcceleration() * dt), info[5]);

        return window;
    }
}
