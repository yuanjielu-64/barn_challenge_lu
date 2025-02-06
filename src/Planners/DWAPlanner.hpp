/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */

#ifndef Antipatrea__DWAPlanner_HPP_
#define Antipatrea__DWAPlanner_HPP_

#include "Components/GroupKeyTour.hpp"
#include "Components/GroupSelector.hpp"
#include "Components/TourGenerator.hpp"
#include "Planners/MPTree.hpp"
#include <numeric>
#include <thread>
#include <mutex>
#include <chrono>
#include "Utils/Timer.hpp"
#include "Utils/Stats.hpp"


namespace Antipatrea {
    using PoseState = Robot_config::PoseState;

    class DWAPlanner : public MPTree,
                       public GroupSelectorContainer {
    public:
        DWAPlanner(void) : MPTree(),
                           GroupSelectorContainer() {
        }

        virtual ~DWAPlanner(void) {
        }

        virtual bool Solve(const int nrIters, const double tmax, bool &canBeSolved);

        class Cost {
        public:
            Cost();

            Cost(double obs_cost, double to_goal_cost, double speed_cost, double path_cost, double ori_cost,
                 double aw_cost,
                 double total_cost);

            void show() const;

            void calc_total_cost();

            double obs_cost_;
            double to_goal_cost_;
            double speed_cost_;
            double path_cost_;
            double ori_cost_;
            double aw_cost_;

            double total_cost_;
        };

        class RobotBox {
        public:
            RobotBox();

            RobotBox(double x_min_, double x_max_, double y_min_, double y_max_);

            double x_min, x_max;
            double y_min, y_max;
        };

        class Window {
        public:
            Window();

            void show() const;

            double min_velocity_;
            double max_velocity_;
            double min_angular_velocity_;
            double max_angular_velocity_;
        };

        Robot_config *robot;

    protected:
        virtual GroupKey *NewGroupKey() const {
            return new GroupKeyTour();
        }

        virtual void commonParameters(Robot_config &robot);

        virtual void frontBackParameters(Robot_config &robot);

        virtual void normalParameters(Robot_config &robot);

        virtual void lowSpeedParameters(Robot_config &robot);

        virtual void recoverParameters(Robot_config &robot);

        virtual bool handleNoMapPlanning(geometry_msgs::Twist &cmd_vel);

        virtual bool handleNormalSpeedPlanning(geometry_msgs::Twist &cmd_vel,
                                               std::pair<std::vector<PoseState>, bool> &best_traj, double dt);

        virtual bool handleLowSpeedPlanning(geometry_msgs::Twist &cmd_vel,
                                            std::pair<std::vector<PoseState>, bool> &best_traj, double dt);

        virtual bool handleAbnormalPlaning(geometry_msgs::Twist &cmd_vel,
                                           std::pair<std::vector<PoseState>, bool> &best_traj, double dt);

        virtual void publishCommand(geometry_msgs::Twist &cmd_vel, double linear, double angular);

        virtual bool hasRotateFirst(PoseState &state, PoseState &state_odom, double angle_to_goal);

        virtual double recover(PoseState &state, PoseState &state_odom,
                               std::pair<std::vector<PoseState>, bool> &best_traj, bool &results);

        virtual bool collisionCheck(std::vector<PoseState> &trajectory);

        virtual bool dwa_planning(PoseState &state, PoseState &state_odom,
                                  std::pair<std::vector<PoseState>, bool> &best_traj, double time);

        virtual RobotBox calculateMovingBoundingBox(const PoseState &state1, const PoseState &state2,
                                                    double robot_width, double robot_length);

        virtual bool isBoxIntersectingBox(const RobotBox &bbox1, const std::vector<double> &obs) {
            return !(bbox1.x_max < obs[0] || bbox1.x_min > obs[0] ||
                     bbox1.y_max < obs[1] || bbox1.y_min > obs[1]);
        }

        virtual double calculateDistanceToCarEdge(
            double carX, double carY, double cosTheta, double sinTheta,
            double halfLength, double halfWidth, const std::vector<double> &obs);

        virtual std::pair<std::vector<PoseState>, std::vector<PoseState> > generateTrajectory(
            PoseState &state, PoseState &state_odom, double angular_velocity);

        virtual std::pair<std::vector<PoseState>, std::vector<PoseState> > generateTrajectory(
            PoseState &state, PoseState &state_odom, double v, double w);

        virtual void motion(PoseState &state, double velocity, double angular_velocity);

        virtual void normalize_costs(std::vector<Cost> &costs);

        virtual void process_segment(int thread_id, int start, int end, PoseState &state, PoseState &state_odom,
                                     double velocity_resolution,
                                     double angularVelocity_resolution, Window &dw,
                                     std::vector<Cost> &thread_costs,
                                     std::vector<std::pair<std::vector<PoseState>, std::vector<PoseState> > > &
                                     thread_trajectories);

        virtual double calc_to_goal_cost(const std::vector<PoseState> &traj) {
            if (use_goal_cost_ == false)
                return 0.0;

            return Algebra::PointDistance(2, &traj[traj.size() - 1].pose()[0], &local_goal[0]);
        }

        virtual double calc_speed_cost(const std::vector<PoseState> &traj);

        virtual Cost evaluate_trajectory(std::pair<std::vector<PoseState>, std::vector<PoseState> > &traj, double &dist,
                                         std::vector<double> &last_position);

        virtual Cost evaluate_trajectory(std::vector<PoseState> &traj, double &dist,
                                         std::vector<double> &last_position);

        virtual double calc_obs_cost(const std::vector<PoseState> &traj);

        virtual double calc_ori_cost(const std::vector<PoseState> &traj) {
            if (!use_ori_cost_)
                return 0.0;

            double theta = calculateTheta(traj[traj.size() - 1], &local_goal[0]);

            return fabs(theta);
        }

        virtual double calc_angular_velocity(const std::vector<PoseState> &traj) {
            if (use_angular_cost_) {
                double angular_velocity = std::abs(traj.front().angular_velocity_);
                double angular_velocity_cost = angular_velocity * angular_velocity;
                return angular_velocity_cost;
            }

            return 0.0;
        }

        virtual double calc_path_cost(const std::vector<PoseState> &traj) {
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

        virtual double calc_dist_to_path(const std::vector<double> &state);

        virtual Window calc_dynamic_window(PoseState &state);

        virtual double calculateTheta(const PoseState &x, const double *y);

        virtual double normalizeAngle(double angle);

        bool use_goal_cost_ = false;
        bool use_speed_cost_ = false;
        bool use_path_cost_ = false;
        bool use_ori_cost_ = false;
        bool use_angular_cost_ = false;

        double angle_to_goal_ = M_PI / 2;

        double robot_radius_ = 0.03;
        double distance = 0.0;

        int num_threads = 10;
        double obs_range_ = 4;
        int nr_steps_ = 20;
        int v_steps_ = 20;
        int w_steps_ = 20;
        int state_dims = 5;

        PoseState parent;
        PoseState parent_odom;

        std::vector<std::vector<double> > obstacles;
        std::vector<double> global_goal;
        std::vector<double> local_goal;
        std::vector<double> timeInterval;

        double to_goal_cost_gain_ = 0.8;
        double obs_cost_gain_ = 0.5;
        double speed_cost_gain_ = 0.4;
        double path_cost_gain_ = 0.4;
        double ori_cost_gain_ = 0.3;
        double aw_cost_gain_ = 0.2;

        double minAccelerSpeed;
        double maxAccelerSpeed;
        double minAngularAccelerSpeed;
        double maxAngularAccelerSpeed;

        double dt;
        double n;

        std::mutex mtx;

        std::vector<std::vector<double> > local_paths;
    };

    ClassContainer(DWAPlanner, m_DWAPlanner);
}

#endif
