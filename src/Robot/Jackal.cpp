#include "Jackal.hpp"
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <utility>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <ctime>
#include <message_filters/sync_policies/approximate_time.h>
#include <filesystem>
#include "Programs/Setup.hpp"
#include <cmath>
#include <nav_msgs/GetPlan.h>

#include "std_srvs/Empty.h"
#include <algorithm>
#include "utility.hpp"

void Robot_config::robotStatusCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    double q1 = msg->pose.pose.orientation.x;
    double q2 = msg->pose.pose.orientation.y;
    double q3 = msg->pose.pose.orientation.z;
    double q0 = msg->pose.pose.orientation.w;

    robot_state.x_ = msg->pose.pose.position.x;
    robot_state.y_ = msg->pose.pose.position.y;
    robot_state.theta_ = atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3));
    robot_state.velocity_ = msg->twist.twist.linear.x;
    robot_state.angular_velocity_ = msg->twist.twist.angular.z;

    robot_state.valid_ = true;
}

void Robot_config::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    laserData.clear();
    laserDataDistance.clear();

    laserData.reserve(msg->ranges.size());
    laserDataDistance.reserve(msg->ranges.size());

    double angle = msg->angle_min;

    std::vector<double> last_valid_point = {INFINITY, INFINITY};
    front_obs = INFINITY;

    for (const auto &range: msg->ranges) {
        if (range > msg->range_min && range < msg->range_max && range <= 2 * v + 1) {
            double laser_x = range * cos(angle);
            double laser_y = range * sin(angle);

            if (last_valid_point[0] != INFINITY) {
                double dx = laser_x - last_valid_point[0];
                double dy = laser_y - last_valid_point[1];
                double distance = sqrt(dx * dx + dy * dy);

                if (distance < 0.01) {
                    angle += msg->angle_increment;
                    continue;
                }
            }

            laserData.emplace_back(std::vector<double>{laser_x, laser_y});
            laserDataDistance.emplace_back(range);

            last_valid_point = {laser_x, laser_y};

            if (angle >= -M_PI/4 && angle <= M_PI/4) {
                front_obs = std::min(front_obs, static_cast<double>(range));
            }
        }

        angle += msg->angle_increment;
    }

    front_obs = front_obs - 0.33;
}

void Robot_config::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    costmapData.clear();

    if (getRobotState() == LOW_SPEED_PLANNING || getRobotState() == NORMAL_PLANNING) {
        return;
    }

    const int width = msg->info.width;
    const int height = msg->info.height;
    const double resolution = msg->info.resolution;

    const geometry_msgs::Pose origin = msg->info.origin;

    const PoseState &robotPose = getPoseState();

    latter_obs = INFINITY;

    if (robotPose.valid_) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                int index = x + y * width;
                int8_t value = msg->data[index];

                if (value >= 0 && value != 0) {
                    double obs_x = origin.position.x + x * resolution;
                    double obs_y = origin.position.y + y * resolution;

                    std::vector<double> lg = transform_lg(
                        obs_x,
                        obs_y,
                        robot_state.x_, robot_state.y_, robot_state.theta_);

                    costmapData.push_back(lg);

                    double dx = obs_x - robotPose.x_;
                    double dy = obs_y - robotPose.y_;
                    double distance = std::sqrt(dx * dx + dy * dy);

                    double angle = std::atan2(dy, dx) - robotPose.theta_;
                    angle = normalizeAngle(angle);

                    if (angle >= M_PI - M_PI_4 && angle <= M_PI + M_PI_4)
                        latter_obs = std::min(latter_obs, distance);
                }
            }
        }
    }
}

void Robot_config::globalPathCallback(const nav_msgs::Path::ConstPtr &msg) {
    getGoal = false;
    local_goal.clear();
    local_paths.clear();
    local_paths_odom.clear();
    std::vector<double> goals;
    goals.reserve(2);

    std::vector<std::pair<double, double> > path_points;

    // std::vector<double> xhat, yhat;
    // for (const auto &pose: msg->poses) {
    //     xhat.push_back(pose.pose.position.x);
    //     yhat.push_back(pose.pose.position.y);
    // }

    std::vector<double> X, Y;
    for (const auto& pose : msg->poses) {
        X.push_back(pose.pose.position.x);
        Y.push_back(pose.pose.position.y);
    }

    std::vector<double> xhat = savgolFilter(X, 9, 2);
    std::vector<double> yhat = savgolFilter(Y, 9, 2);

    if (global_goal_odom.empty()) {
        global_goal_odom = {0, 15};
        setRobotState(NORMAL_PLANNING);
    }

    std::vector<double> lg = transform_lg(global_goal_odom[0],
                                          global_goal_odom[1],
                                          robot_state.x_,
                                          robot_state.y_,
                                          robot_state.theta_);

    global_goal = lg;
    goals = {global_goal_odom[0], global_goal_odom[1]};

    std::vector<double> last_point = {INFINITY, INFINITY};

    bool flag = false;
    double thresholdSq = 0;
    double length = 0;

    for (size_t i = 1; i < xhat.size(); ++i) {
        if (last_point[0] != INFINITY) {
            double dx = xhat[i] - last_point[0];
            double dy = yhat[i] - last_point[1];
            double distance = sqrt(dx * dx + dy * dy);

            if (distance >= 0.1) {
                lg = transform_lg(xhat[i], yhat[i], robot_state.x_, robot_state.y_, robot_state.theta_);
                local_paths.emplace_back(std::vector<double> {lg[0], lg[1]});
                local_paths_odom.emplace_back(std::vector<double> {xhat[i], yhat[i]});
                last_point = {xhat[i], yhat[i]};
            }
        }else {
            last_point = {xhat[i], yhat[i]};
        }

        double dist = l2_distance(xhat[i], yhat[i], xhat[i - 1],yhat[i - 1]);

        length += dist;

        // double distance = l2_distance(robot_state.x_, robot_state.y_, xhat[i], yhat[i]);

        if (getAlgorithm() == DWA || getAlgorithm() == DDPDWA) {
            v = 1.5;
            thresholdSq = 2 * v + 1;

            if (length >= thresholdSq && flag == false) {
                lg = transform_lg(xhat[i], yhat[i], robot_state.x_, robot_state.y_, robot_state.theta_);
                setLocalGoal(lg, xhat[i], yhat[i]);
                flag = true;
                break;
            }
        } else if (getAlgorithm() == LuPlanner || getAlgorithm() == DDPLuPlanner) {
            v = 2;
            thresholdSq = 2 * v + 1;

            if (length >= thresholdSq && flag == false) {
                lg = transform_lg(xhat[i], yhat[i], robot_state.x_, robot_state.y_, robot_state.theta_);
                setLocalGoal(lg, xhat[i], yhat[i]);
                flag = true;
                break;
            }
        } else {
            if (getRobotState() == NORMAL_PLANNING) {
                v = 1.5;
                thresholdSq = 2 * v + 1;
                if (length >= thresholdSq && flag == false) {
                    lg = transform_lg(xhat[i], yhat[i], robot_state.x_, robot_state.y_, robot_state.theta_);
                    setLocalGoal(lg, xhat[i], yhat[i]);
                    flag = true;
                    break;
                }

                // if (distance >= 0.2) {
                //     lg = transform_lg(xhat[i], yhat[i], robot_state.x_, robot_state.y_, robot_state.theta_);
                //     double probabilities = std::exp(-0.05 * n);
                //
                //     if (Antipatrea::RandomUniformReal() < probabilities) {
                //         local_goal_point.push_back(lg);
                //         local_goal_point_odom.push_back({xhat[i], yhat[i]});
                //     }
                //
                //     if (probabilities <= 0.2) {
                //         setLocalGoal(lg, xhat[i], yhat[i]);
                //         flag = true;
                //         break;
                //     }
                //
                //     ++n;
                // }
            } else if (getRobotState() == LOW_SPEED_PLANNING) {
                v = 0.75;

                thresholdSq = 2 * v + 0.25;

                if (length >= thresholdSq && flag == false) {
                    lg = transform_lg(xhat[i], yhat[i], robot_state.x_, robot_state.y_, robot_state.theta_);
                    setLocalGoal(lg, xhat[i], yhat[i]);
                    flag = true;
                    break;
                }
                // if (distance >= 0.2) {
                //     lg = transform_lg(xhat[i], yhat[i], robot_state.x_, robot_state.y_, robot_state.theta_);
                //     double probabilities = std::exp(-0.05 * n);
                //
                //     if (Antipatrea::RandomUniformReal() < probabilities) {
                //         local_goal_point.push_back(lg);
                //         local_goal_point_odom.push_back({xhat[i], yhat[i]});
                //     }
                //
                //     if (probabilities <= 0.3) {
                //         setLocalGoal(lg, xhat[i], yhat[i]);
                //         flag = true;
                //         break;
                //     }
                //
                //     ++n;
                // }
            } else if (getRobotState() == NO_MAP_PLANNING){
                v = 1;
                thresholdSq = v;
                if (length >= thresholdSq && flag == false) {
                    lg = transform_lg(xhat[i], yhat[i], robot_state.x_, robot_state.y_, robot_state.theta_);
                    setLocalGoal(lg, xhat[i], yhat[i]);
                    flag = true;
                    break;
                }
            }else {
                v = 0.8;
                thresholdSq = v;
                if (length >= thresholdSq && flag == false) {
                    lg = transform_lg(xhat[i], yhat[i], robot_state.x_, robot_state.y_, robot_state.theta_);
                    setLocalGoal(lg, xhat[i], yhat[i]);
                    flag = true;
                    break;
                }
            }
        }
    }

    if (!flag) {
        lg = transform_lg(global_goal[0], global_goal[1], robot_state.x_, robot_state.y_, robot_state.theta_);
        setLocalGoal(lg, global_goal_odom[0], global_goal_odom[1]);
    }

    view_Goal(goals, local_goal_odom);

    update_angular_velocity();

    getGoal = true;
}

void Robot_config::update_angular_velocity() {
    if (getAlgorithm() == DWA || getAlgorithm() == DDPLuPlanner) {
        if (getRobotState() == NORMAL_PLANNING)
            w = 2;
        else if (getRobotState() == LOW_SPEED_PLANNING)
            w = 1;
    } else {
        if (getRobotState() == NORMAL_PLANNING) {
            if (abs(getPoseState().angular_velocity_) <= 1 && abs(getPoseState().velocity_) <=  1 * v / 3)
                w = 2;
            else if ((abs(getPoseState().angular_velocity_) <= 2 && abs(getPoseState().angular_velocity_) > 1 * v / 3) || (
                         abs(getPoseState().velocity_) > 1 && abs(getPoseState().velocity_) <= 2 * v / 3))
                w = 1.5;
            else
                w = 1.0;
        } else if (getRobotState() == LOW_SPEED_PLANNING) {
            if (abs(getPoseState().angular_velocity_) <= 1 && abs(getPoseState().velocity_) <= 1 * v / 3)
                w = 2.5;
            else if ((abs(getPoseState().angular_velocity_) <= 2 && abs(getPoseState().angular_velocity_) > 1 * v / 3) || (
                         abs(getPoseState().velocity_) > 0.2 && abs(getPoseState().velocity_) <= 2 * v / 3))
                w = 2;
            else
                w = 1.5;
        }
    }
}

void Robot_config::goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg) {
    ROS_INFO("Received goal to move to position x: %f, y: %f", msg->goal.target_pose.pose.position.x,
             msg->goal.target_pose.pose.position.y);

    global_goal_odom.clear();
    global_goal_odom = {msg->goal.target_pose.pose.position.x, msg->goal.target_pose.pose.position.y};
    setRobotState(NORMAL_PLANNING);
}

void Robot_config::velocityCallback(const geometry_msgs::Twist &cmd_vel) {

    if (getAlgorithm() == DWA || getAlgorithm() == DDPDWA || getAlgorithm() == LuPlanner || getAlgorithm() == DDPLuPlanner)
       return;

    double linear_speed = fabs(getPoseState().velocity_);

    double LOW_SPEED_THRESHOLD = v * 0.8 + 0.05;
    double LOW_SPEED_HYSTERESIS = 0.05;
    double HIGH_SPEED_THRESHOLD = v * 0.5 + 0.1;

    if (getRobotState() == NORMAL_PLANNING) {
        low_speed_timer_active = false;
        break_speed_timer_active = false;
        is_stopped = false;

        if (linear_speed < HIGH_SPEED_THRESHOLD) {
            if (!high_speed_timer_active) {
                high_speed_start_time = ros::Time::now();
                high_speed_timer_active = true;
            }else if ((ros::Time::now() - high_speed_start_time).toSec() >= 0.5) {
                ROS_INFO("The robot is back to LOW_SPEED_PLANNING after 0.8s in high speed.");
                setRobotState(LOW_SPEED_PLANNING);
                high_speed_timer_active = false;
            }
        }else {
            high_speed_timer_active = false;
        }
    } else if (getRobotState() == LOW_SPEED_PLANNING) {
        high_speed_timer_active = false;

        if (linear_speed >= LOW_SPEED_THRESHOLD + LOW_SPEED_HYSTERESIS) {
            if (!low_speed_timer_active) {
                low_speed_start_time = ros::Time::now();
                low_speed_timer_active = true;
            } else if ((ros::Time::now() - low_speed_start_time).toSec() >= 0.5) {
                ROS_INFO("The robot is back to NORMAL_PLANNING after 0.8s in low speed.");
                setRobotState(NORMAL_PLANNING);
                low_speed_timer_active = false;
            }
        } else {
            low_speed_timer_active = false;
        }

        static constexpr double BRAKE_WAIT_TIME = 0.5; // 触发 BRAKE_PLANNING 的时间
        if (linear_speed < MIN_SPEED) {
            if (!break_speed_timer_active) {
                stopped_time = ros::Time::now();
                break_speed_timer_active = true;
            } else if ((ros::Time::now() - stopped_time).toSec() > BRAKE_WAIT_TIME * STOPPED_TIME_THRESHOLD) {
                ROS_INFO("The robot needs to brake after 0.5 second in low speed");
                setRobotState(BRAKE_PLANNING);
                break_speed_timer_active = false;
            }
        } else {
            break_speed_timer_active = false;
        }
    } else {  // recover
        static constexpr double RECOVERY_WAIT_TIME = 8.0;
        if (linear_speed < MIN_SPEED) {
            if (!is_stopped) {
                stopped_time = ros::Time::now();
                is_stopped = true;
            } else if ((ros::Time::now() - stopped_time).toSec() > RECOVERY_WAIT_TIME * STOPPED_TIME_THRESHOLD) {
                ROS_INFO("Cleaning the costmap: Triggering recovery.");
                triggerRecovery();
                is_stopped = false;
            }
        } else {
            is_stopped = false;
        }
    }
}

void Robot_config::triggerRecovery() {
    std_srvs::Empty srv;
    if (clear_costmaps_clt.call(srv)) {
        ROS_INFO("Recovery behavior triggered.");
        local_goal_odom.clear();
    } else
        ROS_ERROR("Failed to call clear_costmaps service.");

    resetStoppedStatus();
}

void Robot_config::resetStoppedStatus() {
    if (is_stopped) {
        is_stopped = false;
        ROS_INFO("Robot resumed moving.");
    }
}

bool Robot_config::setup() {
    if (global_goal_odom.empty()) {
        global_goal_odom = {0, 10};
        setRobotState(NORMAL_PLANNING);
    }

    if (getRobotState() != INITIALIZING && getPoseState().valid_ && getGoal) {
        return true;
    }

    return false;
}

std::vector<double> Robot_config::getSize() const {
    std::vector<double> info;
    //size, velocity, angular velocity
    if (currentMap == ONLY_COSTMAP_RECEIVED && getRobotState() == RECOVERY)
        info = {0.02, 0.02, 0, v, -w, w};
    else if (currentMap == ONLY_COSTMAP_RECEIVED && getRobotState() == BACKWARD)
        info = {0.02, 0.02, -2.0, 0.0, -2, 2};
    else if (currentMap == ONLY_COSTMAP_RECEIVED && getRobotState() == FORWARD)
        info = {0.02, 0.02, 0, 2.0, -2, 2};
    else if (currentMap == ONLY_LASER_RECEIVED && getRobotState() == NORMAL_PLANNING)
        info = {0.508, 0.430, 0, v, -w, w};
    else if (currentMap == ONLY_LASER_RECEIVED && getRobotState() == LOW_SPEED_PLANNING)
        info = {0.508, 0.430, 0, v, -w, w};
    else
        info = {0.508, 0.430, 0, v, -w, w};

    return info;
}

void Robot_config::generateMap(Map newMap) {
    map.clear();

    if (newMap == ONLY_COSTMAP_RECEIVED) {
        map = costmapData;

        if (map.empty()) {
            map = laserData;
            currentMap = ONLY_LASER_RECEIVED;
        } else
            currentMap = ONLY_COSTMAP_RECEIVED;
    }

    if (newMap == ONLY_LASER_RECEIVED) {
        map = laserData;
        if (map.empty()) {
            map = costmapData;
            currentMap = ONLY_COSTMAP_RECEIVED;
        } else
            currentMap = ONLY_LASER_RECEIVED;
    }

    if (laserData.empty()) {
        setRobotState(NO_MAP_PLANNING);
        currentMap = NO_ANY_RECEIVED;
    }
}

Robot_config::PoseState::PoseState(): x_(0.0), y_(0.0), theta_(0.0), velocity_(0.0), angular_velocity_(0.0),
                                      valid_(false) {
}

Robot_config::PoseState::PoseState(double x, double y, double theta, double velocity, double angular_velocity,
                                   bool valid) : x_(x), y_(y), theta_(theta), velocity_(velocity),
                                                 angular_velocity_(angular_velocity), valid_(valid) {
}

Robot_config::RobotBox::RobotBox() : x_max(0.0), x_min(0.0), y_max(0.0), y_min(0.0) {
}

Robot_config::RobotBox::RobotBox(double x_min_, double x_max_, double y_min_, double y_max_)
    : x_max(x_max_), x_min(x_min_), y_min(y_min_), y_max(y_max_) {
}

void Robot_config::setLaserSize() {
    double min_x = FLT_MAX;
    double min_y = FLT_MAX;
    double max_x = FLT_MIN;
    double max_y = FLT_MIN;

    // for (const auto &data: map) {
    //     if (data[0] < min_x)
    //         min_x = data[0] < -5 ? -5 : data[0];
    //     if (data[1] < min_y)
    //         min_y = data[1] < -5 ? -5 : data[1];
    //     if (data[0] > max_x)
    //         max_x = data[0] > 5 ? 5 : data[0];
    //     if (data[1] > max_y)
    //         max_y = data[1] > 5 ? 5 : data[1];
    // }

    // grid_min = {std::min(getGlobalGoalCfg()[0] - 0.5, min_x), std::min(getGlobalGoalCfg()[1] - 0.5, min_y)};
    // grid_max = {std::max(getGlobalGoalCfg()[0] + 0.5, max_x), std::max(getGlobalGoalCfg()[1] + 0.5, max_y)};
    grid_min = {-10, -10};
    grid_max = {10, 10};
}

Robot_config::Robot_config()
    : algorithm(DWA),
      currentState(INITIALIZING),
      currentMap(ONLY_LASER_RECEIVED),
      getGoal(false),
      canBeSolved(true),
      rotating_angle(0.0),
      los(2),
      latter_obs(INFINITY),
      front_obs(INFINITY),
      dt(0.05),
      tfListener(tfBuffer) {
    global_goal.reserve(2);
    local_goal.reserve(2);
    local_goal_odom.reserve(2);

    recover_times = 0;

    local_goal = {0, 0};
    robot_state = {0, 0, 0, 0, 0, false};
    actions = {{0, 0}};
    grid_min = {-10, -10};
    grid_max = {10, 10};

    robot_pose_sub = nh.subscribe("/odometry/filtered", 10, &Robot_config::robotStatusCallback, this);
    laser_scan_sub = nh.subscribe("/front/scan", 10, &Robot_config::laserScanCallback, this);
    goal_sub = nh.subscribe("/move_base/goal", 10, &Robot_config::goalCallback, this);
    costmap_update_sub = nh.subscribe("/move_base/local_costmap/costmap", 10, &Robot_config::costmapCallback, this);
    velocity_sub = nh.subscribe("/cmd_vel", 10, &Robot_config::velocityCallback, this);
    global_path_sub = nh.subscribe<nav_msgs::Path>("/move_base/NavfnROS/plan", 10, &Robot_config::globalPathCallback,
                                                   this);

    trajectory_pub = nh.advertise<nav_msgs::Path>("trajectory", 10);
    global_path_pub = nh.advertise<nav_msgs::Path>("global_path", 10);
    local_goal_pub = nh.advertise<visualization_msgs::Marker>("local_goal", 1);
    global_goal_pub = nh.advertise<visualization_msgs::Marker>("global_goal", 1);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    costmap = new costmap_2d::Costmap2DROS("local_costmap", tfBuffer);

    global_path_clt = nh.serviceClient<nav_msgs::GetPlan>("/move_base/NavfnROS/make_plan");

    clear_costmaps_clt = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
}

void Robot_config::view_Goal(std::vector<double> &goal, std::vector<double> &goal1) const {
    visualization_msgs::Marker marker;

    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now();

    marker.ns = "point_marker";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    geometry_msgs::Point p;
    p.x = goal[0];
    p.y = goal[1];
    p.z = 0.0;

    marker.points.push_back(p);

    global_goal_pub.publish(marker);

    marker.points.clear();
    marker.id = 1;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    geometry_msgs::Point p1;
    p1.x = goal1[0];
    p1.y = goal1[1];
    p1.z = 0.0;

    marker.points.push_back(p1);

    local_goal_pub.publish(marker);
}

void Robot_config::viewTrajectories(std::vector<PoseState> &trajectories, int nr_steps_, double theta_,
                                    std::vector<double> &t) const {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";

    double x = robot_state.x_;
    double y = robot_state.y_;
    double theta = normalizeAngle(robot_state.theta_ + theta_);

    for (int i = 0; i < nr_steps_; ++i) {
        x = x + trajectories[i].velocity_ * cos(theta) * t[i];
        y = y + trajectories[i].velocity_ * sin(theta) * t[i];
        theta = normalizeAngle(theta + trajectories[i].angular_velocity_ * t[i]);

        geometry_msgs::PoseStamped pose;

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "odom";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        path.poses.push_back(pose);
    }

    trajectory_pub.publish(path);
}

void Robot_config::viewTrajectories(std::vector<PoseState> &trajectories, int nr_steps_, std::vector<double> &t) const {
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";

    for (int i = 0; i < nr_steps_; ++i) {
        double x = trajectories[i].x_;
        double y = trajectories[i].y_;
        double theta = normalizeAngle(trajectories[i].angular_velocity_);

        geometry_msgs::PoseStamped pose;

        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "odom";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0;

        path.poses.push_back(pose);
    }

    trajectory_pub.publish(path);
}
