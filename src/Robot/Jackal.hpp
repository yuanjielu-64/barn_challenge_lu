//
// Created by lu on 7/23/24.
//

#ifndef INC_2024_WS_JACKAL_HPP
#define INC_2024_WS_JACKAL_HPP

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <utility>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d_publisher.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <visualization_msgs/Marker.h>
#include <numeric>
#include "Utils/Stats.hpp"

class Robot_config {
public:
    Robot_config();

    class Obstacle {
    public:
        geometry_msgs::Point center;
        double radius{};
    };

    class PoseState
    {
    public:
        PoseState();

        PoseState(double x, double y, double theta, double velocity, double angular_velocity, bool valid);

        std::vector<double> pose() const {
            return {x_, y_};
        }

        double x_;
        double y_;
        double theta_;
        double velocity_;
        double angular_velocity_;

        bool valid_;

    private:
    };

    class RobotBox {
    public:
        RobotBox();

        RobotBox(double x_min_, double x_max_, double y_min_, double y_max_);

        double x_min, x_max;
        double y_min, y_max;

        std::pair<double, double> front_left;
        std::pair<double, double> front_right;
        std::pair<double, double> rear_left;
        std::pair<double, double> rear_right;
    };

    enum Algorithm {
        DWA,
        DDPDWA,
        LuPlanner,
        DDPLuPlanner,
        DDP,
    };

    enum RobotState {
        INITIALIZING,
        NORMAL_PLANNING,
        LOW_SPEED_PLANNING,
        NO_MAP_PLANNING,
        BRAKE_PLANNING,
        RECOVERY,
        ROTATE_PLANNING,
        BACKWARD,
        FORWARD,
        TEST,
        IDIE
        // INITIALIZING,  // 0
        // PLANNING,      // 1
        // CONTROLLING,   // 2
        // CLEARING,      // 3
        // RECOVERY,      // 4
        // ARRIVED,       // 5
        // ABORTED,       // 6
        // WAITING,       // 7
        // IDLE           // 8
    };

    enum Map {
        ONLY_COSTMAP_RECEIVED,  // 0
        ONLY_LASER_RECEIVED,    // 1
        NO_ANY_RECEIVED         // 2
    };

    void setAlgorithm(Algorithm a) {
        algorithm = a;
    }

    Algorithm getAlgorithm() const {
        return algorithm;
    }

    void setDt(double t) {
        dt = t;
    }

    double getDt() const {
        return dt;
    }

    void setRobotState(RobotState newState) {
        currentState = newState;
    }

    RobotState getRobotState() const {
        return currentState;
    }

    void setMapType(Map newMap) {
        currentMap = newMap;
    }

    Map getMapType() const {
        return currentMap;
    }

    bool setting(Map newMap, double distanceToGoal) {
        los = distanceToGoal;
        generateMap(newMap);
        //setLocalGoal();
        setLaserSize();

        if (getMapType() != newMap)
            return false;

        return true;
    }


    void globalPathCallback(const nav_msgs::Path::ConstPtr& msg);

    void robotStatusCallback(const nav_msgs::Odometry::ConstPtr &msg);

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &msg);

    void velocityCallback(const geometry_msgs::Twist& cmd_vel);

    void costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    void goalCallback(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);

    bool setup();

    void update_angular_velocity();

    void view_Goal(std::vector<double> &goal, std::vector<double> &goal1) const;

    void setLocalGoal(std::vector<double> &lg, double x, double y) {
        local_goal_odom.clear();
        local_goal.clear();
        local_goal.push_back(lg[0]);
        local_goal.push_back(lg[1]);
        local_goal_odom.push_back(x);
        local_goal_odom.push_back(y);
    }

    static RobotBox calculateMovingBoundingBox(const PoseState &state, double robot_width, double robot_length);

    void setLaserSize();

    void generateMap(Map newMap);

    void triggerRecovery();

    void resetStoppedStatus();

    std::vector<std::vector<double>> getDataMap(){
        return map;
    }

    PoseState getPoseState() {
        return robot_state;
    }

    std::vector<double> getLocalGoalCfg() {
        return local_goal;
    }

    std::vector<double> getGlobalGoalCfg() {
        return global_goal;
    }

    std::vector<double> getSize() const;

    std::vector<std::vector<double>> getLaserData() {
        return laserData;
    }

    std::vector<std::vector<double>> getCostmapDataOdom() const {
        return costmapDataOdom;
    }

    double getDistanceToGoal() const {
        return los;
    }

    static double calculateTheta(const PoseState &state, const std::vector<double> &y) {
        double deltaX = y[0] - state.x_;
        double deltaY = y[1] - state.y_;
        double theta = atan2(deltaY, deltaX);

        double normalizedTheta = normalizeAngle(state.theta_);

        return fabs(normalizeAngle(theta - normalizedTheta));
    }

    static double normalizeAngle(double a) {
        a = fmod(a + M_PI, 2 * M_PI);
        if (a < 0)
            a += 2 * M_PI;

        return a - M_PI;
    }

    void viewTrajectories(std::vector<PoseState> &trajectories, int nr_steps_, double theta_, std::vector<double> &t) const;

    void viewTrajectories(std::vector<PoseState> &trajectories, int nr_steps_, std::vector<double> &t) const;

    ros::Publisher Control() {
        return cmd_vel_pub;
    }

    double getVelocity() const {
        return robot_state.velocity_;
    }

    double getAngularVelocity() const {
        return robot_state.angular_velocity_;
    }

    costmap_2d::Costmap2DROS *getCostMap(){
        return costmap;
    }

    double rotating_angle;

    double dt{};
    double latter_obs{};
    double front_obs{};

    Algorithm algorithm;
    Map currentMap;
    RobotState currentState;

    ros::NodeHandle nh;

    bool canBeSolved{};
    bool getGoal{};

    std::vector<std::vector<double>> local_paths;
    std::vector<std::vector<double>> local_paths_odom;

    std::vector<std::vector<double>> actions;
    std::vector<double> grid_min;
    std::vector<double> grid_max;
    std::vector<double> local_goal_odom;
    std::vector<std::vector<double>> local_goal_point;
    std::vector<std::vector<double>> local_goal_point_odom;

    std::vector<double> global_goal_odom;

    std::vector<std::vector<geometry_msgs::Point>> polygons;

    ros::Publisher trajectory_pub;
    ros::Publisher global_path_pub;
    ros::Publisher local_goal_pub;
    ros::Publisher global_goal_pub;

    std::vector<std::vector<double>> costmapDataOdom;
    std::vector<std::vector<double>> costmapData;
    std::vector<double> laserDataDistance;

    double v = 1;
    double w = 1;

    int recover_times = 0;
    int re = 1;

    ros::Time last_cover_exit_time = ros::Time::now();
    int recover_to_low_count = 0;
    double dynamic_recovery_wait_time = 0.5;

protected:

    ros::Subscriber robot_pose_sub;
    ros::Subscriber laser_scan_sub;
    ros::Subscriber dist_to_goal_th_sub_;
    ros::Subscriber costmap_update_sub;
    ros::Subscriber goal_sub;
    ros::Subscriber velocity_sub;
    ros::Subscriber global_path_sub;

    costmap_2d::Costmap2DROS *costmap{};

    ros::ServiceClient clear_costmaps_clt;
    ros::ServiceClient global_path_clt;
    ros::ServiceClient path_clt;
    ros::Publisher cmd_vel_pub;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    std::vector<double> global_goal;
    std::vector<double> local_goal;

    std::vector<double> costmapDataDistance;
    std::vector<double> costmapDataAngle;

    std::vector<std::vector<double>> laserData;

    std::vector<std::vector<double>> map;

    PoseState robot_state;

    double los{};

    bool is_stopped = false;

    ros::Time high_speed_start_time;
    bool high_speed_timer_active = false;

    ros::Time low_speed_start_time;
    bool low_speed_timer_active = false;

    ros::Time break_speed_start_time;
    bool break_speed_timer_active = false;

    ros::Time stopped_time;
    ros::Time last_time;
    const double MIN_SPEED = 0.15;
    const double STOPPED_TIME_THRESHOLD = 1.0;

};

#endif //INC_2024_WS_JACKAL_HPP
