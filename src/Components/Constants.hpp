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

#ifndef Antipatrea__ComponentsConstants_HPP_
#define Antipatrea__ComponentsConstants_HPP_

#include "Utils/Constants.hpp"

namespace Antipatrea::Constants {
    // scene

    const double SCENE_OBSTACLE_HEIGHT = 0.2;
    const double SCENE_BOUNDARY_HEIGHT = 0.3;
    const double SCENE_BOUNDARY_THICKNESS = 0.04;

    const char KW_Scene[] = "Scene";
    const char KW_Scene2D[] = "Scene2D";
    const char KW_Scene3D[] = "Scene3D";
    const char KW_ObstaclesCollisionMeshFile[] = "ObstaclesCollisionMeshFile";
    const char KW_ObstaclesDrawMeshFile[] = "ObstaclesDrawMeshFile";
    const char KW_ObstaclesPolygonsFile[] = "ObstaclesPolygonsFile";
    const char KW_TerrainMeshFile[] = "TerrainMeshFile";
    const char KW_ObstacleHeight[] = "ObstacleHeight";
    const char KW_Boundaries[] = "Boundaries";
    const char KW_Height[] = "Height";
    const char KW_Thickness[] = "Thickness";
    const char KW_AdjustGrid[] = "AdjustGrid";
    const char KW_VehicleMeshFile[] = "VehicleMeshFile";

    // allocator
    const char KW_Dim[] = "Dim";

    // simulator
    const double SIMULATOR_TIME_STEP = 0.01;
    const double SIMULATOR_MIN_DISTANCE_ONE_STEP = 0.05;
    const double SIMULATOR_MAX_DISTANCE_ONE_STEP = 0.08;
    const double SIMULATOR_DISTANCE_REACHED_STEER_POSITION = 1;
    const double SIMULATOR_STOP_DISTANCE = 0.6;
    const double SIMULATOR_STOP_EXPONENT = 0.3;
    const double SIMULATOR_VELOCITY_SCALE_SAMPLING = 0.95;
    const double SIMULATOR_VELOCITY_SCALE_CONVERSION = 0.7; //0.5
    const double SIMULATOR_VELOCITY_SCALE_MIN = 0.01;
    const double SIMULATOR_DRAW_POSITION_RADIUS = 0.1;
    const double SIMULATOR_SNAKE_SX = 1.75;
    const double SIMULATOR_SNAKE_SY = 1.5;

    const double SIMULATOR_CAR_SX = 1.5;
    const double SIMULATOR_CAR_SY = 1.25;

    const double SIMULATOR_ROBOT_SX = 1.230;
    const double SIMULATOR_ROBOT_SY = 1;

    const double SIMULATOR_BLIMP_SX = 2.0;
    const double SIMULATOR_BLIMP_SY = 2.0;
    const double SIMULATOR_BLIMP_SZ = 2.0;
    const double SIMULATOR_BLIMP_PATH_CFG_WIDTH = 0.15;

    const char KW_Simulator[] = "Simulator";
    const char KW_SimulatorCarTrailers[] = "SimulatorCarTrailers";
    const char KW_SimulatorBlimp[] = "SimulatorBlimp";
    const char KW_SimulatorRobot[] = "SimulatorRobot";
    const char KW_TimeStep[] = "TimeStep";
    const char KW_MinDistanceOneStep[] = "MinDistanceOneStep";
    const char KW_MaxDistanceOneStep[] = "MaxDistanceOneStep";
    const char KW_DistanceReachedSteerPosition[] = "DistanceReachedSteerPosition";
    const char KW_StopDistance[] = "StopDistance";
    const char KW_StopExponent[] = "StopExponent";
    const char KW_VelocityScaleSampling[] = "VelocityScaleSampling";
    const char KW_VelocityScaleConversion[] = "VelocityScaleConversion";
    const char KW_VelocityScaleMin[] = "VelocityScaleMin";
    const char KW_DrawPositionRadius[] = "DrawPositionRadius";

    const char KW_ProjectName[] = "barn_challenge_lu";

    // car
    const char CAR_CHASSIS_MESH_FILE[] = "gdata/models/porsche.obj";
    const char CAR_WHEEL_MESH_FILE[] = "gdata/textures/wheel.mesh.xml";
    const char CAR_TIRE_TEXTURE_FILE[] = "gdata/textures/Tire_tex.png";
    const char BLIMP_MESH_FILE[] = "gdata/models/blimp.off";

    // SimulatorCarTrailers
    const bool CAR_FRONT_WHEEL_DRIVING = true;
    const double CAR_BODY_LENGTH = 1.75;
    const double CAR_BODY_WIDTH = 1.0;
    const double CAR_MIN_STEER_ANGLE = -85 * Constants::DEG2RAD;
    const double CAR_MAX_STEER_ANGLE = 85 * Constants::DEG2RAD;
    const double CAR_MIN_VELOCITY = -2.0;
    const double CAR_MAX_VELOCITY = 2.0;
    const double CAR_MIN_ACCELERATION = -1.0;
    const double CAR_MAX_ACCELERATION = 1.0;
    const double CAR_MIN_STEER_VELOCITY = -2.7;
    const double CAR_MAX_STEER_VELOCITY = 2.7;
    const double CAR_NR_TRAILERS = 5;
    const double CAR_ATTACH_DISTANCE = 0.01;
    const double BLIMP_MIN_CONTROL_Z = -2.7;
    const double BLIMP_MAX_CONTROL_Z = 2.7;
    const double BLIMP_BODY_HEIGHT = 1.0;

    const char KW_CurrentState[] = "CurrentState";
    const char KW_CarFrontWheelDriving[] = "CarFrontWheelDriving";
    const char KW_CarBodyLength[] = "CarBodyLength";
    const char KW_CarBodyWidth[] = "CarBodyWidth";
    const char KW_BodyHeight[] = "BodyHeight";
    const char KW_CarMinSteerAngleInDegrees[] = "CarMinSteerAngleInDegrees";
    const char KW_CarMaxSteerAngleInDegrees[] = "CarMaxSteerAngleInDegrees";
    const char KW_CarMinVelocity[] = "CarMinVelocity";
    const char KW_CarMaxVelocity[] = "CarMaxVelocity";
    const char KW_CarMinSteerVelocity[] = "CarMinSteerVelocity";
    const char KW_CarMaxSteerVelocity[] = "CarMaxSteerVelocity";
    const char KW_CarMinAcceleration[] = "CarMinAcceleration";
    const char KW_CarMaxAcceleration[] = "CarMaxAcceleration";
    const char KW_CarNrTrailers[] = "CarNrTrailers";
    const char KW_CarAttachDistance[] = "CarAttachDistance";
    const char KW_MinControlZ[] = "MinControlZ";
    const char KW_MaxControlZ[] = "MaxControlZ";

    // robot
    const char ROBOT_CHASSIS_MESH_FILE[] = "gdata/models/robot.obj";
    const char ROBOT_WHEEL_MESH_FILE[] = "gdata/textures/wheel.mesh.xml";
    const char ROBOT_TIRE_TEXTURE_FILE[] = "gdata/textures/Tire_tex.png";

    // SimulatorRobot
    const double ROBOT_BODY_LENGTH = 0.508;
    const double ROBOT_BODY_WIDTH = 0.430;
    const double ROBOT_WHEEL_BASE = 0.277;
    const double ROBOT_MIN_ANGULAR_VELOCITY = -14.44;
    const double ROBOT_MAX_ANGULAR_VELOCITY = 14.44;
    const double ROBOT_MIN_VELOCITY = -2.0;
    const double ROBOT_MAX_VELOCITY = 2.0;
    const double ROBOT_MIN_ACCELERATION = -1.0;
    const double ROBOT_MAX_ACCELERATION = 1.0;

    const char KW_RobotBodyLength[] = "RobotBodyLength";
    const char KW_RobotBodyWidth[] = "RobotBodyWidth";
    const char KW_RobotWheelBase[] = "RobotWheelBase";
    const char KW_RobotBodyHeight[] = "RobotBodyHeight";
    const char KW_RobotMinAngularVelocity[] = "RobotMinAngularVelocity";
    const char KW_RobotMaxAngularVelocity[] = "RobotMaxAngularVelocity";
    const char KW_RobotMinAngularAcceleration[] = "RobotMinAngularAcceleration";
    const char KW_RobotMaxAngularAcceleration[] = "RobotMaxAngularAcceleration";
    const char KW_RobotMinVelocity[] = "RobotMinVelocity";
    const char KW_RobotMaxVelocity[] = "RobotMaxVelocity";
    const char KW_RobotMinAcceleration[] = "RobotMinAcceleration";
    const char KW_RobotMaxAcceleration[] = "RobotMaxAcceleration";
    const char KW_RobotAttachDistance[] = "RobotAttachDistance";
    const char KW_RobotControlZ[] = "MinControlZ";
    const char KW_RobotMaxControlZ[] = "MaxControlZ";


    // Decomposition
    const char KW_Decomposition[] = "Decomposition";
    const char KW_DecompositionGrid[] = "DecompositionGrid";
    const char KW_DecompositionGeometric[] = "DecompositionGeometric";
    const char KW_RegionClearances[] = "RegionClearances";
    const char KW_BasedOnCfg[] = "BasedOnCfg";
    const char KW_BasedOnShape[] = "BasedOnShape";
    const char KW_ScaleFactorForCollisionStatus[] = "ScaleFactorForCollisionStatus";
    const double DECOMPOSITION_SCALE_FACTOR_FOR_COLLISION_STATUS = 1.0;
    const double MAX_RUNTIME_DECOMPOSITION_CONSTRUCT = 2.0;

    const double DECOMPOSITION_TRIANGLES_TRIANGLE_MIN_AREA = 0.05;
    const double DECOMPOSITION_TRIANGLES_TRIANGLE_AVG_AREA = 2.0;
    const char KW_TriangleMinArea[] = "TriangleMinArea";
    const char KW_TriangleAvgArea[] = "TriangleAvgArea";
    const char KW_DecompositionTriangles[] = "DecompositionTriangles";

    // PRM
    const int PRM_NR_NEIGHBORS = 15;
    const int PRM_BATCH_SIZE = 100;
    const double PRM_PROBABILITY_ALLOW_CYCLES = 0.0;
    const double PRM_ONE_STEP_DISTANCE = 1.0;
    const int PRM_MAX_NR_VERTICES = 5000;

    const char KW_DecompositionPRM[] = "DecompositionPRM";
    const char KW_NrNeighbors[] = "NrNeighbors";
    const char KW_BatchSize[] = "BatchSize";
    const char KW_ProbabilityAllowCycles[] = "ProbabilityAllowCycles";
    const char KW_OneStepDistance[] = "OneStepDistance";
    const char KW_StopWhenConnected[] = "StopWhenConnected";
    const char KW_MaxNrVertices[] = "MaxNrVertices";

    // Problem
    const char KW_Problem[] = "Problem";
    const char KW_NrGoals[] = "NrGoals";
    const char KW_Goal[] = "Goal";
    const char KW_Box2D[] = "Box2D";
    const char KW_Triangle2D[] = "Triangle2D";
    const char KW_Box3D[] = "Box3D";

    //Goal
    const double KW_GoalRewardMin = 100;
    const double KW_GoalRewardMax = 100;
    const double KW_GoalDecayMin = 1.0;
    const double KW_GoalDecayMax = 1.0;
    const int KW_GoalDecayPerSecond = 60;

    // Tour
    const char KW_NrSites[] = "NrSites";
    const char KW_TimeBounds[] = "TimeBounds";
    const char KW_Durations[] = "Durations";
    const char KW_StartTime[] = "StartTime";

    // Follow
    const char KW_Follow[] = "Follow";
    const char KW_WeightBase[] = "WeightBase";
    const char KW_ReachTolerance[] = "ReachTolerance";
    const char KW_SamplingBias[] = "SamplingBias";
    const char KW_Radius[] = "Radius";

    const double FOLLOW_WEIGHT_BASE = 100000000.0;
    const double FOLLOW_REACH_TOLERANCE = 0.2;
    const double FOLLOW_RADIUS = 3.0;
    const double FOLLOW_SAMPLING_BIAS = 0.1;

    // Group selector
    const char KW_GroupSelector[] = "GroupSelector";
    const char KW_GroupSelectorProbabilityUniform[] = "GroupSelectorProbabilityUniform";
    const char KW_GroupSelectorProbabilityWeights[] = "GroupSelectorProbabilityWeights";
    const char KW_GroupSelectorMaxWeight[] = "GroupSelectorMaxWeight";
    const char KW_GroupVertexSelectorProbabilityUniform[] = "GroupVertexSelectorProbabilityUniform";

    const char KW_TourGenerator[] = "TourGenerator";
    const char KW_TourGeneratorRandom[] = "TourGeneratorRandom";
    const char KW_TourGeneratorOptimal[] = "TourGeneratorOptimal";
    const char KW_TourGeneratorBNB[] = "TourGeneratorBNB";
    const char KW_TourGeneratorMC[] = "TourGeneratorMC";
    const char KW_TourGeneratorPDDL[] = "TourGeneratorPDDL";
    const char KW_TourGeneratorLKH[] = "TourGeneratorLKH";
    const char KW_TourGeneratorExactNormalOrder[] = "TourGeneratorExactNormalOrder";
    const char KW_TourGeneratorExactReverseOrder[] = "TourGeneratorExactReverseOrder";
}


#endif
