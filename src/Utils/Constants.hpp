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

#ifndef Antipatrea__Constants_HPP_
#define Antipatrea__Constants_HPP_

#include <cmath>
#include <ros/ros.h>
#include <ros/package.h>

namespace Antipatrea {
/**
     *@brief Keywords and default values.
     */
    namespace Constants {

        const int NEITHER_PARALLEL_NOR_ANTI_PARALLEL = 0;
        const int PARALLEL = 1;
        const int ANTI_PARALLEL = 2;

        const double EPSILON = ldexp(1.0, -36);
        const double EPSILON_SQUARED = ldexp(1.0, -72);
        const double SQRT_EPSILON = ldexp(1.0, -18);

        const double RAD2DEG = 180 / M_PI;
        const double DEG2RAD = M_PI / 180;

        const int ID_UNDEFINED = -1;

        const char KW_Grid[] = "Grid";
        const char KW_NrDims[] = "NrDims";
        const char KW_Min[] = "Min";
        const char KW_Max[] = "Max";
        const char KW_Dims[] = "Dims";

        const int GRID_NR_DIMS = 2;
        const double GRID_MIN[] = {-30, -30, 0};
        const double GRID_MAX[] = {30, 30, 20};
        const int GRID_DIMS[] = {50, 50, 16};

        const char KW_ReadFromFile[] = "ReadFromFile";
        const char KW_WriteToFile[] = "WriteToFile";

        const char KW_Graphics[] = "Graphics";

        const char KW_CameraMove[] = "CameraMove";
        const char KW_OrthoNearPlane[] = "OrthoNearPlane";
        const char KW_OrthoFarPlane[] = "OrthoFarPlane";
        const char KW_PerspectiveNearPlane[] = "PerspectiveNearPlane";
        const char KW_PerspectiveFarPlane[] = "PerspectiveFarPlane";
        const char KW_PerspectiveAngleInDegrees[] = "PerspectiveAngleInDegrees";
        const char KW_OnTimerInterval[] = "OnTimerInterval";

        const double GRAPHICS_CAMERA_MOVE = 0.5;
        const double GRAPHICS_ORTHO_NEAR_PLANE = -10;
        const double GRAPHICS_ORTHO_FAR_PLANE = 10;
        const double GRAPHICS_PERSPECTIVE_NEAR_PLANE = 10;
        const double GRAPHICS_PERSPECTIVE_FAR_PLANE = 1000;
        const double GRAPHICS_PERSPECTIVE_ANGLE_IN_DEGREES = 18; //in degrees
        const int GRAPHICS_TIMER_INTERVAL = 5;

        const char KW_Camera[] = "Camera";
        const char KW_Eye[] = "Eye";
        const char KW_Center[] = "Center";
        const char KW_Right[] = "Right";
        const char KW_Forward[] = "Forward";

        const char KW_NrStacks[] = "NrStacks";
        const char KW_NrSlices[] = "NrSlices";
        const char KW_FontSizes[] = "FontSize";

        const int GRAPHICS_NR_STACKS = 50;
        const int GRAPHICS_NR_SLICES = 50;
        const double GRAPHICS_FONT_SIZES[] = {0.2, 0.2, 0.4};

        const char KW_Illumination[] = "Illumination";
        const char KW_Ambient[] = "Ambient";
        const char KW_Diffuse[] = "Diffuse";
        const char KW_Specular[] = "Specular";
        const char KW_Emissive[] = "Emissive";
        const char KW_Shininess[] = "Shininess";
        const char KW_Position[] = "Position";
        const char KW_Light[] = "Light";
        const char KW_NrLights[] = "NrLights";

        const char KW_ProjectName_[] = "barn_challenge_lu";

        const char FONT_FILE[] = "gdata/models/font";
        const char COLORMAP_FILE[] = "gdata/cmaps/jet.cmap";

        const int KEY_CAMERA_ROTATE_X = 'x';
        const int KEY_CAMERA_ROTATE_Y = 'y';
        const int KEY_CAMERA_ROTATE_Z = 'z';
        const int KEY_CAMERA_ROTATE_XNEG = 'X';
        const int KEY_CAMERA_ROTATE_YNEG = 'Y';
        const int KEY_CAMERA_ROTATE_ZNEG = 'Z';
        const int KEY_CAMERA_ROTATE_CENTER = 'c';
        const int KEY_CAMERA_ROTATE_GLOBAL_AXIS = 'g';
        const int KEY_CAMERA_INFO = 'i';
        const int KEY_DRAW_3D = '3';
        const int KEY_DRAW_SAVE_FRAME = 'f';
        const int KEY_DRAW_SAVE_FRAMES = 'm';
        const int KEY_PRINT_STATS = 's';

        const char KW_Polygon2D[] = "Polygon2D";

    }
}

#endif
