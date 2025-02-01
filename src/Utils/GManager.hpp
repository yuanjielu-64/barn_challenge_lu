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

#ifndef Antipatrea__GManager_HPP_
#define Antipatrea__GManager_HPP_

#include "Utils/Flags.hpp"
#include "Utils/GCamera.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/GManagerComponent.hpp"
#include "Utils/GMenuItem.hpp"
#include "Utils/Params.hpp"
#include <unordered_map>
#include <vector>

namespace Antipatrea {

    class GManager {
    public:
        GManager(void);

        virtual ~GManager(void);

        virtual void SetupFromParams(Params &params);

        virtual void Help(void);

        virtual void MainLoop(const char *const title, const int width = 1920, const int height = 1080);

        static void
        MousePosFromScreenToWorld(const int x, const int y, double *const wx, double *const wy, double *const wz);

        static void SaveFrameAsImage(void);

        static void SaveFrameAsImage(const char fname[]);

        virtual void HandleEventOnTimer(void);

        virtual void HandleEventOnDisplay(void);

        virtual bool HandleEventOnMouseLeftBtnDown(const int x, const int y);

        virtual bool HandleEventOnMouseLeftBtnUp(const int x, const int y);

        virtual bool HandleEventOnActiveMouseMove(const int x, const int y);

        virtual bool HandleEventOnPassiveMouseMove(const int x, const int y);

        virtual bool HandleEventOnNormalKeyPress(const int key);

        virtual bool HandleEventOnSpecialKeyPress(const int key);

        virtual bool HandleEventOnNormalKeyUp(const int key);

        virtual bool HandleEventOnSpecialKeyUp(const int key);

        virtual bool HandleEventOnIdle(void);

        virtual bool HandleEventOnMenu(const int item);

        enum {
            INDEX_CAMERA_MOVE = 0,
            INDEX_MINX,
            INDEX_MINY,
            INDEX_MAXX,
            INDEX_MAXY,
            INDEX_ORTHO_NEAR_PLANE,
            INDEX_ORTHO_FAR_PLANE,
            INDEX_PERSPECTIVE_NEAR_PLANE,
            INDEX_PERSPECTIVE_FAR_PLANE,
            INDEX_PERSPECTIVE_ANGLE,
            NR_INDICES
        };

        virtual double GetValue(const int i) {
            return m_values[i];
        }

        virtual void SetValue(const int i, const double val) {
            m_values[i] = val;
        }

        virtual std::vector<GMenuItem *> *GetMenuItems(void) {
            return &m_menuItems;
        }

        virtual const std::vector<GMenuItem *> *GetMenuItems(void) const {
            return &m_menuItems;
        }

        virtual std::vector<GManagerComponent *> *GetComponents(void) {
            return &m_components;
        }

        virtual const std::vector<GManagerComponent *> *GetComponents(void) const {
            return &m_components;
        }

        static int CreateMenu(void);

        static void SetMenu(const int menu);

        static void AddSubMenu(const char name[], const int item);

        static void AddMenuEntry(const char name[], const int item);

        static void ChangeToMenuEntry(const int pos, const char name[], const int item);

        virtual GCamera *GetCamera(void) {
            return &m_gCamera;
        }

    protected:
        static void CallbackEventOnActiveMouseMove(int x, int y);

        static void CallbackEventOnPassiveMouseMove(int x, int y);

        static void CallbackEventOnMouse(int button, int state, int x, int y);

        static void CallbackEventOnTimer(int id);

        static void CallbackEventOnMenu(int item);

        static void CallbackEventOnSpecialKeyPress(int key, int x, int y);

        static void CallbackEventOnNormalKeyPress(unsigned char key, int x, int y);

        static void CallbackEventOnSpecialKeyUp(int key, int x, int y);

        static void CallbackEventOnNormalKeyUp(unsigned char key, int x, int y);

        static void CallbackEventOnDisplay(void);

        static void CallbackEventOnIdle(void);

        virtual void PrepareMenuCamera(void);

        virtual void PrepareMenuDraw(void);

        virtual void PrepareMenuMain(void);

        virtual bool HandleEventOnCameraRotation(const double thetax, const double thetay, const double thetaz);

        int m_idWindow;
        int m_timer;
        int m_mousePrevX;
        int m_mousePrevY;
        int m_frames;
        GCamera m_gCamera;
        GIllumination m_gIllumination;
        std::vector<double> m_values;
        std::unordered_map<int, int> m_mapKeysToMenuItems;

        int m_menuMain;
        int MENU_PAUSE;
        int m_menuCamera;
        int MENU_CAMERA_ROTATE_X;
        int MENU_CAMERA_ROTATE_Y;
        int MENU_CAMERA_ROTATE_Z;
        int MENU_CAMERA_ROTATE_CENTER;
        int MENU_CAMERA_ROTATE_GLOBAL_AXIS;
        int MENU_CAMERA_INFO;
        int m_menuCameraFirstItem;
        int m_menuCameraLastItem;
        int m_menuDraw;
        int MENU_DRAW_3D;
        int MENU_DRAW_TEXTURES;
        int MENU_DRAW_SAVE_FRAME;
        int MENU_DRAW_SAVE_FRAMES;
        int m_menuDrawFirstItem;
        int m_menuDrawLastItem;
        int m_menuSelectedItem;
        std::vector<GMenuItem *> m_menuItems;

        std::vector<GManagerComponent *> m_components;
    };
}

#endif
