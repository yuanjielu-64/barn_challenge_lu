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
#include "Utils/GManager.hpp"
#include "Utils/Constants.hpp"
#include "Utils/Logger.hpp"
#include "Utils/Params.hpp"
#include "Utils/Stats.hpp"
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace Antipatrea {
    GManager *m_gManager = NULL;

    void GManager::CallbackEventOnActiveMouseMove(int x, int y) {
        if (m_gManager) {
            if (m_gManager->HandleEventOnActiveMouseMove(x, y))
                glutPostRedisplay();
            m_gManager->m_mousePrevX = x;
            m_gManager->m_mousePrevY = y;
        }
    }

    void GManager::CallbackEventOnPassiveMouseMove(int x, int y) {
        if (m_gManager) {
            if (m_gManager->HandleEventOnPassiveMouseMove(x, y))
                glutPostRedisplay();
            m_gManager->m_mousePrevX = x;
            m_gManager->m_mousePrevY = y;
        }
    }

    void GManager::CallbackEventOnMouse(int button, int state, int x, int y) {
        if (m_gManager) {
            if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN && m_gManager->HandleEventOnMouseLeftBtnDown(x, y))
                glutPostRedisplay();
            else if (button == GLUT_LEFT_BUTTON && state == GLUT_UP && m_gManager->HandleEventOnMouseLeftBtnUp(x, y))
                glutPostRedisplay();

            m_gManager->m_mousePrevX = x;
            m_gManager->m_mousePrevY = y;
        }
    }

    void GManager::CallbackEventOnTimer(int id) {
        if (m_gManager) {
            m_gManager->HandleEventOnTimer();
            glutTimerFunc(m_gManager->m_timer, CallbackEventOnTimer, id);
            glutPostRedisplay();
        }
    }

    void GManager::CallbackEventOnIdle(void) {
        if (m_gManager && m_gManager->HandleEventOnIdle())
            glutPostRedisplay();
    }

    void GManager::CallbackEventOnSpecialKeyPress(int key, int x, int y) {
        if (m_gManager && m_gManager->HandleEventOnSpecialKeyPress(key))
            glutPostRedisplay();
    }

    void GManager::CallbackEventOnNormalKeyPress(unsigned char key, int x, int y) {
        if (m_gManager && m_gManager->HandleEventOnNormalKeyPress(key))
            glutPostRedisplay();
    }

    void GManager::CallbackEventOnSpecialKeyUp(int key, int x, int y) {
        if (m_gManager && m_gManager->HandleEventOnSpecialKeyUp(key))
            glutPostRedisplay();
    }

    void GManager::CallbackEventOnNormalKeyUp(unsigned char key, int x, int y) {
        if (m_gManager && m_gManager->HandleEventOnNormalKeyUp(key))
            glutPostRedisplay();
    }

    void GManager::CallbackEventOnDisplay(void) {
        if (m_gManager) {
            glClearColor(1.0f, 1.0f, 1.0f, 0.0f);
            glClearDepth(1.0);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            glViewport(0, 0, glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();

            const bool is3D = HasAllFlags(m_gManager->m_menuItems[m_gManager->MENU_DRAW_3D]->GetFlags(),
                                          GMenuItem::FLAG_ON);

            if (is3D)
                gluPerspective(m_gManager->m_values[INDEX_PERSPECTIVE_ANGLE],
                               (double) glutGet(GLUT_WINDOW_WIDTH) / glutGet(GLUT_WINDOW_HEIGHT),
                               m_gManager->m_values[INDEX_PERSPECTIVE_NEAR_PLANE],
                               m_gManager->m_values[INDEX_PERSPECTIVE_FAR_PLANE]);
            else
                glOrtho(m_gManager->m_values[INDEX_MINX], m_gManager->m_values[INDEX_MAXX],
                        m_gManager->m_values[INDEX_MINY], m_gManager->m_values[INDEX_MAXY],
                        m_gManager->m_values[INDEX_ORTHO_NEAR_PLANE], m_gManager->m_values[INDEX_ORTHO_FAR_PLANE]);

            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();

            glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);

            if (HasAllFlags(m_gManager->m_menuItems[m_gManager->MENU_DRAW_TEXTURES]->GetFlags(), GMenuItem::FLAG_ON))
                glEnable(GL_TEXTURE_2D);
            else
                glDisable(GL_TEXTURE_2D);

            glEnable(GL_DEPTH_TEST);
            glShadeModel(GL_SMOOTH);

            if (is3D) {
                double m[16];
                m_gManager->m_gCamera.GetModelViewMatrixOpenGL(m);
                glMultMatrixd(m);
                GDraw3D();
                GDrawIllumination(m_gManager->m_gIllumination);
            } else
                GDraw2D();

            m_gManager->HandleEventOnDisplay();

            if (HasAllFlags(m_gManager->m_menuItems[m_gManager->MENU_DRAW_SAVE_FRAMES]->GetFlags(), GMenuItem::FLAG_ON))
                SaveFrameAsImage();

            glutSwapBuffers();
        }
    }

    GManager::GManager(void) {
        m_idWindow = -1;
        m_timer = Constants::GRAPHICS_TIMER_INTERVAL;
        m_mousePrevX = 0;
        m_mousePrevY = 0;
        m_frames = 0;

        m_values.resize(NR_INDICES);
        m_values[INDEX_CAMERA_MOVE] = Constants::GRAPHICS_CAMERA_MOVE;
        m_values[INDEX_MINX] = Constants::GRID_MIN[0];
        m_values[INDEX_MINY] = Constants::GRID_MIN[1];
        m_values[INDEX_MAXX] = Constants::GRID_MAX[0];
        m_values[INDEX_MAXY] = Constants::GRID_MAX[1];
        m_values[INDEX_ORTHO_NEAR_PLANE] = Constants::GRAPHICS_ORTHO_NEAR_PLANE;
        m_values[INDEX_ORTHO_FAR_PLANE] = Constants::GRAPHICS_ORTHO_FAR_PLANE;
        m_values[INDEX_PERSPECTIVE_ANGLE] = Constants::GRAPHICS_PERSPECTIVE_ANGLE_IN_DEGREES;
        m_values[INDEX_PERSPECTIVE_NEAR_PLANE] = Constants::GRAPHICS_PERSPECTIVE_NEAR_PLANE;
        m_values[INDEX_PERSPECTIVE_FAR_PLANE] = Constants::GRAPHICS_PERSPECTIVE_FAR_PLANE;

        m_menuMain = Constants::ID_UNDEFINED;
        m_menuCamera = Constants::ID_UNDEFINED;
        m_menuDraw = Constants::ID_UNDEFINED;
        m_menuSelectedItem = Constants::ID_UNDEFINED;
    }

    GManager::~GManager(void) {
        if (m_idWindow >= 0)
            glutDestroyWindow(m_idWindow);
    }

    void GManager::SetupFromParams(Params &p) {
        m_values[INDEX_CAMERA_MOVE] = p.GetValueAsDouble(Constants::KW_CameraMove, m_values[INDEX_CAMERA_MOVE]);
        m_values[INDEX_ORTHO_NEAR_PLANE] = p.GetValueAsDouble(Constants::KW_OrthoNearPlane,
                                                              m_values[INDEX_ORTHO_NEAR_PLANE]);
        m_values[INDEX_ORTHO_FAR_PLANE] = p.GetValueAsDouble(Constants::KW_OrthoFarPlane,
                                                             m_values[INDEX_ORTHO_FAR_PLANE]);
        m_values[INDEX_PERSPECTIVE_ANGLE] = p.GetValueAsDouble(Constants::KW_PerspectiveAngleInDegrees,
                                                               m_values[INDEX_PERSPECTIVE_ANGLE]);
        m_values[INDEX_PERSPECTIVE_NEAR_PLANE] = p.GetValueAsDouble(Constants::KW_PerspectiveNearPlane,
                                                                    m_values[INDEX_PERSPECTIVE_NEAR_PLANE]);
        m_values[INDEX_PERSPECTIVE_FAR_PLANE] = p.GetValueAsDouble(Constants::KW_PerspectiveFarPlane,
                                                                   m_values[INDEX_PERSPECTIVE_FAR_PLANE]);
        m_timer = p.GetValueAsInt(Constants::KW_OnTimerInterval, m_timer);

        auto data = p.GetData(Constants::KW_Min);
        if (data) {
            if (data->m_values.size() > 0)
                m_values[INDEX_MINX] = StrToDouble(data->m_values[0]->c_str());
            if (data->m_values.size() > 1)
                m_values[INDEX_MINY] = StrToDouble(data->m_values[0]->c_str());
        }

        data = p.GetData(Constants::KW_Max);
        if (data) {
            if (data->m_values.size() > 0)
                m_values[INDEX_MAXX] = StrToDouble(data->m_values[0]->c_str());
            if (data->m_values.size() > 1)
                m_values[INDEX_MAXY] = StrToDouble(data->m_values[0]->c_str());
        }

        data = p.GetData(Constants::KW_Camera);
        if (data && data->m_params)
            m_gCamera.SetupFromParams(*(data->m_params));
        data = p.GetData(Constants::KW_Illumination);
        if (data && data->m_params)
            m_gIllumination.SetupFromParams(*(data->m_params));
    }

    void GManager::MainLoop(const char *const title, const int width, const int height) {
        m_gManager = this;

        static int argc = 1;
        static char *args = (char *) "args";

        glutInit(&argc, &args);
        glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
        glutInitWindowSize(width, height);
        glutInitWindowPosition(0, 0);
        m_idWindow = glutCreateWindow(title);

        glutDisplayFunc(CallbackEventOnDisplay);
        glutMouseFunc(CallbackEventOnMouse);
        glutMotionFunc(CallbackEventOnActiveMouseMove);
        glutPassiveMotionFunc(CallbackEventOnPassiveMouseMove);
        glutIdleFunc(CallbackEventOnIdle);
        glutTimerFunc(0, CallbackEventOnTimer, 0);
        glutKeyboardFunc(CallbackEventOnNormalKeyPress);
        glutSpecialFunc(CallbackEventOnSpecialKeyPress);
        glutKeyboardUpFunc(CallbackEventOnNormalKeyUp);
        glutSpecialUpFunc(CallbackEventOnSpecialKeyUp);

        PrepareMenuMain();
        glutAttachMenu(GLUT_RIGHT_BUTTON);

        glutMainLoop();
    }

    void GManager::Help(void) {
    }

    void GManager::MousePosFromScreenToWorld(const int x, const int y, double *const wx, double *const wy,
                                             double *const wz) {
        GLint viewport[4];
        GLdouble modelview[16];
        GLdouble projection[16];
        GLfloat winX, winY, winZ = 0;
        double px = 0, py = 0, pz = 0;

        glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
        glGetDoublev(GL_PROJECTION_MATRIX, projection);
        glGetIntegerv(GL_VIEWPORT, viewport);

        winX = (float) x;
        winY = (float) viewport[3] - (float) y;
        glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
        gluUnProject(winX, winY, winZ, modelview, projection, viewport, &px, &py, &pz);

        if (wx)
            *wx = px;
        if (wy)
            *wy = py;
        if (wz)
            *wz = pz;
    }

    void GManager::SaveFrameAsImage(void) {
        int ignore;
        char fname[50];
        sprintf(fname, "frames_%05d.ppm", (m_gManager->m_frames)++);
        SaveFrameAsImage(fname);

        char cmd[300];
        sprintf(cmd, "convert -quality 100 %s %s.jpg", fname, fname);
        ignore = system(cmd);
        sprintf(cmd, "\\rm %s", fname);
        ignore = system(cmd);
    }

    void GManager::SaveFrameAsImage(const char fname[]) {

        const int width = glutGet(GLUT_WINDOW_WIDTH);
        const int height = glutGet(GLUT_WINDOW_HEIGHT);

        char *temp = new char[3 * width * height];
        char *image = new char[3 * width * height];

        FILE *fp = fopen(fname, "w");

        printf("Writing %s\n", fname);

        glPixelStorei(GL_PACK_ALIGNMENT, 1);
        glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, temp);

        int a, b, row_sz = 3 * width;
        // Reverse rows
        for (int i = 0; i < height; i += 1) {
            for (int j = 0; j < width; j += 1) {
                a = i * row_sz + 3 * j;
                b = (height - i - 1) * row_sz + 3 * j;
                image[a] = temp[b];
                image[a + 1] = temp[b + 1];
                image[a + 2] = temp[b + 2];
            }
        }
        fprintf(fp, "P6\n");
        fprintf(fp, "%i %i\n 255\n", width, height);
        fwrite(image, sizeof(char), 3 * width * height, fp);
        fclose(fp);
        delete[] temp;
        delete[] image;
    }

    void GManager::CallbackEventOnMenu(int item) {
        if (m_gManager)
            m_gManager->m_menuSelectedItem = item;
    }

    int GManager::CreateMenu(void) {
        return glutCreateMenu(CallbackEventOnMenu);
    }

    void GManager::SetMenu(const int menu) {
        glutSetMenu(menu);
    }

    void GManager::AddSubMenu(const char name[], const int menu) {
        glutAddSubMenu(name, menu);
    }

    void GManager::AddMenuEntry(const char name[], const int item) {
        glutAddMenuEntry(name, item);
    }

    void GManager::ChangeToMenuEntry(const int pos, const char name[], const int item) {
        glutChangeToMenuEntry(pos, name, item);
    }

    void GManager::PrepareMenuMain(void) {
        PrepareMenuCamera();
        PrepareMenuDraw();

        m_menuMain = CreateMenu();

        auto items = GetMenuItems();
        MENU_PAUSE = items->size();
        items->push_back(new GMenuItem("pause", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));
        AddMenuEntry((*items)[MENU_PAUSE]->GetExtendedName(), MENU_PAUSE);

        AddSubMenu("Camera", m_menuCamera);
        AddSubMenu("Draw", m_menuDraw);

        for (auto &comp: m_components) {
            auto id = comp->PrepareMenu();
            SetMenu(m_menuMain);
            AddSubMenu(comp->GetMenuName(), id);
        }
    }

    void GManager::PrepareMenuCamera(void) {
        MENU_CAMERA_ROTATE_X = m_menuItems.size();
        m_menuItems.push_back(new GMenuItem("rotate x", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));
        MENU_CAMERA_ROTATE_Y = m_menuItems.size();
        m_menuItems.push_back(new GMenuItem("rotate y", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));
        MENU_CAMERA_ROTATE_Z = m_menuItems.size();
        m_menuItems.push_back(new GMenuItem("rotate z", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));
        MENU_CAMERA_ROTATE_CENTER = m_menuItems.size();
        m_menuItems.push_back(new GMenuItem("rotate center", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));
        MENU_CAMERA_ROTATE_GLOBAL_AXIS = m_menuItems.size();
        m_menuItems.push_back(new GMenuItem("rotate global axis", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));
        MENU_CAMERA_INFO = m_menuItems.size();
        m_menuItems.push_back(new GMenuItem("info", GMenuItem::FLAG_OFF));

        m_menuCamera = CreateMenu();
        m_menuCameraFirstItem = MENU_CAMERA_ROTATE_X;
        m_menuCameraLastItem = MENU_CAMERA_INFO;
        AddMenuEntry(m_menuItems[MENU_CAMERA_ROTATE_X]->GetExtendedName(), MENU_CAMERA_ROTATE_X);
        AddMenuEntry(m_menuItems[MENU_CAMERA_ROTATE_Y]->GetExtendedName(), MENU_CAMERA_ROTATE_Y);
        AddMenuEntry(m_menuItems[MENU_CAMERA_ROTATE_Z]->GetExtendedName(), MENU_CAMERA_ROTATE_Z);
        AddMenuEntry(m_menuItems[MENU_CAMERA_ROTATE_CENTER]->GetExtendedName(), MENU_CAMERA_ROTATE_CENTER);
        AddMenuEntry(m_menuItems[MENU_CAMERA_ROTATE_GLOBAL_AXIS]->GetExtendedName(), MENU_CAMERA_ROTATE_GLOBAL_AXIS);
        AddMenuEntry(m_menuItems[MENU_CAMERA_INFO]->GetExtendedName(), MENU_CAMERA_INFO);

        m_mapKeysToMenuItems.insert(std::make_pair(Constants::KEY_CAMERA_INFO, MENU_CAMERA_INFO));
    }

    void GManager::PrepareMenuDraw(void) {
        MENU_DRAW_3D = m_menuItems.size();
        m_menuItems.push_back(new GMenuItem("3D", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_TEXTURES = m_menuItems.size();
        m_menuItems.push_back(new GMenuItem("textures", GMenuItem::FLAG_ON | GMenuItem::FLAG_TOGGLES));
        MENU_DRAW_SAVE_FRAME = m_menuItems.size();
        m_menuItems.push_back(new GMenuItem("save frame", GMenuItem::FLAG_OFF));
        MENU_DRAW_SAVE_FRAMES = m_menuItems.size();
        m_menuItems.push_back(new GMenuItem("save frames", GMenuItem::FLAG_OFF | GMenuItem::FLAG_TOGGLES));

        m_menuDraw = CreateMenu();
        m_menuDrawFirstItem = MENU_DRAW_3D;
        m_menuDrawLastItem = MENU_DRAW_SAVE_FRAMES;
        AddMenuEntry(m_menuItems[MENU_DRAW_3D]->GetExtendedName(), MENU_DRAW_3D);
        AddMenuEntry(m_menuItems[MENU_DRAW_TEXTURES]->GetExtendedName(), MENU_DRAW_TEXTURES);
        AddMenuEntry(m_menuItems[MENU_DRAW_SAVE_FRAME]->GetExtendedName(), MENU_DRAW_SAVE_FRAME);
        AddMenuEntry(m_menuItems[MENU_DRAW_SAVE_FRAMES]->GetExtendedName(), MENU_DRAW_SAVE_FRAMES);

        m_mapKeysToMenuItems.insert(std::make_pair(Constants::KEY_DRAW_3D, MENU_DRAW_3D));
        m_mapKeysToMenuItems.insert(std::make_pair(Constants::KEY_DRAW_SAVE_FRAMES, MENU_DRAW_SAVE_FRAMES));
    }

    void GManager::HandleEventOnTimer(void) {
        if (m_menuSelectedItem >= 0) {
            HandleEventOnMenu(m_menuSelectedItem);
            m_menuSelectedItem = Constants::ID_UNDEFINED;
        }

        if (!HasAllFlags(m_menuItems[MENU_PAUSE]->GetFlags(), GMenuItem::FLAG_ON))
            for (auto &comp: m_components)
                comp->HandleEventOnTimer();
    }

    void GManager::HandleEventOnDisplay(void) {
        for (auto &comp: m_components)
            comp->HandleEventOnDisplay();
    }

    bool GManager::HandleEventOnMouseLeftBtnDown(const int x, const int y) {
        bool res = false;

        for (auto &comp: m_components)
            res = res || comp->HandleEventOnMouseLeftBtnDown(x, y);
        return res;
    }

    bool GManager::HandleEventOnMouseLeftBtnUp(const int x, const int y) {
        bool res = false;

        for (auto &comp: m_components)
            res = res || comp->HandleEventOnMouseLeftBtnUp(x, y);
        return res;
    }

    bool GManager::HandleEventOnActiveMouseMove(const int x, const int y) {
        const double thetax = 0.25 * 2 * M_PI * (y - m_mousePrevY) / glutGet(GLUT_WINDOW_HEIGHT);
        const double thetay = 0.25 * 2 * M_PI * (x - m_mousePrevX) / glutGet(GLUT_WINDOW_WIDTH);
        const double thetaz = thetay;

        bool res = HandleEventOnCameraRotation(thetax, thetay, thetaz);

        for (auto &comp: m_components)
            res = res || comp->HandleEventOnActiveMouseMove(x, y);
        return res;
    }

    bool GManager::HandleEventOnPassiveMouseMove(const int x, const int y) {
        bool res = false;

        for (auto &comp: m_components)
            res = res || comp->HandleEventOnPassiveMouseMove(x, y);
        return res;
    }

    bool GManager::HandleEventOnNormalKeyPress(const int key) {
        if (key == 27) // escape key
        {
            exit(0);
            return true;
        } else if (glutGetModifiers() & GLUT_ACTIVE_ALT) {
            switch (key) {
                case Constants::KEY_PRINT_STATS:
                    Logger::m_out << *(Stats::GetSingleton()) << std::endl;
                    return true;

                case Constants::KEY_CAMERA_ROTATE_X:
                    return HandleEventOnCameraRotation(Constants::DEG2RAD, 0, 0);
                case Constants::KEY_CAMERA_ROTATE_Y:
                    return HandleEventOnCameraRotation(0, Constants::DEG2RAD, 0);
                case Constants::KEY_CAMERA_ROTATE_Z:
                    return HandleEventOnCameraRotation(0, 0, Constants::DEG2RAD);
                case Constants::KEY_CAMERA_ROTATE_XNEG:
                    return HandleEventOnCameraRotation(-Constants::DEG2RAD, 0, 0);
                case Constants::KEY_CAMERA_ROTATE_YNEG:
                    return HandleEventOnCameraRotation(0, -Constants::DEG2RAD, 0);
                case Constants::KEY_CAMERA_ROTATE_ZNEG:
                    return HandleEventOnCameraRotation(0, 0, -Constants::DEG2RAD);
            };

            auto cur = m_mapKeysToMenuItems.find(key);
            if (cur != m_mapKeysToMenuItems.end())
                return HandleEventOnMenu(cur->second);
        }

        bool res = false;

        for (auto &comp: m_components)
            res = res || comp->HandleEventOnNormalKeyPress(key);
        return res;
    }

    bool GManager::HandleEventOnSpecialKeyPress(const int key) {
        if (key == GLUT_KEY_F1) {
            Help();
            return true;
        } else if (HasAllFlags(m_menuItems[MENU_DRAW_3D]->GetFlags(), GMenuItem::FLAG_ON)) {
            switch (key) {
                case GLUT_KEY_UP:
                    m_gCamera.MoveForward(m_values[INDEX_CAMERA_MOVE]);
                    return true;
                case GLUT_KEY_DOWN:
                    m_gCamera.MoveForward(-m_values[INDEX_CAMERA_MOVE]);
                    return true;
                case GLUT_KEY_RIGHT:
                    m_gCamera.MoveRight(m_values[INDEX_CAMERA_MOVE]);
                    return true;
                case GLUT_KEY_LEFT:
                    m_gCamera.MoveRight(-m_values[INDEX_CAMERA_MOVE]);
                    return true;
                case GLUT_KEY_PAGE_DOWN:
                    m_gCamera.MoveUp(m_values[INDEX_CAMERA_MOVE]);
                    return true;
                case GLUT_KEY_PAGE_UP:
                    m_gCamera.MoveUp(-m_values[INDEX_CAMERA_MOVE]);
                    return true;
            }
        }

        bool res = false;
        for (auto &comp: m_components)
            res = res || comp->HandleEventOnSpecialKeyPress(key);
        return res;
    }

    bool GManager::HandleEventOnNormalKeyUp(const int key) {
        bool res = false;

        for (auto &comp: m_components)
            res = res || comp->HandleEventOnNormalKeyUp(key);
        return res;
    }

    bool GManager::HandleEventOnSpecialKeyUp(const int key) {
        bool res = false;

        for (auto &comp: m_components)
            res = res || comp->HandleEventOnSpecialKeyUp(key);
        return res;
    }

    bool GManager::HandleEventOnIdle(void) {
        bool res = false;

        for (auto &comp: m_components)
            res = res || comp->HandleEventOnIdle();
        return res;
    }

    bool GManager::HandleEventOnMenu(const int item) {
        if (item == MENU_PAUSE) {
            if (HasAllFlags(m_menuItems[item]->GetFlags(), GMenuItem::FLAG_TOGGLES)) {
                SetMenu(m_menuMain);
                m_menuItems[item]->SetFlags(FlipFlags(m_menuItems[item]->GetFlags(), GMenuItem::FLAG_ON));
                ChangeToMenuEntry(item - item + 1, m_menuItems[item]->GetExtendedName(), item);
                return true;
            }
        } else if (item >= m_menuCameraFirstItem && item <= m_menuCameraLastItem) {
            if (item == MENU_CAMERA_INFO)
                m_gCamera.Print(Logger::m_out);

            if (HasAllFlags(m_menuItems[item]->GetFlags(), GMenuItem::FLAG_TOGGLES)) {
                SetMenu(m_menuCamera);
                m_menuItems[item]->SetFlags(FlipFlags(m_menuItems[item]->GetFlags(), GMenuItem::FLAG_ON));
                ChangeToMenuEntry(item - m_menuCameraFirstItem + 1, m_menuItems[item]->GetExtendedName(), item);
                return true;
            }
        } else if (item >= m_menuDrawFirstItem && item <= m_menuDrawLastItem) {
            if (item == MENU_DRAW_SAVE_FRAME)
                SaveFrameAsImage();

            if (HasAllFlags(m_menuItems[item]->GetFlags(), GMenuItem::FLAG_TOGGLES)) {
                SetMenu(m_menuDraw);
                m_menuItems[item]->SetFlags(FlipFlags(m_menuItems[item]->GetFlags(), GMenuItem::FLAG_ON));
                ChangeToMenuEntry(item - m_menuDrawFirstItem + 1, m_menuItems[item]->GetExtendedName(), item);
                return true;
            }
        } else {
            for (auto &comp: m_components)
                if (comp->HandleEventOnMenu(item))
                    return true;
        }

        return false;
    }

    bool GManager::HandleEventOnCameraRotation(const double thetax, const double thetay, const double thetaz) {
        const double *p = HasAllFlags(m_menuItems[MENU_CAMERA_ROTATE_CENTER]->GetFlags(), GMenuItem::FLAG_ON)
                          ? m_gCamera.GetCenter() : m_gCamera.GetEye();
        bool event = false;

        if (HasAllFlags(m_menuItems[MENU_CAMERA_ROTATE_X]->GetFlags(), GMenuItem::FLAG_ON) && thetax != 0) {
            if (HasAllFlags(m_menuItems[MENU_CAMERA_ROTATE_GLOBAL_AXIS]->GetFlags(), GMenuItem::FLAG_ON))
                m_gCamera.RotateAroundAxisAtPoint(thetax, 1, 0, 0, p[0], p[1], p[2]);
            else
                m_gCamera.RotateAroundRightAxisAtPoint(thetax, p[0], p[1], p[2]);
            event = true;
        }

        if (HasAllFlags(m_menuItems[MENU_CAMERA_ROTATE_Y]->GetFlags(), GMenuItem::FLAG_ON) && thetay != 0) {
            if (HasAllFlags(m_menuItems[MENU_CAMERA_ROTATE_GLOBAL_AXIS]->GetFlags(), GMenuItem::FLAG_ON))
                m_gCamera.RotateAroundAxisAtPoint(thetay, 0, 1, 0, p[0], p[1], p[2]);
            else
                m_gCamera.RotateAroundUpAxisAtPoint(thetay, p[0], p[1], p[2]);
            event = true;
        }

        if (HasAllFlags(m_menuItems[MENU_CAMERA_ROTATE_Z]->GetFlags(), GMenuItem::FLAG_ON) && thetaz != 0) {
            if (HasAllFlags(m_menuItems[MENU_CAMERA_ROTATE_GLOBAL_AXIS]->GetFlags(), GMenuItem::FLAG_ON))
                m_gCamera.RotateAroundAxisAtPoint(thetaz, 0, 0, 1, p[0], p[1], p[2]);
            else
                m_gCamera.RotateAroundForwardAxisAtPoint(thetaz, p[0], p[1], p[2]);
            event = true;
        }
        return event;
    }
}
