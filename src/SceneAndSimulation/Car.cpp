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
#include "Car.hpp"
#include "Components/Constants.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/TriMeshStreamer.hpp"
#include "Utils/GDraw.hpp"

namespace Antipatrea {
    Car::Car(void) : Component() {
        m_angleTire = 0.0;
        m_angleSteer = 0.0;

        m_bodyLengthFrac = 2 * 0.85 / 3.0;

        double Rx[Algebra3D::Rot_NR_ENTRIES];
        double Rz[Algebra3D::Rot_NR_ENTRIES];
        double R[Algebra3D::Rot_NR_ENTRIES];

        std::string package_path = ros::package::getPath(Constants::KW_ProjectName);
        auto CAR_CHASSIS_MESH_FILE = package_path + "/" + Constants::CAR_CHASSIS_MESH_FILE;

        TriMeshReader(CAR_CHASSIS_MESH_FILE.c_str(), m_chassis);
        Algebra3D::XAxisAngleAsRot(0.5 * M_PI, Rx);
        Algebra3D::ZAxisAngleAsRot(0.5 * M_PI, Rz);
        Algebra3D::RotMultRotAsRot(Rz, Rx, R);
        m_chassis.ApplyRot(0, m_chassis.GetNrVertices() - 1, R);

        const double *min = m_chassis.GetBoundingBoxMin();
        const double *max = m_chassis.GetBoundingBoxMax();

        m_chassis.ApplyTrans(0, m_chassis.GetNrVertices() - 1, (0.88 / 3.0) * (max[0] - min[0]), 0,
                             0.5 * (max[2] - min[2]));

        m_chassis.GetGroupsNotForDrawing()->push_back(6);
        m_chassis.GetGroupsNotForDrawing()->push_back(7);
        m_chassis.GetGroupsNotForDrawing()->push_back(8);
        m_chassis.GetGroupsNotForDrawing()->push_back(9);
        for (int i = m_chassis.GetNrGroups() - 1; i >= 11; --i)
            m_chassis.GetGroupsNotForDrawing()->push_back(i);

        //m_chassis.GetMaterial(1)->SetRuby();
        //m_chassis.GetMaterial(1)->SetDiffuse(1, 0, 0); //0: pasqyra, 1: body; 2: xhami; 3: spotlight; 4: backlight

        auto CAR_WHEEL_MESH_FILE =  package_path + "/" + Constants::CAR_WHEEL_MESH_FILE;
        auto CAR_TIRE_TEXTURE_FILE =  package_path + "/" + Constants::CAR_TIRE_TEXTURE_FILE;

        m_wheel.ReadOgreMesh(CAR_WHEEL_MESH_FILE.c_str(), CAR_TIRE_TEXTURE_FILE.c_str());
        m_wheel.GetMainMaterial()->SetDiffuse(0.2, 0.2, 0.2);
        m_wheel.ApplyRot(0, m_wheel.GetNrVertices() - 1, R);
        m_wheel.ApplyScaling(0, m_wheel.GetNrVertices() - 1, 20, 20, 18);
    }

    double Car::GetLengthFromEndToBackWheels(void) {
        return fabs(m_chassis.GetBoundingBoxMin()[0]);
    }

    double Car::GetBodyLength(void) {
        return m_chassis.GetBoundingBoxMax()[0];
    }

    double Car::GetDimension(const int i) {
        return m_chassis.GetBoundingBoxMax()[i] - m_chassis.GetBoundingBoxMin()[i];
    }

    void Car::AdjustDimensions(const int i, const double dim) {
        const double scale = dim / GetDimension(i);

        SetDimensions(GetDimension(0) * scale, GetDimension(1) * scale, GetDimension(2) * scale);
    }

    void Car::SetDimensions(const double dimx,
                            const double dimy,
                            const double dimz) {

        const double *min = m_chassis.GetBoundingBoxMin();
        const double *max = m_chassis.GetBoundingBoxMax();
        const double sx = dimx / (max[0] - min[0]);
        const double sy = dimy / (max[1] - min[1]);
        const double sz = dimz / (max[2] - min[2]);

        m_chassis.ApplyScaling(0, m_chassis.GetNrVertices() - 1, sx, sy, sz);
        m_wheel.ApplyScaling(0, m_wheel.GetNrVertices() - 1, sx, sy, sz);
    }

    void Car::Draw(void) {
        double R[Algebra3D::Rot_NR_ENTRIES];
        double Rs[Algebra3D::Rot_NR_ENTRIES];

        const double *min = m_chassis.GetBoundingBoxMin();
        const double *max = m_chassis.GetBoundingBoxMax();
        const double *wmin = m_wheel.GetBoundingBoxMin();
        const double *wmax = m_wheel.GetBoundingBoxMax();

        Algebra3D::YAxisAngleAsRot(m_angleTire, R);
        Algebra3D::ZAxisAngleAsRot(m_angleSteer, Rs);

        m_chassis.Draw();

        GDrawPushTransformation();
        GDrawMultTrans(0, -0.35 * (max[1] - min[1]), 0.5 * (wmax[2] - wmin[2]));
        GDrawMultRot(R);
        m_wheel.Draw();
        GDrawPopTransformation();

        GDrawPushTransformation();
        GDrawMultTrans(m_bodyLengthFrac * (max[0] - min[0]), -0.35 * (max[1] - min[1]), 0.5 * (wmax[2] - wmin[2]));
        GDrawMultRot(Rs);
        GDrawMultRot(R);
        m_wheel.Draw();
        GDrawPopTransformation();

        GDrawPushTransformation();
        GDrawMultTrans(0, 0.35 * (max[1] - min[1]), 0.5 * (wmax[2] - wmin[2]));
        GDrawMultRot(R);
        m_wheel.Draw();
        GDrawPopTransformation();

        GDrawPushTransformation();
        GDrawMultTrans(m_bodyLengthFrac * (max[0] - min[0]), 0.35 * (max[1] - min[1]), 0.5 * (wmax[2] - wmin[2]));
        GDrawMultRot(Rs);
        GDrawMultRot(R);
        m_wheel.Draw();
        GDrawPopTransformation();
    }
}
