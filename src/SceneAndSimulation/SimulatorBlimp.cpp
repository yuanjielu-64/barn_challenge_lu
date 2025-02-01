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
#include "SimulatorBlimp.hpp"
#include "Components/Constants.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/TriMeshStreamer.hpp"

namespace Antipatrea {
    SimulatorBlimp::SimulatorBlimp(void)
            : SimulatorVehicle() {
        STATE_X = 0;
        STATE_Y = 1;
        STATE_Z = 2;
        STATE_THETA = 3;
        STATE_VELOCITY = 4;
        STATE_STEER_ANGLE = 5;
        CONTROL_ACCELERATION = 0;
        CONTROL_STEER_VELOCITY = 1;
        CONTROL_Z = 2;
        CFG_X = 0;
        CFG_Y = 1;
        CFG_Z = 2;
        CFG_THETA = 3;

        m_bodyHeight = Constants::BLIMP_BODY_HEIGHT;
        m_minCtrlZ = Constants::BLIMP_MIN_CONTROL_Z;
        m_maxCtrlZ = Constants::BLIMP_MAX_CONTROL_Z;

        GetStateAllocator()->SetDim(6);
        GetControlAllocator()->SetDim(3);
        GetCfgAllocator()->SetDim(4);
        GetPositionAllocator()->SetDim(3);
    }

    void SimulatorBlimp::SetupFromParams(Params &params) {
        SimulatorVehicle::SetupFromParams(params);

        SetBodyHeight(params.GetValueAsDouble(Constants::KW_BodyHeight, GetBodyHeight()));
        SetMinControlZ(params.GetValueAsDouble(Constants::KW_MinControlZ, GetMinControlZ()));
        SetMaxControlZ(params.GetValueAsDouble(Constants::KW_MaxControlZ, GetMinControlZ()));

        std::string package_path = ros::package::getPath(Constants::KW_ProjectName);
        auto BLIMP_MESH_FILE = package_path + "/" + Constants::BLIMP_MESH_FILE;

        auto name = params.GetValue(Constants::KW_VehicleMeshFile, BLIMP_MESH_FILE.c_str());
        if (name) {

            TriMeshReader(name, m_meshDraw);

            double R1[Algebra3D::Rot_NR_ENTRIES];
            double R2[Algebra3D::Rot_NR_ENTRIES];


            Algebra3D::XAxisAngleAsRot(Constants::DEG2RAD * 90, R1);
            Algebra3D::ZAxisAngleAsRot(Constants::DEG2RAD * 180, R2);
            Algebra3D::RotMultRotAsRot(R2, R1, R2);
            m_meshDraw.ApplyRot(0, m_meshDraw.GetNrVertices() - 1, R2);

        }


    }

    void SimulatorBlimp::CompleteSetup(void) {
        SimulatorVehicle::CompleteSetup();

        m_meshCollision.Clear();
        m_meshCollision.AddBox(-GetBodyLength(), -0.5 * GetBodyWidth(), -0.5 * GetBodyHeight(), 0.0,
                               0.5 * GetBodyWidth(), GetBodyHeight());

        if (m_meshDraw.GetNrVertices() == 0)
            m_meshDraw.AddTriMesh(m_meshCollision);
        else
            m_meshDraw.AdjustToFitBoundingBoxMinMax(0, m_meshDraw.GetNrVertices() - 1,
                                                    m_meshCollision.GetBoundingBoxMin(),
                                                    m_meshCollision.GetBoundingBoxMax());

    }

    void SimulatorBlimp::SampleState(double s[]) {
        SimulatorVehicle::SampleState(s);
        s[STATE_Z] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[2], GetScene()->GetGrid()->GetMax()[2]);
    }

    void SimulatorBlimp::MotionEqs(const double s[], const double t, const double u[], double ds[]) {
        SimulatorVehicle::MotionEqs(s, t, u, ds);
        ds[STATE_Z] = u[CONTROL_Z];
    }

    bool SimulatorBlimp::IsWithinLimitsState(void) {
        return m_currState[STATE_Z] <= GetScene()->GetGrid()->GetMax()[2] &&
               m_currState[STATE_Z] >= GetScene()->GetGrid()->GetMin()[2] &&
               SimulatorVehicle::IsWithinLimitsState();
    }

    void SimulatorBlimp::SampleControl(double u[]) {
        SimulatorVehicle::SampleControl(u);
        u[CONTROL_Z] = RandomUniformReal(GetMinControlZ(), GetMaxControlZ());
    }

    void SimulatorBlimp::StartSteerToPosition(const double target[]) {
        SimulatorVehicle::StartSteerToPosition(target);
        m_pidZ.Reset();
        m_pidZ.SetDesiredValue(target[2]);
    }

    void SimulatorBlimp::SteerToPosition(const double target[], const bool stop) {
        SimulatorVehicle::SteerToPosition(target, stop);
        m_currControl[CONTROL_Z] = m_pidZ.Update(m_currState[STATE_Z], m_dt);
        if (m_currControl[CONTROL_Z] > m_maxCtrlZ)
            m_currControl[CONTROL_Z] = m_maxCtrlZ;
        else if (m_currControl[CONTROL_Z] < m_minCtrlZ)
            m_currControl[CONTROL_Z] = m_minCtrlZ;
    }


    void SimulatorBlimp::SampleCfg(double cfg[]) {
        SimulatorVehicle::SampleCfg(cfg);
        cfg[CFG_Z] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[2], GetScene()->GetGrid()->GetMax()[2]);
    }

    bool SimulatorBlimp::IsWithinLimitsCfg(const double cfg[]) {
        return m_currState[STATE_Z] <= GetScene()->GetGrid()->GetMax()[2] &&
               m_currState[STATE_Z] >= GetScene()->GetGrid()->GetMin()[2] &&
               SimulatorVehicle::IsWithinLimitsCfg(cfg);
    }

    void SimulatorBlimp::PathCfgs(const double cfg1[], const double cfg2[], const double t, double cfg[]) {
        SimulatorVehicle::PathCfgs(cfg1, cfg2, t, cfg);
        cfg[CFG_Z] = cfg1[CFG_Z] + t * (cfg2[CFG_Z] - cfg1[CFG_Z]);
    }


    void SimulatorBlimp::DrawPosition(const double pos[]) const {
        GDrawSphere3D(pos, GetDrawPositionRadius());
    }


    void SimulatorBlimp::AddToMeshCfg(TriMesh &tmesh, const double cfg[]) const {
        auto ptr = const_cast<TriMeshDefault *>(&m_meshCollision);

        double TR[Algebra3D::TransRot_NR_ENTRIES];

        TR[0] = cfg[CFG_X];
        TR[1] = cfg[CFG_Y];
        TR[2] = cfg[CFG_Z];
        Algebra3D::ZAxisAngleAsRot(cfg[CFG_THETA], &TR[Algebra3D::Trans_NR_ENTRIES]);
        const int n = tmesh.GetNrVertices();
        tmesh.AddTriMesh(*ptr);
        tmesh.ApplyTransRot(n, tmesh.GetNrVertices() - 1, TR);

    }

    void
    SimulatorBlimp::AddToMeshPathCfgs(TriMesh &tmesh, const double cfg1[], const double cfg2[], const double tstart,
                                      const double dt, const double tend) {
        //Simulator::AddToMeshPathCfgs(tmesh, cfg1, cfg2, tstart, dt, tend);

        //return;

        tmesh.Clear();

        const double *pmin = m_meshCollision.GetBoundingBoxMin();
        const double *pmax = m_meshCollision.GetBoundingBoxMax();
        const double dmax = Constants::SIMULATOR_BLIMP_PATH_CFG_WIDTH;//0.15 * std::min(pmax[0] - pmin[0], std::max(pmax[1] - pmin[1], pmax[2] - pmin[2]));
        const double d = Algebra3D::PointDistance(cfg1, cfg2);

        if (d <= Constants::EPSILON)
            return;

        const double xaxis[] = {1, 0, 0};
        const double other[] = {(cfg2[0] - cfg1[0]) / d, (cfg2[1] - cfg1[1]) / d, (cfg2[2] - cfg1[2]) / d};
        double TR[Algebra3D::TransRot_NR_ENTRIES];

        tmesh.AddBox(-0.5 * d - dmax, -dmax, -dmax,
                     0.5 * d + dmax, dmax, dmax);

        TR[0] = 0.5 * (cfg1[0] + cfg2[0]);
        TR[1] = 0.5 * (cfg1[1] + cfg2[1]);
        TR[2] = 0.5 * (cfg1[2] + cfg2[2]);
        Algebra3D::FromToUnitAxisAsRot(xaxis, other, &TR[Algebra3D::Trans_NR_ENTRIES]);
        tmesh.ApplyTransRot(0, tmesh.GetNrVertices() - 1, TR);

    }

    void SimulatorBlimp::DrawCfg(const double cfg[]) const {
        //place camera in proper transformation
        double R[Algebra3D::Rot_NR_ENTRIES];

        GMaterial mat;
        mat.SetRuby();
        GDrawMaterial(mat);


        Algebra3D::ZAxisAngleAsRot(cfg[CFG_THETA], R);
        GDrawPushTransformation();
        GDrawMultTransRot(cfg, R);
        const_cast<TriMesh *>(&m_meshDraw)->Draw();
        //const_cast<TriMeshDefault*>(&m_meshCollision)->Draw();
        GDrawPopTransformation();

    }

    void SimulatorBlimp::DrawState(void) {
        DrawCfg(m_currState);

    }

}
