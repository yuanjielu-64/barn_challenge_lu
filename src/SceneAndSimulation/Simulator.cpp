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
#include "Simulator.hpp"
#include "Components/Constants.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/Constants.hpp"
#include "Utils/Misc.hpp"
#include "Utils/TriMeshDefault.hpp"

namespace Antipatrea {
    Simulator::Simulator()
            : Component(), SceneContainer(), m_dt(Constants::SIMULATOR_TIME_STEP),
              m_minDistanceOneStep(Constants::SIMULATOR_MIN_DISTANCE_ONE_STEP),
              m_maxDistanceOneStep(Constants::SIMULATOR_MAX_DISTANCE_ONE_STEP),
              m_distanceReachedSteerPosition(Constants::SIMULATOR_DISTANCE_REACHED_STEER_POSITION),
              m_stopDistance(Constants::SIMULATOR_STOP_DISTANCE), m_stopExponent(Constants::SIMULATOR_STOP_EXPONENT),
              m_velocityScaleSampling(Constants::SIMULATOR_VELOCITY_SCALE_SAMPLING),
              m_velocityScaleConversion(Constants::SIMULATOR_VELOCITY_SCALE_CONVERSION),
              m_velocityScaleMin(Constants::SIMULATOR_VELOCITY_SCALE_MIN),
              m_drawPositionRadius(Constants::SIMULATOR_DRAW_POSITION_RADIUS), m_currState(NULL), m_currControl(NULL) {
    }

    Simulator::~Simulator() {
        GetStateAllocator()->Delete(m_currState);
        GetControlAllocator()->Delete(m_currControl);
    }

    void Simulator::SetupFromParams(Params &params) {
        Component::SetupFromParams(params);
        SetTimeStep(params.GetValueAsDouble(Constants::KW_TimeStep, GetTimeStep()));
        SetMinDistanceOneStep(params.GetValueAsDouble(Constants::KW_MinDistanceOneStep, GetMinDistanceOneStep()));
        SetMaxDistanceOneStep(params.GetValueAsDouble(Constants::KW_MaxDistanceOneStep, GetMaxDistanceOneStep()));
        SetStopDistance(params.GetValueAsDouble(Constants::KW_StopDistance, GetStopDistance()));
        SetStopExponent(params.GetValueAsDouble(Constants::KW_StopExponent, GetStopExponent()));
        SetDistanceReachedSteerPosition(
                params.GetValueAsDouble(Constants::KW_DistanceReachedSteerPosition, GetDistanceReachedSteerPosition()));
        SetVelocityScaleSampling(
                params.GetValueAsDouble(Constants::KW_VelocityScaleSampling, GetVelocityScaleSampling()));
        SetVelocityScaleConversion(
                params.GetValueAsDouble(Constants::KW_VelocityScaleConversion, GetVelocityScaleConversion()));
        SetVelocityScaleMin(params.GetValueAsDouble(Constants::KW_VelocityScaleMin, GetVelocityScaleMin()));
        SetDrawPositionRadius(params.GetValueAsDouble(Constants::KW_DrawPositionRadius, GetDrawPositionRadius()));
    }

    void Simulator::CompleteSetup(void) {
        m_currState = GetStateAllocator()->New();
        GetStateAllocator()->Fill(m_currState, 0.0);

        m_currControl = GetControlAllocator()->New();
        GetControlAllocator()->Fill(m_currControl, 0.0);
    }

    void Simulator::AddToMeshState(TriMesh &tmesh) {
        std::vector<double> cfg;
        cfg.resize(GetCfgAllocator()->GetDim());
        GetCfgState(m_currState, &cfg[0]);
        AddToMeshCfg(tmesh, &cfg[0]);
    }

    bool Simulator::IsObstacleCollisionFreeState(void) {
        TriMeshDefault tmesh;
        AddToMeshState(tmesh);
        return tmesh.Collision(NULL, NULL, GetScene()->GetObstaclesCollisionMesh(), NULL, NULL) == false;
    }

    bool Simulator::IsObstacleCollisionFreeCfg(const double cfg[]) {
        TriMeshDefault tmesh;
        AddToMeshCfg(tmesh, cfg);
        return tmesh.Collision(NULL, NULL, GetScene()->GetObstaclesCollisionMesh(), NULL, NULL) == false;
    }

    void Simulator::PathCfgs(const double cfg1[], const double cfg2[], const double t, double cfg[]) {
        for (int i = GetCfgAllocator()->GetDim() - 1; i >= 0; --i)
            cfg[i] = (1 - t) * cfg1[i] + t * cfg2[i];
    }

    void Simulator::AddToMeshPathCfgs(TriMesh &tmesh, const double cfg1[], const double cfg2[], const double tstart,
                                      const double dt, const double tend) {
        std::vector<double> cfg;
        cfg.resize(GetCfgAllocator()->GetDim());
        for (double t = tstart; t <= (tend + Constants::EPSILON); t += dt) {
            PathCfgs(cfg1, cfg2, t, &cfg[0]);
            AddToMeshCfg(tmesh, &cfg[0]);
        }
    }

    bool
    Simulator::IsWithinLimitsPathCfgs(const double cfg1[], const double cfg2[], const double tstart, const double dt,
                                      const double tend) {
        std::vector<double> cfg;
        cfg.resize(GetCfgAllocator()->GetDim());
        for (double t = tstart; t <= (tend + Constants::EPSILON); t += dt) {
            PathCfgs(cfg1, cfg2, t, &cfg[0]);
            if (IsWithinLimitsCfg(&cfg[0]) == false)
                return false;
        }
        return true;
    }

    bool Simulator::IsObstacleCollisionFreePathCfgs(const double cfg1[], const double cfg2[], const double tstart,
                                                    const double dt, const double tend) {
        TriMeshDefault tmesh;
        AddToMeshPathCfgs(tmesh, cfg1, cfg2, tstart, dt, tend);
        return tmesh.Collision(NULL, NULL, GetScene()->GetObstaclesCollisionMesh(), NULL, NULL) == false;
    }

    bool Simulator::IsSelfCollisionFreePathCfgs(const double cfg1[], const double cfg2[], const double tstart,
                                                const double dt, const double tend) {
        std::vector<double> cfg;
        cfg.resize(GetCfgAllocator()->GetDim());
        for (double t = tstart; t <= (tend + Constants::EPSILON); t += dt) {
            PathCfgs(cfg1, cfg2, t, &cfg[0]);
            if (IsSelfCollisionFreeCfg(&cfg[0]) == false)
                return false;
        }
        return true;
    }

// distances
    double Simulator::DistanceStatePosition(const double s[], const double pos[]) const {
        std::vector<double> spos;
        spos.resize(GetPositionAllocator()->GetDim());
        GetPositionState(s, &spos[0]);
        return DistancePositions(&spos[0], pos);
    }

    double Simulator::DistanceStateCfg(const double s[], const double cfg[]) const {
        std::vector<double> scfg;
        scfg.resize(GetCfgAllocator()->GetDim());
        GetCfgState(s, &scfg[0]);
        return DistanceCfgs(&scfg[0], cfg);
    }

// clearances
    double Simulator::ClearancePosition(const double pos[]) {
        TriMeshDefault tmesh;
        double bbox[6];

        const int nrDims = GetScene()->GetGrid()->GetNrDims();
        const double hdim = Constants::EPSILON;

        for (int j = 0; j < nrDims; ++j) {
            bbox[j] = pos[j] - hdim;
            bbox[j + nrDims] = pos[j] + hdim;
        }
        if (nrDims == 2)
            tmesh.AddBox2D(bbox[0], bbox[1], bbox[2], bbox[3]);
        else
            tmesh.AddBox(bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5]);
        return tmesh.Distance(NULL, NULL, GetScene()->GetObstaclesCollisionMesh(), NULL, NULL);
    }

    double Simulator::ClearanceSegment(const double p1[], const double p2[]) {
        TriMeshDefault tmesh;
        double bbox[6];

        const int nrDims = GetScene()->GetGrid()->GetNrDims();
        const double hdim = Constants::EPSILON;

        if (nrDims == 2) {
            const double skel[] = {p1[0], p1[1], p2[0], p2[1]};
            std::vector<double> vertices;
            Polygon2D poly;

            FromSkeletonToPolygon2D(2, skel, 2 * hdim, vertices);
            poly.AddVertices(vertices.size() / 2, &vertices[0]);
            tmesh.AddPolygon(poly);
        } else {
            const double d = Algebra3D::PointDistance(p1, p2);
            const double bbox[] = {-0.5 * d, -hdim, -hdim, 0.5 * d, hdim, hdim};
            tmesh.AddBox(bbox[0], bbox[1], bbox[2], bbox[3], bbox[4], bbox[5]);

            const double xaxis[] = {1, 0, 0};
            const double other[] = {(p2[0] - p1[0]) / d, (p2[1] - p1[1]) / d, (p2[2] - p1[2]) / d};
            double TR[Algebra3D::TransRot_NR_ENTRIES];

            TR[0] = 0.5 * (p1[0] + p2[0]);
            TR[1] = 0.5 * (p1[1] + p2[1]);
            TR[2] = 0.5 * (p1[2] + p2[2]);
            Algebra3D::FromToUnitAxisAsRot(xaxis, other, &TR[Algebra3D::Trans_NR_ENTRIES]);
            tmesh.ApplyTransRot(0, tmesh.GetNrVertices() - 1, TR);
        }

        return tmesh.Distance(NULL, NULL, GetScene()->GetObstaclesCollisionMesh(), NULL, NULL);
    }

    double Simulator::ClearanceCfg(const double cfg[]) {
        TriMeshDefault tmesh;
        AddToMeshCfg(tmesh, cfg);
        return tmesh.Distance(NULL, NULL, GetScene()->GetObstaclesCollisionMesh(), NULL, NULL);
    }

    double Simulator::ClearancePathCfgs(const double cfg1[], const double cfg2[], const double tstart, const double dt,
                                        const double tend) {
        TriMeshDefault tmesh;
        AddToMeshPathCfgs(tmesh, cfg1, cfg2, tstart, dt, tend);
        return tmesh.Distance(NULL, NULL, GetScene()->GetObstaclesCollisionMesh(), NULL, NULL) == false;
    }

    double Simulator::ClearanceState(void) {
        TriMeshDefault tmesh;
        AddToMeshState(tmesh);
        return tmesh.Distance(NULL, NULL, GetScene()->GetObstaclesCollisionMesh(), NULL, NULL);
    }

// other functionality
    double Simulator::TimeStepConstantVelocity(const double v, const double d) const {
        const double absv = fabs(v);

        return absv < Constants::EPSILON ? (d / Constants::EPSILON) : (d / absv);
    }

    double Simulator::TimeStepConstantAcceleration(const double v, const double a, const double d) const {
        if (fabs(a) < Constants::EPSILON)
            return TimeStepConstantVelocity(v, d);

        if (a >= 0.0 && v >= 0.0)
            return fabs((-v + sqrt(v * v + 2 * a * d)) / a);
        else if (a <= 0.0 && v <= 0.0)
            return fabs((v + sqrt(fabs(v * v - 2 * a * d))) / (-a));
        else if (a <= 0.0 && v >= 0.0) {
            const double b2_4ac = v * v + 2 * a * d;
            if (b2_4ac >= 0.0)
                return fabs((-v + sqrt(b2_4ac)) / a);
            const double dist_remaining = d - v * v / (2 * a);
            // traveled from speed zero in reverse
            return sqrt(2 * dist_remaining / fabs(a));
        } else if (a >= 0 && v <= 0.0) {
            const double b2_4ac = v * v - 2 * a * d;
            if (b2_4ac >= 0.0)
                return fabs((-v - sqrt(b2_4ac)) / a);
            const double dist_remaining = d + v * v / (2 * a);
            return sqrt(2 * dist_remaining / a);
        }

        return 0.0;
    }

    void Simulator::DrawCfg(const double cfg[]) const {
        TriMesh tmesh;
        AddToMeshCfg(tmesh, cfg);
        tmesh.Draw();
    }
}
