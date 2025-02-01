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

#ifndef Antipatrea__SimulatorBlimp_HPP_
#define Antipatrea__SimulatorBlimp_HPP_

#include "Car.hpp"
#include "SimulatorVehicle.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/PIDController.hpp"
#include "Utils/TriMeshDefault.hpp"
#include <vector>

namespace Antipatrea {
    class SimulatorBlimp : public SimulatorVehicle {
    public:
        SimulatorBlimp(void);

        virtual ~SimulatorBlimp(void) {
        }

        int STATE_Z;
        int CONTROL_Z;
        int CFG_Z;

        // parameter setup
        virtual void SetBodyHeight(const double h) {
            m_bodyHeight = h;
        }

        virtual void SetMinControlZ(const double u) {
            m_minCtrlZ = u;
        }

        virtual void SetMaxControlZ(const double u) {
            m_maxCtrlZ = u;
        }

        virtual void SetupFromParams(Params &params);

        virtual void CompleteSetup(void);

        // parameter access
        virtual double GetBodyHeight(void) const {
            return m_bodyHeight;
        }

        virtual double GetMinControlZ(void) const {
            return m_minCtrlZ;
        }

        virtual double GetMaxControlZ(void) const {
            return m_maxCtrlZ;
        }

        // state functionality
        virtual void SampleState(double s[]);

        virtual bool IsWithinLimitsState(void);

        virtual bool IsSelfCollisionFreeState(void) {
            return true;
        }

        // control functionality
        virtual void SampleControl(double u[]);

        virtual void StartSteerToPosition(const double target[]);

        virtual void SteerToPosition(const double target[], const bool stop = false);

        // cfg functionality
        virtual void SampleCfg(double cfg[]);

        virtual void AddToMeshCfg(TriMesh &tmesh, const double cfg[]) const;


        virtual bool IsWithinLimitsCfg(const double cfg[]);

        // cfg path functionality
        virtual void PathCfgs(const double cfg1[], const double cfg2[], const double t, double cfg[]);

        // projections and unprojections
        virtual void GetPositionState(const double s[], double pos[]) const {
            SimulatorVehicle::GetPositionState(s, pos);
            pos[2] = s[STATE_Z];
        }

        virtual void GetCfgState(const double s[], double cfg[]) const {
            SimulatorVehicle::GetCfgState(s, cfg);
            cfg[CFG_Z] = s[STATE_Z];
        }

        virtual void GetPositionCfg(const double cfg[], double pos[]) const {
            SimulatorVehicle::GetPositionCfg(cfg, pos);
            pos[2] = cfg[CFG_Z];
        }

        virtual void SetPositionState(const double pos[], double s[]) const {
            SimulatorVehicle::SetPositionState(pos, s);
            s[STATE_Z] = pos[2];
        }

        virtual void SetCfgState(const double cfg[], double s[]) const {
            SimulatorVehicle::SetCfgState(cfg, s);
            s[STATE_Z] = cfg[CFG_Z];
        }

        virtual void SetPositionCfg(const double pos[], double cfg[]) const {
            SimulatorVehicle::SetPositionCfg(pos, cfg);
            cfg[CFG_Z] = pos[2];
        }

        // other functionality
        virtual double GetVelocityState(const double s[]) const {
            return s[STATE_VELOCITY];
        }

        virtual void MotionEqs(const double s[], const double t, const double u[], double ds[]);

        virtual void AddToMeshPathCfgs(TriMesh &tmesh, const double cfg1[], const double cfg2[], const double tstart,
                                       const double dt, const double tend);

        virtual void DrawState(void);

        virtual void DrawCfg(const double cfg[]) const;

        virtual void DrawPosition(const double pos[]) const;

        virtual void DrawSegment(const double pos1[], const double pos2[]) const {
            GDrawSegment3D(pos1, pos2);
        }

    protected:
        double m_bodyHeight;
        double m_minCtrlZ;
        double m_maxCtrlZ;

        PIDController m_pidSteer;
        PIDController m_pidVel;
        PIDController m_pidZ;

        TriMesh m_meshDraw;
        TriMeshDefault m_meshCollision;
    };
}

#endif
