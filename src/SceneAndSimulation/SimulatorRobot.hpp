#ifndef BARN_CHALLENGE_LU_SIMULATORJACKAL_HPP
#define BARN_CHALLENGE_LU_SIMULATORJACKAL_HPP

#include "Robot.hpp"
#include "SimulatorJackal.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/GDraw.hpp"
#include "Utils/PIDController.hpp"
#include "Utils/TriMeshDefault.hpp"
#include <vector>

namespace Antipatrea {
    class SimulatorRobot : public SimulatorJackal {
    public:
        SimulatorRobot(void);

        virtual ~SimulatorRobot(void) {
        }

        // parameter setup

        virtual void SetAttachDistance(const double d) {
            m_attachDistance = d;
        }

        virtual void SetupFromParams(Params &params);

        virtual void SetupFromParams(Params &params, Robot_config &robot);

        virtual void CompleteSetup(void);

        // parameter access

        virtual double GetAttachDistance(void) const {
            return m_attachDistance;
        }

        // state functionality
        virtual void SetState(const double s[]);

        virtual void SampleState(double s[]);

        virtual void AddToMeshState(TriMesh &tmesh);

        virtual bool IsSelfCollisionFreeState(void);

        // cfg functionality
        virtual void AddToMeshCfg(TriMesh &tmesh, const double cfg[]) const;

        // other functionality
        virtual double GetVelocityState(const double s[]) const {
            return s[STATE_VELOCITY];
        }

        virtual void AddToMeshPathCfgs(TriMesh &tmesh, const double cfg1[], const double cfg2[], const double tstart,
                                       const double dt, const double tend);


        virtual void MotionEqs(const double s[], const double t, const double u[], double ds[]);

        virtual void DrawState(void);

        virtual void DrawPosition(const double pos[]) const;

        virtual void DrawSegment(const double pos1[], const double pos2[]) const {
            GDrawSegment2D(pos1, pos2);
        }

    protected:
        double m_attachDistance;

        std::vector<double> m_bodies;
        double m_orig[8];
        double m_origScaled[8];

        std::vector<double> m_TRs;
        TriMeshDefault m_tmeshBodies;
        Robot m_robot;
    };
}


#endif //BARN_CHALLENGE_LU_SIMULATORJACKAL_HPP
