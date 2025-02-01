#ifndef BARN_CHALLENGE_LU_SIMULATORTWOWHEELSVEHICLE_HPP
#define BARN_CHALLENGE_LU_SIMULATORTWOWHEELSVEHICLE_HPP

#include "SimulatorMotionEqs.hpp"
#include "Utils/PIDController.hpp"

namespace Antipatrea {
    class SimulatorJackal : public SimulatorMotionEqs {
    public:
        SimulatorJackal();

        virtual ~SimulatorJackal() {}

        int STATE_X;
        int STATE_Y;
        int STATE_THETA;
        int STATE_VELOCITY;
        int STATE_ANGULAR_VELOCITY;

        int CONTROL_ACCELERATION;
        int CONTROL_ANGULAR_VELOCITY;

        int CFG_X;
        int CFG_Y;
        int CFG_THETA;

        // parameter setup
        virtual void SetBodyLength(const double d) {
            m_bodyLength = d;
        }

        virtual void SetBodyWidth(const double d) {
            m_bodyWidth = d;
        }

        virtual void SetWheelBase(const double d) {
            m_wheelBase = d;
        };

        virtual void SetMinAngularVelocity(const double v) {
            m_minAngularVelocity = v;
        }

        virtual void SetMaxAngularVelocity(const double v) {
            m_maxAngularVelocity = v;
        }

        virtual void SetMinAngularAcceleration(const double a) {
            m_minAngularAcceleration = a;
        }

        virtual void SetMaxAngularAcceleration(const double a) {
            m_maxAngularAcceleration = a;
        }

        virtual void SetMinVelocity(const double v) {
            m_minVelocity = v;
        }

        virtual void SetMaxVelocity(const double v) {
            m_maxVelocity = v;
        }

        virtual void SetMinAcceleration(const double a) {
            m_minAcceleration = a;
        }

        virtual void SetMaxAcceleration(const double a) {
            m_maxAcceleration = a;
        }

        virtual void SetupFromParams(Params &params);

        virtual void SetupFromParams(Params &params, Robot_config &robot);

        virtual double GetBodyLength(void) const {
            return m_bodyLength;
        }

        virtual double GetBodyWidth(void) const {
            return m_bodyWidth;
        }

        virtual double GetWheelBase() const {
            return m_wheelBase;
        }

        virtual double GetMinAngularVelocity() const {
            return m_minAngularVelocity;
        }

        virtual double GetMaxAngularVelocity() const {
            return m_maxAngularVelocity;
        }

        virtual double GetMinAngularAcceleration() const {
            return m_minAngularAcceleration;
        }

        virtual double GetMaxAngularAcceleration() const {
            return m_maxAngularAcceleration;
        }

        virtual double GetMinVelocity(void) const {
            return m_minVelocity;
        }

        virtual double GetMaxVelocity(void) const {
            return m_maxVelocity;
        }

        virtual double GetMinAcceleration(void) const {
            return m_minAcceleration;
        }

        virtual double GetMaxAcceleration(void) const {
            return m_maxAcceleration;
        }

        virtual double GetVelocity(void) {
            return m_currState[STATE_VELOCITY];
        }

        virtual double GetAngularVelocity(void) {
            return m_currState[STATE_ANGULAR_VELOCITY];
        }

        virtual double GetAcceleration(void) {
            return m_currControl[CONTROL_ACCELERATION];
        }

        virtual void SampleState(double s[]);

        virtual void SampleControl(double u[]);

        virtual void MotionEqs(const double s[], const double t, const double u[], double ds[]);

        virtual bool IsWithinLimitsState(void);

        // distances
        virtual double DistanceStates(const double s1[], const double s2[]) const {
            return Algebra::PointDistance(GetPositionAllocator()->GetDim(), s1, s2);
        }

        virtual double DistanceStatePosition(const double s[], const double pos[]) const {
            return Algebra::PointDistance(GetPositionAllocator()->GetDim(), s, pos);
        }

        virtual double DistanceStateCfg(const double s[], const double cfg[]) const {
            return Algebra::PointDistance(GetPositionAllocator()->GetDim(), s, cfg);
        }

        virtual double DistanceCfgs(const double cfg1[], const double cfg2[]) const {
            return Algebra::PointDistance(GetPositionAllocator()->GetDim(), cfg1, cfg2);
        }

        virtual double DistancePositions(const double p1[], const double p2[]) const {
            return Algebra::PointDistance(GetPositionAllocator()->GetDim(), p1, p2);
        }

        // projections and unprojections
        virtual void GetPositionState(const double s[], double pos[]) const {
            pos[0] = s[STATE_X];
            pos[1] = s[STATE_Y];
        }

        virtual void GetCfgState(const double s[], double cfg[]) const {
            cfg[CFG_X] = s[STATE_X];
            cfg[CFG_Y] = s[STATE_Y];
            cfg[CFG_THETA] = s[STATE_THETA];
        }

        virtual void GetPositionCfg(const double cfg[], double pos[]) const {
            pos[0] = cfg[CFG_X];
            pos[1] = cfg[CFG_Y];
        }

        virtual void SetPositionState(const double pos[], double s[]) const {
            s[STATE_X] = pos[0];
            s[STATE_Y] = pos[1];
        }

        virtual void SetCfgState(const double cfg[], double s[]) const {
            s[STATE_X] = cfg[CFG_X];
            s[STATE_Y] = cfg[CFG_Y];
            s[STATE_THETA] = cfg[CFG_THETA];
        }

        virtual void SetPositionCfg(const double pos[], double cfg[]) const {
            cfg[CFG_X] = pos[0];
            cfg[CFG_Y] = pos[1];
        }

        // control functionality
        virtual void StartSteerToPosition(const double target[]);

        virtual void StartToPosition(double v, double w);

        virtual void SteerToPosition(const double target[], const bool stop = false);

        virtual void ToPosition(double v, double w);

        virtual void SampleCfg(double cfg[]);

        virtual bool IsWithinLimitsCfg(const double cfg[]);

        virtual void PathCfgs(const double cfg1[], const double cfg2[], const double t, double cfg[]);

    protected:
        double m_bodyLength;
        double m_bodyWidth;
        double m_wheelBase;
        double m_minAngularVelocity;
        double m_maxAngularVelocity;
        double m_minAngularAcceleration;
        double m_maxAngularAcceleration;
        double m_minVelocity;
        double m_maxVelocity;
        double m_minAcceleration;
        double m_maxAcceleration;
        PIDController m_pidAngularVel;
        PIDController m_pidVel;
    };
}
#endif
