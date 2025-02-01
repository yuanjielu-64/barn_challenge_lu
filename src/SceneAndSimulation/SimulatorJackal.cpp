#include "SimulatorJackal.hpp"
#include "Components/Constants.hpp"
#include "Utils/Algebra2D.hpp"

namespace Antipatrea {

    SimulatorJackal::SimulatorJackal(void)
            : SimulatorMotionEqs() {

        STATE_X = 0;
        STATE_Y = 1;
        STATE_THETA = 2;
        STATE_VELOCITY = 3;
        STATE_ANGULAR_VELOCITY = 4;

        CONTROL_ACCELERATION = 0;
        CONTROL_ANGULAR_VELOCITY = 1;

        CFG_X = 0;
        CFG_Y = 1;
        CFG_THETA = 2;

        m_bodyLength = Constants::ROBOT_BODY_LENGTH;
        m_bodyWidth = Constants::ROBOT_BODY_WIDTH;
        m_wheelBase = Constants::ROBOT_WHEEL_BASE;
        m_minAngularVelocity = Constants::ROBOT_MIN_ANGULAR_VELOCITY;
        m_maxAngularVelocity = Constants::ROBOT_MAX_ANGULAR_VELOCITY;
        m_minVelocity = Constants::ROBOT_MIN_VELOCITY;
        m_maxVelocity = Constants::ROBOT_MAX_VELOCITY;
        m_minAcceleration = Constants::ROBOT_MIN_ACCELERATION;
        m_maxAcceleration = Constants::ROBOT_MAX_ACCELERATION;
    }

    void SimulatorJackal::SetupFromParams(Params &params) {
        SimulatorMotionEqs::SetupFromParams(params);

        SetBodyLength(params.GetValueAsDouble(Constants::KW_RobotBodyLength, GetBodyLength()));
        SetBodyWidth(params.GetValueAsDouble(Constants::KW_RobotBodyWidth, GetBodyWidth()));
        SetWheelBase(params.GetValueAsDouble(Constants::KW_RobotWheelBase, GetWheelBase()));
        SetMinAngularVelocity(params.GetValueAsDouble(Constants::KW_RobotMinAngularVelocity,GetMinAngularVelocity()));
        SetMaxAngularVelocity(params.GetValueAsDouble(Constants::KW_RobotMaxAngularVelocity,GetMaxAngularVelocity()));
        SetMinAngularAcceleration(params.GetValueAsDouble(Constants::KW_RobotMinAngularAcceleration,GetMinAngularAcceleration()));
        SetMaxAngularAcceleration(params.GetValueAsDouble(Constants::KW_RobotMaxAngularAcceleration,GetMaxAngularAcceleration()));
        SetMinVelocity(params.GetValueAsDouble(Constants::KW_RobotMinVelocity, GetMinVelocity()));
        SetMaxVelocity(params.GetValueAsDouble(Constants::KW_RobotMaxVelocity, GetMaxVelocity()));
        SetMinAcceleration(params.GetValueAsDouble(Constants::KW_RobotMinAcceleration, GetMinAcceleration()));
        SetMaxAcceleration(params.GetValueAsDouble(Constants::KW_RobotMaxAcceleration, GetMaxAcceleration()));
    }

    void SimulatorJackal::SetupFromParams(Params &params, Robot_config &robot) {
        // SetBodyLength(robot.getRobotSize()[0]);
        // SetBodyWidth(robot.getRobotSize()[1]);
        // SetMinVelocity(robot.getRobotSize()[2]);
        // SetMaxVelocity(robot.getRobotSize()[3]);
    }

    void SimulatorJackal::SampleState(double s[]) {
        s[STATE_X] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[0], GetScene()->GetGrid()->GetMax()[0]);
        s[STATE_Y] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[1], GetScene()->GetGrid()->GetMax()[1]);
        s[STATE_THETA] = RandomUniformReal(-M_PI, M_PI);
        s[STATE_VELOCITY] = RandomUniformReal(m_minVelocity, m_maxVelocity);
        s[STATE_ANGULAR_VELOCITY] = RandomUniformReal(m_minAngularVelocity, m_maxAngularVelocity);
    }

    void SimulatorJackal::MotionEqs(const double s[], const double t, const double u[], double ds[]) {
        ds[STATE_X] = s[STATE_VELOCITY] * cos(s[STATE_THETA]);
        ds[STATE_Y] = s[STATE_VELOCITY] * sin(s[STATE_THETA]);
        ds[STATE_THETA] = s[STATE_ANGULAR_VELOCITY];

        ds[STATE_VELOCITY] = u[CONTROL_ACCELERATION];
        ds[STATE_ANGULAR_VELOCITY] = u[CONTROL_ANGULAR_VELOCITY];
    }

    bool SimulatorJackal::IsWithinLimitsState(void) {
        // const double asteer = Algebra2D::AngleNormalize(m_currState[STATE_STEER_ANGLE], -M_PI);
        const double *pmin = GetScene()->GetGrid()->GetMin();
        const double *pmax = GetScene()->GetGrid()->GetMax();

        return /*asteer >= GetMinSteerAngle() && asteer <= GetMaxSteerAngle() &&*/
                m_currState[STATE_X] >= pmin[0] && m_currState[STATE_X] <= pmax[0] && m_currState[STATE_Y] >= pmin[1] &&
                m_currState[STATE_Y] <= pmax[1] /*&& m_currState[STATE_VELOCITY] >= GetMinVelocity() &&
									      m_currState[STATE_VELOCITY] <= GetMaxVelocity()*/;
    }

    void SimulatorJackal::SampleControl(double u[]) {
        u[CONTROL_ACCELERATION] = RandomUniformReal(m_minAcceleration, m_maxAcceleration);

    }

    void SimulatorJackal::StartSteerToPosition(const double target[]) {
        m_pidAngularVel.Reset();
        m_pidVel.Reset();

        m_pidAngularVel.SetDesiredValue(0.0);
        m_pidVel.SetDesiredValue(RandomUniformReal(GetVelocityScaleSampling(), 1.0) * m_maxVelocity);
    }

    void SimulatorJackal::StartToPosition(double v, double w) {
//        m_pidVel.Reset();
//        m_pidAngle.Reset();
//
//        m_pidVel.SetDesiredValue(v);
//        m_pidAngle.SetDesiredValue(w);
    }

    void SimulatorJackal::ToPosition(double v, double w) {
//        m_currControl[CONTROL_ACCELERATION] = v;
//        m_currControl[CONTROL_ANGULAR_VELOCITY] = w;
//        m_currControl[CONTROL_ANGULAR_VELOCITY] = m_pidAngle.Update(m_currState[STATE_ANGULAR_VELOCITY], m_dt);
//        m_currControl[CONTROL_ANGULAR_VELOCITY] = std::clamp(m_currControl[CONTROL_ANGULAR_VELOCITY], m_minAngularAcceleration, m_maxAngularAcceleration);
//
//        m_currControl[CONTROL_ACCELERATION] = m_pidVel.Update(m_currState[STATE_VELOCITY], m_dt);
//        m_currControl[CONTROL_ACCELERATION] = std::clamp(m_currControl[CONTROL_ACCELERATION], m_minAcceleration, m_maxAcceleration);

    }

    void SimulatorJackal::SteerToPosition(const double target[], const bool stop) {

        const double u[2] = {target[0] - m_currState[STATE_X], target[1] - m_currState[STATE_Y]};

        const double v[2] = {cos(m_currState[STATE_THETA]),
                             sin(m_currState[STATE_THETA])};

        m_currControl[CONTROL_ANGULAR_VELOCITY] = m_pidAngularVel.Update(Algebra2D::VecFromToAngleCCW(u, v), m_dt);

        if (m_currControl[CONTROL_ANGULAR_VELOCITY] > m_maxAngularVelocity)
            m_currControl[CONTROL_ANGULAR_VELOCITY] = m_maxAngularVelocity;
        else if (m_currControl[CONTROL_ANGULAR_VELOCITY] < m_minAngularVelocity)
            m_currControl[CONTROL_ANGULAR_VELOCITY] = m_minAngularVelocity;

        if (stop) {
            const double use = std::min(1.0, Algebra2D::VecNorm(u) / GetStopDistance());

            m_pidVel.SetDesiredValue(m_maxVelocity * pow(use, GetStopExponent()));
        }

        m_currControl[CONTROL_ACCELERATION] = m_pidVel.Update(m_currState[STATE_VELOCITY], m_dt);
        if (m_currControl[CONTROL_ACCELERATION] > m_maxAcceleration)
            m_currControl[CONTROL_ACCELERATION] = m_maxAcceleration;
        else if (m_currControl[CONTROL_ACCELERATION] < m_minAcceleration)
            m_currControl[CONTROL_ACCELERATION] = m_minAcceleration;

    }

    void SimulatorJackal::SampleCfg(double cfg[]) {
        cfg[CFG_X] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[0], GetScene()->GetGrid()->GetMax()[0]);
        cfg[CFG_Y] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[1], GetScene()->GetGrid()->GetMax()[1]);
        cfg[CFG_THETA] = RandomUniformReal(-M_PI, M_PI);
    }

    bool SimulatorJackal::IsWithinLimitsCfg(const double cfg[]) {
        const double *pmin = GetScene()->GetGrid()->GetMin();
        const double *pmax = GetScene()->GetGrid()->GetMax();

        return cfg[CFG_X] >= pmin[0] && cfg[CFG_X] <= pmax[0] && cfg[CFG_Y] >= pmin[1] && cfg[CFG_Y] <= pmax[1];
    }

    void SimulatorJackal::PathCfgs(const double cfg1[], const double cfg2[], const double t, double cfg[]) {
        cfg[CFG_X] = cfg1[CFG_X] + t * (cfg2[CFG_X] - cfg1[CFG_X]);
        cfg[CFG_Y] = cfg1[CFG_Y] + t * (cfg2[CFG_Y] - cfg1[CFG_Y]);
        cfg[CFG_THETA] = cfg1[CFG_THETA] + t * Algebra2D::AngleSignedDistance(cfg1[CFG_THETA], cfg2[CFG_THETA]);
    }
}
