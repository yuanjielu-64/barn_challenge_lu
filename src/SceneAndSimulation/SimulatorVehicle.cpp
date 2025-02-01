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
#include "SimulatorVehicle.hpp"
#include "Components/Constants.hpp"
#include "Utils/Algebra2D.hpp"

namespace Antipatrea {

    SimulatorVehicle::SimulatorVehicle(void)
            : SimulatorMotionEqs() {

        STATE_X = 0;
        STATE_Y = 1;
        STATE_THETA = 2;
        STATE_VELOCITY = 3;
        STATE_STEER_ANGLE = 4;
        CONTROL_ACCELERATION = 0;
        CONTROL_STEER_VELOCITY = 1;
        CFG_X = 0;
        CFG_Y = 1;
        CFG_THETA = 2;

        m_frontWheelDriving = Constants::CAR_FRONT_WHEEL_DRIVING;
        m_bodyLength = Constants::CAR_BODY_LENGTH;
        m_bodyWidth = Constants::CAR_BODY_WIDTH;
        m_minSteerAngle = Constants::CAR_MIN_STEER_ANGLE;
        m_maxSteerAngle = Constants::CAR_MAX_STEER_ANGLE;
        m_minVelocity = Constants::CAR_MIN_VELOCITY;
        m_maxVelocity = Constants::CAR_MAX_VELOCITY;
        m_minAcceleration = Constants::CAR_MIN_ACCELERATION;
        m_maxAcceleration = Constants::CAR_MAX_ACCELERATION;
        m_minSteerVelocity = Constants::CAR_MIN_STEER_VELOCITY;
        m_maxSteerVelocity = Constants::CAR_MAX_STEER_VELOCITY;
    }

    void SimulatorVehicle::SetupFromParams(Params &params) {
        SimulatorMotionEqs::SetupFromParams(params);

        SetFrontWheelDriving(params.GetValueAsBool(Constants::KW_CarFrontWheelDriving, IsFrontWheelDriving()));
        SetBodyLength(params.GetValueAsDouble(Constants::KW_CarBodyLength, GetBodyLength()));
        SetBodyWidth(params.GetValueAsDouble(Constants::KW_CarBodyWidth, GetBodyWidth()));
        SetMinSteerAngle(Constants::DEG2RAD * params.GetValueAsDouble(Constants::KW_CarMinSteerAngleInDegrees,
                                                                      Constants::RAD2DEG * GetMinSteerAngle()));
        SetMaxSteerAngle(Constants::DEG2RAD * params.GetValueAsDouble(Constants::KW_CarMaxSteerAngleInDegrees,
                                                                      Constants::RAD2DEG * GetMaxSteerAngle()));
        SetMinVelocity(params.GetValueAsDouble(Constants::KW_CarMinVelocity, GetMinVelocity()));
        SetMaxVelocity(params.GetValueAsDouble(Constants::KW_CarMaxVelocity, GetMaxVelocity()));
        SetMinAcceleration(params.GetValueAsDouble(Constants::KW_CarMinAcceleration, GetMinAcceleration()));
        SetMaxAcceleration(params.GetValueAsDouble(Constants::KW_CarMaxAcceleration, GetMaxAcceleration()));
        SetMinSteerVelocity(params.GetValueAsDouble(Constants::KW_CarMinSteerVelocity, GetMinSteerVelocity()));
        SetMaxSteerVelocity(params.GetValueAsDouble(Constants::KW_CarMaxSteerVelocity, GetMaxSteerVelocity()));
        SetMinAngularVelocity(params.GetValueAsDouble(Constants::KW_RobotMinAngularVelocity,0));
        SetMaxAngularVelocity(params.GetValueAsDouble(Constants::KW_RobotMaxAngularVelocity,0));
    }

    void SimulatorVehicle::SampleState(double s[]) {
        s[STATE_X] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[0], GetScene()->GetGrid()->GetMax()[0]);
        s[STATE_Y] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[1], GetScene()->GetGrid()->GetMax()[1]);
        s[STATE_THETA] = RandomUniformReal(-M_PI, M_PI);
        s[STATE_VELOCITY] = RandomUniformReal(m_minVelocity, m_maxVelocity);
        s[STATE_STEER_ANGLE] = RandomUniformReal(m_minSteerAngle, m_maxSteerAngle);
    }

    void SimulatorVehicle::MotionEqs(const double s[], const double t, const double u[], double ds[]) {

        const double steer = s[STATE_STEER_ANGLE];

        if (m_frontWheelDriving) {
            const double cpsi = cos(steer);
            ds[STATE_X] = s[STATE_VELOCITY] * cos(s[STATE_THETA]) * cpsi;
            ds[STATE_Y] = s[STATE_VELOCITY] * sin(s[STATE_THETA]) * cpsi;
            ds[STATE_THETA] = s[STATE_VELOCITY] * sin(steer) / m_bodyLength;
        } else {
            ds[STATE_X] = s[STATE_VELOCITY] * cos(s[STATE_THETA]);
            ds[STATE_Y] = s[STATE_VELOCITY] * sin(s[STATE_THETA]);
            ds[STATE_THETA] = s[STATE_VELOCITY] * tan(steer) / m_bodyLength;
        }

        ds[STATE_VELOCITY] = u[CONTROL_ACCELERATION];
        ds[STATE_STEER_ANGLE] = u[CONTROL_STEER_VELOCITY];
    }

    bool SimulatorVehicle::IsWithinLimitsState(void) {
        // const double asteer = Algebra2D::AngleNormalize(m_currState[STATE_STEER_ANGLE], -M_PI);
        const double *pmin = GetScene()->GetGrid()->GetMin();
        const double *pmax = GetScene()->GetGrid()->GetMax();

        return /*asteer >= GetMinSteerAngle() && asteer <= GetMaxSteerAngle() &&*/
                m_currState[STATE_X] >= pmin[0] && m_currState[STATE_X] <= pmax[0] && m_currState[STATE_Y] >= pmin[1] &&
                m_currState[STATE_Y] <= pmax[1] /*&& m_currState[STATE_VELOCITY] >= GetMinVelocity() &&
									      m_currState[STATE_VELOCITY] <= GetMaxVelocity()*/;
    }

    void SimulatorVehicle::SampleControl(double u[]) {
        u[CONTROL_ACCELERATION] = RandomUniformReal(m_minAcceleration, m_maxAcceleration);
        u[CONTROL_STEER_VELOCITY] = RandomUniformReal(m_minSteerVelocity, m_maxSteerVelocity);
    }

    void SimulatorVehicle::StartSteerToPosition(const double target[]) {
        m_pidSteer.Reset();
        m_pidVel.Reset();

        m_pidSteer.SetDesiredValue(0.0);
        m_pidVel.SetDesiredValue(RandomUniformReal(GetVelocityScaleSampling(), 1.0) * m_maxVelocity);
    }

    void SimulatorVehicle::StartToPosition(double v, double w) {
        m_pidSteer.Reset();
        m_pidVel.Reset();

        m_pidSteer.SetDesiredValue(0.0);
        m_pidVel.SetDesiredValue(RandomUniformReal(GetVelocityScaleSampling(), 1.0) * m_maxVelocity);
    }

    void SimulatorVehicle::ToPosition(double v, double w) {

    }

    void SimulatorVehicle::SteerToPosition(const double target[], const bool stop) {
        const double u[2] = {target[0] - m_currState[STATE_X], target[1] - m_currState[STATE_Y]};
        const double v[2] = {cos(m_currState[STATE_THETA] + m_currState[STATE_STEER_ANGLE]),
                             sin(m_currState[STATE_THETA] + m_currState[STATE_STEER_ANGLE])};

        m_currControl[CONTROL_STEER_VELOCITY] = m_pidSteer.Update(Algebra2D::VecFromToAngleCCW(u, v), m_dt);
        if (m_currControl[CONTROL_STEER_VELOCITY] > m_maxSteerVelocity)
            m_currControl[CONTROL_STEER_VELOCITY] = m_maxSteerVelocity;
        else if (m_currControl[CONTROL_STEER_VELOCITY] < m_minSteerVelocity)
            m_currControl[CONTROL_STEER_VELOCITY] = m_minSteerVelocity;

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

    void SimulatorVehicle::SampleCfg(double cfg[]) {
        cfg[CFG_X] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[0], GetScene()->GetGrid()->GetMax()[0]);
        cfg[CFG_Y] = RandomUniformReal(GetScene()->GetGrid()->GetMin()[1], GetScene()->GetGrid()->GetMax()[1]);
        cfg[CFG_THETA] = RandomUniformReal(-M_PI, M_PI);
    }

    bool SimulatorVehicle::IsWithinLimitsCfg(const double cfg[]) {
        const double *pmin = GetScene()->GetGrid()->GetMin();
        const double *pmax = GetScene()->GetGrid()->GetMax();

        return cfg[CFG_X] >= pmin[0] && cfg[CFG_X] <= pmax[0] && cfg[CFG_Y] >= pmin[1] && cfg[CFG_Y] <= pmax[1];
    }

    void SimulatorVehicle::PathCfgs(const double cfg1[], const double cfg2[], const double t, double cfg[]) {
        cfg[CFG_X] = cfg1[CFG_X] + t * (cfg2[CFG_X] - cfg1[CFG_X]);
        cfg[CFG_Y] = cfg1[CFG_Y] + t * (cfg2[CFG_Y] - cfg1[CFG_Y]);
        cfg[CFG_THETA] = cfg1[CFG_THETA] + t * Algebra2D::AngleSignedDistance(cfg1[CFG_THETA], cfg2[CFG_THETA]);
    }
}
