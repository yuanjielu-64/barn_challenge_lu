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
#include "SimulatorMotionEqs.hpp"
#include "Utils/PseudoRandom.hpp"

namespace Antipatrea {

    double SimulatorMotionEqs::SimulateOneStep(void) {
        double dt = fabs(TimeStepConstantAcceleration(GetVelocity(),
                                                      GetAcceleration(),
                                                      RandomUniformReal(GetMinDistanceOneStep(),
                                                                        GetMaxDistanceOneStep())));

        if (dt > GetTimeStep())
            dt = GetTimeStep();


        const double *s = m_currState;
        const double hhalf = 0.5 * dt;
        const double hthird = dt / 3.0;
        const double hsixth = dt / 6.0;
        const int dimState = GetStateAllocator()->GetDim();

        std::vector<double> waux;
        waux.resize(3 * dimState);

        double *wa = &waux[0];
        double *wb = &waux[dimState];
        double *snew = &waux[2 * dimState];

        MotionEqs(s, 0, m_currControl, wa);
        for (int i = 0; i < dimState; ++i) {
            snew[i] = s[i] + hsixth * wa[i];
            wa[i] = s[i] + hhalf * wa[i];
        }
        MotionEqs(wa, hhalf, m_currControl, wb);
        for (int i = 0; i < dimState; ++i) {
            snew[i] += hthird * wb[i];
            wb[i] = s[i] + hhalf * wb[i];
        }
        MotionEqs(wb, hhalf, m_currControl, wa);
        for (int i = 0; i < dimState; ++i) {
            snew[i] += hthird * wa[i];
            wa[i] = s[i] + dt * wa[i];
        }
        MotionEqs(wa, dt, m_currControl, wb);
        for (int i = 0; i < dimState; ++i)
            snew[i] += hsixth * wb[i];

        SetState(snew);

        return dt;
    }

    void SimulatorMotionEqs::SimulateOneStepFromParams(double v, double w, double dt) {

        //double dt = GetTimeStep();

        const double *s = m_currState;
        std::vector<double> waux;
        waux.resize(GetStateAllocator()->GetDim());

        waux[2] = s[2] + s[4] * dt;
        waux[2] = normalizeAngle(waux[2]);
        waux[0] = s[0] + dt * v * cos(s[2]);
        waux[1] = s[1] + dt * v * sin(s[2]);
        waux[3] = v;
        waux[4] = w;

        double *snew = &waux[0];
        SetState(snew);

    }
}
