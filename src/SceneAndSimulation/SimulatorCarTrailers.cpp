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
#include "SimulatorCarTrailers.hpp"
#include "Components/Constants.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/GDraw.hpp"

namespace Antipatrea {
    SimulatorCarTrailers::SimulatorCarTrailers(void)
            : SimulatorVehicle() {
        STATE_TRAILERS = 5;

        m_nrTrailers = Constants::CAR_NR_TRAILERS;
        m_attachDistance = Constants::CAR_ATTACH_DISTANCE;

        m_car.AdjustDimensions(1, m_bodyWidth);

        GetStateAllocator()->SetDim(5 + GetNrTrailers());
        GetControlAllocator()->SetDim(2);
        GetCfgAllocator()->SetDim(3);
        GetPositionAllocator()->SetDim(2);
    }

    void SimulatorCarTrailers::SetupFromParams(Params &params) {
        SimulatorVehicle::SetupFromParams(params);

        SetNrTrailers(params.GetValueAsInt(Constants::KW_CarNrTrailers, GetNrTrailers()));
        SetAttachDistance(params.GetValueAsDouble(Constants::KW_CarAttachDistance, GetAttachDistance()));

        m_car.AdjustDimensions(1, m_bodyWidth);
    }

    void SimulatorCarTrailers::CompleteSetup(void) {
        GetStateAllocator()->SetDim(5 + GetNrTrailers());

        SimulatorVehicle::CompleteSetup();

        if (m_nrTrailers == 0) {
            const double *min = m_car.GetBoundingBoxMin();
            const double *max = m_car.GetBoundingBoxMax();

            m_orig[0] = min[0];
            m_orig[1] = min[1];
            m_orig[2] = max[0];
            m_orig[3] = min[1];
            m_orig[4] = max[0];
            m_orig[5] = max[1];
            m_orig[6] = min[0];
            m_orig[7] = max[1];

            m_origScaled[0] = Constants::SIMULATOR_CAR_SX * min[0];
            m_origScaled[1] = Constants::SIMULATOR_CAR_SY * min[1];
            m_origScaled[2] = Constants::SIMULATOR_CAR_SX * max[0];
            m_origScaled[3] = Constants::SIMULATOR_CAR_SY * min[1];
            m_origScaled[4] = Constants::SIMULATOR_CAR_SX * max[0];
            m_origScaled[5] = Constants::SIMULATOR_CAR_SY * max[1];
            m_origScaled[6] = Constants::SIMULATOR_CAR_SX * min[0];
            m_origScaled[7] = Constants::SIMULATOR_CAR_SY * max[1];

            m_bodyLength = m_car.GetBodyLength();
            m_bodyWidth = max[1] - min[1];

            Logger::m_out << " SimulatorCarTrailers car model: "
                          << " length = " << (max[0] - min[0]) << " width = " << m_bodyWidth << std::endl;
        } else {
            const double hw = 0.5 * m_bodyWidth;

            m_orig[0] = 0;
            m_orig[1] = -hw;
            m_orig[2] = m_bodyLength;
            m_orig[3] = -hw;
            m_orig[4] = m_bodyLength;
            m_orig[5] = hw;
            m_orig[6] = 0;
            m_orig[7] = hw;


            m_origScaled[0] = 0;
            m_origScaled[1] = -hw * Constants::SIMULATOR_SNAKE_SY;
            m_origScaled[2] = m_bodyLength * Constants::SIMULATOR_SNAKE_SX;
            m_origScaled[3] = -hw * Constants::SIMULATOR_SNAKE_SY;
            m_origScaled[4] = m_bodyLength * Constants::SIMULATOR_SNAKE_SX;
            m_origScaled[5] = hw * Constants::SIMULATOR_SNAKE_SY;
            m_origScaled[6] = 0;
            m_origScaled[7] = hw * Constants::SIMULATOR_SNAKE_SY;

            Logger::m_out << " SimulatorCarTrailers snake model: "
                          << " length = " << m_bodyLength << " width = " << m_bodyWidth << " nrTrailers = "
                          << m_nrTrailers << std::endl;
        }

        m_bodies.resize(8 * (m_nrTrailers + 1));
        m_TRs.resize(Algebra2D::TransRot_NR_ENTRIES * (1 + m_nrTrailers));

        SetState(m_currState);

        // m_tmeshCfg.Clear();

        // const double quad[] = {m_orig[0], m_orig[1], 0.0, m_orig[2], m_orig[3], 0.0, m_orig[4], m_orig[5], 0.0, m_orig[6], m_orig[7], 0.0};
        // m_tmeshCfg.AddQuad(quad);
    }

    void SimulatorCarTrailers::SampleState(double s[]) {
        SimulatorVehicle::SampleState(s);
        for (int i = 0; i < m_nrTrailers; ++i)
            s[STATE_TRAILERS + i] = RandomUniformReal(-M_PI, M_PI);
    }

    void SimulatorCarTrailers::SetState(const double s[]) {
        m_tmeshBodies.Clear();

        SimulatorVehicle::SetState(s);

        double TR[Algebra2D::TransRot_NR_ENTRIES];
        double T[Algebra2D::Trans_NR_ENTRIES];
        double p[2];

        TR[0] = p[0] = s[STATE_X];
        TR[1] = p[1] = s[STATE_Y];
        Algebra2D::AngleAsRot(s[STATE_THETA], &TR[Algebra2D::Trans_NR_ENTRIES]);
        ApplyTransRotToPolygon2D(TR, 4, m_orig, &m_bodies[0]);

        Algebra2D::TransRotAsTransRot(TR, &m_TRs[0]);

        for (int i = 0; i < m_nrTrailers; ++i) {
            T[0] = p[0] - (m_attachDistance + m_bodyLength);
            T[1] = p[1];
            Algebra2D::RotateAroundPointAsTransRot(s[STATE_TRAILERS + i], p, TR);
            Algebra2D::TransRotMultTransAsTransRot(TR, T, TR);
            ApplyTransRotToPolygon2D(TR, 4, m_orig, &m_bodies[8 * (i + 1)]);
            Algebra2D::TransRotAsTransRot(TR, &m_TRs[(i + 1) * Algebra2D::TransRot_NR_ENTRIES]);
            p[0] = TR[0];
            p[1] = TR[1];
        }
    }

    void SimulatorCarTrailers::MotionEqs(const double s[], const double t, const double u[], double ds[]) {
        SimulatorVehicle::MotionEqs(s, t, u, ds);

        double coeff = s[STATE_VELOCITY] / (m_bodyLength + m_attachDistance);
        double prev = s[STATE_THETA];

        for (int i = 0; i < m_nrTrailers; ++i) {
            ds[STATE_TRAILERS + i] = coeff * sin(prev - s[STATE_TRAILERS + i]);
            coeff = coeff * cos(prev - s[STATE_TRAILERS + i]);
            prev = s[STATE_TRAILERS + i];
        }
    }

    void SimulatorCarTrailers::AddToMeshState(TriMesh &tmesh) {
        double quad3[12];

        for (int i = 0; i < (int) m_bodies.size(); i += 8) {
            quad3[0] = m_bodies[i];
            quad3[1] = m_bodies[i + 1];
            quad3[2] = 0.0;
            quad3[3] = m_bodies[i + 2];
            quad3[4] = m_bodies[i + 3];
            quad3[5] = 0.0;
            quad3[6] = m_bodies[i + 4];
            quad3[7] = m_bodies[i + 5];
            quad3[8] = 0.0;
            quad3[9] = m_bodies[i + 6];
            quad3[10] = m_bodies[i + 7];
            quad3[11] = 0.0;
            tmesh.AddQuad(quad3, true);
        }
    }

    bool SimulatorCarTrailers::IsSelfCollisionFreeState(void) {
        const int n = m_bodies.size();
        for (int i = 0; i < n; i += 8)
            for (int j = i + 32; j < n; j += 8)
                if (CollisionConvexPolygons2D(4, &m_bodies[i], 4, &m_bodies[j]))
                    return false;
        return true;
    }

    void SimulatorCarTrailers::AddToMeshCfg(TriMesh &tmesh, const double cfg[]) const {
        double TR[Algebra2D::TransRot_NR_ENTRIES];
        double body[8];

        TR[0] = cfg[CFG_X];
        TR[1] = cfg[CFG_Y];
        Algebra2D::AngleAsRot(cfg[CFG_THETA], &TR[Algebra2D::Trans_NR_ENTRIES]);
        ApplyTransRotToPolygon2D(TR, 4, m_origScaled, body);

        const double quad[] = {body[0], body[1], 0.0, body[2], body[3], 0.0, body[4], body[5], 0.0, body[6], body[7],
                               0.0};

        tmesh.AddQuad(quad);
    }

    void SimulatorCarTrailers::AddToMeshPathCfgs(TriMesh &tmesh, const double cfg1[], const double cfg2[],
                                                 const double tstart, const double dt, const double tend) {
        double skel[4] =
                {
                        cfg1[0], cfg1[1], cfg2[0], cfg2[1]
                };
        std::vector<double> vertices;
        Polygon2D poly;

        FromSkeletonToPolygon2D(2, skel, std::max(m_bodyWidth, m_bodyLength), vertices);
        poly.AddVertices(vertices.size() / 2, &vertices[0]);
        tmesh.AddPolygon(poly);

    }

    void SimulatorCarTrailers::DrawState(void) {
        double T3[Algebra3D::Trans_NR_ENTRIES];
        double R3[Algebra3D::Rot_NR_ENTRIES];
        const double *TR;
        GMaterial gmat;

        if (m_nrTrailers == 0) // just the car
        {
            Algebra3D::IdentityAsRot(R3);
            Algebra3D::ZAxisAngleAsRot(m_currState[STATE_THETA], R3);

            GDrawPushTransformation();
            GDrawMultTrans(m_currState[STATE_X], m_currState[STATE_Y], 0.0);
            GDrawMultRot(R3);
            m_car.SetAngleSteer(m_currState[STATE_STEER_ANGLE]);
            m_car.Draw();
            GDrawPopTransformation();

            return;
        }

        if (GDrawIs2D()) {
            GDrawColor(1, 0, 0);
            GDrawQuad2D(&m_bodies[0]);
            GDrawColor(0, 1, 0);
            for (int i = 8; i < (int) m_bodies.size(); i += 8)
                GDrawQuad2D(&m_bodies[i]);
        } else {

            gmat.SetPearl();

            Algebra3D::IdentityAsRot(R3);
            T3[2] = 0.0;

            for (int i = 0; i <= m_nrTrailers; ++i) {
                TR = &m_TRs[i * Algebra2D::TransRot_NR_ENTRIES];

                T3[0] = TR[0];
                T3[1] = TR[1];

                R3[0] = R3[4] = TR[2];
                R3[3] = TR[3];
                R3[1] = -R3[3];

                if (i == 0)
                    gmat.SetDiffuse(1.0, 0.0, 0.0);
                else
                    gmat.SetDiffuse(0.0, 1, 0.0);
                GDrawMaterial(gmat);
                GDrawPushTransformation();
                GDrawMultTransRot(T3, R3);
                GDrawMultTrans(0, 0, 0.5 * m_bodyWidth);
                GDrawMultFromZAxisToAxis(1, 0, 0);
                GDrawCappedCone3D(0.5 * m_bodyWidth, (i == 0 ? 0.25 : 0.5) * m_bodyWidth, m_bodyLength);
                GDrawPopTransformation();
            }
        }
    }

    void SimulatorCarTrailers::DrawPosition(const double pos[]) const {
        GDrawCircle2D(pos, GetDrawPositionRadius());
    }
}
