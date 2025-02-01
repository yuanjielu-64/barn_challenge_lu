#include "SimulatorRobot.hpp"
#include "Components/Constants.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/GDraw.hpp"

namespace Antipatrea {
    SimulatorRobot::SimulatorRobot(void)
            : SimulatorJackal() {

        m_robot.AdjustDimensions(1, m_bodyWidth);

        GetStateAllocator()->SetDim(5);
        GetControlAllocator()->SetDim(2);
        GetCfgAllocator()->SetDim(3);
        GetPositionAllocator()->SetDim(2);
    }

    void SimulatorRobot::SetupFromParams(Params &params) {
        SimulatorJackal::SetupFromParams(params);

        SetAttachDistance(params.GetValueAsDouble(Constants::KW_RobotAttachDistance, GetAttachDistance()));

        m_robot.AdjustDimensions(1, m_bodyWidth);
    }

    void SimulatorRobot::SetupFromParams(Params &params, Robot_config &robot) {
        SimulatorJackal::SetupFromParams(params, robot);

        m_robot.AdjustDimensions(1, m_bodyWidth);
    }

    void SimulatorRobot::CompleteSetup(void) {
        GetStateAllocator()->SetDim(5);

        SimulatorJackal::CompleteSetup();

        const double *min = m_robot.GetBoundingBoxMin();
        const double *max = m_robot.GetBoundingBoxMax();

        m_orig[0] = min[0];
        m_orig[1] = min[1];
        m_orig[2] = max[0];
        m_orig[3] = min[1];
        m_orig[4] = max[0];
        m_orig[5] = max[1];
        m_orig[6] = min[0];
        m_orig[7] = max[1];

        m_origScaled[0] = Constants::SIMULATOR_ROBOT_SX * min[0];
        m_origScaled[1] = Constants::SIMULATOR_ROBOT_SY * min[1];
        m_origScaled[2] = Constants::SIMULATOR_ROBOT_SX * max[0];
        m_origScaled[3] = Constants::SIMULATOR_ROBOT_SY * min[1];
        m_origScaled[4] = Constants::SIMULATOR_ROBOT_SX * max[0];
        m_origScaled[5] = Constants::SIMULATOR_ROBOT_SY * max[1];
        m_origScaled[6] = Constants::SIMULATOR_ROBOT_SX * min[0];
        m_origScaled[7] = Constants::SIMULATOR_ROBOT_SY * max[1];

        m_bodyLength = m_robot.GetBodyLength();
        m_bodyWidth = max[1] - min[1];

        // Logger::m_out << " SimulatorRobot robot model: " << " length = " << (max[0] - min[0]) << " width = "
        //               << m_bodyWidth << std::endl;

        m_bodies.resize(8);
        m_TRs.resize(Algebra2D::TransRot_NR_ENTRIES);

        SetState(m_currState);
    }

    void SimulatorRobot::SampleState(double s[]) {
        SimulatorJackal::SampleState(s);
    }

    void SimulatorRobot::SetState(const double s[]) {
        m_tmeshBodies.Clear();

        SimulatorJackal::SetState(s);

        double TR[Algebra2D::TransRot_NR_ENTRIES];
        double T[Algebra2D::Trans_NR_ENTRIES];
        double p[2];

        TR[0] = p[0] = s[STATE_X];
        TR[1] = p[1] = s[STATE_Y];

        Algebra2D::AngleAsRot(s[STATE_THETA], &TR[Algebra2D::Trans_NR_ENTRIES]);
        ApplyTransRotToPolygon2D(TR, 4, m_orig, &m_bodies[0]);

        Algebra2D::TransRotAsTransRot(TR, &m_TRs[0]);

    }

    void SimulatorRobot::MotionEqs(const double s[], const double t, const double u[], double ds[]) {
        SimulatorJackal::MotionEqs(s, t, u, ds);
    }

    void SimulatorRobot::AddToMeshState(TriMesh &tmesh) {
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

    bool SimulatorRobot::IsSelfCollisionFreeState(void) {
        const int n = m_bodies.size();
        for (int i = 0; i < n; i += 8)
            for (int j = i + 32; j < n; j += 8)
                if (CollisionConvexPolygons2D(4, &m_bodies[i], 4, &m_bodies[j]))
                    return false;
        return true;
    }

    void SimulatorRobot::AddToMeshCfg(TriMesh &tmesh, const double cfg[]) const {
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

    void SimulatorRobot::AddToMeshPathCfgs(TriMesh &tmesh, const double cfg1[], const double cfg2[],
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

    void SimulatorRobot::DrawState(void) {
        double T3[Algebra3D::Trans_NR_ENTRIES];
        double R3[Algebra3D::Rot_NR_ENTRIES];
        const double *TR;
        GMaterial gmat;

        Algebra3D::IdentityAsRot(R3);
        Algebra3D::ZAxisAngleAsRot(m_currState[STATE_THETA], R3);

        GDrawPushTransformation();
        GDrawMultTrans(m_currState[STATE_X], m_currState[STATE_Y], 0.0);
        GDrawMultRot(R3);
        m_robot.Draw();
        GDrawPopTransformation();
    }

    void SimulatorRobot::DrawPosition(const double pos[]) const {
        GDrawCircle2D(pos, GetDrawPositionRadius());
    }


}
