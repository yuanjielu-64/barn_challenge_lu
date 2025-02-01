#include "Components/Constants.hpp"
#include "Utils/Algebra3D.hpp"
#include "Utils/TriMeshStreamer.hpp"
#include "Utils/GDraw.hpp"
#include "Robot.hpp"

namespace Antipatrea {
    Robot::Robot(void) : Component() {
        m_angleTire = 0.0;

        m_bodyLengthFrac = 2 * 0.85 / 3.0;

        double Rx[Algebra3D::Rot_NR_ENTRIES];
        double Rz[Algebra3D::Rot_NR_ENTRIES];
        double R[Algebra3D::Rot_NR_ENTRIES];

        std::string package_path = ros::package::getPath(Constants::KW_ProjectName);
        auto ROBOT_CHASSIS_MESH_FILE = package_path + "/" + Constants::ROBOT_CHASSIS_MESH_FILE;

        TriMeshReader(ROBOT_CHASSIS_MESH_FILE.c_str(), m_chassis);

        Algebra3D::XAxisAngleAsRot(0.5 * M_PI, Rx);
        Algebra3D::ZAxisAngleAsRot(0.5 * M_PI, Rz);
        Algebra3D::RotMultRotAsRot(Rz, Rx, R);
        m_chassis.ApplyRot(0, m_chassis.GetNrVertices() - 1, R);

        const double *min = m_chassis.GetBoundingBoxMin();
        const double *max = m_chassis.GetBoundingBoxMax();

        double x = max[0] - min[0];
        double y = max[1] - min[1];

        m_chassis.ApplyTrans(0, m_chassis.GetNrVertices() - 1, -x / 2 - min[0], -y/2 - min[1],15);

        m_chassis.GetGroupsNotForDrawing()->push_back(6);
        m_chassis.GetGroupsNotForDrawing()->push_back(7);
        m_chassis.GetGroupsNotForDrawing()->push_back(8);
        m_chassis.GetGroupsNotForDrawing()->push_back(9);
        for (int i = m_chassis.GetNrGroups() - 1; i >= 11; --i)
            m_chassis.GetGroupsNotForDrawing()->push_back(i);
        m_chassis.ApplyScaling(0, m_chassis.GetNrVertices() - 1, 1.229, 1.00, 1);

        auto ROBOT_WHEEL_MESH_FILE =  package_path + "/" + Constants::ROBOT_WHEEL_MESH_FILE;
        auto ROBOT_TIRE_TEXTURE_FILE =  package_path + "/" + Constants::ROBOT_TIRE_TEXTURE_FILE;

        m_wheel.ReadOgreMesh(ROBOT_WHEEL_MESH_FILE.c_str(), ROBOT_TIRE_TEXTURE_FILE.c_str());
        m_wheel.GetMainMaterial()->SetDiffuse(0.2, 0.2, 0.2);
        m_wheel.ApplyRot(0, m_wheel.GetNrVertices() - 1, R);
        m_wheel.ApplyScaling(0, m_wheel.GetNrVertices() - 1, 100, 100, 90);
    }

    double Robot::GetLengthFromEndToBackWheels(void) {
        return fabs(m_chassis.GetBoundingBoxMin()[0]);
    }

    double Robot::GetBodyLength(void) {
        return m_chassis.GetBoundingBoxMax()[0];
    }

    double Robot::GetDimension(const int i) {
        return m_chassis.GetBoundingBoxMax()[i] - m_chassis.GetBoundingBoxMin()[i];
    }

    void Robot::AdjustDimensions(const int i, const double dim) {
        const double scale = dim / GetDimension(i);

        SetDimensions(GetDimension(0) * scale, GetDimension(1) * scale, GetDimension(2) * scale);
    }

    void Robot::SetDimensions(const double dimx,
                            const double dimy,
                            const double dimz) {

        const double *min = m_chassis.GetBoundingBoxMin();
        const double *max = m_chassis.GetBoundingBoxMax();

        const double sx = dimx / (max[0] - min[0]);
        const double sy = dimy / (max[1] - min[1]);
        const double sz = dimz / (max[2] - min[2]);

        m_chassis.ApplyScaling(0, m_chassis.GetNrVertices() - 1, sx, sy, sz);
        m_wheel.ApplyScaling(0, m_wheel.GetNrVertices() - 1, sx, sy, sz);
    }

    void Robot::Draw(void) {
        double R[Algebra3D::Rot_NR_ENTRIES];
        Algebra3D::YAxisAngleAsRot(m_angleTire, R);

        m_chassis.Draw();

    }
}
