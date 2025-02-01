#ifndef BARN_CHALLENGE_LU_ROBOT_HPP
#define BARN_CHALLENGE_LU_ROBOT_HPP
#include "Components/Component.hpp"
#include "Utils/TriMesh.hpp"

namespace Antipatrea {

    class Robot : public Component {
    public:
        Robot(void);

        virtual ~Robot(void) {}

        virtual double GetLengthFromEndToBackWheels(void);

        virtual double GetBodyLength(void);

        virtual double GetDimension(const int i);

        virtual const double *GetBoundingBoxMin(void) {
            return m_chassis.GetBoundingBoxMin();
        }

        virtual const double *GetBoundingBoxMax(void) {
            return m_chassis.GetBoundingBoxMax();
        }

        virtual void AdjustDimensions(const int i, const double dim);

        virtual void SetDimensions(const double dimx, const double dimy,
                                   const double dimz);

        virtual void SetAngleTire(const double a) { m_angleTire = a; }

        virtual void Draw(void);

    protected:
        TriMesh m_chassis;
        TriMesh m_wheel;
        double m_bodyLengthFrac;
        double m_angleTire;
    };
}


#endif //BARN_CHALLENGE_LU_ROBOT_HPP
