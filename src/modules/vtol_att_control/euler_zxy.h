#ifndef EULER_ZXY_H
#define EULER_ZXY_H

//#include "math.hpp"
#include <matrix/matrix/math.hpp>

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

namespace matrix
{

template <typename Type>
class Dcm;

template <typename Type>
class Quaternion;

/**
 * Euler angles class
 *
 * This class describes the rotation from frame 1
 * to frame 2 via 3-2-1 intrinsic Tait-Bryan rotation sequence.
 */
template<typename Type>
class Euler_zxy : public Vector<Type, 3>
{
public:
    /**
     * Standard constructor
     */
    Euler_zxy() = default;

    /**
     * Copy constructor
     *
     * @param other vector to copy
     */
    Euler_zxy(const Vector<Type, 3> &other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from Matrix31
     *
     * @param other Matrix31 to copy
     */
    Euler_zxy(const Matrix<Type, 3, 1> &other) :
        Vector<Type, 3>(other)
    {
    }

    /**
     * Constructor from euler angles
     *
     * Instance is initialized from an 3-2-1 intrinsic Tait-Bryan
     * rotation sequence representing transformation from frame 1
     * to frame 2.
     *
     * @param phi_ rotation angle about X axis
     * @param theta_ rotation angle about Y axis
     * @param psi_ rotation angle about Z axis
     */
    Euler_zxy(Type phi_, Type theta_, Type psi_) : Vector<Type, 3>()
    {
        phi() = phi_;
        theta() = theta_;
        psi() = psi_;
    }

    /**
     * Constructor from DCM matrix
     *
     * In order to avoid singularity
     * This instance will hold the angles defining the 3-1-2 intrinsic
     * Tait-Bryan rotation sequence from frame 1 to frame 2.
     *
     * @param dcm Direction cosine matrix
    */
    Euler_zxy(const Dcm<Type> &dcm)
    {
        Type phi_val = Type(asin(dcm(2, 1)));
        //Type phi_val = Type(atan2(dcm(2, 1), dcm(2, 2)));
        Type theta_val = Type(atan2(-dcm(2, 0), dcm(2, 2)));
        Type psi_val = Type(atan2(-dcm(0, 1), dcm(1, 1)));
        Type pi = Type(M_PI);

        if (Type(fabs(theta_val - pi / Type(2))) < Type(1.0e-3)) {
            phi_val = Type(0.0);
            psi_val = Type(atan2(-dcm(0, 1), dcm(1, 1)));

        } else if (Type(fabs(theta_val + pi / Type(2))) < Type(1.0e-3)) {
            phi_val = Type(0.0);
            psi_val = Type(atan2(dcm(0, 1), -dcm(1, 1)));
        }

        phi() = phi_val;
        theta() = theta_val;
        psi() = psi_val;
    }

    /**
     * Constructor from quaternion instance.
     *
     * Instance is set from a quaternion representing transformation
     * from frame 2 to frame 1.
     * This instance will hold the angles defining the 3-2-1 intrinsic
     * Tait-Bryan rotation sequence from frame 1 to frame 2.
     *
     * @param q quaternion
    */
    Euler_zxy(const Quaternion<Type> &q)
    {
        *this = Euler_zxy(Dcm<Type>(q));
    }

    inline Type phi() const
    {
        return (*this)(0);
    }
    inline Type theta() const
    {
        return (*this)(1);
    }
    inline Type psi() const
    {
        return (*this)(2);
    }

    inline Type &phi()
    {
        return (*this)(0);
    }
    inline Type &theta()
    {
        return (*this)(1);
    }
    inline Type &psi()
    {
        return (*this)(2);
    }

};

typedef Euler_zxy<float> Eulerf_zxy;

}

#endif
