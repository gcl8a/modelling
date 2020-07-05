//
//  quaternion.h
//  kalman
//
//  Created by Gregory C Lewin on 12/1/15.
//  Copyright © 2015 Gregory C Lewin. All rights reserved.
//

#ifndef quaternion_h
#define quaternion_h

#include <linalg/matrix.h>

class Quaternion
{
    dvector x;
    
public:
    Quaternion(void) : x(4)
    {
        x.Zero();
        x[0] = 1;
    }
    
    Quaternion(const dvector& v)
    {
        x = v;
    }
    
    Quaternion(double a, double b, double c, double d) : x(4)
    {
        x[0] = a;
        x[1] = b;
        x[2] = c;
        x[3] = d;
    }
    
    double& operator [] (int i) const { return x[i]; }
    
    Quaternion operator * (const Quaternion& q)
    {
        Quaternion result;
        
        result[0] = x[0] * q[0] - x[1] * q[1] - x[2] * q[2] - x[3] * q[3];
        result[1] = x[0] * q[1] + x[1] * q[0] + x[2] * q[3] - x[3] * q[2];
        result[2] = x[0] * q[2] - x[1] * q[3] + x[2] * q[0] + x[3] * q[1];
        result[3] = x[0] * q[3] + x[1] * q[2] - x[2] * q[1] + x[3] * q[0];
        
        return result;
    }
    
    Quaternion Normalize(void)
    {
        double norm = x.Dot(x);
        x /= sqrt(norm);
        return *this;
    }

    Quaternion NormalizeFast(void) //approximated normalization based on first-order Taylor series
    {
        double norm = x.Dot(x);
        x *= 1.5 - 0.5 * norm;
        
        return *this;
    }
    
    dvector AsVector(void) const
    {
        return x;
    }
    
    dmatrix AsRotationMatrix(void) const
    {
        dmatrix R(3,3);
        
        R[0][0] = x[0]*x[0] + x[1]*x[1] - x[2]*x[2] - x[3]*x[3];
        R[0][1] = 2 * (x[1]*x[2] - x[0]*x[3]);
        R[0][2] = 2 * (x[1]*x[3] + x[0]*x[2]);

        R[1][0] = 2 * (x[1]*x[2] + x[0]*x[3]);
        R[1][1] = x[0]*x[0] - x[1]*x[1] + x[2]*x[2] - x[3]*x[3];
        R[1][2] = 2 * (x[2]*x[3] - x[0]*x[1]);
        
        R[2][0] = 2 * (x[1]*x[3] - x[0]*x[2]);
        R[2][1] = 2 * (x[2]*x[3] + x[0]*x[1]);
        R[2][2] = x[0]*x[0] - x[1]*x[1] - x[2]*x[2] + x[3]*x[3];

        return R;
    }
    
    dvector AsEulerAngles(void) const
    {
        dvector eulers(3);
        eulers[0] = atan2(2 * (x[0] * x[1] + x[2] * x[3]), 1 - 2 * (x[1] * x[1] + x[2] * x[2]));
        eulers[1] = asin (2 * (x[0] * x[2] - x[1] * x[3]));
        eulers[2] = atan2(2 * (x[0] * x[3] + x[1] * x[2]), 1 - 2 * (x[3] * x[3] + x[2] * x[2]));

        return eulers;
    }
    
    dvector FromEulers(dvector eulers) //only does yaw at the moment
    {
        x[0] = cos(eulers[2] / 2);
        x[3] = sin(eulers[2] / 2);
        return x;
    }
};

#endif /* quaternion_h */
