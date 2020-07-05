/* 
 * File:   kalman.h
 * Author: Greg Lewin
 *
 * Created on October 2, 2014, 2:08 PM
 */

#ifndef KALMAN_H
#define	KALMAN_H

#include <stdlib.h>
#include <math.h>
#include <cstdlib>

#include <random>
#include <cmath>

#include <linalg/matrix.h>

#define ranf (((double)rand() / (double)RAND_MAX));

//there's now a library for normal distributions -- I have no idea how it works, but it seems sufficient for this purpose
double randn(double average, double stdev)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<> d(average, stdev);
    
    double dd = d(gen);
    return dd;
}

dvector RandomNoise(dmatrix cov)
{
    dvector noise(cov.CountRows());
    
    for(int i = 0; i < cov.CountRows(); i++)
    {
        //zero-centered, normal dist sample
        noise[i] = randn(0, sqrt(cov[i][i]));
    }
    
    return noise;
}

class DynamicalSystem
{
private:
protected:
    dvector x,u,z; //x is the state, u is control input, z is observables
    dmatrix A,B,C,D;
    dmatrix Q; //process noise
    dmatrix R; //observation noise -- for better or worse, noise added here
    
public:
    DynamicalSystem(int a, int b, int c) : x(a), u(b), z(c), A(a,a), B(a,b), C(c,a), D(c,b), Q(a,a), R(c,c)
    {
        //not sure I need anything here...
    }
    
    void SetDynamics(const dmatrix& a, const dmatrix& b, const dmatrix& c)
    {
        A = a;
        B = b;
        C = c;
    }
    
    void Initialize(const dvector& x0)
    {
        x = x0;
    }
    
    void AddNoise(const dmatrix& q, const dmatrix& r)
    {
        Q = q;
        R = r;
    }
    
    virtual dvector Step(const dvector& u, double dt) //dt unused at the moment!!
    {
        return x = A * x + B * u + RandomNoise(Q); //Q is diagonal here...need to improve
    }

    dvector Observe(const dvector& u)  //CLEAN UP u!!!!!!!!!!!
    {
        z = C * x + D * u;

        return z + RandomNoise(R);
    }
};

class KalmanFilter : public DynamicalSystem
{
protected:
    dmatrix I; //useful to pre-compute here
    dvector x_hat, x_hat_minus; //x_hat is estimate (after correction); x_hat_minus is predictive step
    dmatrix P, P_minus;
    dmatrix K, H;
    
public:    
    KalmanFilter(int a, int b, int c) : DynamicalSystem(a,b,c), x_hat(a), P(a,a)
    {
        I = dmatrix::Eye(a);
    }
    
    void Initialize(const dvector& x0)
    {
        DynamicalSystem::Initialize(x0);
        x_hat = x0;
    }
    
    
    dvector Step(const dvector& u, const dvector& z, double dt) //dt unused at the moment!
    {
        //process update
        x_hat_minus = A * x_hat + B * u;
        
        P_minus = A * P * A.MakeTranspose() + Q;
        
        H = C; ///????
        K = P_minus * H.MakeTranspose() * (H * P_minus * H.MakeTranspose() + R).FindInverse();

        x_hat = x_hat_minus + K * (z - H * x_hat_minus);
        P = (I - K * H) * P_minus;
                
        return x_hat;
    }

    dvector Observe(const dvector& u_hat)  //CLEAN UP u!!!!!!!!!!!
    {
        z = C * x_hat + D * u_hat;
        
        return z + RandomNoise(R);
    }

};

#endif	/* KALMAN_H */

