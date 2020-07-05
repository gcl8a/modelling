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
#include <fstream>

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

dvector RandomNoise(const dvector& var)
{
    dvector noise(var.Length());
    
    for(int i = 0; i < var.Length(); i++)
    {
        //zero-centered, normal dist sample
        noise[i] = randn(0, sqrt(var[i]));
    }
    
    return noise;
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
    dvector x;//,u,z; //x is the state, u is control input, z is observables
    dmatrix A,B,C,D;
    dmatrix Q; //process noise
    dmatrix R; //observation noise -- for better or worse, noise added here
    
public:
    DynamicalSystem(int a, int b, int c) : x(a), /*u(b), z(c), */A(a,a), B(a,b), C(c,a), D(c,b), Q(a,a), R(c,c)
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
        dvector z = C * x + D * u;

        return z + RandomNoise(R);
    }
};

class KalmanFilter : public DynamicalSystem
{
protected:
    dmatrix I; //useful to pre-compute here
    dvector x_hat_minus; //x_hat is estimate (after correction); x_hat_minus is predictive step
    dmatrix P, P_minus;
    dmatrix K, H;
    
public:
    KalmanFilter(int a, int b, int c) : DynamicalSystem(a,b,c), P(a,a)//x_hat(a),
    {
        I = dmatrix::Eye(a);
    }
    
/*    void Initialize(const dvector& x0)
    {
        DynamicalSystem::Initialize(x0);
        x_hat = x0;
    }
*/
    
    dvector Step(const dvector& u, const dvector& z, double dt) //dt unused at the moment!
    {
        //process update
        x_hat_minus = A * x/*_hat*/ + B * u;
        
        P_minus = A * P * A.MakeTranspose() + Q;
        
        H = C; ///????
        K = P_minus * H.MakeTranspose() * (H * P_minus * H.MakeTranspose() + R).FindInverse();

        x/*_hat*/ = x_hat_minus + K * (z - H * x_hat_minus);
        P = (I - K * H) * P_minus;
                
        return x/*_hat*/;
    }

    dvector Observe(const dvector& u_hat)  //CLEAN UP u!!!!!!!!!!!
    {
        dvector z_hat = C * x/*_hat*/ + D * u_hat;
        
        return z_hat + RandomNoise(R);
    }

};

int ConvertHex2Int(char c)
{
    switch(c)
    {
        case '0': return 0;
        case '1': return 1;
        case '2': return 2;
        case '3': return 3;
        case '4': return 4;
        case '5': return 5;
        case '6': return 6;
        case '7': return 7;
        case '8': return 8;
        case '9': return 9;
        case 'A': return 10;
        case 'B': return 11;
        case 'C': return 12;
        case 'D': return 13;
        case 'E': return 14;
        case 'F': return 15;
    }
    
    cout << "Error!!!!!!" << endl;
    return 0;
}

int ParseTestData(char* fileName)
{
    ifstream inFile(fileName);
    ofstream outFile("imu_data.txt");
    
    int lineCount = 0;
    while(!inFile.eof())
    {
        char line[128];
        inFile.getline(line, 128);
        
        //cout<<line<<endl;
        
        int16_t gyro_x = (ConvertHex2Int(line[18]) << 12) + (ConvertHex2Int(line[19]) << 8) +
            (ConvertHex2Int(line[21]) << 4) + (ConvertHex2Int(line[22]));
        int16_t gyro_y = (ConvertHex2Int(line[24]) << 12) + (ConvertHex2Int(line[25]) << 8) +
            (ConvertHex2Int(line[27]) << 4) + (ConvertHex2Int(line[28]));
        int16_t gyro_z = (ConvertHex2Int(line[30]) << 12) + (ConvertHex2Int(line[31]) << 8) +
            (ConvertHex2Int(line[33]) << 4) + (ConvertHex2Int(line[34]));
        
        int16_t accel_x = (ConvertHex2Int(line[36]) << 12) + (ConvertHex2Int(line[37]) << 8) +
            (ConvertHex2Int(line[39]) << 4) + (ConvertHex2Int(line[40]));
        int16_t accel_y = (ConvertHex2Int(line[42]) << 12) + (ConvertHex2Int(line[43]) << 8) +
            (ConvertHex2Int(line[45]) << 4) + (ConvertHex2Int(line[46]));
        int16_t accel_z = (ConvertHex2Int(line[48]) << 12) + (ConvertHex2Int(line[49]) << 8) +
            (ConvertHex2Int(line[51]) << 4) + (ConvertHex2Int(line[52]));
        
        
        cout << gyro_x << '\t' << gyro_y << '\t' << gyro_z << '\t' << accel_x << '\t' << accel_y << '\t' << accel_z << endl;
        
        outFile << gyro_x << '\t' << gyro_y << '\t' << gyro_z << '\t' << accel_x << '\t' << accel_y << '\t' << accel_z << endl;
        
        
        lineCount++;
    }
    
    
    return lineCount;
}

#endif	/* KALMAN_H */

