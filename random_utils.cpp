//
//  random.h
//  copter
//
//  Created by Gregory C Lewin on 12/14/15.
//  Copyright © 2015 Gregory C Lewin. All rights reserved.
//

#include <modelling/random_utils.h>

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

double RandomNoise(double var)
{
    return randn(0, sqrt(var));
}
