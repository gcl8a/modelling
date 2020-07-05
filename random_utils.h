//
//  random.h
//  copter
//
//  Created by Gregory C Lewin on 12/14/15.
//  Copyright © 2015 Gregory C Lewin. All rights reserved.
//

#ifndef __MODELLING__RANDOM_UTILS_H
#define __MODELLING__RANDOM_UTILS_H

#include <linalg/matrix.h>

double randn(double average, double stdev);
dvector RandomNoise(const dvector& var);
dvector RandomNoise(dmatrix cov);
double RandomNoise(double var);

#endif /* random_h */
