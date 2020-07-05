/*
 * nn.cpp
 *
 *  Created on: May 24, 2011
 *      Author: greg
 */

//#include "utilities.h"
#include <modelling/nn.h>
//#include <modelling/som.h>
#include <cstdlib>
#include <template/random.h>

MATRIX NeuralNet::GenerateRandomWeights(FLOAT range)
{
    for (int i = 0; i < weights.CountRows(); i++)
    {
        for (int j = 0; j < i; j++)
        {
            if (map[i][j])
                weights[i][j] = d_random(-range, range);
            else weights[i][j] = 0;
        }
    }

    return weights;
}

FLOAT NeuralNet::TrainBP(const MATRIX& training_inputs, const MATRIX& training_outputs,
            long unsigned maxIter, FLOAT threshold, FLOAT b) 
    {
        debugFile.open("debug.training.txt");
        if (!debugFile) throw XError("Cannot open debug.trainbp.txt!\n");

        FLOAT mse = CalcMSE(training_inputs, training_outputs);
        if (b != 0) beta = b;

        FLOAT old_mse = 0;
        int changed_beta = 1;

        for (long unsigned i = 0; i <= maxIter; i++)
        {
            dw.Zero();
            mse = 0;

            int trainingStart=rand() % training_inputs.CountRows();
            for (long j = 0; j < training_inputs.CountRows(); j++)
            {
                int record=(trainingStart + j) % training_inputs.CountRows();

                FeedForward(training_inputs.GetRow(record));
                mse += BackProp(training_outputs.GetRow(record));
            }

            mse /= (training_inputs.CountRows() * training_inputs.CountRows());

            //update the weights if we're using all trials combined
            if (backPropagateAvg) weights += dw * beta/training_inputs.CountRows();

            if (!(i % BP_STEPS)) {
                FLOAT mse_direct = CalcMSE(training_inputs, training_outputs);

                FLOAT mse_improvement = (old_mse - mse) / mse;
                if(backPropagateAvg) {}
                else if (changed_beta) changed_beta--; //don't want to jump around too much

                else if (mse_improvement < 0.01) //we're stuck
                {
                    FLOAT mse_ratio = (mse_direct) / mse;
                    if (mse_ratio > 1.01) //we're following the training too closely)
                    {
                        beta *= 0.97; //so lower beta
                        changed_beta = 8;
                    }
                    else
                    {
                        beta *= 1.01;
                    } //probably want to end this
                }

                cout << "Iter.: " << i << ", beta= " << beta << ", mse= " << mse << ", direct mse= " << mse_direct << endl;
                old_mse = mse;

                ifstream stop_file("stop.txt");
                if (stop_file) {
                    char stop_char;
                    stop_file >> stop_char;
                    if (stop_char == '1') {
#ifdef __DEBUG_TRAINBP__
                        debugFile << "Stop detected. Bye.\n";
#endif
                        break;
                        ;
                    }
                }
            }

            if (mse < threshold) break;
        }

#define __DEBUG_TRAINBP__

#ifdef __DEBUG_TRAINBP__

    for (int j = 0; j < training_inputs.CountRows(); j++)
    {
        VECTOR results = FeedForward(training_inputs.GetRow(j));
        debugFile << j;
        debugFile << '\t' << results
                << '\t' << (training_outputs.GetRow(j))
                << '\t' << (training_inputs.GetRow(j));
        debugFile << endl;
        }

#endif
        return mse;
    }
