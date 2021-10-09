/*
 * nn.h
 *
 *  Created on: Apr 27, 2011
 *      Author: greg
 */

/* THINGS TO DO!!!!!!!!!
 * remove many of the zeros from the map
 * auto-normalization??
 */

#ifndef NN_H_
#define NN_H_

#include <fstream>

#include <linalg/vector.h>
#include <linalg/matrix.h>

#include <template/array.h>

#include "prec_defs.h"

#define BIAS 1.0
#define BP_STEPS 100

enum OUTPUT_TYPE {
    ZERO, SIGMOID, EXPONENTIAL, VALUE
};

#define __DEBUG_TRAINBP__  //outputs results of back propagation training to a file

class NeuralNet
{
    int nInputs, nOutputs;
    MATRIX inputNorms;
    MATRIX outputNorms;

    VECTOR values;
    VECTOR errors;

    imatrix map; //connectivity mapping
    MATRIX weights; //the actual weights
    MATRIX best_weights;
    MATRIX dw; //for updating the weights

    FLOAT beta;
    int backPropagateAvg; //determines if we update with each trial or as a whole
    int autoLearning; //adjusts the learning rate based on convergence rates
    int trainRandomOrder; //picks randomly from the training set to avoid over-weighting the end

    //debug file
    ofstream debugFile;
public:

    NeuralNet(void)
    {
        nInputs = 0;
        nOutputs = 0;

        backPropagateAvg = 0;
        autoLearning = 1;
        trainRandomOrder = 1;
    }

    NeuralNet(const char* defFile)
    {
        nInputs = 0;
        nOutputs = 0;

        backPropagateAvg = 0;
        autoLearning = 1;
        trainRandomOrder = 1;

        ReadFile(defFile);
    }

    int ReadFile(const char* filename)
    {
        ifstream inFile(filename);
        if (!inFile)
        {
            string error("Can't find file in NeuralNet::ReadFile: ");
            error+=filename+string("\n");
            throw XError(error.c_str());
        }

        int total_nodes;
        inFile>>total_nodes>>nInputs>>nOutputs;

        values = VECTOR(total_nodes);
        errors = VECTOR(total_nodes);
        weights = MATRIX(total_nodes, total_nodes);
        dw = MATRIX(total_nodes, total_nodes);
        map = imatrix(total_nodes, total_nodes);

        inputNorms=MATRIX(nInputs, 2);
        outputNorms=MATRIX(nOutputs, 2);

        inFile>>inputNorms;
        inFile>>outputNorms;

        inFile >> map;

        inFile>>weights;

        inFile>>beta;

        return 0;
    }

    int WriteFile(const char* filename)
    {
        ofstream outFile(filename);

        if (!outFile) throw XError("Can't find file in WriteFile. Bye!\n");

        outFile << weights.CountRows() << " " << nInputs << " " << nOutputs << endl;

        outFile << inputNorms;
        outFile << outputNorms;
        outFile << map;
        outFile << weights;

        outFile << beta;

        return 0;
    }

/*    int ReadSFile(const char* filename, int l)
    {
        ifstream inFile(filename);
        if (!inFile) throw XError("Can't find file in ReadSFile. Bye!\n");

        char waste[512];
        inFile >> waste;
        inFile >> waste;

        ivector m(l);
        for (int i = 0; i < l; i++)
            inFile >> m[i];

        CreateStdMap(m, VALUE);

        inFile.getline(waste, 512);

        int firstCol = 0;
        int firstRow = 0;

        for (int k = 1; k < l; k++)
        {
            firstRow += m[k - 1];
            for (int j = 1; j <= m[k]; j++)
            {
                for (int i = 1; i <= m[k - 1]; i++)
                {
                    inFile >> weights[firstRow + j][firstCol + i];
                    inFile.getline(waste, 512);
                }

                inFile >> weights[firstRow + j][0]; //bias
                inFile.getline(waste, 512);
            }

            firstCol += m[k - 1];
        }

        return 0;
    }*/

    VECTOR GetValues(void)
    {
        return values;
    }

    MATRIX GetWeights(void)
    {
        return weights;
    }

protected:
    int SetWeights(const MATRIX& w)
    {
        weights = w;
        return 0;
    }
    
    imatrix GetMap(void) {return map;}

public:
    imatrix CreateStdMap(const ivector& nodes_per_level, OUTPUT_TYPE output_type)
    {
        int total_nodes = 1; //bias
        for (int i = 0; i < nodes_per_level.Length(); i++)
            total_nodes += nodes_per_level[i];

        values = VECTOR(total_nodes);
        errors = VECTOR(total_nodes);
        weights = MATRIX(total_nodes, total_nodes);
        dw = MATRIX(total_nodes, total_nodes);
        map = imatrix(total_nodes, total_nodes);

        nInputs = nodes_per_level[0];
        nOutputs = nodes_per_level.Last();

        inputNorms = MATRIX(nInputs, 2);
        for(int i=0; i<nInputs; i++) SetInputNorms(i, 0, 1); //default to no normalization
        outputNorms = MATRIX(nOutputs, 2);
        for(int i=0; i<nOutputs; i++) SetOutputNorms(i, 0, 1); //and again

        map.Zero();
        int start_node = nInputs + 1;

        for (int i = 1; i < nodes_per_level.Length(); i++)
        {
            for (int j = start_node; j < start_node + nodes_per_level[i]; j++)
            {
                map[j][0] = 1;
                for (int k = start_node - nodes_per_level[i - 1]; k < start_node; k++)
                {
                    map[j][k] = 1;
                }
                map[j][j] = SIGMOID;
            }

            start_node += nodes_per_level[i];
        }

        //outputs
        for (int k = total_nodes - nOutputs; k < total_nodes; k++)
            map[k][k] = output_type;

        return map;
    }

    int SetInputNorms(int i, FLOAT offset, FLOAT scale)
    {
        inputNorms[i][0]=offset;
        inputNorms[i][1]=scale ? scale : 1;

        return 0;
    }

    int SetOutputNorms(int i, FLOAT offset, FLOAT scale)
    {
        outputNorms[i][0]=offset;
        outputNorms[i][1]=scale;

        return 0;
    }

    int SetInputNorms(VECTOR offset, VECTOR scale)
    {
        inputNorms.SetColumn(0, offset);
        inputNorms.SetColumn(1, scale);

        return 0;
    }

    int SetOutputNorms(VECTOR offset, VECTOR scale)
    {
        outputNorms.SetColumn(0, offset);
        outputNorms.SetColumn(1, scale);

        return 0;
    }

    VECTOR FeedForward(const VECTOR& inputs)
    {
        if (nInputs != inputs.Length())
		{
			string err("Incorrect input length in NN::FeedForward.\n");
			cout<<"Expected "<<nInputs<<". Got "<<inputs.Length()<<endl;
			throw XError(err.c_str());
		}

        values[0] = BIAS;

        //copy the inputs
        for (int i = 0; i < nInputs; i++)
        {
            values[i + 1] = (inputs[i]-inputNorms[i][0])/inputNorms[i][1];
        }

        //feed forward
        for (int i = nInputs + 1; i < values.Length(); i++)
        {
            FLOAT sum = 0;
            for (int j = 0; j < i; j++) //a little wasteful for all the zeros
            {
                sum += weights[i][j] * values[j];
            }

            //output function
            switch (map[i][i])
            {
                case 0:
                    values[i] = 0;
                    break;
                case SIGMOID:
                    values[i] = 1.0 / (1.0 + exp(-sum));
                    break;
                case EXPONENTIAL:
                    values[i] = exp(sum);
                    break;
                case VALUE:
                    values[i] = sum;
                    break;
                default:
                    throw XError("Illegal output type in map\n");
            }
        }

        VECTOR result(nOutputs);
        for (int k = 0; k < nOutputs; k++)
        {
            result[k] = values[values.Length() - nOutputs + k] *
                    outputNorms[k][1]+outputNorms[k][0];
        }

        return result;
    }

    FLOAT BackProp(const VECTOR& targets)
    {
        if (nOutputs != targets.Length()) throw XError("Incorrect target length.\n");

        VECTOR delta(values.Length());
        if (!backPropagateAvg) dw.Zero();

        //back-propagate the error
        int firstOutput = values.Length() - nOutputs;
        for (int i = values.Length() - 1; i > nInputs; i--)
        {
            FLOAT deriv=0;
            switch (map[i][i])
            {
                case ZERO:
                    deriv = 0;
                    break;
                case SIGMOID:
                    deriv = values[i]*(1.0 - values[i]);
                    break;
                case EXPONENTIAL:
                    deriv = values[i];
                    break;
                case VALUE:
                    deriv = 1.0;
                    break;
                default:
                    throw XError("Illegal output type in map\n");
            }

            delta[i] = 0;

            if (i >= firstOutput)
            {
                int io=i-firstOutput;
                FLOAT target=(targets[io]-outputNorms[io][0])/outputNorms[io][1];
                errors[i] = target - values[i];
                delta[i] = errors[i] * deriv;
            }

            else for (int j = values.Length() - 1; j > i; j--)
            {
                delta[i] += deriv * delta[j] * weights[j][i];
            }

            //store the weight update values
            for (int j = 0; j < i; j++)
            {
                if (map[i][j]) dw[i][j] += delta[i] * values[j];
            }
        }

        //then update the weights if we're doing it with each record
        if (!backPropagateAvg) weights += dw * beta;

        return errors.Dot(errors);
    }

    FLOAT CalcMSE(const MATRIX& inputs, const MATRIX& outputs)
    {
        FLOAT mse = 0;
        for (int i = 0; i < inputs.CountRows(); i++)
        {
            VECTOR result = FeedForward(inputs.GetRow(i));
            VECTOR error = result-outputs.GetRow(i);
            for(int j=0; j< nOutputs; j++) error[j]/=outputNorms[j][1];
            mse += error.Dot(error);
        }

        return mse / (inputs.CountRows() * inputs.CountRows());
    }

    MATRIX GenerateRandomWeights(FLOAT range);

    FLOAT TrainBP(const MATRIX& training_inputs, const MATRIX& training_outputs,
            long unsigned maxIter, FLOAT threshold, FLOAT b);

    FLOAT TestNN(const MATRIX& training_inputs, const MATRIX& training_outputs);
};

#endif /* NN_H_ */
