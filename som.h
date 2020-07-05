/* 
 * File:   som.h
 * Author: gregl
 *
 * Created on July 3, 2012, 10:56 AM
 */

#ifndef SOM_H
#define	SOM_H

#include <fstream>
#include <modelling/prec_defs.h>
#include <template/random.h>

class SelfOrganizingMap2D
{
protected:
    TMatrix<VECTOR> map;
    
    FLOAT eta0;
    FLOAT sigma0;
    FLOAT tau_eta;
    FLOAT tau_sigma;
    
    MATRIX inputNorms;

    FLOAT t_k; //some semblance of time

public:
    SelfOrganizingMap2D(void)
    {
        
    }

    int Resize(int m, int n, int v, FLOAT range = 1.0)
            /*
             * sets up an m-by-n map with input vector lengths, v
             */
    {
        map=TMatrix<VECTOR>(m,n);
        for(int i=0; i<m; i++)
            for(int j=0; j<n; j++)
            {
                map[i][j]=VECTOR(v);
                for(int k=0; k<v; k++)
                    map[i][j][k]=d_random(-range, range);
            }  
        
        inputNorms = MATRIX(v, 2);
        for(int i=0; i<v; i++) SetInputNorms(i, 0, 1); //default to no normalization
         
        t_k=0;
 
        return m*n*v;
    }
    
    int SetTrainingParams(float e, float s, float t_e, float t_s)
    {
        eta0=e;
        sigma0=s;
        tau_eta=t_e;
        tau_sigma=t_s;
        
        t_k=0;
        
        return 0;
    }
    
    ivector FindClosestMatch(VECTOR v)
    {
        FLOAT min_dist=1.0e99; //distance in vector space
        ivector best(2);
        
        for(int i=0; i<map.CountRows(); i++)
            for(int j=0; j<map.CountColumns(); j++)
            {
                FLOAT dist=(v-map[i][j]).CalcL2Norm();
                //norms[i][j]=dist;
                if(dist < min_dist)
                {
                    min_dist=dist;
                    best[0]=i; best[1]=j;
                }
            } 
        
        return best;
    }
    
    ivector Classify(const VECTOR& v)
    {
        ivector best=FindClosestMatch(Normalize(v));
        return best;
    }
    
    ivector Train(const VECTOR& v)
    {
        VECTOR v_star=Normalize(v);
        
        ivector best=FindClosestMatch(v_star);
        
        FLOAT eta=eta0*exp(-t_k/tau_eta);
        FLOAT sigma=sigma0*exp(-t_k/tau_sigma);
        FLOAT sigma2=sigma*sigma;
        
        for(int i=0; i<map.CountRows(); i++)
            for(int j=0; j<map.CountColumns(); j++)
            {
                FLOAT dist2=(i-best[0])*(i-best[0])+(j-best[1])*(j-best[1]); //Euclidian dist in computational space
                FLOAT T=exp(-dist2/(2.0*sigma2));
                
                VECTOR delta=(v_star-map[i][j])*(eta*T);
                map[i][j]+=delta;
            }
        
        return best;
    }
    
    int Train(MATRIX& training_inputs, long unsigned epochs, long unsigned maxIter) 
    {
        for(long unsigned t = 0; t <= epochs; t++)
        {
            t_k=t;
            for(long unsigned i = 0; i <= maxIter; i++)
            {
                int sample=rand() % training_inputs.CountRows();
                Train(training_inputs.GetRow(sample));
            }
        }
        
        return 0;
    }
    
    int ReadFile(const char* filename)
    {
        ifstream inFile(filename);
        if(!inFile) throw XError("Can't find file in SOM::ReadFile()! Bye.\n");
        
        int m,n,v;
        inFile>>m>>n>>v;
        Resize(m,n,v);

        float e, s, t_e, t_s;
        inFile>>e>>s>>t_e>>t_s;
        SetTrainingParams(e, s, t_e, t_s);
        
        inputNorms=MATRIX(v, 2);
        inFile>>inputNorms;
 
        int data;
        inFile>>data;
        
        if(data)
        {
            for(int i=0; i<m; i++)
            for(int j=0; j<n; j++)
                for(int k=0; k<v; k++)
                    inFile>>map[i][j][k];
         }
        
        return 0;
    }

    int WriteFile(const char* filename)
    {
        ofstream outFile(filename);
        if(!outFile) throw XError("Can't find file in SOM::WriteFile()! Bye.\n");
        
        //TODO: check for [0][0] before we access it!!!!!
        outFile<<map.CountRows()<<' '<<map.CountColumns()<<' '<<map[0][0].Length()<<endl;

        FLOAT eta=eta0*exp(-t_k/tau_eta);
        FLOAT sigma=sigma0*exp(-t_k/tau_sigma);
        outFile<<eta<<' '<<sigma<<' '<<tau_eta<<' '<<tau_sigma<<endl;
        
        outFile<<inputNorms;
        
        outFile<<1<<endl;

        for (int i = 0; i < map.CountRows(); i++)
            for (int j = 0; j < map.CountColumns(); j++) 
            {
                for (int k = 0; k < map[i][j].Length(); k++)
                {
                    if(k) outFile<<' ';
                    outFile << map[i][j][k];
                }                    
                
                outFile<<endl;
            }                    

            return 0;
    }
    
    int SetInputNorms(int i, FLOAT offset, FLOAT scale)
    {
        inputNorms[i][0]=offset;
        inputNorms[i][1]=scale ? scale : 1;

        return 0;
    }
    
    VECTOR Normalize(const VECTOR& v)
    {
        VECTOR norm=v;
        for (int i = 0; i < v.Length(); i++)
        {
            norm[i] = (v[i]-inputNorms[i][0])/inputNorms[i][1];
        }

        return norm;
    }
};

#endif	/* SOM_H */

