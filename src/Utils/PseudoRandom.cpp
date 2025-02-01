/*
 * Copyright (C) 2018 Erion Plaku
 * All Rights Reserved
 * 
 *       Created by Erion Plaku
 *       Computational Robotics Group
 *       Department of Electrical Engineering and Computer Science
 *       Catholic University of America
 *
 *       www.robotmotionplanning.org
 *
 * Code should not be distributed or used without written permission from the
 * copyright holder.
 */
#include "Utils/PseudoRandom.hpp"
#include <cstdio>
#include <ctime>
#include <cmath>
#include <vector>
#include <algorithm>

namespace Antipatrea
{
    unsigned int RandomSeed(void)
    {
	FILE        *fp = fopen("/dev/urandom", "r");    
	unsigned int s;
	int          size = 0;
	
	if(fp != NULL)
	{
	    size = fread(&s, sizeof(unsigned int), 1, fp);    
	    fclose(fp);    	        
	}
	else
	    s = (unsigned int) time(NULL);
	
	RandomSeed(s);
	
	return s;
    }
    
    void RandomSeed(const unsigned int s)
    {
	srandom(s);
    }
    
    double RandomGaussianReal(const double mean, const double stddev)
    {
	double        x1, x2, w, y1;    
	static double y2;
	static int    use_last = 0;
	
	if (use_last)                                /* use gaussian sample from previous time. */
	{
	    y1       = y2;
	    use_last = 0;
	}
	else
	{
	    do
	    {
		x1 = 2.0 * RandomUniformReal() - 1.0;   /* select uniformly at random a point inside */
		x2 = 2.0 * RandomUniformReal() - 1.0;   /* the square [-1, 1] x [-1, 1]. */	    
		w  = x1 * x1 + x2 * x2;              /* continue the selection as long as */
	    }                                        /* the point is not inside */
	    while (w >= 1.0 || w == 0.0);            /* the unit circle. */
	    
	    w = sqrt ((-2.0 * log(w) ) / w);  
	    
	    y1       = x1 * w;                       /* complete the Box-Muller transformations to get */
	    y2       = x2 * w;                       /* two gaussian samples. */
	    use_last = 1;                            /* use one now and save the other for the next time. */
	} 
	/* transform the sample to a Gaussian distribution */
	return mean + y1 * stddev;                   /* with mean mean and standard deviation stddev. */
    }

      void RandomPointOnSphereSurface(const int    nrDims,
				      const double center[],
				      const double r,
				      double       p[])
    {
	std::vector<double> radii;
	radii.resize(nrDims);
	std::fill(radii.begin(), radii.end(), r);
	RandomPointOnEllipsoidSurface(nrDims, center, &radii[0], p);
    }

    void RandomPointInsideSphere(const int nrDims,
				 const double center[],
				 const double r,
				 double       p[])
    {
	std::vector<double> radii;
	radii.resize(nrDims);
	std::fill(radii.begin(), radii.end(), r);
	RandomPointInsideEllipsoid(nrDims, center, &radii[0], p);
	
    }

    void RandomPointOnEllipsoidSurface(const int    nrDims,
				       const double center[],
				       const double radii[],
				       double       p[])
    {
	double s = 0.0;
	
	for(int i = 0; i < nrDims; i++)
	{
	    p[i] = RandomGaussianReal(0.0, 1.0);	
	    s   += p[i] * p[i];
	}    
	s = 1.0 / sqrt(s);    
	for(int i = 0; i < nrDims; ++i) 
	    p[i] = center[i] + p[i] * radii[i] * s;
    }

    void RandomPointInsideEllipsoid(const int    nrDims,
				    const double center[],
				    const double radii[],
				    double       p[])
    {
	double s = 0.0;
	
	for(int i = 0; i < nrDims; ++i)
	{
	    p[i] = RandomGaussianReal(0.0, 1.0);	
	    s   += p[i] * p[i];
	}    
	s = pow(RandomUniformReal(), 1.0/nrDims) / sqrt(s);    
	for(int i = 0; i < nrDims; ++i) 
	    p[i] = center[i] + p[i] * radii[i] * s;
    }
    
    
}





    
    
    
    
    
    
