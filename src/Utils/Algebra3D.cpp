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
#include "Utils/Algebra3D.hpp"

namespace Antipatrea
{ 
    namespace Algebra3D 	
    {
// utils operations
	void VecNormal(const double v[], double vnormal[])
	{
	    if(fabs(v[0] - v[2]) <= Constants::EPSILON &&
	       fabs(v[0] + v[1]) <= Constants::EPSILON)
	    {
		const double x = -v[0];		    
		vnormal[0] = v[1];
		vnormal[1] = x;
		vnormal[2] = 0;		    
	    }
	    else
	    {
		const double x = -v[0];
		const double y =  v[1];
		vnormal[0] = y + v[2];
		vnormal[1] = x + v[2];
		vnormal[2] = x - y;
	    }	
	}
	
//conversion	    
	void RotAsQuat(const double R[], double Q[])
	{
	    const double trace = R[0] + R[4] + R[8];
	    double       s;
	    
	    if(trace > 0)
	    {
		s    = sqrt(trace + 1);
		Q[3] = 0.5 * s;
		s    = 0.5 / s;
		Q[0] = s * (R[7] - R[5]);
		Q[1] = s * (R[2] - R[6]);
		Q[2] = s * (R[3] - R[1]);
	    }
	    else if(R[8] > R[4] && R[8] > R[0]) 
	    {
		//i = 2, j = 0, k = 1
		s    = sqrt(R[8] - R[0] - R[4] + 1);
		Q[2] = 0.5 * s;
		s    = 0.5 / s;
		Q[3] = s * (R[3] - R[1]);
		Q[0] = s * (R[2] + R[6]);
		Q[1] = s * (R[5] + R[7]);
	    }
	    else if(R[4] > R[0])
	    {
		//i = 1, j = 2, k = 0
		s    = sqrt(R[4] - R[8] - R[0] + 1);
		Q[1] = 0.5 * s;
		s    = 0.5 / s;
		Q[3] = s * (R[2] - R[6]);
		Q[2] = s * (R[7] + R[5]);
		Q[0] = s * (R[1] + R[3]);
	    }
	    else
	    {
		//i = 0, j = 1, k = 2
		s    = sqrt(R[0] - R[4] - R[8] + 1);
		Q[0] = 0.5 * s;
		s    = 0.5 / s;
		Q[3] = s * (R[7] - R[5]);
		Q[1] = s * (R[3] + R[1]);
		Q[2] = s * (R[6] + R[2]);
	    }
	}
	void FromToAxisAsQuat(const double Afrom[], const double Ato[], const double normProd,
			      double Q[])
	{
	    //Q = [((Afrom x Ato) / (|Afrom x Ato|)) * sin(theta/2), cos(theta/2)]
	    //Now
	    //  |Afrom x Ato| = |Afrom| |Ato| sin(theta) = |Afrom||Ato| 2sin(theta/2)cos(theta/2)
	    //So
	    //Q = [(Afrom x Ato / |Afrom||Ato|) * 1/(2cos(theta/2)), cos(theta/2)]
	    //Moreover,
	    //  cos(theta/2) = sqrt((1+cos(theta))/2)
	    //  cos(theta)   = Afrom.Ato / (|Afrom||Ato|)
	    
	    if(normProd <= Constants::EPSILON)
		IdentityAsQuat(Q);
	    else
	    { 
		const double n = 1/normProd;
		Q[3] = n * VecDotProduct(Afrom, Ato);
		if(Q[3] <= -1 + Constants::EPSILON)
		{		    
		    VecNormal(Afrom, Q);
		    VecUnit(Q, Q);
		    Q[3] = 0;
		}
		else if(Q[3] + Constants::EPSILON >= 1)
		    IdentityAsQuat(Q);
		else
		{
		    Q[3] = sqrt(0.5 * (1 + Q[3]));
		    const double s = n * 0.5 / Q[3];
		    const double x = s * (Afrom[1] * Ato[2] - Afrom[2] * Ato[1]);
		    const double y = s * (Afrom[2] * Ato[0] - Afrom[0] * Ato[2]);
		    Q[2] = s * (Afrom[0] * Ato[1] - Afrom[1] * Ato[0]);
		    Q[0] = x;
		    Q[1] = y;
		}		
	    }
	}
	void FromToAxisAsRot(const double Afrom[], const double Ato[], const double normProd, 
			     double R[])
	{
	    double A[3];
	    if(normProd <= Constants::EPSILON)
		IdentityAsRot(R);
	    else
	    {
		const double c = VecDotProduct(Afrom, Ato) / normProd;
		if(c + Constants::EPSILON >= 1) //rotation by 0
		    IdentityAsRot(R);
		else if(c <= -1 + Constants::EPSILON) //rotation by PI: c = -1, s = 0
		{
		    VecNormal(Afrom, A);
		    VecUnit(A, A);
		    R[0] = A[0]*A[0]*2 - 1; R[1] = A[0]*A[1]*2; 	R[2] = A[0]*A[2]*2;
		    R[3] = A[0]*A[1]*2; 	R[4] = A[1]*A[1]*2 - 1;	R[5] = A[1]*A[2]*2;
		    R[6] = A[0]*A[2]*2; 	R[7] = A[1]*A[2]*2;	R[8] = A[2]*A[2]*2 - 1;
		}
		else
		{
		    VecCrossProduct(Afrom, Ato, A);
		    const double norm = VecNorm(A);
		    const double s    = norm / normProd;			
		    const double ninv = 1/norm;
		    A[0] *= ninv;
		    A[1] *= ninv;
		    A[2] *= ninv;
		    
		    R[0] = A[0]*A[0]*(1-c) + c;
		    R[1] = A[0]*A[1]*(1-c) - A[2]*s;
		    R[2] = A[0]*A[2]*(1-c) + A[1]*s;
		    R[3] = A[0]*A[1]*(1-c) + A[2]*s;
		    R[4] = A[1]*A[1]*(1-c) + c;
		    R[5] = A[1]*A[2]*(1-c) - A[0]*s;
		    R[6] = A[0]*A[2]*(1-c) - A[1]*s;
		    R[7] = A[1]*A[2]*(1-c) + A[0]*s;
		    R[8] = A[2]*A[2]*(1-c) + c;
		}
	    }
	}
	void FromToAxisAsAxisAngle(const double Afrom[], const double Ato[], const double normProd,
				   double AA[])
	{
	    if(normProd <= Constants::EPSILON)
	    {
		VecNormal(Afrom, AA);
		AA[3] = 0;
	    }
	    else
	    {
		const double c = VecDotProduct(Afrom, Ato) / normProd;
		if(c + Constants::EPSILON >= 1) //rotation by 0
		{
		    VecNormal(Afrom, AA);
		    AA[3] = 0;
		}
		else if(c <= -1 + Constants::EPSILON) //rotation by PI
		{
		    VecNormal(Afrom, AA);
		    AA[3] = M_PI;
		}
		else
		{
		    VecCrossProduct(Afrom, Ato, AA);
		    AA[3] = acos(c);
		}
	    }
	}
	
//combine
	void RotMultRotAsRot(const double Ra[], const double Rb[], double R[])
	{
	    double *useR = R;
	    double tmp[Rot_NR_ENTRIES];
	    
	    if(R == Ra || R == Rb)
		useR = tmp;
	    
	    useR[0] = Ra[0] * Rb[0] + Ra[1] * Rb[3] + Ra[2] * Rb[6];
	    useR[1] = Ra[0] * Rb[1] + Ra[1] * Rb[4] + Ra[2] * Rb[7];
	    useR[2] = Ra[0] * Rb[2] + Ra[1] * Rb[5] + Ra[2] * Rb[8];
	    
	    useR[3] = Ra[3] * Rb[0] + Ra[4] * Rb[3] + Ra[5] * Rb[6];
	    useR[4] = Ra[3] * Rb[1] + Ra[4] * Rb[4] + Ra[5] * Rb[7];
	    useR[5] = Ra[3] * Rb[2] + Ra[4] * Rb[5] + Ra[5] * Rb[8];
	    
	    useR[6] = Ra[6] * Rb[0] + Ra[7] * Rb[3] + Ra[8] * Rb[6];
	    useR[7] = Ra[6] * Rb[1] + Ra[7] * Rb[4] + Ra[8] * Rb[7];
	    useR[8] = Ra[6] * Rb[2] + Ra[7] * Rb[5] + Ra[8] * Rb[8];
	    
	    if(useR != R)
		RotAsRot(useR, R);
	}
	
	
	
//sample
	void QuatSampleUniform_Marsaglia(double Q[])
	{	
	    double x, y, z, w, s, s1, s2;
	    
	    do
	    {
		x = RandomUniformReal(-1, 1);
		y = RandomUniformReal(-1, 1);	
	    }
	    while((s1 = (x * x + y * y)) >= 1.0);
	    
	    do
	    {
		z = RandomUniformReal(-1, 1);
		w = RandomUniformReal(-1, 1);			
		s2= z * z + w * w;	
	    }
	    while(s2 >= 1.0 || s2 == 0.0);
	    
	    s = sqrt((1.0 - s1) / s2);	
	    
	    Q[0] = x;
	    Q[1] = y;
	    Q[2] = z * s;
	    Q[3] = w * s;
	}
	void QuatSampleUniform_Gaussian(double Q[])
	{	
	    double x, y, z, w, s1, s2, n1, n2, r1, r2, r;
	    
	    do
	    {
		x = RandomUniformReal(-1, 1);
		y = RandomUniformReal(-1, 1);		
		s1= x * x + y * y;			
	    } while(s1 >= 1.0 || s1 == 0.0);
	    
	    do
	    {
		z = RandomUniformReal(-1, 1);
		w = RandomUniformReal(-1, 1);		
		s2= z * z + w * w;			
	    } while(s2 >= 1.0 || s2 == 0.0);
	    
	    n1 = -2 * log(s1);
	    n2 = -2 * log(s2);
	    r  =  n1 + n2;
	    r1 =  sqrt(n1 / (s1 * r));
	    r2 =  sqrt(n2 / (s2 * r));
	    
	    Q[0] = x * r1;
	    Q[1] = y * r1;
	    Q[2] = z * r2;
	    Q[3] = w * r2;
	}
	void QuatSampleUniform_Shoemake(double Q[])
	{
	    const double twoPi = 2 * M_PI;    
	    const double s     = RandomUniformReal();
	    const double s1    = sqrt(1 - s);
	    const double s2    = sqrt(s);
	    const double a1    = twoPi * RandomUniformReal();
	    const double a2    = twoPi * RandomUniformReal();
	    
	    Q[0] = sin(a1) * s1;
	    Q[1] = cos(a1) * s1;
	    Q[2] = sin(a2) * s2;
	    Q[3] = cos(a2) * s2;
	}	    
    }
}

