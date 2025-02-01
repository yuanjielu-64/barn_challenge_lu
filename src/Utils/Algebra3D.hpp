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

#ifndef Antipatrea__Algebra3D_HPP_
#define Antipatrea__Algebra3D_HPP_

#include "Utils/Constants.hpp"
#include "Utils/PseudoRandom.hpp"
#include <cmath>
#include <cstring>

namespace Antipatrea
{
    namespace Algebra3D
    {
	enum
	    {
		Trans_NR_ENTRIES = 3,
		Quat_NR_ENTRIES = 4,
		Rot_NR_ENTRIES = 9,
		AxisAngle_NR_ENTRIES = 4,
		TransQuat_NR_ENTRIES = 7,
		TransRot_NR_ENTRIES = 12,
		TransAxisAngle_NR_ENTRIES = 7
	    };
	
//Vec operations
	static inline double VecDotProduct(const double v1[], const double v2[])
	{
	    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
	}	    
	static inline void VecCrossProduct(const double v1[], const double v2[], double v[])
	{
	    const double x = v1[1] * v2[2] - v1[2] * v2[1];
	    const double y = v1[2] * v2[0] - v1[0] * v2[2];
	    v[2] = v1[0] * v2[1] - v1[1] * v2[0];		
	    v[0] = x;		
	    v[1] = y;		
	}

	static inline void VecNormalToTriangle(const double ta[],
					       const double tb[],
					       const double tc[],
					       double v[])
	{
	    const double ab[3] = {tb[0] - ta[0], tb[1] - ta[1], tb[2] - ta[2]};
	    const double ac[3] = {tc[0] - ta[0], tc[1] - ta[1], tc[2] - ta[2]};
	    
	    VecCrossProduct(ab, ac, v);
	}
	

	static inline double PointDistanceSquared(const double p1[], const double p2[])
	{
	    return 
		(p1[0]-p2[0])*(p1[0]-p2[0]) + 
		(p1[1]-p2[1])*(p1[1]-p2[1]) + 
		(p1[2]-p2[2])*(p1[2]-p2[2]);
	}	    
	static inline double PointDistance(const double p1[], const double p2[])
	{
	    return sqrt(PointDistanceSquared(p1, p2));
	}	    
	static inline double VecNormSquared(const double v[])
	{
	    return VecDotProduct(v, v);
	}
	static inline double VecNorm(const double v[])
	{
	    return sqrt(VecNormSquared(v));
	}
	static inline void VecUnit(const double v[], double vunit[])
	{
	    const double d = 1.0 / VecNorm(v);
	    vunit[0] = d * v[0];
	    vunit[1] = d * v[1];
	    vunit[2] = d * v[2];
	}
	static inline void VecScale(const double v[], const double s, double vscaled[])
	{
	    vscaled[0] = s * v[0];
	    vscaled[1] = s * v[1];
	    vscaled[2] = s * v[2];
	}
	static inline void VecClampMagnitude(double v[], const double dmax)
	{
	    const double d = VecNormSquared(v);
	    if(d  > (dmax * dmax))
		VecScale(v, dmax / sqrt(d), v);
	}	    
	static inline void VecAdd(const double v1[], const double v2[], double v[])
	{
	    v[0] = v1[0] + v2[0];
	    v[1] = v1[1] + v2[1];
	    v[2] = v1[2] + v2[2];		
	}	    
	static inline void VecSubtract(const double v1[], const double v2[], double v[])
	{
	    v[0] = v1[0] - v2[0];
	    v[1] = v1[1] - v2[1];
	    v[2] = v1[2] - v2[2];		
	}	    
	static inline void VecLinearInterpolation(const double v1[],
						  const double t1,
						  const double v2[],
						  const double t2,
						  double v[])
	{
	    v[0] = t1 * v1[0] + t2 * v2[0];
	    v[1] = t1 * v1[1] + t2 * v2[1];		
	    v[2] = t1 * v1[2] + t2 * v2[2];		
	}
	static inline double VecAngle(const double v1[], const double v2[])
	{
	    return acos(VecDotProduct(v1, v2) / (VecNorm(v1) * VecNorm(v2)));
	}
	static inline double VecAngleUnitArgs(const double v1[], const double v2[])
	{
	    return acos(VecDotProduct(v1, v2));
	}
	void VecNormal(const double v[], double vnormal[]);
	static inline void VecParallelProjection(const double v[],  const double u[],  double v_on_u[])
	{
	    const double d = VecDotProduct(v, u) / VecNorm(u);
	    v_on_u[0] = d * u[0];
	    v_on_u[1] = d * u[1];
	    v_on_u[2] = d * u[2];
	}
	static inline void VecParallelProjectionUnitArgs(const double v[],  const double u[], double v_on_u[])
	{
	    const double d = VecDotProduct(v, u);
	    v_on_u[0] = d * u[0];
	    v_on_u[1] = d * u[1];
	    v_on_u[2] = d * u[2];
	}	    
	static inline void VecOrthogonalProjection(const double v[], const double u[], double v_on_u[])
	{
	    const double d = VecDotProduct(v, u) / VecNorm(u);
	    v_on_u[0] = v[0] - d * u[0];
	    v_on_u[1] = v[1] - d * u[1];
	    v_on_u[2] = v[2] - d * u[2];
	}
	static inline void VecOrthogonalProjectionUnitArgs(const double v[], const double u[], double v_on_u[])
	{
	    const double d = VecDotProduct(v, u);
	    v_on_u[0] = v[0] - d * u[0];
	    v_on_u[1] = v[1] - d * u[1];
	    v_on_u[2] = v[2] - d * u[2];
	}  
	static inline bool VecAreNormal(const double v1[], const double v2[])
	{
	    return fabs(VecDotProduct(v1,v2)) <= Constants::EPSILON;
	}	    
	static inline bool VecAreParallelOrAntiParallel(const double v1[], const double v2[])
	{
	    const double x = v1[1] * v2[2] - v1[2] * v2[1];
	    const double y = v1[2] * v2[0] - v1[0] * v2[2];
	    const double z = v1[0] * v2[1] - v1[1] * v2[0];		
	    return (x * x + y * y + z * z) <= Constants::EPSILON_SQUARED;
	}	
	static inline bool VecAreParallel(const double v1[], const double v2[])
	{
	    return VecAreParallelOrAntiParallel(v1, v2) && (!(VecDotProduct(v1, v2) < 0));
	}	    
	static inline bool VecAreAntiParallel(const double v1[], const double v2[])
	{
	    return VecAreParallelOrAntiParallel(v1, v2) && VecDotProduct(v1, v2) < 0;
	}	    
	static inline int VecAlignment(const double v1[], const double v2[])
	{
	    return VecAreParallelOrAntiParallel(v1, v2) ? 
		(VecDotProduct(v1, v2) < 0 ? Constants::ANTI_PARALLEL : Constants::PARALLEL) : 
		Constants::NEITHER_PARALLEL_NOR_ANTI_PARALLEL;
	}	    
	static inline bool VecAreParallelOrAntiParallelUnitArgs(const double v1[], const double v2[])
	{
	    const double d = VecDotProduct(v1, v2);
	    return fabs(1 - d) <= Constants::EPSILON || fabs(1 + d) <= Constants::EPSILON;
	}   
	static inline bool VecAreParallelUnitArgs(const double v1[], const double v2[])
	{
	    return fabs(1 - VecDotProduct(v1, v2)) <= Constants::EPSILON;
	}
	static inline bool VecAreAntiParallelUnitArgs(const double v1[], const double v2[])
	{
	    return fabs(1 + VecDotProduct(v1, v2)) <= Constants::EPSILON;
	}
	static inline int VecAlignmentUnitArgs(const double v1[], const double v2[])
	{
	    const double d = VecDotProduct(v1, v2);
	    return 
		fabs(1 - d) <= Constants::EPSILON ? Constants::PARALLEL : 
		(fabs(1 + d) < Constants::EPSILON ? Constants::ANTI_PARALLEL : 
		 Constants::NEITHER_PARALLEL_NOR_ANTI_PARALLEL);
	}	    
	
// utils operations
	static inline double QuatDotProduct(const double Q1[], const double Q2[])
	{
	    return Q1[0] * Q2[0] + Q1[1] * Q2[1] + Q1[2] * Q2[2] + Q1[3] * Q2[3];
	}
	static inline double QuatNormSquared(const double Q[])
	{
	    return QuatDotProduct(Q, Q);
	}
	static inline double QuatNorm(const double Q[])
	{
	    return sqrt(QuatNormSquared(Q));
	}
	static inline void QuatUnit(const double Q[], double Qunit[])
	{
	    const double d = 1.0 / QuatNorm(Q);
	    Qunit[0] = d * Q[0];
	    Qunit[1] = d * Q[1];
	    Qunit[2] = d * Q[2];
	    Qunit[3] = d * Q[3];		
	}
	
//identity
	static inline void IdentityAsTrans(double T[])
	{
	    T[0] = T[1] = T[2] = 0;
	}
	static inline void IdentityAsQuat(double Q[])
	{
	    Q[0] = Q[1] = Q[2] = 0;
	    Q[3] = 1;
	}
	static inline void IdentityAsRot(double R[])
	{
	    R[0] = R[4] = R[8] = 1;
	    R[1] = R[2] = R[3] = R[5] = R[6] = R[7] = 0;
	}
	static inline void IdentityAsAxisAngle(double AA[])
	{
	    AA[0] = AA[1] = AA[3] = 0;
	    AA[2] = 1;		
	}	    
	static inline void IdentityAsTransQuat(double TQ[])
	{
	    TQ[0] = TQ[1] = TQ[2] = TQ[3] = TQ[4] = TQ[5] = 0;
	    TQ[6] = 1;
	}
	static inline void IdentityAsTransRot(double TR[])
	{
	    TR[3] = TR[7] = TR[11] = 1;
	    TR[0] = TR[1] = TR[2]  = TR[4] = TR[5] = TR[6] = TR[8] = TR[9] = TR[10] = 0;
	}
	static inline void IdentityAsTransAxisAngle(double TAA[])
	{
	    TAA[0] = TAA[1] = TAA[2] = TAA[3] = TAA[4] = TAA[6] = 0;
	    TAA[5] = 1;				
	}
	
//comparison: same as identity
	static inline bool IdentitySameAsTrans(const double T[])
	{
	    return 
		fabs(T[0]) <= Constants::EPSILON &&
		fabs(T[1]) <= Constants::EPSILON &&
		fabs(T[2]) <= Constants::EPSILON;
	}
	static inline bool IdentitySameAsQuat(const double Q[])
	{
	    return 
		fabs(Q[0]) <= Constants::EPSILON &&
		fabs(Q[1]) <= Constants::EPSILON &&
		fabs(Q[2]) <= Constants::EPSILON &&
		fabs(Q[3] - 1) <= Constants::EPSILON;
	}
	static inline bool IdentitySameAsRot(const double R[])
	{
	    return
		fabs(R[0] - 1) <= Constants::EPSILON &&
		fabs(R[4] - 1) <= Constants::EPSILON &&
		fabs(R[8] - 1) <= Constants::EPSILON &&
		fabs(R[1]) <= Constants::EPSILON &&
		fabs(R[2]) <= Constants::EPSILON &&
		fabs(R[3]) <= Constants::EPSILON &&
		fabs(R[5]) <= Constants::EPSILON &&
		fabs(R[6]) <= Constants::EPSILON &&
		fabs(R[7]) <= Constants::EPSILON;
	}
	static inline bool IdentitySameAsTransQuat(const double TQ[])
	{
	    return
		IdentitySameAsTrans(TQ) &&
		IdentitySameAsQuat(&(TQ[3]));
	}
	static inline bool IdentitySameAsTransRot(const double TR[])
	{
	    return
		IdentitySameAsTrans(TR) &&
		IdentitySameAsRot(&(TR[3]));
	}
	
//conversion	    
	static inline void TransAsTrans(const double x, const double y, const double z,
					double Tresult[])
	{
	    Tresult[0] = x;
	    Tresult[1] = y;
	    Tresult[2] = z;
	}
	
	static inline void TransAsTrans(const double T[], double Tresult[])
	{
	    Tresult[0] = T[0];
	    Tresult[1] = T[1];
	    Tresult[2] = T[2];
	}
	static inline void QuatAsQuat(const double Q[], double Qresult[])
	{
	    Qresult[0] = Q[0];
	    Qresult[1] = Q[1];
	    Qresult[2] = Q[2];
	    Qresult[3] = Q[3];
	}
	static inline void QuatAsRot(const double Q[], double R[])
	{
	    R[0] =  1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2]);
	    R[1] =      2 * (Q[0] * Q[1] - Q[2] * Q[3]);
	    R[2] =      2 * (Q[0] * Q[2] + Q[1] * Q[3]);
	    R[3] =      2 * (Q[0] * Q[1] + Q[2] * Q[3]);
	    R[4] =  1 - 2 * (Q[0] * Q[0] + Q[2] * Q[2]);
	    R[5] =      2 * (Q[1] * Q[2] - Q[0] * Q[3]);
	    R[6] =      2 * (Q[0] * Q[2] - Q[1] * Q[3]);
	    R[7] =      2 * (Q[1] * Q[2] + Q[0] * Q[3]);
	    R[8] =  1 - 2 * (Q[0] * Q[0] + Q[1] * Q[1]);	
	}
	static inline void QuatAsAxisAngle(const double Q[], double AA[])
	{
	    AA[0] = Q[0];
	    AA[1] = Q[1];
	    AA[2] = Q[2];
	    AA[3] = 2 * acos(Q[3]);
	}
	static inline void QuatAsUnitAxisAngle(const double Q[], double AA[])
	{
	    double d = VecDotProduct(Q, Q);
	    if(d <= Constants::EPSILON_SQUARED)
	    {
		AA[0] = AA[1] = AA[3] = 0;
		AA[2] = 1;
	    }
	    else
	    {
		d = 1 / sqrt(d);
		AA[0] = d * Q[0];
		AA[1] = d * Q[1];
		AA[2] = d * Q[2];
		AA[3] = 2 * acos(Q[3]);
	    }
	}
	void RotAsQuat(const double R[], double Q[]);
	static inline void RotAsRot(const double R[], double Rresult[])
	{
	    memcpy(Rresult, R, Rot_NR_ENTRIES * sizeof(double));
	}
	static inline void RotAsAxisAngle(const double R[], double AA[])
	{
	    RotAsQuat(R, AA);
	    AA[3] = 2 * acos(AA[3]);
	}
	static inline void RotAsUnitAxisAngle(const double R[], double AA[])
	{
	    RotAsQuat(R, AA);
	    QuatAsUnitAxisAngle(AA, AA);
	}
	static inline void UnitAxisAngleAsQuat(const double AA[], double Q[])
	{
	    const double s = sin(0.5 * AA[3]);
	    Q[0] = s * AA[0];
	    Q[1] = s * AA[1];
	    Q[2] = s * AA[2];
	    Q[3] = cos(0.5 * AA[3]);
	}
	static inline void AxisAngleAsQuat(const double AA[], double Q[])
	{
	    double d = sqrt(AA[0] * AA[0] + AA[1] * AA[1] + AA[2] * AA[2]);
	    if(d == 0)
		IdentityAsQuat(Q);
	    else
	    {
		d    = sin(0.5 * AA[3]) / d;	
		Q[0] = d * AA[0];
		Q[1] = d * AA[1];
		Q[2] = d * AA[2];
		Q[3] = cos(0.5 * AA[3]);
	    }
	}
	static inline void UnitAxisAngleAsRot(const double AA[], double R[])
	{
	    const double c = cos(AA[3]);
	    const double s = sin(AA[3]);
	    
	    R[0] = AA[0]*AA[0]*(1-c) + c;
	    R[1] = AA[0]*AA[1]*(1-c) - AA[2]*s;
	    R[2] = AA[0]*AA[2]*(1-c) + AA[1]*s;
	    R[3] = AA[0]*AA[1]*(1-c) + AA[2]*s;
	    R[4] = AA[1]*AA[1]*(1-c) + c;
	    R[5] = AA[1]*AA[2]*(1-c) - AA[0]*s;
	    R[6] = AA[0]*AA[2]*(1-c) - AA[1]*s;
	    R[7] = AA[1]*AA[2]*(1-c) + AA[0]*s;
	    R[8] = AA[2]*AA[2]*(1-c) + c;
	}
	static inline void XAxisAngleAsRot(const double theta, double R[])
	{
	    R[0] = 1;
	    R[1] = R[2] = R[3] = R[6] = 0;
	    R[4] = R[8] = cos(theta);
	    R[7] = sin(theta);
	    R[5] = -R[7];	    
	}
	static inline void YAxisAngleAsRot(const double theta, double R[])
	{
	    R[4] = 1;
	    R[1] = R[3] = R[5] = R[7] = 0;
	    R[0] = R[8] = cos(theta);
	    R[2] = sin(theta);
	    R[6] = -R[2];	    
	}
	static inline void ZAxisAngleAsRot(const double theta, double R[])
	{
	    R[8] = 1;
	    R[2] = R[5] = R[6] = R[7] = 0;
	    R[0] = R[4] = cos(theta);
	    R[3] = sin(theta);
	    R[1] = -R[3];	    
	}

	
	static inline void AxisAngleAsRot(const double AA[],  double R[])
	{
	    double d = VecNorm(AA);
	    if(d == 0)
		IdentityAsRot(R);
	    else
	    {
		d = 1/d;
		const double v[4]= {d * AA[0], d * AA[1], d * AA[2], AA[3]};
		UnitAxisAngleAsRot(v, R);
	    }
	}
	static inline void AxisAngleAsAxisAngle(const double AA[], double AAresult[])
	{
	    memcpy(AAresult, AA, AxisAngle_NR_ENTRIES * sizeof(double));
	}					
	static inline void AxisAngleAsUnitAxisAngle(const double AA[], double AAresult[])
	{
	    VecUnit(AA, AAresult);
	    AAresult[3] = AA[3];
	}
	void FromToAxisAsQuat(const double Afrom[], const double Ato[], const double normProd,
			      double Q[]);
	static inline void FromToUnitAxisAsQuat(const double Afrom[], const double Ato[], double Q[])
	{
	    FromToAxisAsQuat(Afrom, Ato, 1, Q);
	}
	static inline void FromToAxisAsQuat(const double Afrom[], const double Ato[], double Q[])
	{
	    FromToAxisAsQuat(Afrom, Ato, 
			     sqrt(VecDotProduct(Afrom, Afrom) * VecDotProduct(Ato, Ato)), Q);
	}
	void FromToAxisAsRot(const double Afrom[], const double Ato[], const double normProd, 
			     double R[]);
	static inline void FromToUnitAxisAsRot(const double Afrom[], const double Ato[], double R[])
	{
	    FromToAxisAsRot(Afrom, Ato, 1, R);
	}
	static inline void FromToAxisAsRot(const double Afrom[], const double Ato[], double R[])
	{
	    FromToAxisAsRot(Afrom, Ato, 
			    sqrt(VecDotProduct(Afrom, Afrom) * VecDotProduct(Ato, Ato)), R);
	}
	void FromToAxisAsAxisAngle(const double Afrom[], const double Ato[], const double normProd,
				   double AA[]);
	static inline void FromToUnitAxisAsAxisAngle(const double Afrom[], const double Ato[], double AA[])
	{
	    FromToAxisAsAxisAngle(Afrom, Ato, 1, AA);
	}
	static inline void FromToAxisAsAxisAngle(const double Afrom[], const double Ato[], double AA[])
	{
	    FromToAxisAsAxisAngle(Afrom, Ato, 
				  sqrt(VecDotProduct(Afrom, Afrom) * VecDotProduct(Ato, Ato)), AA);
	}
	static inline void TransQuatAsTransQuat(const double TQ[], double TQresult[])
	{
	    memcpy(TQresult, TQ, TransQuat_NR_ENTRIES * sizeof(double));
	}
	static inline void TransQuatAsTransRot(const double TQ[], double TR[])
	{
	    TransAsTrans(TQ, TR);
	    QuatAsRot(&(TQ[3]), &(TR[3]));
	}
	static inline void TransQuatAsTransAxisAngle(const double TQ[], double TAA[])
	{
	    TransAsTrans(TQ, TAA);
	    QuatAsAxisAngle(&(TQ[3]), &(TAA[3]));
	}
	static inline void TransQuatAsTransUnitAxisAngle(const double TQ[], double TAA[])
	{
	    TransAsTrans(TQ, TAA);
	    QuatAsUnitAxisAngle(&(TQ[3]), &(TAA[3]));
	}
	static inline void TransRotAsTransQuat(const double TR[], double TQ[])
	{
	    TransAsTrans(TR, TQ);
	    RotAsQuat(&(TR[3]), &(TQ[3]));
	}
	static inline void TransRotAsTransRot(const double TR[], double TRresult[])
	{
	    memcpy(TRresult, TR, TransRot_NR_ENTRIES * sizeof(double));
	}
	static inline void TransRotAsTransAxisAngle(const double TR[], double TAA[])
	{
	    TransAsTrans(TR, TAA);
	    RotAsAxisAngle(&(TR[3]), &(TAA[3]));
	}
	static inline void TransRotAsTransUnitAxisAngle(const double TR[], double TAA[])
	{
	    TransAsTrans(TR, TAA);
	    RotAsUnitAxisAngle(&(TR[3]), &(TAA[3]));
	}
	static inline void TransAxisAngleAsTransQuat(const double TAA[], double TQ[])
	{
	    TransAsTrans(TAA, TQ);
	    AxisAngleAsQuat(&(TAA[3]), &(TQ[3]));
	}
	static inline void TransAxisAngleAsTransRot(const double TAA[], double TR[])
	{
	    TransAsTrans(TAA, TR);
	    AxisAngleAsRot(&(TAA[3]), &(TR[3]));
	}
	static inline void TransAxisAngleAsTransAxisAngle(const double TAA[], double TAAresult[])
	{
	    memcpy(TAAresult, TAA, TransAxisAngle_NR_ENTRIES * sizeof(double));
	}
	static inline void TransAxisAngleAsTransUnitAxisAngle(const double TAA[], double TAAresult[])
	{
	    TransAsTrans(TAA, TAAresult);
	    AxisAngleAsUnitAxisAngle(&(TAA[3]), &(TAAresult[3]));
	}
	static inline void TransUnitAxisAngleAsTransQuat(const double TAA[], double TQ[])
	{
	    TransAsTrans(TAA, TQ);
	    UnitAxisAngleAsQuat(&(TAA[3]), &(TQ[3]));
	}
	static inline void TransUnitAxisAngleAsTransRot(const double TAA[], double TR[])
	{
	    TransAsTrans(TAA, TR);
	    UnitAxisAngleAsRot(&(TAA[3]), &(TR[3]));
	}
	static inline void TransUnitAxisAngleAsTransAxisAngle(const double TAA[], double TAAresult[])
	{
	    TransAxisAngleAsTransAxisAngle(TAA, TAAresult);
	}
	static inline void TransUnitAxisAngleAsTransUnitAxisAngle(const double TAA[], double TAAresult[])
	{
	    TransAxisAngleAsTransAxisAngle(TAA, TAAresult);
	}
	
	
//comparison with each other
	static inline bool TransSameAsTrans(const double Ta[], const double Tb[])
	{
	    return 
		fabs(Ta[0] - Tb[0]) <= Constants::EPSILON &&
		fabs(Ta[1] - Tb[1]) <= Constants::EPSILON &&
		fabs(Ta[2] - Tb[2]) <= Constants::EPSILON;
	}
	static inline bool QuatSameAsQuat(const double Qa[], const double Qb[])
	{
	    return
		(fabs(Qa[0] - Qb[0]) <= Constants::EPSILON &&
		 fabs(Qa[1] - Qb[1]) <= Constants::EPSILON &&
		 fabs(Qa[2] - Qb[2]) <= Constants::EPSILON &&
		 fabs(Qa[3] - Qb[3]) <= Constants::EPSILON)||
		(fabs(Qa[0] + Qb[0]) <= Constants::EPSILON &&
		 fabs(Qa[1] + Qb[1]) <= Constants::EPSILON &&
		 fabs(Qa[2] + Qb[2]) <= Constants::EPSILON &&
		 fabs(Qa[3] + Qb[3]) <= Constants::EPSILON);
	}
	static inline bool QuatSameAsRot(const double Q[], const double R[])
	{	
	    return 
		fabs(R[0] - 1 + 2 * (Q[1] * Q[1] + Q[2] * Q[2])) <= Constants::EPSILON &&
		fabs(R[1]     - 2 * (Q[0] * Q[1] - Q[2] * Q[3])) <= Constants::EPSILON &&
		fabs(R[2]     - 2 * (Q[0] * Q[2] + Q[1] * Q[3])) <= Constants::EPSILON &&
		fabs(R[3]     - 2 * (Q[0] * Q[1] + Q[2] * Q[3])) <= Constants::EPSILON &&
		fabs(R[4] - 1 + 2 * (Q[0] * Q[0] + Q[2] * Q[2])) <= Constants::EPSILON &&
		fabs(R[5]     - 2 * (Q[1] * Q[2] - Q[0] * Q[3])) <= Constants::EPSILON &&
		fabs(R[6]     - 2 * (Q[0] * Q[2] - Q[1] * Q[3])) <= Constants::EPSILON &&
		fabs(R[7]     - 2 * (Q[1] * Q[2] + Q[0] * Q[3])) <= Constants::EPSILON &&
		fabs(R[8] - 1 + 2 * (Q[0] * Q[0] + Q[1] * Q[1])) <= Constants::EPSILON;		
	}
	static inline bool RotSameAsQuat(const double R[], const double Q[])
	{
	    return QuatSameAsRot(Q, R);
	}
	static inline bool RotSameAsRot(const double Ra[], const double Rb[])
	{
	    return 
		fabs(Ra[0] - Rb[0]) <= Constants::EPSILON &&
		fabs(Ra[1] - Rb[1]) <= Constants::EPSILON &&
		fabs(Ra[2] - Rb[2]) <= Constants::EPSILON &&
		fabs(Ra[3] - Rb[3]) <= Constants::EPSILON &&
		fabs(Ra[4] - Rb[4]) <= Constants::EPSILON &&
		fabs(Ra[5] - Rb[5]) <= Constants::EPSILON &&
		fabs(Ra[6] - Rb[6]) <= Constants::EPSILON &&
		fabs(Ra[7] - Rb[7]) <= Constants::EPSILON &&
		fabs(Ra[8] - Rb[8]) <= Constants::EPSILON;
	}
	static inline bool TransQuatSameAsTransQuat(const double TQa[], const double TQb[])
	{
	    return 
		TransSameAsTrans(TQa, TQb) &&
		QuatSameAsQuat(&(TQa[3]), &(TQb[3]));
	}
	static inline bool TransQuatSameAsTransRot(const double TQ[], const double TR[])
	{
	    return 
		TransSameAsTrans(TQ, TR) &&
		QuatSameAsRot(&(TQ[3]), &(TR[3]));
	}
	static inline bool TransRotSameAsTransQuat(const double TR[], const double TQ[])
	{
	    return TransQuatSameAsTransRot(TQ, TR);
	}
	static inline bool TransRotSameAsTransRot(const double TRa[], const double TRb[])
	{
	    return 
		TransSameAsTrans(TRa, TRb) &&
		RotSameAsRot(&(TRa[3]), &(TRb[3]));
	}
	
//transform point
	static inline void TransMultPoint(const double T[], const double p[], double pnew[])
	{
	    pnew[0] = p[0] + T[0];
	    pnew[1] = p[1] + T[1];
	    pnew[2] = p[2] + T[2];
	}
	static inline void TransMultInvPoint(const double T[], const double p[], double pnew[])
	{
	    pnew[0] = -p[0] + T[0];
	    pnew[1] = -p[1] + T[1];
	    pnew[2] = -p[2] + T[2];
	}
	static inline void InvTransMultPoint(const double T[], const double p[], double pnew[])
	{
	    pnew[0] = p[0] - T[0];
	    pnew[1] = p[1] - T[1];
	    pnew[2] = p[2] - T[2];
	}
	static inline void InvTransMultInvPoint(const double T[], const double p[], double pnew[])
	{
	    pnew[0] = -p[0] - T[0];
	    pnew[1] = -p[1] - T[1];
	    pnew[2] = -p[2] - T[2];
	}
	static inline void QuatMultPoint(const double Q[], const double p[], double pnew[])
	{
	    const double x = p[0];
	    const double y = p[1];
	    
	    pnew[0] = 
		x    * (1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2])) +
		y    * (    2 * (Q[0] * Q[1] - Q[2] * Q[3])) +
		p[2] * (    2 * (Q[0] * Q[2] + Q[1] * Q[3]));
	    pnew[1] = 
		x    * (    2 * (Q[0] * Q[1] + Q[2] * Q[3])) +
		y    * (1 - 2 * (Q[0] * Q[0] + Q[2] * Q[2])) +
		p[2] * (    2 * (Q[1] * Q[2] - Q[0] * Q[3]));
	    pnew[2] = 
		x    * (    2 * (Q[0] * Q[2] - Q[1] * Q[3])) +
		y    * (    2 * (Q[1] * Q[2] + Q[0] * Q[3])) +
		p[2] * (1 - 2 * (Q[0] * Q[0] + Q[1] * Q[1]));	
	}
	static inline void QuatMultInvPoint(const double Q[], const double p[], double pnew[])
	{
	    double pinv[3] = {-p[0], -p[1], -p[2]};
	    QuatMultPoint(Q, pinv, pnew);
	}
	static inline void InvQuatMultPoint(const double Q[], const double p[], double pnew[])
	{
	    const double x = p[0];
	    const double y = p[1];
	    
	    pnew[0] = 
		x    * (1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2])) +
		y    * (    2 * (Q[0] * Q[1] + Q[2] * Q[3])) +
		p[2] * (    2 * (Q[0] * Q[2] - Q[1] * Q[3]));
	    
	    pnew[1] = 
		x    * (    2 * (Q[0] * Q[1] - Q[2] * Q[3])) +
		y    * (1 - 2 * (Q[0] * Q[0] + Q[2] * Q[2])) +
		p[2] * (    2 * (Q[1] * Q[2] + Q[0] * Q[3]));
	    
	    
	    pnew[2] = 
		x    * (    2 * (Q[0] * Q[2] + Q[1] * Q[3])) +
		y    * (    2 * (Q[1] * Q[2] - Q[0] * Q[3])) +
		p[2] * (1 - 2 * (Q[0] * Q[0] + Q[1] * Q[1]));	
	}
	static inline void InvQuatMultInvPoint(const double Q[], const double p[], double pnew[])
	{
	    double pinv[3] = {-p[0], -p[1], -p[2]};
	    InvQuatMultPoint(Q, pinv, pnew);
	}
	static inline void QuatMultXAxis(const double Q[], double axis[])
	{
	    axis[0] = (1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2]));
	    axis[1] = (    2 * (Q[0] * Q[1] + Q[2] * Q[3]));
	    axis[2] = (    2 * (Q[0] * Q[2] - Q[1] * Q[3]));	
	}	
	static inline void QuatMultNegXAxis(const double Q[], double axis[])
	{
	    axis[0] = -(1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2]));
	    axis[1] = -(    2 * (Q[0] * Q[1] + Q[2] * Q[3]));
	    axis[2] = -(    2 * (Q[0] * Q[2] - Q[1] * Q[3]));
	}
	static inline void InvQuatMultXAxis(const double Q[], double axis[])
	{
	    axis[0] = (1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2]));
	    axis[1] = (    2 * (Q[0] * Q[1] - Q[2] * Q[3]));
	    axis[2] = (    2 * (Q[0] * Q[2] + Q[1] * Q[3]));
	}
	
	static inline void QuatMultYAxis(const double Q[], double ynew[])
	{
	    
	    ynew[0] = (    2 * (Q[0] * Q[1] - Q[2] * Q[3]));
	    ynew[1] = (1 - 2 * (Q[0] * Q[0] + Q[2] * Q[2]));
	    ynew[2] = (    2 * (Q[1] * Q[2] + Q[0] * Q[3]));
	}
	static inline void QuatMultZAxis(const double Q[], double znew[])
	{
	    znew[0] = (    2 * (Q[0] * Q[2] + Q[1] * Q[3]));
	    znew[1] = (    2 * (Q[1] * Q[2] - Q[0] * Q[3]));
	    znew[2] = (1 - 2 * (Q[0] * Q[0] + Q[1] * Q[1]));	
	}	
	
	static inline void RotMultPoint(const double R[], const double p[], double pnew[])
	{
	    const double x = p[0];
	    const double y = p[1];
	    
	    pnew[0] = R[0] * x + R[1] * y + R[2] * p[2];
	    pnew[1] = R[3] * x + R[4] * y + R[5] * p[2];
	    pnew[2] = R[6] * x + R[7] * y + R[8] * p[2];
	}
	static inline void RotMultInvPoint(const double R[], const double p[], double pnew[])
	{
	    const double x = -p[0];
	    const double y = -p[1];
	    
	    pnew[0] = R[0] * x + R[1] * y - R[2] * p[2];
	    pnew[1] = R[3] * x + R[4] * y - R[5] * p[2];
	    pnew[2] = R[6] * x + R[7] * y - R[8] * p[2];
	}
	static inline void InvRotMultPoint(const double R[], const double p[], double pnew[])
	{
	    const double x = p[0];
	    const double y = p[1];
	    
	    pnew[0] = R[0] * x + R[3] * y + R[6] * p[2];
	    pnew[1] = R[1] * x + R[4] * y + R[7] * p[2];
	    pnew[2] = R[2] * x + R[5] * y + R[8] * p[2];
	}
	static inline void InvRotMultInvPoint(const double R[], const double p[], double pnew[])
	{
	    const double x = -p[0];
	    const double y = -p[1];
	    
	    pnew[0] = R[0] * x + R[3] * y - R[6] * p[2];
	    pnew[1] = R[1] * x + R[4] * y - R[7] * p[2];
	    pnew[2] = R[2] * x + R[5] * y - R[8] * p[2];
	}
	static inline void RotMultXAxis(const double R[], double xnew[])
	{
	    xnew[0] = R[0];
	    xnew[1] = R[3];
	    xnew[2] = R[6];
	}	
	static inline void RotMultYAxis(const double R[], double ynew[])
	{
	    ynew[0] = R[1];
	    ynew[1] = R[4];
	    ynew[2] = R[7];
	}
	static inline void RotMultZAxis(const double R[], double znew[])
	{
	    znew[0] = R[2];
	    znew[1] = R[5];
	    znew[2] = R[8];
	}	
	
	static inline void TransQuatMultPoint(const double TQ[], const double p[], double pnew[])
	{
	    QuatMultPoint(&(TQ[3]), p, pnew);
	    TransMultPoint(TQ, pnew, pnew);
	}
	static inline void TransQuatMultInvPoint(const double TQ[], const double p[], double pnew[])
	{
	    QuatMultInvPoint(&(TQ[3]), p, pnew);
	    TransMultPoint(TQ, pnew, pnew);
	}
	static inline void InvTransQuatMultPoint(const double TQ[], const double p[], double pnew[])
	{
	    InvTransMultPoint(TQ, p, pnew);
	    InvQuatMultPoint(&(TQ[3]), pnew, pnew);
	}
	static inline void InvTransQuatMultInvPoint(const double TQ[], const double p[], double pnew[])
	{
	    InvTransMultInvPoint(TQ, p, pnew);
	    InvQuatMultPoint(&(TQ[3]), pnew, pnew);
	}
	static inline void TransRotMultPoint(const double TR[], const double p[], double pnew[])
	{
	    RotMultPoint(&(TR[3]), p, pnew);
	    TransMultPoint(TR, pnew, pnew);
	}
	static inline void TransRotMultInvPoint(const double TR[], const double p[], double pnew[])
	{
	    RotMultInvPoint(&(TR[3]), p, pnew);
	    TransMultPoint(TR, pnew, pnew);
	}
	static inline void InvTransRotMultPoint(const double TR[], const double p[], double pnew[])
	{
	    InvTransMultPoint(TR, p, pnew);
	    InvRotMultPoint(&(TR[3]), pnew, pnew);
	}
	static inline void InvTransRotMultInvPoint(const double TR[], const double p[], 
						   double pnew[])
	{
	    InvTransMultInvPoint(TR, p, pnew);
	    InvRotMultPoint(&(TR[3]), pnew, pnew);
	}
	
//inverse
	static inline void InvTransAsTrans(const double T[], double Tresult[])
	{
	    Tresult[0] = -T[0];
	    Tresult[1] = -T[1];
	    Tresult[2] = -T[2];
	}
	static inline void InvQuatAsQuat(const double Q[], double Qresult[])
	{
	    Qresult[0] = -Q[0];
	    Qresult[1] = -Q[1];
	    Qresult[2] = -Q[2];
	    Qresult[3] = Q[3];
	}
	static inline void InvQuatAsRot(const double Q[], double R[])
	{
	    R[0] =  1 - 2 * (Q[1] * Q[1] + Q[2] * Q[2]);
	    R[1] =      2 * (Q[0] * Q[1] + Q[2] * Q[3]);
	    R[2] =      2 * (Q[0] * Q[2] - Q[1] * Q[3]);
	    R[3] =      2 * (Q[0] * Q[1] - Q[2] * Q[3]);
	    R[4] =  1 - 2 * (Q[0] * Q[0] + Q[2] * Q[2]);
	    R[5] =      2 * (Q[1] * Q[2] + Q[0] * Q[3]);
	    R[6] =      2 * (Q[0] * Q[2] + Q[1] * Q[3]);
	    R[7] =      2 * (Q[1] * Q[2] - Q[0] * Q[3]);
	    R[8] =  1 - 2 * (Q[0] * Q[0] + Q[1] * Q[1]);	
	}
	static inline void InvRotAsQuat(const double R[], double Q[])
	{
	    RotAsQuat(R, Q);
	    InvQuatAsQuat(Q, Q);
	}
	static inline void InvRotAsRot(const double R[], double Rresult[])
	{
	    double x;
	    
	    Rresult[0] = R[0];
	    Rresult[4] = R[4];
	    Rresult[8] = R[8];
	    x = R[1]; Rresult[1] = R[3]; Rresult[3] = x;
	    x = R[2]; Rresult[2] = R[6]; Rresult[6] = x;
	    x = R[5]; Rresult[5] = R[7]; Rresult[7] = x;
	}
	static inline void InvTransQuatAsTransQuat(const double TQ[], double TQresult[])
	{
	    InvQuatAsQuat(&(TQ[3]), &(TQresult[3]));
	    QuatMultInvPoint(&(TQresult[3]), TQ, TQresult);
	}
	static inline void InvTransQuatAsTransRot(const double TQ[], double TR[])
	{
	    InvQuatAsRot(&(TQ[3]), &(TR[3]));
	    RotMultInvPoint(&(TR[3]), TQ, TR);
	}
	static inline void InvTransRotAsTransQuat(const double TR[], double TQ[])
	{
	    InvRotAsQuat(&(TR[3]), &(TQ[3]));
	    InvRotMultInvPoint(&(TR[3]), TR, TQ);
	}
	static inline void InvTransRotAsTransRot(const double TR[], double TRresult[])
	{
	    InvRotAsRot(&(TR[3]), &(TRresult[3]));
	    RotMultInvPoint(&(TRresult[3]), TR, TRresult);
	}
	
//combine
	static inline void TransMultTransAsTrans(const double Ta[], const double Tb[], double T[])
	{
	    T[0] = Ta[0] + Tb[0];
	    T[1] = Ta[1] + Tb[1];
	    T[2] = Ta[2] + Tb[2];
	}
	static inline void TransMultQuatAsTransQuat(const double T[], const double Q[], double TQ[])
	{
	    TransAsTrans(T, TQ);
	    QuatAsQuat(Q, &(TQ[3]));
	}
	static inline void TransMultQuatAsTransRot(const double T[], const double Q[], double TR[])
	{
	    TransAsTrans(T, TR);
	    QuatAsRot(Q, &(TR[3]));
	}
	static inline void TransMultRotAsTransQuat(const double T[], const double R[], double TQ[])
	{
	    TransAsTrans(T, TQ);
	    RotAsQuat(R, &(TQ[3]));
	}
	static inline void TransMultRotAsTransRot(const double T[], const double R[], double TR[])
	{
	    TransAsTrans(T, TR);
	    RotAsRot(R, &(TR[3]));
	}
	static inline void TransMultTransQuatAsTransQuat(const double T[], const double TQ[], 
							 double TQresult[])
	{
	    TransMultTransAsTrans(T, TQ, TQresult);
	    if(TQ != TQresult)
		QuatAsQuat(&(TQ[3]), &(TQresult[3]));
	}
	static inline void TransMultTransQuatAsTransRot(const double T[], const double TQ[], 
							double TR[])
	{
	    TransMultTransAsTrans(T, TQ, TR);
	    QuatAsRot(&(TQ[3]), &(TR[3]));
	}
	static inline void TransMultTransRotAsTransQuat(const double T[], const double TR[], 
							double TQ[])
	{
	    TransMultTransAsTrans(T, TR, TQ);
	    RotAsQuat(&(TR[3]), &(TQ[3]));
	}
	static inline void TransMultTransRotAsTransRot(const double T[], const double TR[], 
						       double TRresult[])
	{
	    TransMultTransAsTrans(T, TR, TRresult);
	    RotAsRot(&(TR[3]), &(TRresult[3]));
	}
	static inline void QuatMultTransAsTransQuat(const double Q[], const double T[], double TQ[])
	{
	    QuatMultPoint(Q, T, TQ);
	    if(Q != &(TQ[3]))
		QuatAsQuat(Q, &(TQ[3]));
	}
	static inline void QuatMultTransAsTransRot(const double Q[], const double T[], double TR[])
	{
	    QuatAsRot(Q, &(TR[3]));
	    RotMultPoint(&(TR[3]), T, TR);
	}
	static inline void QuatMultQuatAsQuat(const double Qa[], const double Qb[], double Q[])
	{
	    const double x =  Qa[0] * Qb[3] + Qa[1] * Qb[2] - Qa[2] * Qb[1] + Qa[3] * Qb[0];
	    const double y = -Qa[0] * Qb[2] + Qa[1] * Qb[3] + Qa[2] * Qb[0] + Qa[3] * Qb[1];
	    const double z =  Qa[0] * Qb[1] - Qa[1] * Qb[0] + Qa[2] * Qb[3] + Qa[3] * Qb[2];
	    
	    Q[3] = -Qa[0] * Qb[0] - Qa[1] * Qb[1] - Qa[2] * Qb[2] + Qa[3] * Qb[3];
	    Q[0] = x;	    
	    Q[1] = y;
	    Q[2] = z;
	}
	static inline void QuatMultQuatAsRot(const double Qa[], const double Qb[], double R[])
	{
	    double Q[Quat_NR_ENTRIES];
	    QuatMultQuatAsQuat(Qa, Qb, Q);
	    QuatAsRot(Q, R);
	}
	static inline void QuatMultRotAsQuat(const double Q[], const double R[], double Qresult[])
	{
	    if(Q != Qresult)
	    {
		RotAsQuat(R, Qresult);
		QuatMultQuatAsQuat(Q, Qresult, Qresult);
	    }
	    else
	    {
		double Qtmp[Quat_NR_ENTRIES];
		RotAsQuat(R, Qtmp);
		QuatMultQuatAsQuat(Q, Qtmp, Qresult);
	    }
	}
	void RotMultRotAsRot(const double Ra[], const double Rb[], double R[]);
	static inline void QuatMultRotAsRot(const double Q[], const double R[], double Rresult[])
	{
	    if(R != Rresult)
	    {
		QuatAsRot(Q, Rresult);
		RotMultRotAsRot(Rresult, R, Rresult);
	    }
	    else
	    {
		double Rtmp[Rot_NR_ENTRIES];
		QuatAsRot(Q, Rtmp);
		RotMultRotAsRot(Rtmp, R, Rresult);
	    }
	}
	static inline void QuatMultTransQuatAsTransQuat(const double Q[], const double TQ[], 
							double TQresult[])
	{
	    QuatMultPoint(Q, TQ, TQresult);
	    QuatMultQuatAsQuat(Q, &(TQ[3]), &(TQresult[3]));
	}
	static inline void QuatMultTransQuatAsTransRot(const double Q[], const double TQ[], 
						       double TR[])
	{
	    QuatMultPoint(Q, TQ, TR);
	    QuatMultQuatAsRot(Q, &(TQ[3]), &(TR[3]));
	}
	static inline void QuatMultTransRotAsTransQuat(const double Q[], const double TR[],
						       double TQ[])
	{
	    QuatMultPoint(Q, TR, TQ);
	    QuatMultRotAsQuat(Q, &(TR[3]), &(TQ[3]));
	}
	static inline void QuatMultTransRotAsTransRot(const double Q[], const double TR[],
						      double TRresult[])
	{
	    QuatMultPoint(Q, TR, TRresult);
	    QuatMultRotAsRot(Q, &(TR[3]), &(TRresult[3]));
	}
	static inline void RotMultTransAsTransQuat(const double R[], const double T[], double TQ[])
	{
	    RotMultPoint(R, T, TQ);
	    RotAsQuat(R, &(TQ[3]));
	}
	static inline void RotMultTransAsTransRot(const double R[], const double T[], double TR[])
	{
	    RotMultPoint(R, T, TR);
	    if(R != &(TR[3]))
		RotAsRot(R, &(TR[3]));
	}
	static inline void RotMultQuatAsQuat(const double R[], const double Q[], double Qresult[])
	{
	    if(Q != Qresult)
	    {
		RotAsQuat(R, Qresult);
		QuatMultQuatAsQuat(Qresult, Q, Qresult);
	    }
	    else
	    {
		double Qtmp[Quat_NR_ENTRIES];
		RotAsQuat(R, Qtmp);
		QuatMultQuatAsQuat(Qtmp, Q, Qresult);
	    }
	}
	static inline void RotMultQuatAsRot(const double R[], const double Q[], double Rresult[])
	{
	    if(R != Rresult)
	    {
		QuatAsRot(Q, Rresult);
		RotMultRotAsRot(R, Rresult, Rresult);
	    }
	    else
	    {
		double Rtmp[Rot_NR_ENTRIES];
		QuatAsRot(Q, Rtmp);
		RotMultRotAsRot(R, Rtmp, Rresult);
	    }
	}
	static inline void RotMultRotAsQuat(const double Ra[], const double Rb[], double Q[])
	{
	    double R[Rot_NR_ENTRIES];
	    RotMultRotAsRot(Ra, Rb, R);
	    RotAsQuat(R, Q);
	}
	static inline void RotMultTransQuatAsTransQuat(const double R[], const double TQ[],
						       double TQresult[])
	{
	    RotMultPoint(R, TQ, TQresult);
	    RotMultQuatAsQuat(R, &(TQ[3]), &(TQresult[3]));
	}
	static inline void RotMultTransQuatAsTransRot(const double R[], const double TQ[],
						      double TRresult[])
	{
	    RotMultPoint(R, TQ, TRresult);
	    RotMultQuatAsRot(R, &(TQ[3]), &(TRresult[3]));
	}
	static inline void RotMultTransRotAsTransQuat(const double R[], const double TR[],
						      double TQ[])
	{
	    RotMultPoint(R, TR, TQ);
	    RotMultRotAsQuat(R, &(TR[3]), &(TQ[3]));
	}
	static inline void RotMultTransRotAsTransRot(const double R[], const double TR[],
						     double TRresult[])
	{
	    RotMultPoint(R, TR, TRresult);
	    RotMultRotAsRot(R, &(TR[3]), &(TRresult[3]));
	}
	
	static inline void TransQuatMultTransAsTransQuat(const double TQ[], const double T[],
							 double TQresult[])
	{
	    double p[3];
	    QuatMultPoint(&(TQ[3]), T, p);
	    TransMultTransAsTrans(TQ, p, TQresult);
	    if(TQ != TQresult)
		QuatAsQuat(&(TQ[3]), &(TQresult[3]));
	}
	static inline void TransQuatMultTransAsTransRot(const double TQ[], const double T[],
							double TR[])
	{
	    double p[3];
	    QuatMultPoint(&(TQ[3]), T, p);
	    TransMultTransAsTrans(TQ, p, TR);
	    QuatAsRot(&(TQ[3]), &(TR[3]));
	}
	static inline void TransQuatMultQuatAsTransQuat(const double TQ[], const double Q[],
							double TQresult[])
	{
	    QuatMultQuatAsQuat(&(TQ[3]), Q, &(TQresult[3]));
	    TransAsTrans(TQ, TQresult);
	}
	static inline void TransQuatMultQuatAsTransRot(const double TQ[], const double Q[],
						       double TR[])
	{
	    QuatMultQuatAsRot(&(TQ[3]), Q, &(TR[3]));
	    TransAsTrans(TQ, TR);
	}
	static inline void TransQuatMultRotAsTransQuat(const double TQ[], const double R[],
						       double TQresult[])
	{
	    QuatMultRotAsQuat(&(TQ[3]), R, &(TQresult[3]));
	    TransAsTrans(TQ, TQresult);
	}
	static inline void TransQuatMultRotAsTransRot(const double TQ[], const double R[],
						      double TR[])
	{
	    QuatMultRotAsRot(&(TQ[3]), R, &(TR[3]));
	    TransAsTrans(TQ, TR);
	}
	static inline void TransQuatMultTransQuatAsTransQuat(const double TQa[], const double TQb[],
							     double TQ[])
	{
	    double p[3];		
	    QuatMultPoint(&(TQa[3]), TQb, p);
	    TransMultTransAsTrans(TQa, p, TQ);
	    QuatMultQuatAsQuat(&(TQa[3]), &(TQb[3]), &(TQ[3]));
	}
	static inline void TransQuatMultTransQuatAsTransRot(const double TQa[], const double TQb[],
							    double TR[])
	{
	    double p[3];		
	    QuatMultPoint(&(TQa[3]), TQb, p);
	    TransMultTransAsTrans(TQa, p, TR);
	    QuatMultQuatAsRot(&(TQa[3]), &(TQb[3]), &(TR[3]));
	}
	static inline void TransQuatMultTransRotAsTransQuat(const double TQ[], const double TR[],
							    double TQresult[])
	{
	    double p[3];		
	    QuatMultPoint(&(TQ[3]), TR, p);
	    TransMultTransAsTrans(TQ, p, TQresult);
	    QuatMultRotAsQuat(&(TQ[3]), &(TR[3]), &(TQresult[3]));
	}
	static inline void TransQuatMultTransRotAsTransRot(const double TQ[], const double TR[],
							   double TRresult[])
	{
	    double p[3];		
	    QuatMultPoint(&(TQ[3]), TR, p);
	    TransMultTransAsTrans(TQ, p, TRresult);
	    QuatMultRotAsRot(&(TQ[3]), &(TR[3]), &(TRresult[3]));
	}
	static inline void TransRotMultTransAsTransQuat(const double TR[], const double T[],
							double TQresult[])
	{
	    double p[3];
	    RotMultPoint(&(TR[3]), T, p);
	    TransMultTransAsTrans(TR, p, TQresult);
	    RotAsQuat(&(TR[3]), &(TQresult[3]));
	}
	static inline void TransRotMultTransAsTransRot(const double TR[], const double T[],
						       double TRresult[])
	{
	    double p[3];
	    RotMultPoint(&(TR[3]), T, p);
	    TransMultTransAsTrans(TR, p, TRresult);
	    if(TR != TRresult)
		RotAsRot(&(TR[3]), &(TRresult[3]));
	}
	static inline void TransRotMultQuatAsTransQuat(const double TR[], const double Q[],
						       double TQresult[])
	{
	    RotMultQuatAsQuat(&(TR[3]), Q, &(TQresult[3]));
	    TransAsTrans(TR, TQresult);
	}
	static inline void TransRotMultQuatAsTransRot(const double TR[], const double Q[],
						      double TRresult[])
	{
	    RotMultQuatAsRot(&(TR[3]), Q, &(TRresult[3]));
	    TransAsTrans(TR, TRresult);
	}
	static inline void TransRotMultRotAsTransQuat(const double TR[], const double R[],
						      double TQresult[])
	{
	    RotMultRotAsQuat(&(TR[3]), R, &(TQresult[3]));
	    TransAsTrans(TR, TQresult);
	}
	static inline void TransRotMultRotAsTransRot(const double TR[], const double R[],
						     double TRresult[])
	{
	    RotMultRotAsRot(&(TR[3]), R, &(TRresult[3]));
	    TransAsTrans(TR, TRresult);
	}
	static inline void TransRotMultTransQuatAsTransQuat(const double TR[], const double TQ[],
							    double TQresult[])
	{
	    double p[3];		
	    RotMultPoint(&(TR[3]), TQ, p);
	    TransMultTransAsTrans(TR, p, TQresult);
	    RotMultQuatAsQuat(&(TR[3]), &(TQ[3]), &(TQresult[3]));
	}
	static inline void TransRotMultTransQuatAsTransRot(const double TR[], const double TQ[],
							   double TRresult[])
	{
	    double p[3];		
	    RotMultPoint(&(TR[3]), TQ, p);
	    TransMultTransAsTrans(TR, p, TRresult);
	    RotMultQuatAsRot(&(TR[3]), &(TQ[3]), &(TRresult[3]));
	}
	static inline void TransRotMultTransRotAsTransQuat(const double TRa[], const double TRb[],
							   double TQ[])
	{
	    double p[3];		
	    RotMultPoint(&(TRa[3]), TRb, p);
	    TransMultTransAsTrans(TRa, p, TQ);
	    RotMultRotAsQuat(&(TRa[3]), &(TRb[3]), &(TQ[3]));
	}
	static inline void TransRotMultTransRotAsTransRot(const double TRa[], 
							  const double TRb[],
							  double TR[])
	{
	    double p[3];		
	    RotMultPoint(&(TRa[3]), TRb, p);
	    TransMultTransAsTrans(TRa, p, TR);
	    RotMultRotAsRot(&(TRa[3]), &(TRb[3]), &(TR[3]));
	}
	
	//sample
	static inline void TransSampleUniform(double T[])
	{
	    T[0] = RandomUniformReal();
	    T[1] = RandomUniformReal();
	    T[2] = RandomUniformReal();
	}
	static inline void TransSampleUniform(double T[], const double bbox[])
	{
	    T[0] = RandomUniformReal(bbox[0], bbox[1]);
	    T[1] = RandomUniformReal(bbox[2], bbox[3]);
	    T[2] = RandomUniformReal(bbox[4], bbox[5]);
	}
	void QuatSampleUniform_Marsaglia(double Q[]);
	void QuatSampleUniform_Gaussian(double Q[]);
	void QuatSampleUniform_Shoemake(double Q[]);
	static inline void QuatSampleUniform(double Q[])
	{
	    QuatSampleUniform_Marsaglia(Q);
	}
	static inline void RotSampleUniform(double R[])
	{
	    double Q[Quat_NR_ENTRIES];
	    QuatSampleUniform(Q);
	    QuatAsRot(Q, R);
	}
	static inline void TransQuatSampleUniform(double TQ[])
	{
	    TransSampleUniform(TQ);
	    QuatSampleUniform(&(TQ[3]));
	}
	static inline void TransQuatSampleUniform(double TQ[], const double bbox[])
	{
	    TransSampleUniform(TQ, bbox);
	    QuatSampleUniform(&(TQ[3]));
	}
	static inline void TransRotSampleUniform(double TR[])
	{
	    TransSampleUniform(TR);
	    RotSampleUniform(&(TR[3]));
	}
	static inline void TransRotSampleUniform(double TR[], const double bbox[])
	{
	    TransSampleUniform(TR, bbox);
	    RotSampleUniform(&(TR[3]));
	}

	static inline void QuatLERP(const double Q1[], const double Q2[], const double t, double Q[])
	{
	    Q[0] = (1 - t) * Q1[0] + t * Q2[0];
	    Q[1] = (1 - t) * Q1[1] + t * Q2[1];
	    Q[2] = (1 - t) * Q1[2] + t * Q2[2];
	    Q[3] = (1 - t) * Q1[3] + t * Q2[3];

	    QuatUnit(Q, Q);	    
	}

	static inline void TransQuatLERP(const double TQ1[], const double TQ2[], const double t, double TQ[])
	{
	    TQ[0] = (1 - t) * TQ1[0] + t * TQ2[0];
	    TQ[1] = (1 - t) * TQ1[1] + t * TQ2[1];
	    TQ[2] = (1 - t) * TQ1[2] + t * TQ2[2];
	    
	    QuatLERP(&TQ1[Algebra3D::Trans_NR_ENTRIES],
		     &TQ2[Algebra3D::Trans_NR_ENTRIES], t,
		     &TQ[Algebra3D::Trans_NR_ENTRIES]);
	}
	
	static inline void QuatSLERP(const double Q1[], const double Q2[], const double t, double Q[])
	{
	    double dot = QuatDotProduct(Q1, Q2);
	    int    sign= 1;
	    
	    if(dot < 0)
	    {
		dot = -dot;
		sign= -1;		
	    }
	    if(dot > 0.95)
		QuatLERP(Q1, Q2, t, Q);
	    else
	    {
		const double theta = acos(dot);
		const double c     = 1 / sin(theta);		
		const double a     = sin(theta * (1 - t)) * c;
		const double b     = sign * sin(theta * t) * c;
		
		Q[0] = a * Q1[0] + b * Q2[0];
		Q[1] = a * Q1[1] + b * Q2[1];
		Q[2] = a * Q1[2] + b * Q2[2];
		Q[3] = a * Q1[3] + b * Q2[3];
	    }
	}

	static inline void TransQuatSLERP(const double TQ1[], const double TQ2[], const double t, double TQ[])
	{
	    TQ[0] = (1 - t) * TQ1[0] + t * TQ2[0];
	    TQ[1] = (1 - t) * TQ1[1] + t * TQ2[1];
	    TQ[2] = (1 - t) * TQ1[2] + t * TQ2[2];
	    
	    QuatSLERP(&TQ1[Algebra3D::Trans_NR_ENTRIES],
		      &TQ2[Algebra3D::Trans_NR_ENTRIES], t,
		      &TQ[Algebra3D::Trans_NR_ENTRIES]);
	}

    }	
}


#endif

