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

#ifndef Antipatrea__Algebra2D_HPP_
#define Antipatrea__Algebra2D_HPP_

#include "Utils/Constants.hpp"
#include "Utils/PseudoRandom.hpp"
#include <cmath>
#include <cstring>

namespace Antipatrea
{
    namespace Algebra2D
    {	
	enum
	    {
		Trans_NR_ENTRIES = 2,
		Angle_NR_ENTRIES = 1,
		Rot_NR_ENTRIES = 2,
		TransAngle_NR_ENTRIES = 3,
		TransRot_NR_ENTRIES = 4,
		
		Rot_COS = 0,
		Rot_SIN = 1
	    };
	
	static inline double VecDotProduct(const double v1[], const double v2[])
	{
	    return v1[0] * v2[0] + v1[1] * v2[1];
	}	    
	static inline double VecCrossProduct(const double v1[], const double v2[])
	{
	    return v1[0] * v2[1] - v1[1] * v2[0];		
	}	    
	static inline double PointDistanceSquared(const double p1[], const double p2[])
	{
	    return 
		(p1[0]-p2[0])*(p1[0]-p2[0]) + 
		(p1[1]-p2[1])*(p1[1]-p2[1]);
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
	}
	static inline void VecScale(const double v[], const double s, double vscaled[])
	{
	    vscaled[0] = s * v[0];
	    vscaled[1] = s * v[1];
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
	}	    
	static inline void VecSubtract(const double v1[], const double v2[], double v[])
	{
	    v[0] = v1[0] - v2[0];
	    v[1] = v1[1] - v2[1];
	}	    
	static inline void VecLinearInterpolation(const double v1[],
						  const double t1,
						  const double v2[],
						  const double t2,
						  double       v[])
	{
	    v[0] = t1 * v1[0] + t2 * v2[0];
	    v[1] = t1 * v1[1] + t2 * v2[1];		
	}
	static inline bool VecFromToCCW(const double v1[], const double v2[])
	{
	    return !(VecCrossProduct(v1, v2) < 0);
	}	    
	static inline double VecFromToAngleCCW(const double v1[], const double v2[])
	{
	    const double d = 1.0 / (VecNorm(v1) * VecNorm(v2));
	    return atan2(VecCrossProduct(v1, v2) * d, VecDotProduct(v1, v2) * d);
	}
	static inline double VecFromToAngleCCW(const double v1[], const double v2[], const double d)
	{
	    return atan2(VecCrossProduct(v1, v2) * d, VecDotProduct(v1, v2) * d);
	}
	static inline double VecFromToAngleCCWUnitArgs(const double v1[], const double v2[])
	{
	    return atan2(VecCrossProduct(v1, v2), VecDotProduct(v1, v2));
	}
	static inline double VecAngle(const double v1[], const double v2[])
	{
	    return acos(VecDotProduct(v1, v2) / (VecNorm(v1) * VecNorm(v2)));
	}
	static inline double VecAngleUnitArgs(const double v1[], const double v2[])
	{
	    return acos(VecDotProduct(v1, v2));
	}
	static inline void VecNormal(const double v[], double vnormal[])
	{
	    const double x = v[0];
	    vnormal[0] = v[1];
	    vnormal[1] = -x;
	}
	static inline void VecParallelProjection(const double v[],  
						 const double u[],  
						 double       v_on_u[])
	{
	    const double d = VecDotProduct(v, u) / VecNorm(u);
	    v_on_u[0] = d * u[0];
	    v_on_u[1] = d * u[1];
	}
	static inline void VecParallelProjectionUnitArgs(const double v[],  
							 const double u[], 
							 double       v_on_u[])
	{
	    const double d = VecDotProduct(v, u);
	    v_on_u[0] = d * u[0];
	    v_on_u[1] = d * u[1];
	}	    
	static inline void VecOrthogonalProjection(const double v[], 
						   const double u[], 
						   double       v_on_u[])
	{
	    const double d = VecDotProduct(v, u) / VecNorm(u);
	    v_on_u[0] = v[0] - d * u[0];
	    v_on_u[1] = v[1] - d * u[1];
	}
	static inline void VecOrthogonalProjectionUnitArgs(const double v[], 
							   const double u[], 
							   double       v_on_u[])
	{
	    const double d = VecDotProduct(v, u);
	    v_on_u[0] = v[0] - d * u[0];
	    v_on_u[1] = v[1] - d * u[1];
	}  
	static inline bool VecAreNormal(const double v1[], const double v2[])
	{
	    return fabs(VecDotProduct(v1,v2)) <= Constants::EPSILON;
	}	    
	static inline bool VecAreParallelOrAntiParallel(const double v1[], const double v2[])
	{
	    return fabs(VecCrossProduct(v1, v2)) <= Constants::EPSILON;
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
	static inline bool VecAreParallelUnitArgs(const double v1[], const double v2[])
	{
	    return 
		fabs(v1[0] - v2[0]) <= Constants::EPSILON &&
		fabs(v1[1] - v2[1]) <= Constants::EPSILON;		
	}
	static inline bool VecAreAntiParallelUnitArgs(const double v1[], const double v2[])
	{
	    return 
		fabs(v1[0] + v2[0]) <= Constants::EPSILON &&
		fabs(v1[1] + v2[1]) <= Constants::EPSILON;		
	}
	static inline bool VecAreParallelOrAntiParallelUnitArgs(const double v1[], const double v2[])
	{
	    return 
		VecAreParallelUnitArgs(v1, v2) ||
		VecAreAntiParallelUnitArgs(v1, v2);
	}   
	static inline int VecAlignmentUnitArgs(const double v1[], const double v2[])
	{
	    return
		VecAreParallelUnitArgs(v1, v2) ? 
		Constants::PARALLEL :
		(VecAreAntiParallelUnitArgs(v1, v2) ? 
		 Constants::ANTI_PARALLEL : 
		 Constants::NEITHER_PARALLEL_NOR_ANTI_PARALLEL);
	}	    
	
// utils operations
	static inline double AngleNormalize(const double theta, const double low) 
	{
	    return theta - 2 * M_PI * floor((theta - low) / (2 * M_PI));
	}
	static inline double AngleDistance(const double theta1, const double theta2)
	{
	    const double d = AngleNormalize(fabs(theta1 - theta2), 0);
	    return d > M_PI ? (2 * M_PI - d) : d;
	}
	static inline double AngleSignedDistance(const double theta1, const double theta2)
	{	  	    
	    const double d = AngleNormalize(theta2 - theta1, 0);
	    
	    if(fabs(d) > M_PI)
	    {
		if(d < 0) 
		    return 2 * M_PI + d;
		else
		    return -2 * M_PI + d;
	    }
	    else 
		return d;    
	}
	static inline double AngleInterpolation(const double theta1, 
						const double theta2,
						const double t)
	{
	    return theta1 + t * AngleSignedDistance(theta1, theta2);
	}

	static inline bool AngleIsInside(const double theta, 
					 const double mintheta, 
					 const double range)
	{
	    return AngleNormalize(theta, mintheta) <= (mintheta + range);
	}
	
	static inline double AngleClamp(const double theta, const double mintheta, const double range)
	{
	    if(AngleIsInside(theta, mintheta, range))
		return theta;
	    
	    const double d1 = AngleDistance(theta, mintheta);
	    const double d2 = AngleDistance(theta, mintheta + range);
	    
	    return d1 < d2 ? mintheta : mintheta + range;
	}
	
//identity
	static inline void IdentityAsTrans(double T[])
	{
	    T[0] = T[1] = 0;
	}
	static inline double IdentityAsAngle(void)
	{
	    return 0;	    
	}
	static inline void IdentityAsRot(double R[])
	{
	    R[Rot_COS] = 1;
	    R[Rot_SIN] = 0;
	}
	static inline double IdentityAsTransAngle(double T[])
	{
	    IdentityAsTrans(T);	    
	    return IdentityAsAngle();
	}
	static inline void IdentityAsTransRot(double TR[])
	{
	    IdentityAsTrans(TR);
	    IdentityAsRot(&TR[Algebra2D::Trans_NR_ENTRIES]);
	}
	
//comparison: same as identity
	static inline bool IdentitySameAsTrans(const double T[])
	{
	    return 
		fabs(T[0]) <= Constants::EPSILON &&
		fabs(T[1]) <= Constants::EPSILON;
	}
	static inline bool IdentitySameAsAngle(const double A)
	{
	    return fabs(AngleNormalize(A, -M_PI)) <= Constants::EPSILON;
	}
	static inline bool IdentitySameAsRot(const double R[])
	{
	    return
		fabs(R[Rot_COS] - 1) <= Constants::EPSILON &&
		fabs(R[Rot_SIN]) <= Constants::EPSILON;
	}
	static inline bool IdentitySameAsTransAngle(const double T[], const double A)
	{
	    return
		IdentitySameAsTrans(T) &&
		IdentitySameAsAngle(A);
	}
	static inline bool IdentitySameAsTransRot(const double T[], const double R[])
	{
	    return
		IdentitySameAsTrans(T) &&
		IdentitySameAsRot(R);
	}
	
//conversion	    
	static inline void TransAsTrans(const double T[], double Tresult[])
	{
	    Tresult[0] = T[0];
	    Tresult[1] = T[1];
	}
	static inline double AngleAsAngle(const double A)
	{
	    return A;	    
	}
	static inline void AngleAsRot(const double A, double R[])
	{
	    R[Rot_COS] = cos(A);
	    R[Rot_SIN] = sin(A);	    
	}	
	static inline double RotAsAngle(const double R[])
	{
	    return atan2(R[Rot_SIN], R[Rot_COS]);
	}
	static inline void RotAsRot(const double R[], double Rresult[])
	{
	    Rresult[Rot_COS] = R[Rot_COS];		
	    Rresult[Rot_SIN] = R[Rot_SIN];
	}
	static inline void TransAngleAsTransAngle(const double TA[], double TAresult[])
	{
	    TAresult[0] = TA[0];
	    TAresult[1] = TA[1];
	    TAresult[2] = TA[2];
	}
	static inline void TransAngleAsTransRot(const double TA[], double TR[])
	{
	    TransAsTrans(TA, TR);
	    AngleAsRot(TA[2], &(TR[2]));
	}
	static inline void TransRotAsTransAngle(const double TR[], double TA[])
	{
	    TransAsTrans(TR, TA);
	    TA[2] = RotAsAngle(&(TR[2]));
	}
	static inline void TransRotAsTransRot(const double TR[], double TRresult[])
	{
	    memcpy(TRresult, TR, TransRot_NR_ENTRIES * sizeof(double));
	}
	
	
//comparison with each other
	static inline bool TransSameAsTrans(const double Ta[], const double Tb[])
	{
	    return 
		fabs(Ta[0] - Tb[0]) <= Constants::EPSILON &&
		fabs(Ta[1] - Tb[1]) <= Constants::EPSILON;
	}
	static inline bool AngleSameAsAngle(const double Aa, const double Ab)
	{
	    return fabs(AngleNormalize(Aa - Ab, -M_PI))  <= Constants::EPSILON;
	}
	static inline bool AngleSameAsRot(const double A, const double R[])
	{	
	    return 
		fabs(cos(A) - R[Rot_COS]) <= Constants::EPSILON &&
		fabs(sin(A) - R[Rot_SIN]) <= Constants::EPSILON;
	}
	static inline bool RotSameAsAngle(const double R[], const double A)
	{
	    return AngleSameAsRot(A, R);
	}
	static inline bool RotSameAsRot(const double Ra[], const double Rb[])
	{
	    return 
		fabs(Ra[Rot_COS] - Rb[Rot_COS]) <= Constants::EPSILON &&
		fabs(Ra[Rot_SIN] - Rb[Rot_SIN]) <= Constants::EPSILON;
	}
	static inline bool TransAngleSameAsTransAngle(const double TAa[], const double TAb[])
	{
	    return 
		TransSameAsTrans(TAa, TAb) &&
		AngleSameAsAngle(TAa[2], TAb[2]);
	}
	static inline bool TransAngleSameAsTransRot(const double TA[], const double TR[])
	{
	    return 
		TransSameAsTrans(TA, TR) &&
		AngleSameAsRot(TA[2], &(TR[2]));
	}
	static inline bool TransRotSameAsTransAngle(const double TR[], const double TA[])
	{
	    return TransAngleSameAsTransRot(TA, TR);
	}
	static inline bool TransRotSameAsTransRot(const double TRa[], const double TRb[])
	{
	    return 
		TransSameAsTrans(TRa, TRb) &&
		RotSameAsRot(&(TRa[2]), &(TRb[2]));
	}
	
//transform point
	static inline void TransMultPoint(const double T[], const double p[], double pnew[])
	{
	    pnew[0] = p[0] + T[0];
	    pnew[1] = p[1] + T[1];
	}
	static inline void TransMultInvPoint(const double T[], const double p[], double pnew[])
	{
	    pnew[0] = -p[0] + T[0];
	    pnew[1] = -p[1] + T[1];
	}
	static inline void InvTransMultPoint(const double T[], const double p[], double pnew[])
	{
	    pnew[0] = p[0] - T[0];
	    pnew[1] = p[1] - T[1];
	}
	static inline void InvTransMultInvPoint(const double T[], const double p[], double pnew[])
	{
	    pnew[0] = -p[0] - T[0];
	    pnew[1] = -p[1] - T[1];
	}
	static inline void AngleMultPoint(const double A, const double p[], double pnew[])
	{
	    const double c = cos(A);
	    const double s = sin(A);
	    const double x = p[0];
	    
	    pnew[0]= c * x - s * p[1];
	    pnew[1]= s * x + c * p[1];
	}
	static inline void AngleMultInvPoint(const double A, const double p[], double pnew[])
	{
	    const double c = cos(A);
	    const double s = sin(A);
	    const double x = -p[0];
	    
	    pnew[0]= c * x + s * p[1];
	    pnew[1]= s * x - c * p[1];
	}
	static inline void InvAngleMultPoint(const double A, const double p[], double pnew[])
	{
	    const double c = cos(A);
	    const double s = -sin(A);
	    const double x = p[0];
	    
	    pnew[0]= c * x - s * p[1];
	    pnew[1]= s * x + c * p[1];
	}
	static inline void InvAngleMultInvPoint(const double A, const double p[], double pnew[])
	{
	    const double c = cos(A);
	    const double s = -sin(A);
	    const double x = -p[0];
	    
	    pnew[0]= c * x + s * p[1];
	    pnew[1]= s * x - c * p[1];
	}
	static inline void RotMultPoint(const double R[], const double p[], double pnew[])
	{
	    const double x = p[0];
	    pnew[0] = R[Rot_COS] * x - R[Rot_SIN] * p[1];
	    pnew[1] = R[Rot_SIN] * x + R[Rot_COS] * p[1];
	}
	static inline void RotMultInvPoint(const double R[], const double p[], double pnew[])
	{
	    const double x = -p[0];
	    pnew[0] = R[Rot_COS] * x + R[Rot_SIN] * p[1];
	    pnew[1] = R[Rot_SIN] * x - R[Rot_COS] * p[1];
	}
	static inline void InvRotMultPoint(const double R[], const double p[], double pnew[])
	{
	    const double x = p[0];
	    pnew[0] =  R[Rot_COS] * x + R[Rot_SIN] * p[1];
	    pnew[1] = -R[Rot_SIN] * x + R[Rot_COS] * p[1];
	}
	static inline void InvRotMultInvPoint(const double R[], const double p[], double pnew[])
	{
	    const double x = -p[0];
	    pnew[0] =  R[Rot_COS] * x - R[Rot_SIN] * p[1];
	    pnew[1] = -R[Rot_SIN] * x - R[Rot_COS] * p[1];
	}
	static inline void TransAngleMultPoint(const double TA[], const double p[], double pnew[])
	{
	    AngleMultPoint(TA[2], p, pnew);
	    TransMultPoint(TA, pnew, pnew);
	}
	static inline void TransAngleMultInvPoint(const double TA[], const double p[], double pnew[])
	{
	    AngleMultInvPoint(TA[2], p, pnew);
	    TransMultPoint(TA, pnew, pnew);
	}
	static inline void InvTransAngleMultPoint(const double TA[], const double p[], double pnew[])
	{
	    InvTransMultPoint(TA, p, pnew);
	    InvAngleMultPoint(TA[2], pnew, pnew);
	}
	static inline void InvTransAngleMultInvPoint(const double TA[], const double p[], double pnew[])
	{
	    InvTransMultInvPoint(TA, p, pnew);
	    InvAngleMultPoint(TA[2], pnew, pnew);
	}
	static inline void TransRotMultPoint(const double TR[], const double p[], double pnew[])
	{
	    RotMultPoint(&(TR[2]), p, pnew);
	    TransMultPoint(TR, pnew, pnew);
	}
	static inline void TransRotMultInvPoint(const double TR[], const double p[], double pnew[])
	{
	    RotMultInvPoint(&(TR[2]), p, pnew);
	    TransMultPoint(TR, pnew, pnew);
	}
	static inline void InvTransRotMultPoint(const double TR[], const double p[], double pnew[])
	{
	    InvTransMultPoint(TR, p, pnew);
	    InvRotMultPoint(&(TR[2]), pnew, pnew);
	}
	static inline void InvTransRotMultInvPoint(const double TR[], const double p[], 
						   double pnew[])
	{
	    InvTransMultInvPoint(TR, p, pnew);
	    InvRotMultPoint(&(TR[2]), pnew, pnew);
	}
	
//inverse
	static inline void InvTransAsTrans(const double T[], double Tresult[])
	{
	    Tresult[0] = -T[0];
	    Tresult[1] = -T[1];
	}
	static inline double InvAngleAsAngle(const double A)
	{
	    return -A;
	}
	static inline void InvAngleAsRot(const double A, double R[])
	{
	    R[Rot_COS] = cos(A);
	    R[Rot_SIN] = -sin(A);
	}
	static inline double InvRotAsAngle(const double R[])
	{
	    return atan2(-R[Rot_SIN],  R[Rot_COS]);
	}
	static inline void InvRotAsRot(const double R[], double Rresult[])
	{
	    Rresult[Rot_COS] =  R[Rot_COS];
	    Rresult[Rot_SIN] = -R[Rot_SIN];
	}
	static inline void InvTransAngleAsTransAngle(const double TA[], double TAresult[])
	{
	    TAresult[2] = InvAngleAsAngle(TA[2]);
	    AngleMultInvPoint(TAresult[2], TA, TAresult);
	}
	static inline void InvTransAngleAsTransRot(const double TA[], double TR[])
	{
	    InvAngleAsRot(TA[2], &(TR[2]));
	    RotMultInvPoint(&(TR[2]), TA, TR);
	}
	static inline void InvTransRotAsTransAngle(const double TR[], double TA[])
	{
	    TA[2] = InvRotAsAngle(&(TR[2]));
	    InvRotMultInvPoint(&(TR[2]), TR, TA);
	}
	static inline void InvTransRotAsTransRot(const double TR[], double TRresult[])
	{
	    InvRotAsRot(&(TR[2]), &(TRresult[2]));
	    RotMultInvPoint(&(TRresult[2]), TR, TRresult);
	}
	
//combine
	static inline void TransMultTransAsTrans(const double Ta[], const double Tb[], double T[])
	{
	    T[0] = Ta[0] + Tb[0];
	    T[1] = Ta[1] + Tb[1];
	}
	static inline void TransMultAngleAsTransAngle(const double T[], const double A, double TA[])
	{
	    TransAsTrans(T, TA);
	    TA[2] = AngleAsAngle(A);
	}
	static inline void TransMultAngleAsTransRot(const double T[], const double A, double TR[])
	{
	    TransAsTrans(T, TR);
	    AngleAsRot(A, &(TR[2]));
	}
	static inline void TransMultRotAsTransAngle(const double T[], const double R[], double TA[])
	{
	    TransAsTrans(T, TA);
	    TA[2] = RotAsAngle(R);
	}
	static inline void TransMultRotAsTransRot(const double T[], const double R[], double TR[])
	{
	    TransAsTrans(T, TR);
	    RotAsRot(R, &(TR[2]));
	}
	static inline void TransMultTransAngleAsTransAngle(const double T[], const double TA[], 
							   double TAresult[])
	{
	    TransMultTransAsTrans(T, TA, TAresult);
	    TAresult[2] = AngleAsAngle(TA[2]);
	}
	static inline void TransMultTransAngleAsTransRot(const double T[], const double TA[], 
							 double TR[])
	{
	    TransMultTransAsTrans(T, TA, TR);
	    AngleAsRot(TA[2], &(TR[2]));
	}
	static inline void TransMultTransRotAsTransAngle(const double T[], const double TR[], 
							 double TA[])
	{
	    TransMultTransAsTrans(T, TR, TA);
	    TA[2] = RotAsAngle(&(TR[2]));
	}
	static inline void TransMultTransRotAsTransRot(const double T[], const double TR[], 
						       double TRresult[])
	{
	    TransMultTransAsTrans(T, TR, TRresult);
	    RotAsRot(&(TR[2]), &(TRresult[2]));
	}
	static inline void AngleMultTransAsTransAngle(const double A, const double T[], double TA[])
	{
	    AngleMultPoint(A, T, TA);
	    TA[2] = AngleAsAngle(A);
	}
	static inline void AngleMultTransAsTransRot(const double A, const double T[], double TR[])
	{
	    AngleAsRot(A, &(TR[2]));
	    RotMultPoint(&(TR[2]), T, TR);
	}
	static inline double AngleMultAngleAsAngle(const double Aa, const double Ab)
	{
	    return Aa + Ab;
	}
	static inline void AngleMultAngleAsRot(const double Aa, const double Ab, double R[])
	{
	    R[Rot_COS] = cos(Aa + Ab);
	    R[Rot_SIN] = sin(Aa + Ab);
	}
	static inline double AngleMultRotAsAngle(const double A, const double R[])
	{
	    return A + RotAsAngle(R);
	}
	static inline void RotMultRotAsRot(const double Ra[], const double Rb[], double R[])
	{
	    const double ca = Ra[Rot_COS];
	    const double cb = Rb[Rot_COS];
	    
	    R[Rot_COS] = ca * cb - Ra[Rot_SIN] * Rb[Rot_SIN];
	    R[Rot_SIN] = Ra[Rot_SIN] * cb + ca * Rb[Rot_SIN];
	}
	static inline void AngleMultRotAsRot(const double A, const double R[], double Rresult[])
	{
	    double Rtmp[Rot_NR_ENTRIES];
	    AngleAsRot(A, Rtmp);
	    RotMultRotAsRot(Rtmp, R, Rresult);
	}
	static inline void AngleMultTransAngleAsTransAngle(const double A, const double TA[], 
							   double TAresult[])
	{
	    AngleMultPoint(A, TA, TAresult);
	    TAresult[2] = AngleMultAngleAsAngle(A, TA[2]);
	}
	static inline void AngleMultTransAngleAsTransRot(const double A, const double TA[], 
							 double TR[])
	{
	    AngleMultPoint(A, TA, TR);
	    AngleMultAngleAsRot(A, TA[2], &(TR[2]));
	}
	static inline void AngleMultTransRotAsTransAngle(const double A, const double TR[],
							 double TA[])
	{
	    AngleMultPoint(A, TR, TA);
	    TA[2] = AngleMultRotAsAngle(A, &(TR[2]));
	}
	static inline void AngleMultTransRotAsTransRot(const double A, const double TR[],
						       double TRresult[])
	{
	    double Rtmp[Rot_NR_ENTRIES];
	    AngleAsRot(A, Rtmp);
	    RotMultPoint(Rtmp, TR, TRresult);
	    RotMultRotAsRot(Rtmp, &(TR[2]), &(TRresult[2]));
	}
	static inline void RotMultTransAsTransAngle(const double R[], const double T[], double TA[])
	{
	    RotMultPoint(R, T, TA);
	    TA[2] = RotAsAngle(R);
	}
	static inline void RotMultTransAsTransRot(const double R[], const double T[], double TR[])
	{
	    RotMultPoint(R, T, TR);
	    RotAsRot(R, &(TR[2]));
	}
	static inline double RotMultAngleAsAngle(const double R[], const double A)
	{
	    return RotAsAngle(R) + A;
	}
	static inline void RotMultAngleAsRot(const double R[], const double A, double Rresult[])
	{
	    double Rtmp[Rot_NR_ENTRIES];
	    AngleAsRot(A, Rtmp);
	    RotMultRotAsRot(R, Rtmp, Rresult);
	}
	static inline double RotMultRotAsAngle(const double Ra[], const double Rb[])
	{
	    double R[Rot_NR_ENTRIES];
	    RotMultRotAsRot(Ra, Rb, R);
	    return RotAsAngle(R);
	}
	static inline void RotMultTransAngleAsTransAngle(const double R[], const double TA[],
							 double TAresult[])
	{
	    RotMultPoint(R, TA, TAresult);
	    TAresult[2] = RotMultAngleAsAngle(R, TA[2]);
	}
	static inline void RotMultTransAngleAsTransRot(const double R[], const double TA[],
						       double TRresult[])
	{
	    RotMultPoint(R, TA, TRresult);
	    RotMultAngleAsRot(R, TA[2], &(TRresult[2]));
	}
	static inline void RotMultTransRotAsTransAngle(const double R[], const double TR[],
						       double TA[])
	{
	    RotMultPoint(R, TR, TA);
	    TA[2] = RotMultRotAsAngle(R, &(TR[2]));
	}
	static inline void RotMultTransRotAsTransRot(const double R[], const double TR[],
						     double TRresult[])
	{
	    RotMultPoint(R, TR, TRresult);
	    RotMultRotAsRot(R, &(TR[2]), &(TRresult[2]));
	}
	static inline void TransAngleMultTransAsTransAngle(const double TA[], const double T[],
							   double TAresult[])
	{
	    double p[2];
	    AngleMultPoint(TA[2], T, p);
	    TransMultTransAsTrans(TA, p, TAresult);
	    TAresult[2] = AngleAsAngle(TA[2]);
	}
	static inline void TransAngleMultTransAsTransRot(const double TA[], const double T[],
							 double TR[])
	{
	    double p[2];
	    AngleMultPoint(TA[2], T, p);
	    TransMultTransAsTrans(TA, p, TR);
	    AngleAsRot(TA[2], &(TR[2]));
	}
	static inline void TransAngleMultAngleAsTransAngle(const double TA[], const double A,
							   double TAresult[])
	{
	    TAresult[2] = AngleMultAngleAsAngle(TA[2], A);
	    TransAsTrans(TA, TAresult);
	}
	static inline void TransAngleMultAngleAsTransRot(const double TA[], const double A,
							 double TR[])
	{
	    AngleMultAngleAsRot(TA[2], A, &(TR[2]));
	    TransAsTrans(TA, TR);
	}
	static inline void TransAngleMultRotAsTransAngle(const double TA[], const double R[],
							 double TAresult[])
	{
	    TAresult[2] = AngleMultRotAsAngle(TA[2], R);
	    TransAsTrans(TA, TAresult);
	}
	static inline void TransAngleMultRotAsTransRot(const double TA[], const double R[],
						       double TR[])
	{
	    AngleMultRotAsRot(TA[2], R, &(TR[2]));
	    TransAsTrans(TA, TR);
	}
	static inline void TransAngleMultTransAngleAsTransAngle(const double TAa[], const double TAb[],
								double TA[])
	{
	    double p[2];
	    TransAsTrans(TAa, p);
	    AngleMultTransAngleAsTransAngle(TAa[2], TAb, TA);
	    TransMultTransAsTrans(p, TA, TA);
	}
	static inline void TransAngleMultTransAngleAsTransRot(const double TAa[], const double TAb[],
							      double TR[])
	{
	    double p[2];
	    TransAsTrans(TAa, p);
	    AngleMultTransAngleAsTransRot(TAa[2], TAb, TR);
	    TransMultTransAsTrans(p, TR, TR);
	}
	static inline void TransAngleMultTransRotAsTransAngle(const double TA[], const double TR[],
							      double TAresult[])
	{
	    double p[2];
	    TransAsTrans(TA, p);
	    AngleMultTransRotAsTransAngle(TA[2], TR, TAresult);
	    TransMultTransAsTrans(p, TAresult, TAresult);
	}
	static inline void TransAngleMultTransRotAsTransRot(const double TA[], const double TR[],
							    double TRresult[])
	{
	    double p[2];
	    TransAsTrans(TA, p);
	    AngleMultTransRotAsTransRot(TA[2], TR, TRresult);
	    TransMultTransAsTrans(p, TRresult, TRresult);
	}
	static inline void TransRotMultTransAsTransAngle(const double TR[], const double T[],
							 double TAresult[])
	{
	    double p[2];
	    RotMultPoint(&(TR[2]), T, p);
	    TransMultTransAsTrans(TR, p, TAresult);
	    TAresult[2] = RotAsAngle(&(TR[2]));
	}
	static inline void TransRotMultTransAsTransRot(const double TR[], const double T[],
						       double TRresult[])
	{
	    double p[2];
	    RotMultPoint(&(TR[2]), T, p);
	    TransMultTransAsTrans(TR, p, TRresult);
	    RotAsRot(&(TR[2]), &(TRresult[2]));
	}
	static inline void TransRotMultAngleAsTransAngle(const double TR[], const double A,
							 double TAresult[])
	{
	    TAresult[2] = RotMultAngleAsAngle(&(TR[2]), A);
	    TransAsTrans(TR, TAresult);
	}
	static inline void TransRotMultAngleAsTransRot(const double TR[], const double A,
						       double TRresult[])
	{
	    RotMultAngleAsRot(&(TR[2]), A, &(TRresult[2]));
	    TransAsTrans(TR, TRresult);
	}
	static inline void TransRotMultRotAsTransAngle(const double TR[], const double R[],
						       double TAresult[])
	{
	    TAresult[2] = RotMultRotAsAngle(&(TR[2]), R);
	    TransAsTrans(TR, TAresult);
	}
	static inline void TransRotMultRotAsTransRot(const double TR[], const double R[],
						     double TRresult[])
	{
	    RotMultRotAsRot(&(TR[2]), R, &(TRresult[2]));
	    TransAsTrans(TR, TRresult);
	}
	static inline void TransRotMultTransAngleAsTransAngle(const double TR[], const double TA[],
							      double TAresult[])
	{
	    double p[2];
	    TransAsTrans(TR, p);
	    RotMultTransAngleAsTransAngle(&(TR[2]), TA, TAresult);
	    TransMultTransAsTrans(p, TAresult, TAresult);
	}
	static inline void TransRotMultTransAngleAsTransRot(const double TR[], const double TA[],
							    double TRresult[])
	{
	    double p[2];
	    TransAsTrans(TR, p);
	    RotMultTransAngleAsTransRot(&(TR[2]), TA, TRresult);
	    TransMultTransAsTrans(p, TRresult, TRresult);
	}
	static inline void TransRotMultTransRotAsTransAngle(const double TRa[], const double TRb[],
							    double TA[])
	{
	    double p[2];
	    TransAsTrans(TRa, p);
	    RotMultTransRotAsTransAngle(&(TRa[2]), TRb, TA);
	    TransMultTransAsTrans(p, TA, TA);
	}
	static inline void TransRotMultTransRotAsTransRot(const double TRa[], 
							  const double TRb[],
							  double TR[])
	{
	    double p[2];
	    TransAsTrans(TRa, p);
	    RotMultTransRotAsTransRot(&(TRa[2]), TRb, TR);
	    TransMultTransAsTrans(p, TR, TR);
	}
	
	//sample
	static inline void TransSampleUniform(double T[])
	{
	    T[0] = RandomUniformReal();
	    T[1] = RandomUniformReal();
	}
	static inline void TransSampleUniform(double T[], const double bbox[])
	{
	    T[0] = RandomUniformReal(bbox[0], bbox[1]);
	    T[1] = RandomUniformReal(bbox[2], bbox[3]);
	}
	static inline double AngleSampleUniform(void)
	{
	    return RandomUniformReal(-M_PI, M_PI);
	}
	static inline double AngleSampleUniform(const double bbox[])
	{
	    return RandomUniformReal(bbox[0], bbox[1]);
	}
	static inline void RotSampleUniform(double R[])
	{
	    const double c = RandomUniformReal(-1, 1);
	    
	    R[Rot_COS] = c;		
	    R[Rot_SIN] = sqrt(1 - c * c) * (RandomUniformBoolean() ? 1:-1);
	}
	static inline void RotSampleUniform(double R[], const double bbox[])
	{
	    const double theta = RandomUniformReal(bbox[0], bbox[1]);
	    R[Rot_COS] = cos(theta);		
	    R[Rot_SIN] = sin(theta);
	}
	static inline void TransAngleSampleUniform(double TA[])
	{
	    TransSampleUniform(TA);
	    TA[2] = AngleSampleUniform();
	}
	static inline void TransAngleSampleUniform(double TA[], const double bbox[])
	{
	    TransSampleUniform(TA, bbox);
	    TA[2] = AngleSampleUniform(&(bbox[4]));
	}
	static inline void TransRotSampleUniform(double TR[])
	{
	    TransSampleUniform(TR);
	    RotSampleUniform(&(TR[2]));
	}
	static inline void TransRotSampleUniform(double TR[], const double bbox[])
	{
	    TransSampleUniform(TR, bbox);
	    RotSampleUniform(&(TR[2]), &(bbox[4]));
	}

	static inline void RotateAroundPointAsTransRot(const double theta, const double p[2], double TR[])
	{
/*
 T(p) R(theta) T(-p) o

 T(p)     = 1  0  x
            0  1  y
            0  0  1
 
 R(theta) = c -s  0
            s  c  0
            0  0  1

 T(-p)   = 1   0 -x 
           0   1 -y
           0   0  1

 T(p) R(theta) = c -s x
                 s  c y
                 0  0 1

*/
	    AngleAsRot(theta, &TR[Trans_NR_ENTRIES]);

	    const double c = TR[Trans_NR_ENTRIES + Rot_COS];
	    const double s = TR[Trans_NR_ENTRIES + Rot_SIN];	    
	    const double x = -c * p[0] + s * p[1] + p[0];
	    TR[1] = -s * p[0] - c * p[1] + p[1];
	    TR[0] = x;
	}
    }	
}


#endif

