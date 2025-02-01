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
#include "Utils/Algebra.hpp"
#include <vector>

#ifdef HAS_LAPACK
extern "C" void dgesv_(int *n, int *nrhs, double *a, int *lda,
		    int *ipiv, double *b, int *ldb, int *info);

extern "C" void dgesvd_(const char* jobu, const char* jobvt, const int* M, const int* N,
			double* A, const int* lda, double* S, double* U, const int* ldu,
			double* VT, const int* ldvt, double* work,const int* lwork, const
			int* info);
#endif

namespace Antipatrea
{
    namespace Algebra
    {	
	std::vector<int> global_solveLinearSystemPerm;
	std::vector<double> global_svdWork;
	
	bool SolveLinearSystemMatMultVec(const int n,
					 double Atransp[],
					 double b[])
	{
#ifdef HAS_LAPACK
	    int N = n;
	    int NRHS = 1;
	    int LDA = N;
	    int LDB = N;
	    int info;
	    
	    if((int) global_solveLinearSystemPerm.size() < N)
		global_solveLinearSystemPerm.resize(N);
	    
	    dgesv_(&N, &NRHS, Atransp, &LDA, &global_solveLinearSystemPerm[0], b, &LDB, &info);
	    return info == 0;
#else
	    return false;
#endif
	}	    
	
	//Utransp: nrRows x nrRows
	//Sigma: max(nrRows, nrCols)
	//V: nrCols x nrCols
	bool SVD(const int nrRowsA,
		 const int nrColsA,
		 double Atransp[], double Utransp[], double Sigma[], double V[])
	{
#ifdef HAS_LAPACK
	    char JOBU = Utransp == NULL ? 'N' : 'A';
	    char JOBVT= V == NULL ? 'N' : 'A';
	    int  M    = nrRowsA;
	    int  N    = nrColsA;
	    int  LDA  = M;
	    int  LDU  = M;
	    int  LDVT = N;
	    int  LWORK= -1;
	    int  info = 0;
	    double  worksize;
	    
	    
	    dgesvd_(&JOBU, &JOBVT, &M, &N, Atransp, &LDA,
		    Sigma, Utransp, &LDU, V, &LDVT, &worksize, &LWORK, &info);
	    
	    LWORK = (int) worksize;
	    if(LWORK > (int) global_svdWork.size())
		global_svdWork.resize(LWORK);
	    
	    dgesvd_(&JOBU, &JOBVT, &M, &N, Atransp, &LDA,
		    Sigma, Utransp, &LDU, V, &LDVT, &global_svdWork[0], &LWORK, &info);
	    
	    return info == 0;
#else
	    return false;
#endif
	}	    
    }	
}
