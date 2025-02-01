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

#ifndef Antipatrea__Algebra_HPP_
#define Antipatrea__Algebra_HPP_

#include "Utils/Constants.hpp"
#include "Utils/PseudoRandom.hpp"
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cassert>

namespace Antipatrea {
    namespace Algebra {
        static inline double VecDotProduct(const int n, const double v1[], const double v2[]) {
            double d = 0;
            for (int i = 0; i < n; ++i)
                d += v1[i] * v2[i];
            return d;
        }

        static inline double PointDistanceSquared(const int n, const double p1[], const double p2[]) {
            double d = 0;
            for (int i = 0; i < n; ++i)
                d += (p1[i] - p2[i]) * (p1[i] - p2[i]);
            return d;
        }

        static inline double PointDistance(const int n, const double p1[], const double p2[]) {
            return sqrt(PointDistanceSquared(n, p1, p2));
        }

        static inline double VecNormSquared(const int n, const double v[]) {
            return VecDotProduct(n, v, v);
        }

        static inline double VecNorm(const int n, const double v[]) {
            return sqrt(VecNormSquared(n, v));
        }

        static inline void VecScale(const int n, const double v[], const double s, double vscaled[]) {
            for (int i = 0; i < n; ++i)
                vscaled[i] = s * v[i];
        }

        static inline void VecUnit(const int n, const double v[], double vunit[]) {
            VecScale(n, v, 1.0 / VecNorm(n, v), vunit);
        }

        static inline void VecClampMagnitude(const int n, double v[], const double dmax) {
            const double d = VecNormSquared(n, v);
            if (d > (dmax * dmax))
                VecScale(n, v, dmax / sqrt(d), v);
        }

        static inline void VecClampMaxAbs(const int n, double v[], const double amax) {
            double vmax = -1;
            for (int i = 0; i < n; ++i)
                if (fabs(v[i]) > vmax)
                    vmax = fabs(v[i]);
            if (vmax > amax)
                VecScale(n, v, amax / vmax, v);
        }

        static inline double VecMaxAbsDifference(const int n, const double v1[], const double v2[]) {
            double dmax = -INFINITY;
            double dcurr;

            for (int i = 0; i < n; ++i)
                if (dmax < (dcurr = fabs(v1[i] - v2[i])))
                    dmax = dcurr;
            return dmax;
        }

        static inline void VecAdd(const int n, const double v1[], const double v2[], double v[]) {
            for (int i = 0; i < n; ++i)
                v[i] = v1[i] + v2[i];
        }

        static inline void VecSubtract(const int n, const double v1[], const double v2[], double v[]) {
            for (int i = 0; i < n; ++i)
                v[i] = v1[i] - v2[i];
        }

        static inline void VecLinearInterpolation(const int n,
                                                  const double v1[],
                                                  const double t1,
                                                  const double v2[],
                                                  const double t2,
                                                  double v[]) {
            for (int i = 0; i < n; ++i)
                v[i] = t1 * v1[i] + t2 * v2[i];
        }

        static inline double VecAngle(const int n, const double v1[], const double v2[]) {
            return acos(VecDotProduct(n, v1, v2) / (VecNorm(n, v1) * VecNorm(n, v2)));
        }

        static inline double VecAngleUnitArgs(const int n, const double v1[], const double v2[]) {
            return acos(VecDotProduct(n, v1, v2));
        }

        static inline void VecNormal(const int n, const double v[], double vnormal[]) {
            int nonzero = -1;
            double d = 0;

            for (int i = 0; i < n && nonzero < 0; ++i)
                if (fabs(v[i]) > Constants::EPSILON)
                    nonzero = i;
                else {
                    d += v[i] * v[i];
                    vnormal[i] = v[i];
                }

            if (nonzero >= 0) {
                for (int i = nonzero; i < n; ++i) {
                    d += v[i] * v[i];
                    vnormal[i] = v[i];
                }
                vnormal[nonzero] = -d / v[nonzero];
            } else
                for (int i = 0; i < n; ++i)
                    vnormal[i] = 0;
        }

        static inline void VecParallelProjection(const int n,
                                                 const double v[], const double u[], double v_on_u[]) {
            VecScale(n, u, VecDotProduct(n, v, u) / VecNorm(n, u), v_on_u);
        }

        static inline void VecParallelProjectionUnitArgs(const int n,
                                                         const double v[],
                                                         const double u[],
                                                         double v_on_u[]) {
            VecScale(n, u, VecDotProduct(n, v, u), v_on_u);
        }

        static inline void VecOrthogonalProjection(const int n,
                                                   const double v[],
                                                   const double u[],
                                                   double v_on_u[]) {
            const double d = VecDotProduct(n, v, u) / VecNorm(n, u);
            for (int i = 0; i < n; ++i)
                v_on_u[i] = v[i] - d * u[i];
        }

        static inline void VecOrthogonalProjectionUnitArgs(const int n,
                                                           const double v[],
                                                           const double u[],
                                                           double v_on_u[]) {
            const double d = VecDotProduct(n, v, u);
            for (int i = 0; i < n; ++i)
                v_on_u[i] = v[i] - d * u[i];
        }

        static inline bool VecAreNormal(const int n, const double v1[], const double v2[]) {
            return fabs(VecDotProduct(n, v1, v2)) <= Constants::EPSILON;
        }

        static inline bool VecAreParallelOrAntiParallel(const int n, const double v1[], const double v2[]) {
            const double c = VecDotProduct(n, v1, v2) / (VecNorm(n, v1) * VecNorm(n, v2));
            return fabs(c - 1) <= Constants::EPSILON || fabs(c + 1) <= Constants::EPSILON;
        }

        static inline bool VecAreParallel(const int n, const double v1[], const double v2[]) {
            return fabs(VecDotProduct(n, v1, v2) / (VecNorm(n, v1) * VecNorm(n, v2)) - 1) <= Constants::EPSILON;
        }

        static inline bool VecAreAntiParallel(const int n, const double v1[], const double v2[]) {
            return fabs(VecDotProduct(n, v1, v2) / (VecNorm(n, v1) * VecNorm(n, v2)) + 1) <= Constants::EPSILON;
        }

        static inline int VecAlignment(const int n, const double v1[], const double v2[]) {
            const double c = VecDotProduct(n, v1, v2) / (VecNorm(n, v1) * VecNorm(n, v2));
            return fabs(c - 1) <= Constants::EPSILON ? Constants::PARALLEL : (fabs(c + 1) <= Constants::EPSILON
                                                                              ? Constants::ANTI_PARALLEL
                                                                              : Constants::NEITHER_PARALLEL_NOR_ANTI_PARALLEL);
        }

        static inline bool VecAreParallelOrAntiParallelUnitArgs(const int n,
                                                                const double v1[], const double v2[]) {
            const double d = VecDotProduct(n, v1, v2);
            return fabs(1 - d) <= Constants::EPSILON || fabs(1 + d) <= Constants::EPSILON;
        }

        static inline bool VecAreParallelUnitArgs(const int n, const double v1[], const double v2[]) {
            return fabs(1 - VecDotProduct(n, v1, v2)) <= Constants::EPSILON;
        }

        static inline bool VecAreAntiParallelUnitArgs(const int n, const double v1[], const double v2[]) {
            return fabs(1 + VecDotProduct(n, v1, v2)) <= Constants::EPSILON;
        }

        static inline int VecAlignmentUnitArgs(const int n, const double v1[], const double v2[]) {
            const double d = VecDotProduct(n, v1, v2);
            return fabs(1 - d) <= Constants::EPSILON ? Constants::PARALLEL : (fabs(1 + d) < Constants::EPSILON
                                                                              ? Constants::ANTI_PARALLEL
                                                                              : Constants::NEITHER_PARALLEL_NOR_ANTI_PARALLEL);
        }

        static inline double *MatAlloc(const int nrRows, const int nrCols) {
            return (double *) calloc(nrRows * nrCols, sizeof(double));
        }

        static inline void MatFree(double A[]) {
            if (A)
                free(A);
        }

        static inline double MatGetEntry(const int nrRows,
                                         const int nrCols,
                                         const double A[], const int row, const int col) {

            return A[row * nrCols + col];
        }

        static inline const double *MatGetRow(const int nrRows,
                                              const int nrCols,
                                              const double A[], const int row) {
            return &A[row * nrCols];
        }

        static inline double *MatGetRow(const int nrRows,
                                        const int nrCols,
                                        double A[], const int row) {
            return &A[row * nrCols];
        }

        static inline void MatGetCol(const int nrRows,
                                     const int nrCols,
                                     const double A[], const int col, double colEntries[]) {
            for (int i = 0; i < nrRows; ++i)
                colEntries[i] = A[i * nrCols + col];
        }

        static inline void MatSetEntry(const int nrRows,
                                       const int nrCols,
                                       double A[], const int row, const int col, const double val) {
            A[row * nrCols + col] = val;
        }

        static inline void MatSetRow(const int nrRows,
                                     const int nrCols,
                                     double A[], const int row, const double vals[]) {
            memcpy(MatGetRow(nrRows, nrCols, A, row), vals, nrCols * sizeof(double));
        }

        static inline void MatSetCol(const int nrRows,
                                     const int nrCols,
                                     double A[], const int col, const double vals[]) {
            for (int i = 0; i < nrRows; ++i)
                A[i * nrCols + col] = vals[i];
        }

        static inline double MatRowDotVec(const int nrRows,
                                          const int nrCols,
                                          const double A[],
                                          const int row,
                                          const double v[]) {
            return VecDotProduct(nrCols, MatGetRow(nrRows, nrCols, A, row), v);
        }

        static inline double MatColDotVec(const int nrRows,
                                          const int nrCols,
                                          const double A[],
                                          const int col,
                                          const double v[]) {
            double d = 0;
            for (int i = 0; i < nrRows; ++i)
                d += A[i * nrCols + col] * v[i];
            return d;
        }

        static inline double MatRowDotRow(const int nrRows1,
                                          const int nrCols1,
                                          const double A1[],
                                          const int nrRows2,
                                          const int nrCols2,
                                          const double A2[],
                                          const int row1, const int row2) {
            assert(nrCols1 == nrCols2);
            return VecDotProduct(nrCols1,
                                 MatGetRow(nrRows1, nrCols1, A1, row1),
                                 MatGetRow(nrRows2, nrCols2, A2, row2));
        }

        static inline double MatRowDotCol(const int nrRows1,
                                          const int nrCols1,
                                          const double A1[],
                                          const int nrRows2,
                                          const int nrCols2,
                                          const double A2[],
                                          const int row1, const int col2) {
            assert(nrCols1 == nrRows2);
            const double *a1 = MatGetRow(nrRows1, nrCols1, A1, row1);
            double d = 0;
            for (int i = 0; i < nrCols1; ++i)
                d += a1[i] * A2[i * nrCols2 + col2];
            return d;
        }

        static inline double MatColDotRow(const int nrRows1,
                                          const int nrCols1,
                                          const double A1[],
                                          const int nrRows2,
                                          const int nrCols2,
                                          const double A2[],
                                          const int col1, const int row2) {
            return MatRowDotCol(nrRows2, nrCols2, A2, nrRows1, nrCols1, A1, row2, col1);
        }

        static inline double MatColDotCol(const int nrRows1,
                                          const int nrCols1,
                                          const double A1[],
                                          const int nrRows2,
                                          const int nrCols2,
                                          const double A2[],
                                          const int col1, const int col2) {
            assert(nrRows1 == nrRows2);
            double d = 0;
            for (int i = 0; i < nrRows1; ++i)
                d += A1[i * nrCols1 + col1] * A2[i * nrCols2 + col2];
            return d;
        }

        static inline void MatTranspose(const int nrRows,
                                        const int nrCols,
                                        const double A[], double Atransp[]) {
            for (int i = 0; i < nrRows; ++i) {
                const double *a = &A[i * nrCols];
                for (int j = 0; j < nrCols; ++j)
                    Atransp[j * nrRows + i] = a[j];
            }
        }

        static inline void MatAdd(const int nrRows,
                                  const int nrCols,
                                  const double A1[], const double A2[], double A[]) {
            VecAdd(nrRows * nrCols, A1, A2, A);
        }

        static inline void MatSubtract(const int nrRows,
                                       const int nrCols,
                                       const double A1[], const double A2[], double A[]) {
            VecSubtract(nrRows * nrCols, A1, A2, A);
        }

        static inline void MatScale(const int nrRows,
                                    const int nrCols,
                                    const double A[], const double s, double Ascaled[]) {
            VecScale(nrRows * nrCols, A, s, Ascaled);
        }

        static inline void MatLinearInterpolation(const int nrRows,
                                                  const int nrCols,
                                                  const double A1[], const double t1,
                                                  const double A2[], const double t2,
                                                  double A[]) {
            VecLinearInterpolation(nrRows * nrCols, A1, t1, A2, t2, A);
        }

        static inline void MatMultVec(const int nrRows,
                                      const int nrCols,
                                      const double A[], const double v[], double vnew[]) {
            for (int i = 0; i < nrRows; ++i)
                vnew[i] = VecDotProduct(nrCols, &A[i * nrCols], v);
        }

        static inline void MatMultMat(const int nrRows1,
                                      const int nrCols1,
                                      const double A1[],
                                      const int nrRows2,
                                      const int nrCols2,
                                      const double A2[],
                                      double A[]) {
            assert(nrCols1 == nrRows2);
            for (int i = 0; i < nrRows1; ++i)
                for (int j = 0; j < nrCols2; ++j)
                    A[i * nrCols2 + j] = MatRowDotCol(nrRows1, nrCols1, A1,
                                                      nrRows2, nrCols2, A2, i, j);
        }

        static inline void TransposeOfMatMultMat(const int nrRows1,
                                                 const int nrCols1,
                                                 const double A1[],
                                                 const int nrRows2,
                                                 const int nrCols2,
                                                 const double A2[],
                                                 double A[]) {
            assert(nrCols1 == nrRows2);
            for (int i = 0; i < nrRows1; ++i)
                for (int j = 0; j < nrCols2; ++j)
                    A[j * nrRows1 + i] = MatRowDotCol(nrRows1, nrCols1, A1,
                                                      nrRows2, nrCols2, A2, i, j);
        }

        static inline void MatMultMatTranspose(const int nrRows1,
                                               const int nrCols1,
                                               const double A1[],
                                               const int nrRows2,
                                               const int nrCols2,
                                               const double A2[],
                                               double A[]) {
            assert(nrCols1 == nrCols2);
            for (int i = 0; i < nrRows1; ++i)
                for (int j = 0; j < nrRows2; ++j)
                    A[i * nrRows2 + j] = MatRowDotRow(nrRows1, nrCols1, A1,
                                                      nrRows2, nrCols2, A2, i, j);
        }

        static inline void TransposeOfMatMultMatTranspose(const int nrRows1,
                                                          const int nrCols1,
                                                          const double A1[],
                                                          const int nrRows2,
                                                          const int nrCols2,
                                                          const double A2[],
                                                          double A[]) {
            assert(nrCols1 == nrCols2);
            for (int i = 0; i < nrRows1; ++i)
                for (int j = 0; j < nrRows2; ++j)
                    A[j * nrRows1 + i] = MatRowDotRow(nrRows1, nrCols1, A1,
                                                      nrRows2, nrCols2, A2, i, j);
        }

        static inline void MatTransposeMultMat(const int nrRows1,
                                               const int nrCols1,
                                               const double A1[],
                                               const int nrRows2,
                                               const int nrCols2,
                                               const double A2[],
                                               double A[]) {
            assert(nrRows1 == nrRows2);
            for (int i = 0; i < nrCols1; ++i)
                for (int j = 0; j < nrCols2; ++j)
                    A[i * nrCols2 + j] = MatColDotCol(nrRows1, nrCols1, A1,
                                                      nrRows2, nrCols2, A2, i, j);
        }

        static inline void TransposeOfMatTransposeMultMat(const int nrRows1,
                                                          const int nrCols1,
                                                          const double A1[],
                                                          const int nrRows2,
                                                          const int nrCols2,
                                                          const double A2[],
                                                          double A[]) {
            assert(nrRows1 == nrRows2);
            for (int i = 0; i < nrCols1; ++i)
                for (int j = 0; j < nrCols2; ++j)
                    A[j * nrCols1 + i] = MatColDotCol(nrRows1, nrCols1, A1,
                                                      nrRows2, nrCols2, A2, i, j);
        }

        static inline void MatTransposeMultMatTranspose(const int nrRows1,
                                                        const int nrCols1,
                                                        const double A1[],
                                                        const int nrRows2,
                                                        const int nrCols2,
                                                        const double A2[],
                                                        double A[]) {
            assert(nrRows1 == nrCols2);
            for (int i = 0; i < nrCols1; ++i)
                for (int j = 0; j < nrRows2; ++j)
                    A[i * nrRows2 + j] = MatColDotRow(nrRows1, nrCols1, A1,
                                                      nrRows2, nrCols2, A2, i, j);
        }

        static inline void TransposeOfMatTransposeMultMatTranspose(const int nrRows1,
                                                                   const int nrCols1,
                                                                   const double A1[],
                                                                   const int nrRows2,
                                                                   const int nrCols2,
                                                                   const double A2[],
                                                                   double A[]) {
            assert(nrRows1 == nrCols2);
            for (int i = 0; i < nrCols1; ++i)
                for (int j = 0; j < nrRows2; ++j)
                    A[j * nrCols1 + i] = MatColDotRow(nrRows1, nrCols1, A1,
                                                      nrRows2, nrCols2, A2, i, j);
        }

        bool SolveLinearSystemMatMultVec(const int n,
                                         double Atransp[],
                                         double b[]);

//Utransp: nrRows x nrRows
//Sigma: max(nrRows, nrCols)
//V: nrCols x nrCols
        bool SVD(const int nrRowsA,
                 const int nrColsA,
                 double Atransp[], double Utransp[], double Sigma[], double V[]);

        static inline void PointSampleUniform(const int n, double p[], const double bbox[]) {
            for (int i = 0; i < n; ++i)
                p[i] = RandomUniformReal(bbox[i], bbox[i + n]);
        }
    }
}

#endif
