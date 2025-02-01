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

#ifndef Antipatrea__Grid_HPP_
#define Antipatrea__Grid_HPP_

#include "Utils/Params.hpp"
#include <vector>

namespace Antipatrea {
    class Grid {
    public:
        Grid(void) {
            m_ndims = 0;
            m_cvol = 0;
            m_ncells = 0;
        }

        virtual ~Grid(void) {
        }

        virtual void SetupFromParams(Params &params);

        virtual void Setup2D(const int dimsX, const int dimsY,
                             const double minX, const double minY,
                             const double maxX, const double maxY) {
            const int dims[2] = {dimsX, dimsY};
            const double min[2] = {minX, minY};
            const double max[2] = {maxX, maxY};

            Setup(2, dims, min, max);
        }

        virtual void Setup3D(const int dimsX, const int dimsY, const int dimsZ,
                             const double minX, const double minY, const double minZ,
                             const double maxX, const double maxY, const double maxZ) {
            const int dims[3] = {dimsX, dimsY, dimsZ};
            const double min[3] = {minX, minY, minZ};
            const double max[3] = {maxX, maxY, maxZ};

            Setup(3, dims, min, max);
        }

        virtual void Setup(const int ndims,
                           const int dims[],
                           const double min[],
                           const double max[]);

        int GetNrCells(void) const {
            return m_ncells;
        }

        int GetNrDims(void) const {
            return m_dims.size();
        }

        const int *GetDims(void) const {
            return &(m_dims[0]);
        }

        const double *GetMin(void) const {
            return &(m_min[0]);
        }

        const double *GetMax(void) const {
            return &(m_max[0]);
        }

        const double *GetUnits(void) const {
            return &(m_units[0]);
        }

        double GetCellVolume(void) const {
            return m_cvol;
        }

        virtual int GetCellIdFromCoords(const int coords[]) const;

        virtual void GetCoordsFromCellId(const int id, int coords[]) const;

        virtual void GetCoords(const double p[], int coords[]) const;

        virtual int GetCellId(const double p[]) const;

        virtual void GetCellFromCoords(const int coords[],
                                       double min[],
                                       double max[]) const;

        virtual void GetCellFromCoords(const int coords[],
                                       double bbox[]) const {
            GetCellFromCoords(coords, &bbox[0], &bbox[m_ndims]);
        }

        virtual void GetCellFromId(const int id, double min[], double max[]) const;

        virtual void GetCellFromId(const int id, double bbox[]) const {
            GetCellFromId(id, &bbox[0], &bbox[m_ndims]);
        }

        virtual void GetCellCenterFromCoords(const int coords[], double c[]) const;

        virtual void GetCellCenterFromId(const int id, double c[]) const;

        virtual bool IsPointInside(const double p[]) const;

        virtual bool IsPointInsideCell(const int coords[], const double p[]) const;

        virtual void GetNeighbors(const int id, std::vector<int> &neighs);

        virtual void GetNeighbors(const int id, const int coords[], std::vector<int> &neighs);

        virtual void SamplePoint(double p[]) const;

    protected:
        static int GetCoord(const double x,
                            const double min,
                            const double max,
                            const double unit,
                            const int ndims) {
            const int c = ((int) ((x - min) / unit));

            return c < 0 ? 0 : (c > (ndims - 1) ? (ndims - 1) : c);
        }

        int m_ndims;
        std::vector<int> m_dims;
        std::vector<double> m_min;
        std::vector<double> m_max;
        std::vector<double> m_units;
        double m_cvol;
        int m_ncells;
    };
}

#endif
