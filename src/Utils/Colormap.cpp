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
#include "Utils/Colormap.hpp"

namespace Antipatrea {
    Colormap *Colormap::m_singleton = new Colormap();

    double Colormap::GetColor(const int n, const double vals[], const double val) const {

/*
t = (val - start) / range = (val - pos * range) / range = val / range - pos = val * (n-1) - pos
 */
        const int pos = val * (n - 1);

        if (pos < 0)
            return vals[0];
        else if (pos >= n - 1)
            return vals[n - 1];
        else {
            const double t = val * (n - 1.0) - pos;
            return (1 - t) * vals[pos] + t * vals[pos + 1];
        }
    }

    Status Colormap::Read(std::istream &in) {
        double r;
        double g;
        double b;

        m_red.clear();
        m_green.clear();
        m_blue.clear();

        while (in >> r >> g >> b) {
            m_red.push_back(r);
            m_green.push_back(g);
            m_blue.push_back(b);
        }
        return STATUS_OK;
    }

}


    
    







