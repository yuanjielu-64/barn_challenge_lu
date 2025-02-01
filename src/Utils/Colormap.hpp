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

#ifndef Antipatrea__Colormap_HPP_
#define Antipatrea__Colormap_HPP_

#include "Utils/Reader.hpp"
#include "Utils/Constants.hpp"
#include <vector>


namespace Antipatrea {
    class Colormap : public Reader {
    public:
        Colormap(void) : Reader() {
            std::string package_path = ros::package::getPath(Constants::KW_ProjectName_);
            auto COLORMAP_FILE = package_path + "/" + Constants::COLORMAP_FILE;
            ReadFromFile(COLORMAP_FILE.c_str());
        }

        virtual ~Colormap(void) {
        }

        virtual double GetRed(const double val) const {
            return GetColor(m_red.size(), &m_red[0], val);
        }

        virtual double GetGreen(const double val) const {
            return GetColor(m_green.size(), &m_green[0], val);
        }

        virtual double GetBlue(const double val) const {
            return GetColor(m_blue.size(), &m_blue[0], val);
        }

        virtual void GetRGB(const double val, double rgb[]) const {
            rgb[0] = GetRed(val);
            rgb[1] = GetGreen(val);
            rgb[2] = GetBlue(val);
        }

        virtual Status Read(std::istream &in);

        static Colormap *GetSingleton(void) {
            return m_singleton;
        }

    protected:
        static Colormap *m_singleton;

        double GetColor(const int n, const double vals[], const double val) const;

        std::vector<double> m_red;
        std::vector<double> m_green;
        std::vector<double> m_blue;
    };
}

#endif
