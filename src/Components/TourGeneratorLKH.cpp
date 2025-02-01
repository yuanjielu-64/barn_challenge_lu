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
#include "Components/TourGeneratorLKH.hpp"
#include "Utils/Misc.hpp"
#include <fstream>

namespace Antipatrea {
    bool TourGeneratorLKH::GenerateTour(Tour &tour) {
        const double undefval = 100000000;
        std::vector<int> a = tour.m_order;
        const int n = GetNrSites();

        if (n == 0)
            return false;

        if (n == 1) {
            tour.m_order.clear();
            tour.m_order.push_back(0);
            return FromOrderToTimes(tour);

        }

        std::ofstream out("LKHs/LKHprob.txt");

        out
                << "NAME: atsp" << std::endl
                << "TYPE: ATSP" << std::endl
                << "DIMENSION: " << (n + 1) << std::endl
                << "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl
                << "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl
                << "EDGE_WEIGHT_SECTION" << std::endl;

        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (i == j)
                    out << "0 ";
                else if (j == 0)
                    out << undefval << " ";
                else
                    out << GetDuration(i, j) << " ";
            }

            if (i == 0)
                out << undefval << std::endl;
            else
                out << "0" << std::endl;
        }
        out << " 0 ";
        for (int i = 0; i < n; ++i)
            out << undefval << " ";
        out << std::endl << "EOF" << std::endl;

        out.close();

        //run LKH planner
        system("LKHs/./LKH LKHs/LKHparams.txt > LKHs/LKHlog.txt");

        //read solution
        std::ifstream in("LKHs/LKHsol.txt");
        std::string s;

        tour.m_order.clear();
        while (in >> s)
            if (s.compare("TOUR_SECTION") == 0)
                break;
        while (in >> s) {
            auto val = std::stoi(s);
            if (val < 0 || val == (n + 1))
                break;
            tour.m_order.push_back(val - 1);
        }

        return tour.m_order[0] == 0 && tour.m_order.size() == n && FromOrderToTimes(tour);


    }
}
