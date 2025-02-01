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
#include "Utils/Reader.hpp"
#include <fstream>
#include <string>
#include "Utils/Logger.hpp"

namespace Antipatrea {
    Status Reader::ReadDouble(std::istream &in, double &val) {
        std::string s;

        if ((in >> s)) {
            val = std::stod(s);
            return STATUS_OK;
        } else {
            Logger::m_out << "error Reader::ReadDouble : could not read double value" << std::endl;
            return STATUS_ERROR;
        }

    }


    Status Reader::ReadFromFile(const char fname[], std::ios_base::openmode mode) {
        std::ifstream in(fname, mode);

        if (!in.is_open()) {
            Logger::m_out << "... fail to load file" << fname <<  std::endl;
            return STATUS_ERROR;
        }

        auto status = Read(in);
        in.close();

        return status;
    }
}



