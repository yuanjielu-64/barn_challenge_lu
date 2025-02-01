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
#include "Components/Allocator.hpp"

namespace Antipatrea
{
void Allocator::Print(std::ostream &out, const double values[]) const
{
    const int d = GetDim();
    for (int i = 0; i < d; ++i)
        out << values[i] << " ";
    out << std::endl;
}

Status Allocator::Read(std::istream &in, double values[]) const
{
    const int d = GetDim();
    for (int i = 0; i < d; ++i)
        if (!(in >> values[i]))
        {
            Logger::m_out << "error Allocator::Read ... expecting to read " << d << " values instead of " << i << std::endl;

            return STATUS_ERROR;
        }
    return STATUS_OK;
}
}
