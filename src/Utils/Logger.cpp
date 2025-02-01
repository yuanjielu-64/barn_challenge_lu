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
#include "Utils/Logger.hpp"
#include <iostream>

namespace Antipatrea
{
    
    int Logger::m_verbosity = 1;

    std::ostream& Logger::m_out = std::cout;
    
}
