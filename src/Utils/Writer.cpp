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
#include "Utils/Writer.hpp"
#include "Utils/Misc.hpp"
#include <fstream>

namespace Antipatrea
{
    Status Writer::PrintToFile(const char fname[],
					  std::ios_base::openmode mode) const
    {
	std::ofstream out(fname, mode);
	if(!out.is_open())
	    return STATUS_ERROR;
	OutputFormat(out);
	Print(out);
	out.close();

	return STATUS_OK;
	
    }
}



