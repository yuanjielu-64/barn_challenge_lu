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

#ifndef Antipatrea__Writer_HPP_
#define Antipatrea__Writer_HPP_

#include "Utils/Definitions.hpp"
#include <ostream>


namespace Antipatrea
{
    /**
     *@brief Interface for writing to an output stream.
     */ 
    class Writer
    {
    public:
	Writer(void) 
	{
	}
	
	virtual ~Writer(void)
	{
	}
	
	/**
	 *@brief Open a file for writing, write the content, and then close the file.
	 *
	 *@remarks
	 * - The file is opened in ios_base::out mode by default.
	 */
	virtual Status PrintToFile(const char fname[],
					      std::ios_base::openmode mode = std::ios_base::out) const;
	
	/**
	 *@brief Write to the output stream.
	 */
	virtual void Print(std::ostream & out) const = 0;
    };
    
    static inline std::ostream& operator<<(std::ostream& out, const Writer& w)  
    {  
        w.Print(out);
        return out;
    }  
    
}

#endif



