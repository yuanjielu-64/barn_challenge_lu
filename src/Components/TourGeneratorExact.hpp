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

#ifndef Antipatrea__TourGeneratorExact_HPP_
#define Antipatrea__TourGeneratorExact_HPP_

#include "Components/TourGenerator.hpp"

namespace Antipatrea
{
    
    class TourGeneratorExact : public TourGenerator
    {
    public:
	TourGeneratorExact(const bool normalOrder) : TourGenerator(),
				   m_order1(normalOrder)
	{
	}
	
	virtual ~TourGeneratorExact(void)
	{
	}

	virtual void SetupFromParams(Params &params)
	{
	    TourGenerator::SetupFromParams(params);
	    if(params.GetValueAsBool("UseReverseOrder",  false))
		m_order1 = false;
	    else
		m_order1 = true;
	    
	       
		
	}
	

	virtual bool GenerateTour(Tour &tour);

	

	bool m_order1;
	
    };
}

#endif
