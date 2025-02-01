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

#ifndef Antipatrea__GLight_HPP_
#define Antipatrea__GLight_HPP_

#include "Utils/GProperties.hpp"
#include "Utils/Constants.hpp"

namespace Antipatrea
{
    class GLight : public GProperties
    {
    public:
	GLight(void) : GProperties()
	{
	    m_nrEntriesPerProperty = 4;
	    m_values.resize(4 * m_nrEntriesPerProperty);
	    
	    SetPropertyValues(AMBIENT, 0.2, 0.2, 0.2);
	    SetPropertyValues(DIFFUSE, 1.0, 1.0, 1.0);
	    SetPropertyValues(SPECULAR, 0.5, 0.5, 0.5);
	    SetPropertyValues(POSITION, 0.0, 0.0, 0.0);
	}
	
	virtual ~GLight(void)
	{
	}
	
	enum 
	    {
		AMBIENT = 0,
		DIFFUSE,
		SPECULAR,
		POSITION   
	    };

	
	virtual const char* GetPropertyName(const int i) const
	{
	    switch(i)
	    {
	    case AMBIENT:
		return Constants::KW_Ambient;
	    case DIFFUSE:
		return Constants::KW_Diffuse;
	    case SPECULAR:
		return Constants::KW_Specular;
	    case POSITION:
		return Constants::KW_Position;
	    default:
		return NULL;
	    };
	}
	
	virtual int GetPropertyIndex(const char name[]) const
	{
	    if(StrSameContent(name, Constants::KW_Ambient))
		return AMBIENT;
	    else if(StrSameContent(name, Constants::KW_Diffuse))
		return DIFFUSE;
	    else if(StrSameContent(name, Constants::KW_Specular))
		return SPECULAR;
	    else if(StrSameContent(name, Constants::KW_Position))
		return POSITION;
	    else
		return Constants::ID_UNDEFINED;
	}
	
	virtual void SetAmbient(const float a, const float b, const float c, const float d = 1.0)
	{
	    SetPropertyValues(AMBIENT, a, b, c, d);
	}
	virtual void SetDiffuse(const float a, const float b, const float c, const float d = 1.0)
	{
	    SetPropertyValues(DIFFUSE, a, b, c, d);
	}
	virtual void SetSpecular(const float a, const float b, const float c, const float d = 1.0)
	{
	    SetPropertyValues(SPECULAR, a, b, c, d);
	}
	virtual void SetPosition(const float a, const float b, const float c, const float d = 1.0)
	{
	    SetPropertyValues(POSITION, a, b, c, d);
	}	
    };
}


#endif
    
    






