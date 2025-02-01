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

#ifndef Antipatrea__GProperties_HPP_
#define Antipatrea__GProperties_HPP_

#include "Utils/Reader.hpp"
#include "Utils/Writer.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include <vector>

namespace Antipatrea
{
    class GProperties : public Reader,
		       public Writer
    {
    public:
	GProperties(void) : Reader(),
			    Writer(),
			    m_nrEntriesPerProperty(0)
	{
	}
	
	virtual ~GProperties(void)
	{
	}

	virtual int GetNrProperties(void) const
	{
	    return m_values.size() / GetNrEntriesPerProperty();
	}

	virtual int GetNrEntriesPerProperty(void) const
	{
	    return m_nrEntriesPerProperty;
	}
			
	virtual const float* GetPropertyValues(const int i) const
	{
	    return &m_values[i * GetNrEntriesPerProperty()];
	}

	virtual float* GetPropertyValues(const int i)
	{
	    return &m_values[i * GetNrEntriesPerProperty()];
	}
	
	virtual const char* GetPropertyName(const int i) const = 0;

	virtual int GetPropertyIndex(const char name[]) const = 0;
	
	
	virtual void SetPropertyValues(const int i, const float vals[])
	{
	    CopyArray<float>(GetPropertyValues(i), GetNrEntriesPerProperty(), vals);
	}

	
	virtual void SetPropertyValues(const int i,
				       const float a = 0.0f,
				       const float b = 0.0f,
				       const float c = 0.0f,
				       const float d = 1.0f)
	{

	    float       *vals  = GetPropertyValues(i);
	    const int    m     = std::min(4, GetNrEntriesPerProperty());
	    const float  use[] = {a, b, c, d};
	    
	    for(int i = 0; i < m; ++i)
		vals[i] = use[i];
	    
	}
	
	
	virtual void CopyFrom(const GProperties & props)
	{
	    m_values.assign(props.m_values.begin(), props.m_values.end());
	    m_nrEntriesPerProperty = props.m_nrEntriesPerProperty;
	}

	
	virtual void SetupFromParams(Params & params);
	
	virtual Status Read(std::istream & in);

	virtual void Print(std::ostream & out) const;
	

    protected:
	std::vector<float> m_values;
	int                m_nrEntriesPerProperty;
    };
}


#endif
    
    






