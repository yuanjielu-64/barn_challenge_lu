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

#ifndef Antipatrea__GIllumination_HPP_
#define Antipatrea__GIllumination_HPP_

#include "Utils/GLight.hpp"
#include "Utils/Params.hpp"
#include "Utils/Misc.hpp"
#include <vector>

namespace Antipatrea
{
    class GIllumination : public GProperties
    {
    public:
	GIllumination(void) : GProperties()
	{
	    m_nrEntriesPerProperty = 4;
	    m_values.resize(2 * m_nrEntriesPerProperty);

	    SetPropertyValues(AMBIENT, 0.2, 0.2, 0.2);
	    SetPropertyValues(DIFFUSE, 1.0, 1.0, 1.0);
	    AddDefaultLights();
	}
	
	virtual ~GIllumination(void)
	{
	    RemoveLights();
	}
	
	enum
	    {
		AMBIENT = 0,
		DIFFUSE   
	    };

	virtual const char* GetPropertyName(const int i) const
	{
	    switch(i)
	    {
	    case AMBIENT:
		return Constants::KW_Ambient;
	    case DIFFUSE:
		return Constants::KW_Diffuse;
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
	    else
		return Constants::ID_UNDEFINED;
	}

	void SetAmbient(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetPropertyValues(AMBIENT, a, b, c, d);
	}
	
	void SetDiffuse(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetPropertyValues(DIFFUSE, a, b, c, d);
	}

	void CopyFrom(const GIllumination & gIllum);
	
	int GetNrLights(void) const
	{
	    return m_lights.size();
	}
	
	GLight* GetLight(const int i)
	{
	    return m_lights[i];
	}
	
	const GLight* GetLight(const int i) const
	{
	    return m_lights[i];
	}

	virtual void AddLight(GLight * const light)
	{
	    m_lights.push_back(light);
	}

	virtual void AddDefaultLights(void);
	
	virtual void RemoveLights(void)
	{
	    DeleteItems<GLight*>(m_lights);
	    m_lights.clear();
	}

	virtual void SetupFromParams(Params & params);

    protected:
	std::vector<GLight*> m_lights;
    };
}


#endif
    
    






