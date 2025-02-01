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

#ifndef Antipatrea__GMaterial_HPP_
#define Antipatrea__GMaterial_HPP_

#include "Utils/GProperties.hpp"
#include "Utils/Constants.hpp"

namespace Antipatrea
{
    class GMaterial : public GProperties
    {
    public:
	GMaterial(void) : GProperties()
	{
	    m_nrEntriesPerProperty = 4;
	    m_values.resize(5 * m_nrEntriesPerProperty);
	    SetObsidian();
	}
	
	virtual ~GMaterial(void)
	{
	}
	
	enum 
	    {
		AMBIENT     = 0,
		DIFFUSE     = 1,
		SPECULAR    = 2,
		EMISSIVE    = 3,
		SHININESS   = 4
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
	    case EMISSIVE:
		return Constants::KW_Emissive;
	    case SHININESS:
		return Constants::KW_Shininess;
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
	    else if(StrSameContent(name, Constants::KW_Emissive))
		return EMISSIVE;
	    else if(StrSameContent(name, Constants::KW_Shininess))
		return SHININESS;
	    else
		return Constants::ID_UNDEFINED;
	}

	virtual void SetAmbient(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetPropertyValues(AMBIENT, a, b, c, d);
	}
	
	virtual void SetDiffuse(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetPropertyValues(DIFFUSE, a, b, c, d);
	}
	
	virtual void SetSpecular(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetPropertyValues(SPECULAR, a, b, c, d);
	}
	
	virtual void SetEmissive(const float a, const float b, const float c, const float d = 1.0f)
	{
	    SetPropertyValues(EMISSIVE, a, b, c, d);
	}

	virtual void SetShininess(const float a)
	{
	    SetPropertyValues(SHININESS, a);
	}

	
	virtual void SetGold(void)
	{
	    SetAmbient(0.24725, 	0.1995, 	0.0745);
	    SetDiffuse(0.75164, 	0.60648, 	0.22648);
	    SetShininess(20.4);
	    SetEmissive(0, 0, 0, 1.0);
	    SetSpecular(1.0, 1.0, 1.0, 1.0);
	}
	
	virtual void SetPearl(void)
	{
	    SetAmbient(0.25, 	0.20725, 	0.20725);
	    SetDiffuse(1, 	0.829, 	0.829);
	    SetShininess(18.088);
	    SetEmissive(0, 0, 0, 1.0);
	    SetSpecular(1.0, 1.0, 1.0, 1.0);
	    

	}
	
	virtual void SetRuby(void)
	{
	    SetAmbient(0.1745,0.01175,0.01175);
	    SetDiffuse(0.61424,0.04136,0.04136);
	    SetSpecular(0.727811,0.626959,0.626959);
	    SetShininess(0.6*128);
	    
	}

	virtual void SetObsidian(void)
	{
	    SetAmbient(0.05375,0.05,0.06625);
	    SetSpecular(0.332741,0.328634,0.346435);
	    SetDiffuse(0.18275,0.17,0.22525);
	    SetShininess(0.6*128);
	}

	virtual void SetTurquoise(void)
	{
	    SetAmbient(0.1, 0.18725,0.1745);
	    SetDiffuse(0.396,0.74151,0.69102);
	    SetSpecular(0.297254,0.30829,0.306678);
	    SetShininess(0.1*128);
	}
	
	virtual void SetYellowPlastic(void)
	{
	    SetAmbient(0,0,0);
	    SetDiffuse(1,1,0.0);
	    SetSpecular(0.60,0.60,0.50);
	    SetShininess(10.25*128);
	}	
	
    };
}


#endif
    
    






