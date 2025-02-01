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
#include "Utils/GProperties.hpp"
#include "Utils/Logger.hpp"

namespace Antipatrea
{
    void GProperties::SetupFromParams(Params & params)
    {
	const int nrProps   = GetNrProperties();
	const int nrEntries = GetNrEntriesPerProperty();

	for(int i = 0; i < nrProps; ++i)
	{
	    auto data = params.GetData(GetPropertyName(i));
	    if(data && data->m_values.size() == nrEntries)
	    {
		auto vals = GetPropertyValues(i);
		for(int j = 0; j < nrEntries; ++j)
		    vals[j] = (float) StrToDouble(data->m_values[j]->c_str());
	    }
	}
    }
    
    Status GProperties::Read(std::istream & in)
    {
	const int n = m_values.size();
	double val;
	
	for(int i = 0; i < n; ++i)
	{
	    if(!(in >> val))
	    {
		Logger::m_out << "error Properties::Read ... failed to read value" << i << " out of " << n << " values " << std::endl;
		return STATUS_ERROR;
	    }
	    m_values[i] = val;
	}
	return STATUS_OK;
    }
    
    void GProperties::Print(std::ostream & out) const
    {
	const int n = m_values.size();
	for(int i = 0; i < n; ++i)
	{
	    out << m_values[i] << " ";
	    if((i + 1) % m_nrEntriesPerProperty == 0)
		out << std::endl;
	}
    }
}


