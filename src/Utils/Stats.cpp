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
#include "Utils/Stats.hpp"
#include "Utils/Misc.hpp"
#include <string>
#include <iomanip>

namespace Antipatrea
{
    
    Stats* Stats::m_singleton = new Stats();

    double Stats::GetValue(const char id[]) 
    {
	auto iter = m_values.find(id);
	if(iter != m_values.end())
	    return iter->second;
	return 0.0;
    }

    void Stats::SetValue(const char id[], const double t)
    {
	auto iter = m_values.find(id);
	if(iter != m_values.end())
	    iter->second = t;
	else
	    m_values.insert(std::make_pair(id, t));
    }
    
    double Stats::AddValue(const char id[], const double t)
    {
	auto iter = m_values.find(id);
	if(iter != m_values.end())
	{
	    iter->second += t;
	    return iter->second;
	}
	else
	{
	    m_values.insert(std::make_pair(id, t));
	    return t;
	}
    }
    
    void Stats::Print(std::ostream & out) const
    {
	for(auto iter = m_values.begin(); iter != m_values.end(); iter++)
	    out << std::setw(30) << std::left << iter->first.c_str() << " " << iter->second << std::endl;
    }


    Status Stats::Read(std::istream &in)
    {
	std::string id;
	double      val;

	while(in >> id >> val)
	    AddValue(id.c_str(), val);
	return STATUS_OK;
    }
    
    
}



