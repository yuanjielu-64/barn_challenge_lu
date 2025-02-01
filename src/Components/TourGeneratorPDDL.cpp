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
#include "Components/TourGeneratorPDDL.hpp"
#include "Utils/Misc.hpp"
#include <fstream>

namespace Antipatrea
{
bool TourGeneratorPDDL::GenerateTour(Tour &tour)
{
    const int n = GetNrSites();

    if(n == 0)
	return false;
    
    if(n == 1)
    {
	tour.m_order.clear();
	tour.m_order.push_back(0);
	return FromOrderToTimes(tour);
	
    }
    
    std::ofstream out("tw_problem.pddl");

    out << "(define (problem pandora_task)" << std::endl
	<< "(:domain pandora)" << std::endl << std::endl
	<< "(:objects" << std::endl
	<< "   auv - vehicle" << std::endl << "   ";

    out << "wpVirtual ";    
    for(int i = 0; i < n; ++i)
	out << "wp" << i << " ";
    out << " - waypoint " << std::endl << "   ";
    for(int i = 1; i < n; ++i)
	out << "task" << i << " ";
    out << " - task " << std::endl << ")" << std::endl << std::endl;
    

    out << "(:init " << std::endl
	<< "   (at auv wpVirtual)" << std::endl << std::endl;
 
    for(int i = 1; i < n; ++i)
	out << "   (located task" << i << " wp" << i <<")" << std::endl;
    out << std::endl;
    for(int i = 1; i < n; ++i)
	out << "   (= (taskduration task" << i << ") 0)" << std::endl;
    out << std::endl;
    for(int i = 1; i < n; ++i)
	out << "   (todo task" << i << ")" << std::endl;
    out << std::endl;
    
    //intervals
    for(int i = 1; i < n; ++i)
	out << "   (at " << GetLowerBound(i) << " (tw_open task"<<i<<"))" << std::endl
	    << "   (at " << GetUpperBound(i) << " (not (tw_open task"<<i<<")))" << std::endl << std::endl;
        
    //connections
    out << "   (connected wpVirtual wp0) (= (travelduration wpVirtual wp0) " << GetStartTime() <<")" << std::endl;
    
    for(int i = 0; i < n; ++i)
	for(int j = 0; j < n; ++j)
	    if(i != j)
		out << "   (connected wp" << i << " wp" << j << ") (= (travelduration wp" << i << " wp" << j <<") " << GetDuration(i, j) << ")" << std::endl;

    out << std::endl << ")" << std::endl << std::endl; //close init
     
    //goal   
    out << "(:goal ";
    if(n > 1)
	out << "(and " << std::endl;
    for(int i = 1; i < (int) GetNrSites(); ++i)
	out << "   (completed task" << i << ")" << std::endl;
    if(n > 1)
	out << ")";
    out << ")" << std::endl << std::endl;
    

    //close parenthesis
    out << ")" << std::endl;
    
    out.close();

    //run PDDL planner
    system("rm plan.sol");
    system("rm plan.tasks");    
    system("./optic-clp -N tw_domain.pddl tw_problem.pddl");
    system("grep execute_task plan.sol > plan.tasks");

    //read solution
    std::ifstream in("plan.tasks");
    std::string s;

    tour.m_order.clear();    
    tour.m_order.push_back(0);
        
    while(in >> s)
    {
	//std::cout << "<" << s << ">" << std::endl;
	if(s.compare("(execute_task") == 0)
	{
	    in >> s >> s >> s;
	    const int gid = atoi(&s.c_str()[4]);
	    tour.m_order.push_back(gid);
	}
    }
    in.close();

    return tour.m_order.size() == n && FromOrderToTimes(tour);
    
    
}
}
