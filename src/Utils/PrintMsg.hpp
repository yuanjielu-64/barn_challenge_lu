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

#ifndef Antipatrea__PrintMsg_HPP_
#define Antipatrea__PrintMsg_HPP_

#include "Utils/Definitions.hpp"
#include <cstdio>
#include <cstdlib>

#define PRINT_ERROR 31
#define PRINT_WARNING 32
#define PRINT_INFO  34

#define BeginPrint(col) printf("%c[1;%dm%s (%s:%s:%d)\n", 27, col, (col == PRINT_ERROR ? "ERROR" : (col == PRINT_WARNING ? "WARNING":"INFO")), __FILE__, __FUNCTION__, __LINE__)
#define EndPrint(col) printf("%c[0m", 27)

#define PrintWarning(cmd) BeginPrint(PRINT_WARNING); cmd; EndPrint(PRINT_WARNING)
#define PrintError(cmd) BeginPrint(PRINT_ERROR); cmd; EndPrint(PRINT_ERROR)
#define PrintInfo(cmd) BeginPrint(PRINT_INFO); cmd; EndPrint(PRINT_INFO)

#define OnInputError(cmd) {BeginPrint(PRINT_ERROR); cmd; EndPrint(PRINT_ERROR);exit(0);}
    
    
    


#endif
