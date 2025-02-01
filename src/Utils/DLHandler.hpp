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

#ifndef Antipatrea__DLHandler_HPP_
#define Antipatrea__DLHandler_HPP_


namespace Antipatrea::DLHandler {
    void *GetSymbol(void *handle, const char name[]);

    void *GetSymbol(const char name[]);
}


#endif
