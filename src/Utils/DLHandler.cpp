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
#include "Utils/Definitions.hpp"
#include "Utils/DLHandler.hpp"

#if defined OS_WINDOWS
#include <windows.h>
#else

#include <dlfcn.h>

#endif


namespace Antipatrea {
    namespace DLHandler {
        void *GetSymbol(void *handle, const char name[]) {
#if defined OS_WINDOWS
            return GetProcAddress((HMODULE) handle, name);
#else
            return dlsym(handle, (char *) name);
#endif
        }


        void *GetSymbol(const char name[]) {

#if defined OS_WINDOWS
            return GetSymbol(NULL, name);
#else
            return GetSymbol(RTLD_DEFAULT, name);
#endif
        }

    }
}
