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

#ifndef Antipatrea__Definitions_HPP_
#define Antipatrea__Definitions_HPP_

#include <cstdlib>

namespace Antipatrea {
    typedef int Id;

    enum Status {
        STATUS_OK = 0,
        STATUS_WARNING = 1,
        STATUS_ERROR = 2
    };
}

/**
 *@cond
 */

/**
 *@brief Recognize operating system: Linux 
 */
#if defined(linux) || defined(__linux__)
#define OS_LINUX
#endif

/**
 *@brief Recognize operating system: Microsoft Windows
 */
#if defined(WINDOWS) || defined(_WIN32) || defined(__WIN32__)
#define OS_WINDOWS
#endif

/**
 *@brief Recognize operating system: Macintosh OS
 */
#ifdef __APPLE__
#define OS_MAC
#endif

/**
 *@brief Recognize operating system: SunOS/Solaris
 */
#if defined(sun) || defined(__sun)
#define OS_SUNOS
#endif

/**
 *@brief Recognize compiler: GNU C/C++
 */
#ifdef __GNUC__
#define COMPILER_GNU
#endif

/**
 *@brief Recognize compiler: PGI
 */
#ifdef __PGI
#define COMPILER_PGI
#endif

/**
 *@brief Recognize compiler: Microsoft Visual Studio 
 */
#ifdef _MSC_VER
#define COMPILER_VISUAL_STUDIO
#endif

/**
 *@brief Make Visual Studio happy: avoid warnings/errors about
 *       <em>strdup</em> and <em>inline</em> 
 */
#ifdef COMPILER_VISUAL_STUDIO
#define strdup _strdup
#define inline __inline
#endif

/**
 *@endcond
 */

/**
 *@brief Include appropriate header files when compiling code in debug mode 
 */
#if DEBUG
#include <cassert>
#endif

#define ClassContainer3(ContainerName, ClassName, m_variable)   \
    class ContainerName##Container                              \
    {                                                           \
      public:                                                   \
        ContainerName##Container(void) : m_variable(NULL)       \
        {                                                       \
        }                                                       \
        virtual ~ContainerName##Container(void)                 \
        {                                                       \
        }                                                       \
        virtual ClassName *Get##ContainerName(void)             \
        {                                                       \
            return m_variable;                                  \
        }                                                       \
        virtual const ClassName *Get##ContainerName(void) const \
        {                                                       \
            return m_variable;                                  \
        }                                                       \
        virtual void Set##ContainerName(ClassName *const other) \
        {                                                       \
            m_variable = other;                                 \
        }                                                       \
                                                                \
      protected:                                                \
        ClassName *m_variable;                                  \
    }

#define ClassContainer(ContainerName, m_variable) ClassContainer3(ContainerName, ContainerName, m_variable)


#endif
