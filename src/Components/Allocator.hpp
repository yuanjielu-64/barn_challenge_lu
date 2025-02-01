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

#ifndef Antipatrea__Allocator_HPP_
#define Antipatrea__Allocator_HPP_

#include "Components/Component.hpp"
#include "Components/Constants.hpp"
#include "Utils/Constants.hpp"
#include <istream>
#include <ostream>
#include <cstring>
#include <algorithm>

namespace Antipatrea
{

class Allocator : public Component
{
  public:
    Allocator(const int dim = 0) : Component(),
                                   m_dim(dim)
    {
    }

    virtual ~Allocator(void)
    {
    }

    virtual void Info(void) const
    {
        Component::Info();
        Logger::m_out << "Dim = " << GetDim() << std::endl;
    }

    virtual bool CheckSetup(void) const
    {
        return Component::CheckSetup() &&
               GetDim() > 0;
    }

    virtual void SetupFromParams(Params &params)
    {
        Component::SetupFromParams(params);
        SetDim(params.GetValueAsInt(Constants::KW_Dim, GetDim()));
    }

    /**
	 *@brief Set dimension.
	 *
	 *@remarks
	 * - The dimension should be set once at the beginning before any array is allocated.
	 */
    virtual void SetDim(const int dim)
    {
        m_dim = dim;
    }

    /**
	 *@brief Get the dimension.
	 */
    virtual int GetDim(void) const
    {
        return m_dim;
    }

    /**
	 *@brief Allocate memory for the array.
	 */
    virtual double *New(void) const
    {
        return new double[GetDim()];
    }

    /**
	 *@brief Delete the memory allocated to the array.
	 */
    virtual void Delete(double values[]) const
    {
        if (values)
            delete[] values;
    }

    /**
	 *@brief Copy the values from <tt>src</tt> to <tt>dest</tt>.
	 */
    virtual void Copy(double dest[], const double src[]) const
    {
        memcpy(dest, src, GetDim() * sizeof(double));
    }

    /**
	 *@brief Return a copy of the array.
	 */
    virtual double *Copy(const double values[]) const
    {
        double *dest = New();
        Copy(dest, values);
        return dest;
    }

    /**
	 *@brief Set every entry in the array to the specified value.
	 */
    virtual void Fill(double values[], const double val)
    {
        std::fill(values, values + GetDim(), val);
    }

    virtual void Print(std::ostream &out, const double values[]) const;

    virtual Status Read(std::istream &in, double values[]) const;

  protected:
    /**
	 *@brief Array dimension.
	 */
    int m_dim;
};

ClassContainer(Allocator, m_allocator);
}

#endif
