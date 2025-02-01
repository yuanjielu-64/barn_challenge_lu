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

#ifndef Antipatrea__Curve_HPP_
#define Antipatrea__Curve_HPP_

#include "Utils/Misc.hpp"
#include "Utils/Polygon2D.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/Algebra3D.hpp"
#include <vector>

namespace Antipatrea
{
class Curve
{
  public:
	Curve(void)
	{
	}

	virtual ~Curve(void)
	{
	}

	virtual int GetDim(void) const = 0;
	virtual double GetDuration(void) const = 0;

	virtual void GetFirstPoint(double p[]) const
	{
		GetPointAtTime(0, p);
	}
	virtual void GetLastPoint(double p[]) const
	{
		GetPointAtTime(GetDuration(), p);
	}

	virtual Curve *Clone(void) const = 0;
	virtual void GetPointAtTime(const double t, double p[]) const = 0;
	virtual Curve *Subcurve(const double tstart, const double tend) const = 0;
	virtual void Subcurves(const int n, const double gaps[], std::vector<Curve *> & curves) const;
	virtual void ToPolygon(const int n, const double thick, Polygon2D & poly);
	virtual void Draw(const int nrPts) const;
};

class TransformedCurve : public Curve
{
  public:
	TransformedCurve(void) : Curve(),
							 m_curve(NULL)
	{
		m_scaling[0] = m_scaling[1] = m_scaling[2] = 1;
		Algebra3D::IdentityAsTransRot(m_TR);
	}

	virtual ~TransformedCurve(void)
	{
		if (m_curve)
			delete m_curve;
	}

	virtual int GetDim(void) const
	{
		return m_curve->GetDim();
	}

	virtual double GetDuration(void) const
	{
		return m_curve->GetDuration();
	}

	virtual void GetFirstPoint(double p[]) const
	{
		m_curve->GetPointAtTime(0, p);
		Transform(p);
	}

	virtual void GetLastPoint(double p[]) const
	{
		m_curve->GetPointAtTime(GetDuration(), p);
		Transform(p);
	}

	virtual Curve *Clone(void) const
	{
		TransformedCurve *tc = new TransformedCurve();
		tc->m_curve = m_curve->Clone();
		tc->m_scaling[0] = m_scaling[0];
		tc->m_scaling[1] = m_scaling[1];
		tc->m_scaling[2] = m_scaling[2];
		Algebra3D::TransRotAsTransRot(m_TR, tc->m_TR);
		return tc;
	}

	virtual void GetPointAtTime(const double t, double p[]) const
	{
		m_curve->GetPointAtTime(t, p);
		Transform(p);
	}

	virtual Curve *Subcurve(const double tstart, const double tend) const
	{
		TransformedCurve *tc = new TransformedCurve();
		tc->m_curve = m_curve->Subcurve(tstart, tend);
		tc->m_scaling[0] = m_scaling[0];
		tc->m_scaling[1] = m_scaling[1];
		tc->m_scaling[2] = m_scaling[2];
		Algebra3D::TransRotAsTransRot(m_TR, tc->m_TR);
		return tc;
	}

	Curve *m_curve;
	double m_scaling[3];
	double m_TR[Algebra3D::TransRot_NR_ENTRIES];

  protected:
	virtual void Transform(double p[]) const;
};

class MultiCurve : public Curve
{
  public:
	MultiCurve(void) : Curve(),
					   m_duration(0.0)
	{
	}

	virtual ~MultiCurve(void);

	virtual void AddCurve(Curve *const curve);
	virtual void RemoveCurve(const int i);
	virtual int GetNrCurves(void) const
	{
		return m_curves.size();
	}
	virtual const Curve *GetCurve(const int i) const
	{
		return m_curves[i];
	}

	virtual Curve *GetCurve(const int i)
	{
		return m_curves[i];
	}

	////////////////
	virtual int GetDim(void) const
	{
		return m_curves[0]->GetDim();
	}

	virtual double GetDuration(void) const
	{
		return m_duration;
	}

	virtual void GetFirstPoint(double p[]) const
	{
		return m_curves[0]->GetFirstPoint(p);
	}

	virtual void GetLastPoint(double p[]) const
	{
		return m_curves.back()->GetLastPoint(p);
	}

	virtual Curve *Clone(void) const;
	virtual void GetPointAtTime(const double t, double p[]) const;
	virtual Curve *Subcurve(const double tstart, const double tend) const;
	virtual void Draw(const int nrPts) const;

  protected:
	std::vector<Curve *> m_curves;
	double m_duration;
};

class SegmentCurve : public Curve
{
  public:
	SegmentCurve(void) : Curve(),
						 m_dim(0),
						 m_duration(1.0)
	{
	}

	virtual ~SegmentCurve(void)
	{
	}

	virtual void SetDim(const int dim)
	{
		m_dim = dim;
		m_pts.resize(2 * m_dim);
	}

	virtual void SetDuration(const double d)
	{
		m_duration = d;
	}

	virtual void SetFirstPoint(const double p[])
	{
		CopyArray<double>(&m_pts[0], m_dim, p);
	}

	virtual void SetLastPoint(const double p[])
	{
		CopyArray<double>(&m_pts[m_dim], m_dim, p);
	}

	/////////////////
	virtual int GetDim(void) const
	{
		return m_dim;
	}

	virtual double GetDuration(void) const
	{
		return m_duration;
	}

	virtual void GetFirstPoint(double p[]) const
	{
		CopyArray<double>(p, m_dim, &m_pts[0]);
	}

	virtual void GetLastPoint(double p[]) const
	{
		CopyArray<double>(p, m_dim, &m_pts[m_dim]);
	}

	virtual Curve *Clone(void) const
	{
		SegmentCurve *sc = new SegmentCurve();
		sc->SetDim(m_dim);
		sc->SetFirstPoint(&m_pts[0]);
		sc->SetLastPoint(&m_pts[m_dim]);

		return sc;
	}

	virtual void GetPointAtTime(const double t, double p[]) const;
	virtual Curve *Subcurve(const double tstart, const double tend) const;
	virtual void Draw(const int nrPts) const;

  protected:
	int m_dim;
	double m_duration;
	std::vector<double> m_pts;
};

class ArcCurve : public Curve
{
  public:
	ArcCurve(void) : Curve(),
					 m_start(0),
					 m_duration(0),
					 m_radiusX(0),
					 m_radiusY(0)
	{
	}

	virtual ~ArcCurve(void)
	{
	}

	virtual void SetDuration(const double duration)
	{
		m_duration = duration;
	}

	virtual void SetStart(const double start)
	{
		m_start = start;
	}

	virtual void SetRadius(const double r)
	{
		m_radiusX = m_radiusY = r;
	}

	virtual void SetRadiusX(const double r)
	{
		m_radiusX = r;
	}

	virtual void SetRadiusY(const double r)
	{
		m_radiusY = r;
	}

	virtual void SetCenter(const double p[])
	{
		m_center[0] = p[0];
		m_center[1] = p[1];
	}

	virtual void SetCenter(const double x, const double y)
	{
		m_center[0] = x;
		m_center[1] = y;
	}

	//////////////
	virtual int GetDim(void) const
	{
		return 2;
	}

	virtual double GetDuration(void) const
	{
		return m_duration;
	}

	virtual Curve *Clone(void) const
	{
		ArcCurve *ac = new ArcCurve();
		ac->SetStart(m_start);
		ac->SetDuration(m_duration);
		ac->SetRadiusX(m_radiusX);
		ac->SetRadiusY(m_radiusY);
		ac->SetCenter(m_center);

		return ac;
	}

	virtual void GetPointAtTime(const double t, double p[]) const;
	virtual Curve *Subcurve(const double tstart, const double tend) const;

  protected:
	double m_start;
	double m_duration;
	double m_radiusX;
	double m_radiusY;
	double m_center[2];
};

class RaisedCurve : public Curve
{
  public:
	RaisedCurve(void) : Curve(),
						m_base(NULL),
						m_height(NULL)
	{
	}

	virtual ~RaisedCurve(void)
	{
		if (m_base)
			delete m_base;
		if (m_height)
			delete m_height;
	}

	virtual void SetBase(Curve *const b)
	{
		if (m_base != NULL && m_base != b)
			delete m_base;
		m_base = b;
	}

	virtual void SetHeight(Curve *const h)
	{
		if (m_height != NULL && m_height != h)
			delete m_height;
		m_height = h;
	}

	//////////////////
	virtual int GetDim(void) const
	{
		return 3;
	}

	virtual double GetDuration(void) const
	{
		return m_base->GetDuration();
	}

	virtual void GetFirstPoint(double p[]) const
	{
		m_base->GetFirstPoint(p);
		m_height->GetFirstPoint(&p[2]);
	}
	virtual void GetLastPoint(double p[]) const
	{
		m_base->GetLastPoint(p);
		m_height->GetLastPoint(&p[2]);
	}

	virtual Curve *Clone(void) const
	{
		RaisedCurve *rc = new RaisedCurve();
		rc->SetBase(m_base->Clone());
		rc->SetHeight(m_height->Clone());
		return rc;
	}

	void GetPointAtTime(const double t, double p[]) const
	{
		m_base->GetPointAtTime(t, p);
		m_height->GetPointAtTime(t, &p[2]);
	}

	virtual Curve *Subcurve(const double tstart, const double tend) const
	{
		RaisedCurve *rc = new RaisedCurve();
		rc->SetBase(m_base->Subcurve(tstart, tend));
		rc->SetHeight(m_height->Subcurve(tstart, tend));

		return rc;
	}

  protected:
	Curve *m_base;
	Curve *m_height;
};

void PolygonFromCurves(const Curve & curve1,
					   const Curve & curve2,
					   const int n,
					   Polygon2D & poly);
}

#endif
