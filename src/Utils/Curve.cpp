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
#include "Utils/Curve.hpp"
#include "Utils/Algebra2D.hpp"
#include "Utils/GDraw.hpp"

namespace Antipatrea
{
void Curve::Subcurves(const int n, const double gaps[], std::vector<Curve *> &curves) const
{
	Curve *c;
	double start = 0;
	for (int i = 0; i < n; ++i)
	{
		curves.push_back(Subcurve(start, gaps[2 * i]));
		start = gaps[2 * i + 1];
	}
	curves.push_back(Subcurve(start, GetDuration()));
}

void Curve::ToPolygon(const int n, const double thick, Polygon2D &poly)
{
	std::vector<double> skel;
	std::vector<double> vertices;

	const int dim = GetDim();
	const double dur = GetDuration();

	skel.resize(n * dim);
	GetFirstPoint(&skel[0]);
	for (int i = 1; i < n; ++i)
		GetPointAtTime(dur * (i + 0.0) / (n - 1), &skel[i * dim]);

	FromSkeletonToPolygon2D(skel.size() / 2, &skel[0], thick, vertices);
	poly.AddVertices(vertices.size() / 2, &vertices[0]);
	poly.MakeCCW();
	return;
}

void Curve::Draw(const int nrPts) const
{
	const int dim = GetDim();
	const double dur = GetDuration();

	std::vector<double> pts;
	pts.resize(2 * dim);

	GetFirstPoint(&pts[0]);
	for (int i = 1; i <= nrPts; ++i)
	{
		GetPointAtTime(dur * (i + 0.0) / nrPts, &pts[dim]);
		if (dim == 2)
			GDrawSegment2D(&pts[0], &pts[dim]);
		else if (dim >= 3)
			GDrawSegment3D(&pts[0], &pts[dim]);
		CopyArray<double>(&pts[0], dim, &pts[dim]);
	}
}

////TransformedCurve
void TransformedCurve::Transform(double p[]) const
{
	p[0] *= m_scaling[0];
	p[1] *= m_scaling[1];
	if (m_curve->GetDim() == 3)
	{
		p[2] *= m_scaling[2];
		Algebra3D::TransRotMultPoint(m_TR, p, p);
	}
	else
		Algebra2D::TransRotMultPoint(m_TR, p, p);
}

/////////////////////MultiCurve
MultiCurve::~MultiCurve(void)
{
	const int n = m_curves.size();
	for (int i = 0; i < n; ++i)
		delete m_curves[i];
	m_curves.clear();
}

Curve *MultiCurve::Clone(void) const
{
	MultiCurve *mc = new MultiCurve();
	const int n = m_curves.size();

	for (int i = 0; i < n; ++i)
		mc->AddCurve(m_curves[i]->Clone());
	return mc;
}

void MultiCurve::AddCurve(Curve *const curve)
{
	m_curves.push_back(curve);
	m_duration += curve->GetDuration();
}

void MultiCurve::RemoveCurve(const int i)
{
	const int n = m_curves.size();

	if (i >= 0 && i < n)
	{
		Curve *c = m_curves[i];
		for (int j = i + 1; j < n; ++j)
			m_curves[j - 1] = m_curves[j];
		m_curves.pop_back();
		m_duration -= c->GetDuration();

		delete c;
	}
}

void MultiCurve::GetPointAtTime(const double t, double p[]) const
{
	const int n = m_curves.size();
	double d = 0;

	for (int i = 0; i < n; ++i)
	{
		if ((d + m_curves[i]->GetDuration()) >= t)
		{
			m_curves[i]->GetPointAtTime(t - d, p);
			//printf("point at time %f curve %d %f %f\n", t, i, p[0], p[1]);

			return;
		}
		d += m_curves[i]->GetDuration();
	}
	m_curves.back()->GetPointAtTime(t - GetDuration() + m_curves.back()->GetDuration(), p);

	//printf("outside at time %f %f curve %d %f %f\n", t, t - GetDuration(), n-1, p[0], p[1]);
}

Curve *MultiCurve::Subcurve(const double tstart, const double tend) const
{
	const int n = m_curves.size();
	int istart = -1;
	int iend = -1;
	double d = 0;
	double t1 = 0;
	double t2 = 0;

	for (int i = 0; i < n; ++i)
	{
		if (istart < 0 && (d + m_curves[i]->GetDuration()) >= tstart)
		{
			istart = i;
			t1 = d;
		}

		if (istart >= 0 && (d + m_curves[i]->GetDuration()) >= tend)
		{
			iend = i;
			t2 = d;
			break;
		}

		d += m_curves[i]->GetDuration();
	}

	if (istart < 0 || iend < 0)
		return NULL;
	if (istart == iend)
		return m_curves[istart]->Subcurve(tstart - t1, tend - t1);

	MultiCurve *mc = new MultiCurve();
	mc->AddCurve(m_curves[istart]->Subcurve(tstart - t1, m_curves[istart]->GetDuration()));
	for (int i = istart + 1; i < iend; ++i)
		mc->AddCurve(m_curves[i]->Clone());
	mc->AddCurve(m_curves[iend]->Subcurve(0, tend - t2));
	return mc;
}

void MultiCurve::Draw(const int nrPts) const
{
	const int n = m_curves.size();
	for (int i = 0; i < n; ++i)
		m_curves[i]->Draw(nrPts);
}

//SegmentCurve

void SegmentCurve::GetPointAtTime(const double t, double p[]) const
{
	const double s = t / m_duration;
	for (int i = 0; i < m_dim; ++i)
		p[i] = (1 - s) * m_pts[i] + s * m_pts[m_dim + i];

	//	printf("segment time = %f %f point = %f %f [dim = %d]\n", t, s,  p[0], p[1], m_dim);
	//	printf("seg pts = %f %f %f %f\n", m_pts[0], m_pts[1], m_pts[2], m_pts[3]);
}

Curve *SegmentCurve::Subcurve(const double tstart, const double tend) const
{
	SegmentCurve *sc = new SegmentCurve();
	std::vector<double> tmp;
	tmp.resize(GetDim());

	sc->SetDuration(tend - tstart);
	sc->SetDim(GetDim());

	GetPointAtTime(tstart, &tmp[0]);
	sc->SetFirstPoint(&tmp[0]);
	GetPointAtTime(tend, &tmp[0]);
	sc->SetLastPoint(&tmp[0]);

	return sc;
}

void SegmentCurve::Draw(const int nrPts) const
{
	if (m_dim == 2)
		GDrawSegment2D(&m_pts[0], &m_pts[m_dim]);
	else if (m_dim >= 3)
		GDrawSegment3D(&m_pts[0], &m_pts[m_dim]);
}

//ArcCurve
void ArcCurve::GetPointAtTime(const double t, double p[]) const
{
	p[0] = m_center[0] + cos(t + m_start) * m_radiusX;
	p[1] = m_center[1] + sin(t + m_start) * m_radiusY;
}

Curve *ArcCurve::Subcurve(const double tstart, const double tend) const
{
	ArcCurve *ac = new ArcCurve();
	ac->SetStart(m_start + tstart);
	ac->SetDuration(tend - tstart);
	ac->SetRadiusX(m_radiusX);
	ac->SetRadiusY(m_radiusY);
	ac->SetCenter(m_center);

	return ac;
}

//Misc
void PolygonFromCurves(const Curve &curve1,
					   const Curve &curve2,
					   const int n,
					   Polygon2D &poly)
{
	poly.Clear();
	double p[3];

	for (int i = 0; i < n; ++i)
	{
		curve1.GetPointAtTime(curve1.GetDuration() * (i + 0.0) / (n - 1), p);
		poly.AddVertex(p[0], p[1]);
	}

	for (int i = n - 1; i >= 0; --i)
	{
		curve2.GetPointAtTime(curve2.GetDuration() * (i + 0.0) / (n - 1), p);
		poly.AddVertex(p[0], p[1]);
	}

	poly.MakeCCW();
}
}
