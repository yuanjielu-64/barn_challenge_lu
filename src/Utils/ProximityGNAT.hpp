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

#ifndef Antipatrea__ProximityGNAT_HPP_
#define Antipatrea__ProximityGNAT_HPP_

#include "Utils/Proximity.hpp"
#include "Utils/PseudoRandom.hpp"
#include "Utils/Misc.hpp"
#include <queue>

namespace Antipatrea
{
    template <typename Key, typename DistFnData>
    class ProximityGNAT : public Proximity<Key, DistFnData>
    {
    public:
	ProximityGNAT(const int minDegree      =2, 
		      const int mainDegree     =3, 
		      const int maxDegree      =3, 
		      const int maxNrPtsInLeaf =4) :
	    Proximity<Key, DistFnData>()
	{
	    m_root           = NULL;
	    m_mainDegree     = mainDegree;
	    m_minDegree      = minDegree;
	    m_maxDegree      = maxDegree;
	    m_maxNrPtsInLeaf = maxNrPtsInLeaf;
	    m_centers        = (int *) calloc(m_maxDegree, sizeof(int));
	    m_perm           = (int *) calloc(m_maxDegree, sizeof(int));
	    m_dists          = NULL;
	    m_cap_dists      = 0;
	}

	virtual ~ProximityGNAT(void)
	{
	    ClearScheduler();
	    if(m_root)
		delete m_root;
	    if(m_centers)
		free(m_centers);
	    if(m_perm)
		free(m_perm);
	    if(m_dists)
	    {
		for(int i = 0; i < m_cap_dists; i++)
		    if(m_dists[i])
			free(m_dists[i]);
		free(m_dists);
	    }
	}

	virtual void AddKey(const Key key)
	{
	    Proximity<Key,DistFnData>::AddKey(key);
	    if(this->IsDataStructureConstructed())
	    {
		m_root->InsertKey(this, key);
	    }
	}

	virtual void ClearDataStructure(void)
	{
	    Proximity<Key,DistFnData>::ClearDataStructure();	    
	    ClearScheduler();
	    if(m_root)
		delete m_root;
	    m_root = NULL;
	}

	virtual void ConstructDataStructure(void)
	{
	    const int size = this->m_keys.size();
	    
	    Proximity<Key,DistFnData>::ConstructDataStructure();
	    
	    m_root           = new Node(NULL, -1);
	    m_root->m_degree = m_mainDegree;
	    for(int i = 0; i < size; i++)
		m_root->m_keys.push_back(this->m_keys[i]);
	    if(m_root->ShouldExpand(this))
		m_root->Expand(this);
	}

	virtual void Neighbors(ProximityQuery<Key>   & query, 
			       ProximityResults<Key> & results)
	{
	    if(this->IsDataStructureConstructed() == false)
		this->ConstructDataStructure();
	 
	    const Key  qkey = query.GetKey();
	    InnerData *data = NULL;  
	    double     r    = 0.0;

	    results.Clear();
	    results.SetNrNeighborsAndRange(query.GetNrNeighbors(), query.GetRange());
	    
	    m_root->Neighbors(this, qkey, results);
	    while(m_scheduler.size() > 0)
	    {
		r    = results.GetMaxDistance();
		data = m_scheduler.top();
		m_scheduler.pop();
		
		if(r != INFINITY &&
		   (data->m_distToCenter > (data->m_maxRadius + r) ||
		    data->m_distToCenter < (data->m_minRadius - r)))
		    break;
		data->m_child->Neighbors(this, qkey, results);
	    }	    
	    ClearScheduler();
	}
	
    protected:
	virtual void ClearScheduler(void)
	{
	    while(m_scheduler.size() > 0)
		m_scheduler.pop();	    
	}
	
	virtual void EnsureDists(const int npoints)
	{
	    if(npoints > m_cap_dists)
	    {
		const int new_cap = 2 * npoints + 1;
		m_dists = (double **) realloc(m_dists, new_cap * sizeof(double *));
		for(int i = m_cap_dists; i < new_cap; i++)
		    m_dists[i] = (double *) calloc(m_maxDegree, sizeof(double));
		m_cap_dists = new_cap;
	    }
	}

	virtual void KCenters(const int           nkeys,
			      const Key * const   keys,
			      const int           k,
			      int * const         centers,
			      double            **dists)
	{
	    Key ckey;
	    
	    m_kcentersMinDists.resize(nkeys);
	    for(int j = 0; j < nkeys; j++)
		m_kcentersMinDists[j] = INFINITY;
	    centers[0] = RandomUniformInteger(0, nkeys - 1);
	    for(int i = 1; i < k; i++)
	    {
		double dmax = -INFINITY;
		ckey = keys[centers[i - 1]];
		for(int j = 0; j < nkeys; j++)
		{
		    if((dists[j][i - 1] = this->m_distFn(keys[j], ckey, this->m_distFnData)) < m_kcentersMinDists[j])
			m_kcentersMinDists[j] = dists[j][i - 1];
		    if(m_kcentersMinDists[j] > dmax)
		    {
			centers[i] = j;
			dmax       = m_kcentersMinDists[j];
		    }
		}
	    }
	    ckey = keys[centers[k - 1]];   
	    for(int j = 0; j < nkeys; j++)
		dists[j][k - 1] = this->m_distFn(keys[j], ckey, this->m_distFnData);
	}

	std::vector<double> m_kcentersMinDists;
	

	friend class Node;
	
	class Node;
	
	class InnerData
	{
	public:
	    InnerData(Node * const parent, const int indexInParent, const int degree)
	    {
		m_minRange.resize(degree);
		m_maxRange.resize(degree);
		
		m_minRadius = INFINITY;
		m_maxRadius =-INFINITY;
		for(int i = 0; i < degree; i++)
		{
		    m_minRange[i] = INFINITY;
		    m_maxRange[i] =-INFINITY;       
		}
		m_child = new Node(parent, indexInParent);
	    }

	    virtual ~InnerData(void)
	    {
		if(m_child)
		    delete m_child;
	    }

	    Key          m_center;
	    double         m_minRadius;
	    double         m_maxRadius;
	    std::vector<double> m_minRange;
	    std::vector<double> m_maxRange;
	    double         m_distToCenter;
	    Node *m_child;	
	};

	class Node
	{
	public:
	    Node(Node * const parent, const int indexInParent)
	    {
		m_parent        = parent;
		m_indexInParent = indexInParent;
	    }

	    virtual ~Node(void)
	    {
		for(int i = 0; i < (int) m_inner.size(); i++)
		    delete m_inner[i];    
	    }
	    
	    bool ShouldExpand(ProximityGNAT<Key, DistFnData> * const gnat) const
	    {
		return (int) m_keys.size() > gnat->m_maxNrPtsInLeaf &&
		       (int) m_keys.size() > m_degree;
	    }

	    void Expand(ProximityGNAT<Key, DistFnData> * const gnat)
	    {
		m_inner.resize(m_degree);
		for(int i = 0; i < m_degree; i++)
		    m_inner[i] = new InnerData(this, i, m_degree);
		Partition(gnat);
		for(int i = 0; i < m_degree; i++)
		    if(m_inner[i]->m_child->ShouldExpand(gnat))
			m_inner[i]->m_child->Expand(gnat);
	    }

	    void Partition(ProximityGNAT<Key, DistFnData> * const gnat)
	    {
		const int   npoints = m_keys.size();
		int         cdegree = 0;
		int         cindex  = 0;
		InnerData * data    = NULL;
		
		gnat->EnsureDists(npoints);     
		
		double **dists  = gnat->m_dists;
		int     *centers= gnat->m_centers;
		
		gnat->KCenters(npoints, &(m_keys[0]),
			       m_degree, centers, dists);
		
		for(int i = 0; i < m_degree; i++)
		    m_inner[i]->m_center = m_keys[centers[i]];
		
		for(int j = 0; j < npoints; j++)
		{
		    cindex = 0;
		    for(int i = 1; i < m_degree; i++)
			if(dists[j][i] < dists[j][cindex])
			    cindex = i;
		    data = m_inner[cindex];
		    
		    if(j != centers[cindex])
			data->m_child->m_keys.push_back(m_keys[j]);
		    
		    if(j != centers[cindex])
		    {
			if(dists[j][cindex] > data->m_maxRadius)
			    data->m_maxRadius = dists[j][cindex];
			if(dists[j][cindex] < data->m_minRadius)
			    data->m_minRadius = dists[j][cindex];
		    }
		    for(int h = 0; h < m_degree; h++)
		    {
			if(m_inner[h]->m_minRange[cindex] > dists[j][h])
			    m_inner[h]->m_minRange[cindex] = dists[j][h];
			if(m_inner[h]->m_maxRange[cindex] < dists[j][h])
			    m_inner[h]->m_maxRange[cindex] = dists[j][h];
		    }
		}
		
		for(int i = 0; i < m_degree; i++)
		{
		    cdegree = m_degree * (int) (m_inner[i]->m_child->m_keys.size() / npoints);
		    if(cdegree > gnat->m_maxDegree)
			cdegree = gnat->m_maxDegree;
		    if(cdegree < gnat->m_minDegree)
			cdegree = gnat->m_minDegree;
		    m_inner[i]->m_child->m_degree = cdegree;
		    
		    if(m_inner[i]->m_minRadius == INFINITY)
			m_inner[i]->m_minRadius = 0.0;
		    if(m_inner[i]->m_maxRadius == -INFINITY)
			m_inner[i]->m_maxRadius = 0.0;
		}
		
		m_keys.clear();
	    }

	    void InsertKey(ProximityGNAT<Key,DistFnData> * const gnat, const Key key)
	    {
		const int degree = m_inner.size();
		
		if(degree == 0)
		{
		    m_keys.push_back(key);
		    if(ShouldExpand(gnat))
			Expand(gnat);
		}
		else
		{
		    double        dmin       = INFINITY;
		    int           imin       = -1;
		    for(int i = 0; i < degree; i++)
			if((m_inner[i]->m_distToCenter = 
			    gnat->m_distFn(key, m_inner[i]->m_center, gnat->m_distFnData)) < dmin)
			{
			    dmin = m_inner[i]->m_distToCenter;
			    imin = i;
			}
		    if(dmin < m_inner[imin]->m_minRadius)
			m_inner[imin]->m_minRadius = dmin;
		    if(dmin > m_inner[imin]->m_maxRadius)
			m_inner[imin]->m_maxRadius = dmin;
		    for(int i = 0; i < degree; i++)
		    {
			if(m_inner[i]->m_minRange[imin] > m_inner[i]->m_distToCenter)
			    m_inner[i]->m_minRange[imin] = m_inner[i]->m_distToCenter;
			if(m_inner[i]->m_maxRange[imin] < m_inner[i]->m_distToCenter)
			    m_inner[i]->m_maxRange[imin] = m_inner[i]->m_distToCenter;		
		    }
		    m_inner[imin]->m_child->InsertKey(gnat, key);	    
		}
	    }

	    void Neighbors(ProximityGNAT<Key,DistFnData> * const    gnat, 
			   const Key                                qkey, 
			   ProximityResults<Key> &                  results)
	    {
		const int       size       = m_keys.size();
		const int       degree     = m_inner.size();
		int            *perm       = gnat->m_perm;
		InnerData *     data       = NULL;
		double          r          = 0.0;
		
		for(int i = 0; i < size; i++)
		    results.Insert(m_keys[i], gnat->m_distFn(qkey, m_keys[i], gnat->m_distFnData));
		
		if(degree > 0)
		{
		    for(int z = 0; z < degree; ++z)
			perm[z] = z;
		    PermuteItems<int>(degree, perm, degree);
		    
		    for(int i = 0; i < degree; i++)
			if(perm[i] >= 0)
			{
			    data                 = m_inner[perm[i]];
			    data->m_distToCenter = gnat->m_distFn(qkey, data->m_center, gnat->m_distFnData);
			    results.Insert(data->m_center, data->m_distToCenter);
			    if((r = results.GetMaxDistance()) != INFINITY)
				for(int j = 0; j < degree; j++)
				    if(perm[j] >= 0 && i != j &&
				       (data->m_distToCenter - r > data->m_maxRange[perm[j]] ||
					data->m_distToCenter + r < data->m_minRange[perm[j]]))
				    {			
					perm[j] = -1;
				    }
			}
		    
		    r = results.GetMaxDistance();
		    for(int i = 0; i < degree; i++)
			if(perm[i] >= 0)
			{
			    data = m_inner[perm[i]];
			    if(r == INFINITY ||
			       (data->m_distToCenter <= (data->m_maxRadius + r) && 
				data->m_distToCenter >= (data->m_minRadius - r)))
				gnat->m_scheduler.push(data);
			}    
		}
	    }

	    Node *              m_parent;
	    int                 m_indexInParent;
	    int                 m_degree;
	    std::vector<Key>       m_keys;
	    std::vector<InnerData *> m_inner;
	};

	struct LessFnInnerData : public std::binary_function<InnerData*, InnerData*, bool>
	{
	    bool operator()(InnerData* &a, InnerData* &b) const
	    {
		return (a->m_distToCenter - a->m_maxRadius) > (b->m_distToCenter - b->m_maxRadius);
	    }
	};

	Node *      m_root;
	int         m_mainDegree;
	int         m_minDegree;
	int         m_maxDegree;
	int         m_maxNrPtsInLeaf;
	int        *m_centers;
	int        *m_perm;
	double    **m_dists;	
	int         m_cap_dists;
	std::priority_queue<InnerData *, std::vector<InnerData *>, LessFnInnerData> m_scheduler;
    };
}

#endif








