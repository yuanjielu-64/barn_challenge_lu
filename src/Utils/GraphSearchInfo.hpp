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

#ifndef Antipatrea___GraphSearchInfo_HPP_
#define Antipatrea___GraphSearchInfo_HPP_

#include "Utils/Graph.hpp"
#include "Utils/GraphSearch.hpp"

namespace Antipatrea
{	    
    /**
     *@brief Wrapper for the  graph so that it can be used with
     *       the graph searching alogrithms, e.g., DFS, BFS, A*, Dijkstra.
     * 
     *@remarks
     * - The graph-search algorithms in this package work with
     *   general, implicit, graph representation.
     * - This wrapper takes a Graph and implements the functions
     *   needed by the graph-search algorithms, namely GetOutEdges   
     */
    template <typename Key>
    class GraphSearchInfo : public GraphAbstraction<Key>
    {
    public:
	GraphSearchInfo(void) : 
	    GraphAbstraction<Key>(),
	    m_graph(NULL)
	{
	}
	
	virtual ~GraphSearchInfo(void)
	{
	}

	virtual const Graph<Key>* GetGraph(void) const
	{
	    return m_graph;
	}
	
	virtual Graph<Key>* GetGraph(void)
	{
	    return m_graph;
	}

	virtual void SetGraph(Graph<Key> * const graph)
	{
	    m_graph = graph;
	}
	
	virtual void GetOutEdges(const Key key, 
				 std::vector<Key> & edges,
				 std::vector<double> & costs) const
	{
	    edges.clear();
	    costs.clear();
	    
	    const Graph<Key>       *g = GetGraph();
	    const GraphVertex<Key> *v = g->GetVertex(key);
	    
	    if(v == NULL)
		return;
	    
	    auto connections = v->GetConnections();
	    for(auto & keyTo : *connections)
	    {
		edges.push_back(keyTo);
		auto edge = g->FindEdge(key, keyTo);
		if(edge->GetCost() != INFINITY)
		    costs.push_back(edge->GetCost());
	    }
	}
	

    protected:
	Graph<Key> *m_graph;
    };
}

#endif
