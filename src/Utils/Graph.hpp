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

#ifndef Antipatrea___Graph_HPP_
#define Antipatrea___Graph_HPP_

#include "Utils/GraphVertex.hpp"
#include "Utils/HashFn.hpp"
#include "Utils/Misc.hpp"
#include <unordered_map>
#include <vector>

namespace Antipatrea
{
template <typename Key>
class Graph
{
  public:
    Graph(void)
    {
    }

    virtual ~Graph(void)
    {
        for (auto &iter : m_vertices)
            if (iter.second)
                delete iter.second;
        for (auto &iter : m_edges)
            if (iter.second)
                delete iter.second;
    }

    virtual int GetNrVertices(void) const
    {
        return m_vertices.size();
    }

    virtual Key GetVertexKey(const int pos) const
    {
        return m_keys[pos];
    }

    virtual bool HasVertex(const Key key) const
    {
        return m_vertices.find(key) != m_vertices.end();
    }

    virtual const GraphVertex<Key> *GetVertexByIndex(const int i) const
    {
        return GetVertex(GetVertexKey(i));
    }

    virtual GraphVertex<Key> *GetVertexByIndex(const int i)
    {
        return GetVertex(GetVertexKey(i));
    }

    virtual const GraphVertex<Key> *GetVertex(const Key key) const
    {
        auto curr = m_vertices.find(key);
        if (curr == m_vertices.end())
            return NULL;
        return curr->second;
    }

    virtual GraphVertex<Key> *GetVertex(const Key key)
    {
        auto curr = m_vertices.find(key);
        if (curr == m_vertices.end())
            return NULL;
        return curr->second;
    }

    virtual void AddVertex(GraphVertex<Key> *const v)
    {
        if (v->GetDisjointSetElem() == NULL)
            v->SetDisjointSetElem(m_components.Make());
        m_vertices.insert(std::make_pair(v->GetKey(), v));
        m_keys.push_back(v->GetKey());
    }

    /**
   *@brief Get the number of edges.
 */
    virtual int GetNrEdges(void) const
    {
        return m_edges.size();
    }

    /**
   *@brief The key is defined as  <tt>(min(vidFrom, vidTo), max(vidFrom, vidTo))</tt>.
 */
    virtual std::pair<Key, Key> GetEdgeKey(const Key vidFrom, const Key vidTo) const
    {
        return std::make_pair(vidFrom, vidTo);
    }

    /**
   *@brief Return a pointer to the edge <tt>(vidFrom, vidTo)</tt>.
 *
 ******@remarks
 * - If the edge is not in the graph, the function returns <tt>NULL</tt>.
 */
    virtual GraphEdge<Key> *FindEdge(const Key vidFrom, const int vidTo) const
    {
        auto curr = m_edges.find(GetEdgeKey(vidFrom, vidTo));
        if (curr == m_edges.end())
            return NULL;
        else
            return curr->second;
    }

    /**
   *@brief Add the edge to the graph.
 *
 ******@remarks
 * - The function does not check if the edge is already in the graph.
 *   It is the responsibility of the calling function to perform this check (using FindEdge)
 *   if indeed there may be a possibility that the same edge could be added multiple times.
 * - The function also joins the components associated with the end vertices of the edge.
 */
    virtual void AddEdge(GraphEdge<Key> *const edge)
    {
        auto keyFrom = edge->GetVertexKey(GraphEdge<Key>::INDEX_FROM);
        auto keyTo = edge->GetVertexKey(GraphEdge<Key>::INDEX_TO);
        auto vfrom = GetVertex(keyFrom);
        auto vto = GetVertex(keyTo);

        m_edges.insert(std::make_pair(GetEdgeKey(keyFrom, keyTo), edge));

        vfrom->GetConnections()->insert(keyTo);
        vto->GetConnections()->insert(keyFrom);

        m_components.Join(vfrom->GetDisjointSetElem(), vto->GetDisjointSetElem());
    }

    /**
   *@brief Return true iff the two vertices are connected by a path.
 *
 ******@remarks
 * - This function uses the disjoint-set data structure, so it works
 *   correctly only if the graph is undirected, i.e.,
 *   if (A, B) is an edge in the graph, then so is (B, A).
 */
    virtual bool AreVerticesPathConnected(const Key keyFrom, const Key keyTo)
    {
        return m_components.Same(GetVertex(keyFrom)->GetDisjointSetElem(),
                                 GetVertex(keyTo)->GetDisjointSetElem());
    }

    /**
   *@brief Get a pointer to the disjoint-set data structure representing the connected components in the graph.
 */
    virtual const DisjointSet *GetComponents(void) const
    {
        return &m_components;
    }

    /**
   *@brief Get a pointer to the disjoint-set data structure representing the connected components in the graph.
 */
    virtual DisjointSet *GetComponents(void)
    {
        return &m_components;
    }

    /**
   *@brief Define the data structure (unordered map with the vertex pair as the key) to store
 *       all the edges of the graph.
 */
    typedef std::unordered_map<std::pair<Key, Key>, GraphEdge<Key> *, HashStruct<std::pair<Key, Key>>> Edges;

    /**
   *@brief Get a pointer to the data structure storing all the edges of the graph.
 */
    virtual const Edges *GetEdges(void) const
    {
        return &m_edges;
    }

    /**
   *@brief Get a pointer to the data structure storing all the edges of the graph.
 */
    virtual Edges *GetEdges(void)
    {
        return &m_edges;
    }

  protected:
    std::unordered_map<Key, GraphVertex<Key> *, HashStruct<Key>> m_vertices;
    std::vector<Key> m_keys;
    DisjointSet m_components;
    Edges m_edges;
};
}

#endif
