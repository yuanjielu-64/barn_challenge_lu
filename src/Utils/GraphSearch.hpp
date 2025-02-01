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

#ifndef Antipatrea__GraphSearch_HPP_
#define Antipatrea__GraphSearch_HPP_

#include "Utils/Definitions.hpp"
#include "Utils/Heap.hpp"
#include "Utils/Misc.hpp"
#include <unordered_map>
#include <unordered_set>
#include <stack>
#include <queue>

namespace Antipatrea
{

template <typename Key>
class GraphPathData
{
  public:
    GraphPathData(void) : m_cost(0.0)
    {
    }

    double m_cost;
    std::vector<Key> m_path;
    std::vector<double> m_pts;
};

template <typename Key>
class GraphAbstraction
{
  public:
    GraphAbstraction(void)
    {
    }

    virtual ~GraphAbstraction(void)
    {
    }

    virtual void GetOutEdges(const Key u,
                             std::vector<Key> &edges,
                             std::vector<double> &costs) const = 0;

    virtual bool IsGoal(const Key key) const
    {
        return false;
    }

    virtual double HeuristicCostToGoal(const Key u) const
    {
        return 0;
    }
};

template <typename Key>
class GraphSearch
{
  public:
    GraphSearch(void)
    {
        m_info = NULL;
    }

    virtual ~GraphSearch(void)
    {
    }

    void SetGraphAbstraction(GraphAbstraction<Key> *info)
    {
        m_info = info;
    }

    bool DFS(const Key start, Key &goal);

    bool BFS(const Key start, Key &goal);

    bool AStar(const Key start, const bool breakEarly, Key &goal);

    void GetReversePathFromStart(const Key u, std::vector<Key> &rpath) const;

    void GetPathFromStart(const Key u, std::vector<Key> &path) const;

    int GetPathLengthFromStart(const Key u) const;

    double GetPathCostFromStart(const Key u) const;

    void GetPathDataFromStart(const Key u, GraphPathData<Key> &data)
    {
        data.m_cost = GetPathCostFromStart(u);
        GetPathFromStart(u, data.m_path);
    }

  protected:
    GraphAbstraction<Key> *m_info;

    struct Data
    {
        Key m_key;
        Key m_parent;
        double m_gCost;
        double m_hCost;
    };

    struct DataCompare
    {
        DataCompare() : m_gs(NULL)
        {
        }

        bool operator()(Key u, Key v)
        {
            Data datau = m_gs->m_map.find(u)->second;
            Data datav = m_gs->m_map.find(v)->second;

            return (datau.m_gCost + datau.m_hCost) < (datav.m_gCost + datav.m_hCost);
        }

        GraphSearch<Key> *m_gs;
    };

    std::unordered_map<Key, Data> m_map;
};

template <typename Key>
void GraphSearch<Key>::GetPathFromStart(const Key u, std::vector<Key> &path) const
{
    GetReversePathFromStart(u, path);
    ReverseItems<Key>(path);
}

template <typename Key>
double GraphSearch<Key>::GetPathCostFromStart(const Key u) const
{
    auto cur = m_map.find(u);

    if (cur == m_map.end())
        return INFINITY;
    return cur->second.m_gCost;
}

template <typename Key>
int GraphSearch<Key>::GetPathLengthFromStart(const Key u) const
{
    if (m_map.find(u) == m_map.end())
        return -1;

    Key p = u, v;
    int count = 0;
    do
    {
        v = p;
        ++count;
        p = m_map.find(v)->second.m_parent;
    } while (!(v == p));

    return count;
}

template <typename Key>
void GraphSearch<Key>::GetReversePathFromStart(const Key u, std::vector<Key> &rpath) const
{
    rpath.clear();

    if (m_map.find(u) == m_map.end())
        return;

    Key p = u, v;

    do
    {
        v = p;
        rpath.push_back(v);
        p = m_map.find(v)->second.m_parent;
    } while (!(v == p));
}

template <typename Key>
bool GraphSearch<Key>::DFS(const Key start, Key &goal)
{
    std::vector<Key> edges;
    std::vector<int> perm;
    std::vector<double> costs;
    std::stack<Key> stack;
    Data datau;
    Data datav;
    Key u;
    Key v;
    int n;

    m_map.clear();

    datau.m_key = start;
    datau.m_parent = start;
    datau.m_gCost = 0.0;
    datau.m_hCost = 0.0;
    m_map.insert(std::make_pair(start, datau));

    if (m_info->IsGoal(start))
    {
        goal = start;
        return true;
    }

    stack.push(start);
    while (!stack.empty())
    {
        u = stack.top();
        stack.pop();
        datau = m_map.find(u)->second;

        edges.clear();
        costs.clear();
        m_info->GetOutEdges(u, edges, costs);
        n = edges.size();
        perm.resize(n);
        for (int i = 0; i < n; ++i)
            perm[i] = i;
        PermuteItems<Key>(&perm, n);

        for (int i = 0; i < n; ++i)
        {
            v = edges[perm[i]];
            if (m_map.find(v) == m_map.end())
            {
                datav.m_key = v;
                datav.m_parent = u;
                datav.m_gCost = datau.m_gCost + costs[perm[i]];
                m_map.insert(std::make_pair(v, datav));
                if (m_info->IsGoal(v))
                {
                    goal = v;
                    return true;
                }
                stack.push(v);
            }
        }
    }

    return false;
}

template <typename Key>
bool GraphSearch<Key>::BFS(const Key start, Key &goal)
{
    std::vector<Key> edges;
    std::vector<int> perm;
    std::vector<double> costs;
    std::queue<Key> queue;
    Data datau;
    Data datav;
    Key u;
    Key v;
    int n;

    m_map.clear();

    datau.m_key = start;
    datau.m_parent = start;
    datau.m_gCost = 0.0;
    datau.m_hCost = 0.0;
    m_map.insert(std::make_pair(start, datau));

    if (m_info->IsGoal(start))
    {
        goal = start;
        return true;
    }

    queue.push(start);
    while (!queue.empty())
    {
        u = queue.top();
        queue.pop();
        datau = m_map.find(u)->second;

        edges.clear();
        costs.clear();
        m_info->GetOutEdges(u, edges, costs);
        n = edges.size();
        perm.resize(n);
        for (int i = 0; i < n; ++i)
            perm[i] = i;
        PermuteItems<Key>(&perm, n);

        for (int i = 0; i < n; ++i)
        {
            v = edges[perm[i]];
            if (m_map.find(v) == m_map.end())
            {
                datav.m_key = v;
                datav.m_parent = u;
                datav.m_gCost = datau.m_gCost + costs[perm[i]];
                m_map.insert(std::make_pair(v, datav));
                if (m_info->IsGoal(v))
                {
                    goal = v;
                    return true;
                }
                queue.push(v);
            }
        }
    }

    return false;
}

template <typename Key>
bool GraphSearch<Key>::AStar(const Key start, const bool breakEarly, Key &goal)
{
    Key u;
    Key v;
    Data datau;
    Data datav;
    std::vector<Key> edges;
    std::vector<double> costs;
    std::vector<int> perm;
    std::unordered_set<Key> closed;
    Heap<Key, DataCompare> heap;
    DataCompare datacmp;

    datacmp.m_gs = this;
    heap.SetKeyCompare(datacmp);

    m_map.clear();

    datau.m_key = start;
    datau.m_parent = start;
    datau.m_gCost = 0;
    datau.m_hCost = m_info->HeuristicCostToGoal(start);
    m_map.insert(std::make_pair(start, datau));
    heap.Insert(start);

    while (!heap.IsEmpty())
    {
        //remove top
        u = heap.RemoveTop();
        datau = m_map.find(u)->second;
        closed.insert(u);

        if (m_info->IsGoal(u))
        {
            goal = u;
            return true;
        }
        //get edges and costs
        edges.clear();
        costs.clear();
        m_info->GetOutEdges(u, edges, costs);

        //process edges
        for (int i = edges.size() - 1; i >= 0; --i)
            if (closed.find(edges[i]) == closed.end())
            {
                v = edges[i];
                auto cur = m_map.find(v);
                if (cur == m_map.end())
                {
                    datav.m_key = v;
                    datav.m_parent = u;
                    datav.m_gCost = datau.m_gCost + costs[i];
                    datav.m_hCost = m_info->HeuristicCostToGoal(v);
                    if (datav.m_hCost != INFINITY)
                    {
                        m_map.insert(std::make_pair(v, datav));
                        heap.Insert(v);
                        if (breakEarly && m_info->IsGoal(v))
                        {
                            goal = v;
                            return true;
                        }
                    }
                }
                else if ((datau.m_gCost + costs[i]) < cur->second.m_gCost)
                {
                    cur->second.m_parent = u;
                    cur->second.m_gCost = datau.m_gCost + costs[i];
                    heap.Update(v);
                }
            }
    }
    return false;
}
}

#endif
