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

#ifndef Antipatrea___Tree_HPP_
#define Antipatrea___Tree_HPP_

#include "Utils/TreeVertex.hpp"
#include "Utils/HashFn.hpp"
#include "Utils/Misc.hpp"
#include <unordered_map>
#include <vector>

namespace Antipatrea {
    template<typename Key>
    class Tree {
    public:
        Tree(void) {
        }

        virtual ~Tree(void) {
            for (auto &iter: m_vertices)
                if (iter.second)
                    delete iter.second;
        }

        virtual int GetNrVertices(void) const {
            return m_vertices.size();
        }

        virtual Key GetVertexKey(const int pos) const {
            return m_keys[pos];
        }

        virtual bool HasVertex(const Key key) const {
            return m_vertices.find(key) != m_vertices.end();
        }

        virtual const TreeVertex<Key> *GetVertexByIndex(const int i) const {
            return GetVertex(GetVertexKey(i));
        }

        virtual TreeVertex<Key> *GetVertexByIndex(const int i) {
            return GetVertex(GetVertexKey(i));
        }

        virtual const TreeVertex<Key> *GetVertex(const Key key) const {
            auto curr = m_vertices.find(key);
            if (curr == m_vertices.end())
                return NULL;
            return curr->second;
        }

        virtual TreeVertex<Key> *GetVertex(const Key key) {
            auto curr = m_vertices.find(key);
            if (curr == m_vertices.end())
                return NULL;
            return curr->second;
        }

        virtual void AddVertex(TreeVertex<Key> *const v) {
            m_vertices.insert(std::make_pair(v->GetKey(), v));
            m_keys.push_back(v->GetKey());
        }

        virtual double GetPathFromVertexToRoot(const Key key, std::vector<Key> &path) {
            TreeVertex<Key> *v = GetVertex(key);
            double cost = 0.0;

            path.clear();
            while (v != NULL) {
                path.push_back(v->GetKey());
                cost += v->GetEdgeCost();
                v = v->GetParent();
            }
            return cost;
        }

        virtual double GetPathFromRootToVertex(const Key key, std::vector<Key> &path) {
            const double cost = GetPathFromVertexToRoot(key, path);
            ReverseItems<Key>(path);
            return cost;
        }

        virtual void RemoveVertex(const Key key) {
            auto curr = m_vertices.find(key);
            if (curr != m_vertices.end()) {
                auto *v = curr->second;
                m_vertices.erase(curr);
                if (v)
                    delete v;
                auto pos = std::find(m_keys.begin(), m_keys.end(), key);
                if (pos != m_keys.end())
                    m_keys.erase(pos);
            }
        }

    protected:
        std::unordered_map<Key, TreeVertex<Key> *, HashStruct<Key>> m_vertices;
        std::vector<Key> m_keys;
    };

}

#endif
