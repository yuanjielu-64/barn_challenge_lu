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

#ifndef Antipatrea__Ids_HPP_
#define Antipatrea__Ids_HPP_


#include "Utils/Constants.hpp"
#include <vector>
#include <unordered_map>

namespace Antipatrea
{
    template <typename Key,
	      typename Compare=std::less<Key>,
	      typename Equal=std::equal_to<Key>,
	      typename Hash = std::hash<Key> >
    class Ids
    {
    public:
	Ids(void)
	{
	}
	
	virtual ~Ids(void)
	{
	}
	
	bool IsEmpty(void) const
	{
	    return m_keys.empty();
	}
	
	int GetNrKeys(void) const
	{
	    return m_keys.size();
	}
	
	Key GetKeyAtPosition(const int pos) const
	{
	    return m_keys[pos];
	}
	
	bool HasKey(const Key key) const
	{
	    return m_map.find(key) != m_map.end();
	}
	
	int GetKeyPosition(const Key key) const
	{
	    auto cur = m_map.find(key);
	    if(cur == m_map.end())
		return Constants::ID_UNDEFINED;
	    else
		return cur->second;
	}
	
	void Insert(const Key key)
	{
	    const int pos =  m_keys.size();
	    m_keys.push_back(key);
	    m_map.insert(std::make_pair(key, pos));
	}
	
	void Remove(const Key key)
	{
	    RemoveAtPosition(m_map.find(key)->second);
	}
	
	void RemoveAtPosition(const int pos)
	{
	    const Key key  = m_keys[pos];
	    const int size = m_keys.size();
	    
	    m_keys[pos] = m_keys[size - 1];
	    m_map.find(m_keys[pos])->second = pos;
	    m_keys.pop_back();
	    m_map.erase(key);
	}
	
	void Clear(void)
	{
	    m_keys.clear();
	    m_map.clear();
	}
	
	
    protected:
	std::unordered_map<Key, int, Hash, Equal > m_map;
	std::vector<Key>  m_keys;
    };
}

#endif
