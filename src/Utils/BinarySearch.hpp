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

#ifndef Antipatrea__BinarySearch_HPP_
#define Antipatrea__BinarySearch_HPP_

namespace Antipatrea
{
/**
    *@brief Binary search on a sorted array
    */
namespace BinarySearch
{
/**
	 *@brief Find position where <em>key</em> appears in the array
	 *       <em>p[start..end]</em> 
	 *@param p array of elements
	 *@param key item to search for
	 *@param start starting index
	 *@param end ending index
	 *	 
	 *@returns
	 *  - position if <em>key</em> is found in
	 *    <em>p[start..end]</em> 
	 *  - <em>-1</em> otherwise
	 *  .
	 *
	 *@remarks
	 *  - Type <em>Key</em> must support the less-than
	 *    comparison operator <em> < </em>.
	 *  - The invariant <em>p[start] <= p[start + 1] <= ... <=
	 *    p[end]</em> is a precondition on the elements of the
	 *    array <em>p</em>. 
         *  .
	 */
template <typename Key>
int Contains(const Key p[], const Key key, const int start, const int end)
{
	int s = start;
	int e = end;
	int m = 0;

	while (s <= e)
	{
		m = (s + e) / 2;

		if (p[m] < key)
			s = m + 1;
		else if (key < p[m])
			e = m - 1;
		else
			return m;
	}
	return -1;
}

/**
	 *@brief Find position where <em>key</em> can be inserted in the
	 *       array <em>p[start..end]</em> such that array remains
	 *       sorted	  
	 *@param p array of elements
	 *@param key item to search for
	 *@param start starting index
	 *@param end ending index
	 *	 
	 *@returns
	 * - <em>0</em> if <em>end < start</em>
	 * - otherwise, position <em>i</em> in <em>[start, end + 1]</em> such that 
	 *  <CENTER><em>
	 *    p[start] <= p[start + 1] <= p[i - 1] <= key <= p[i] <=
	 *    p[i + 1] <= ... <= p[end] 
         *  </em></CENTER>
	 *
   	 *@remarks
	 *  - Type <em>Key</em> must support the less-than
	 *    comparison operator <em> < </em>.
	 *  - The invariant <em>p[start] <= p[start + 1] <= ... <=
	 *     p[end]</em> is a precondition on the elements of the
	 *     array <em>p</em>. 
         *  .
	 */
template <typename Key>
int SortedPosition(const Key p[], const Key key, const int start, const int end)
{
	int s = start;
	int e = end;
	int m = 0;

	if (e < s)
		return 0;

	while (s <= e)
	{
		m = (s + e) / 2;
		if (p[m] < key)
			s = m + 1;
		else if (key < p[m])
			e = m - 1;
		else
			return m;
	}
	return e + 1;
}
}
}

#endif
