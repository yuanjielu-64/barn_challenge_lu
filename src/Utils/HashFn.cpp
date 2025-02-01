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
#include "Utils/HashFn.hpp"
#include <cstdint>

#undef get16bits
#if (defined(__GNUC__) && defined(__i386__)) || defined(__WATCOMC__) \
  || defined(_MSC_VER) || defined (__BORLANDC__) || defined (__TURBOC__)
#define get16bits(d) (*((const uint16_t *) (d)))
#endif

#if !defined (get16bits)
#define get16bits(d) ((((uint32_t)(((const uint8_t *)(d))[1])) << 8)\
                       +(uint32_t)(((const uint8_t *)(d))[0]) )
#endif

namespace Antipatrea
{
    size_t StringHash(const char *s, const int n)
    {
	int length = n;
	
	
	uint32_t hash = n, tmp;
	int rem;
	
	if (length <= 0 || s == NULL) return 0;
	
	rem = length & 3;
	length >>= 2;
	
	/* Main loop */
	for (;length > 0; length--) 
	{
	    hash  += get16bits (s);
	    tmp    = (get16bits (s+2) << 11) ^ hash;
	    hash   = (hash << 16) ^ tmp;
	    s  += 2*sizeof (uint16_t);
	    hash  += hash >> 11;
	}
	
	/* Handle end cases */
	switch (rem) 
	{
        case 3: hash += get16bits (s);
	    hash ^= hash << 16;
	    hash ^= ((signed char)s[sizeof (uint16_t)]) << 18;
	    hash += hash >> 11;
	    break;
        case 2: hash += get16bits (s);
	    hash ^= hash << 11;
	    hash += hash >> 17;
	    break;
        case 1: hash += (signed char)*s;
	    hash ^= hash << 10;
	    hash += hash >> 1;
	}
	
	/* Force "avalanching" of final 127 bits */
	hash ^= hash << 3;
	hash += hash >> 5;
	hash ^= hash << 4;
	hash += hash >> 17;
	hash ^= hash << 25;
	hash += hash >> 6;
	
	return hash;
    }
    
}

    
    
    






