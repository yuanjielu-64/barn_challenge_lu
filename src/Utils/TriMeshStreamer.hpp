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

#ifndef Antipatrea__TriMeshStreamer_HPP_
#define Antipatrea__TriMeshStreamer_HPP_

#include "Utils/TriMesh.hpp"
#include "Utils/Reader.hpp"
#include "Utils/Writer.hpp"
#include <cstdio>

namespace Antipatrea
{
    bool TriMeshReader(const char fname[], TriMesh & tmesh);
    bool OFFTriMeshReader(const char fname[], TriMesh & tmesh);
    bool PLYTriMeshReader(const char fname[], TriMesh & tmesh);    
    bool OBJTriMeshReader(const char fname[], TriMesh & tmesh);    
    bool STLAsciiTriMeshReader(const char fname[], TriMesh & tmesh);
    bool STLBinaryTriMeshReader(const char fname[], TriMesh & tmesh);
    bool StandardTriMeshReader(std::istream & in, TriMesh & tmesh);


    bool STLAsciiTriMeshWriter(const char fname[], TriMesh & tmesh);
    bool STLBinaryTriMeshWriter(const char fname[], TriMesh & tmesh);
    bool StandardTriMeshWriter(std::ostream & out, TriMesh & tmesh);

}

#endif





    
    

