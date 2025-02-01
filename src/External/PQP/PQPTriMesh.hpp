
#ifndef Antipatrea__PQPTriMesh_HPP_
#define Antipatrea__PQPTriMesh_HPP_

#include "Utils/TriMesh.hpp"
#include "External/PQP/PQP.h"

namespace Antipatrea
{
    class PQPTriMesh : public TriMesh
    {
    public:
	PQPTriMesh(void) : TriMesh(),
			   m_pqpModel(NULL),
			   m_pqpUpdated(false)
	{
	}
	
	virtual ~PQPTriMesh(void)
	{
	    if(m_pqpModel)
		delete m_pqpModel;
	}
	
	PQP_Model* GetPQPModel(void);
	
	virtual bool Collision(const double T[], 
			       const double R[],
			       TriMesh * const other_tmesh, 
			       const double other_T[], 
			       const double other_R[]);

	virtual bool DistanceThreshold(const double T[], 
				       const double R[],
				       TriMesh * const other_tmesh, 
				       const double other_T[], 
				       const double other_R[],
				       const double tol,
				       double * const d = NULL,
				       double p1[] = NULL,
				       double p2[] = NULL);
	
	virtual double Distance(const double T[], 
				const double R[],
				TriMesh * const other_tmesh, 
				const double other_T[], 
				const double other_R[],
				double p1[] = NULL,
				double p2[] = NULL);
	
	
    protected:
	virtual void OnVertexChange(const int i)
	{
	    TriMesh::OnVertexChange(i);
	    
	    m_pqpUpdated = false;
	}
	
	virtual void OnFaceChange(const int i)
	{
	    TriMesh::OnFaceChange(i);
	    
	    m_pqpUpdated = false;
	}

	void RotAsPQPRot(const double R[], double pqpR[3][3]) const;
	
	
	PQP_Model *m_pqpModel;
	bool       m_pqpUpdated;
    };
}

#endif
    
    
    
    
    
    
    

