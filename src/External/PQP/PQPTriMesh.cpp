
#include "External/PQP/PQPTriMesh.hpp"
#include "Utils/Algebra3D.hpp"

namespace Antipatrea
{
    PQP_Model* PQPTriMesh::GetPQPModel(void)
    {
	int count = 0;
	
	if(!m_pqpUpdated)
	{
	    if(m_pqpModel)
		delete m_pqpModel;
	    m_pqpModel = new PQP_Model();
	    
	    const int n = GetNrFaces();
	    if(n == 0)
		return m_pqpModel;
	    
	    m_pqpModel->BeginModel();
	    for(int i = 0; i < n; ++i)
	    {
		const Face *face = GetFace(i);
		if(face->m_vids.size() != 3)
		{
		    for(int j = 1; j < face->m_vids.size() - 1; ++j)
			m_pqpModel->AddTri(GetVertex(face->m_vids[0]),
					   GetVertex(face->m_vids[j]),
					   GetVertex(face->m_vids[j + 1]), count++);
		    
		}
		else		    
		    m_pqpModel->AddTri(GetVertex(face->m_vids[0]), 
				      GetVertex(face->m_vids[1]), 
				      GetVertex(face->m_vids[2]), count++);
		
	    }
	    m_pqpModel->EndModel();
	    m_pqpUpdated = true;

	    
	}
	return m_pqpModel;
    }
    
    void PQPTriMesh::RotAsPQPRot(const double R[], double pqpR[3][3]) const
    {
	if(R)
	{
	    pqpR[0][0] = R[0];
	    pqpR[0][1] = R[1];
	    pqpR[0][2] = R[2];
	    pqpR[1][0] = R[3];
	    pqpR[1][1] = R[4];
	    pqpR[1][2] = R[5];
	    pqpR[2][0] = R[6];
	    pqpR[2][1] = R[7];
	    pqpR[2][2] = R[8];
	}
	else
	{
	    pqpR[0][0] = pqpR[1][1] = pqpR[2][2] = 1;
	    pqpR[0][1] = pqpR[0][2] = pqpR[1][0] = pqpR[1][2] = pqpR[2][0] = pqpR[2][1] = 0;
	}
    }
    
    
    bool PQPTriMesh::Collision(const double T[], 
			       const double R[],
			       TriMesh * const other_tmesh, 
			       const double Tother[], 
			       const double Rother[])
    {
	PQPTriMesh *other = dynamic_cast<PQPTriMesh *>(other_tmesh);
	
	if(other == NULL)
	    return false;
	
	double pqpR[3][3];
	double pqpT[3] = {T ? T[0] : 0, T ? T[1] : 0, T ? T[2] : 0};

	double pqpRother[3][3];
	double pqpTother[3] = {Tother ? Tother[0] : 0, Tother ? Tother[1] : 0, Tother ? Tother[2] : 0};

	RotAsPQPRot(R, pqpR);
	RotAsPQPRot(Rother, pqpRother);
	
	PQP_CollideResult res;
	PQP_Collide(&res, 
		    pqpR, pqpT, GetPQPModel(),
		    pqpRother, pqpTother, other->GetPQPModel(), PQP_FIRST_CONTACT);
	return res.Colliding() != 0;
    }
    
    bool PQPTriMesh::DistanceThreshold(const double T[], 
				       const double R[],
				       TriMesh * const other_tmesh, 
				       const double Tother[], 
				       const double Rother[],
				       const double tol,
				       double * const d,
				       double p1[3],
				       double p2[3])
    {
	PQPTriMesh *other = dynamic_cast<PQPTriMesh *>(other_tmesh);
	
	if(other == NULL)
	    return false;
	
	double pqpR[3][3];
	double pqpT[3] = {T ? T[0] : 0, T ? T[1] : 0, T ? T[2] : 0};
	
	double pqpRother[3][3];
	double pqpTother[3] = {Tother ? Tother[0] : 0, Tother ? Tother[1] : 0, Tother ? Tother[2] : 0};
	
	RotAsPQPRot(R, pqpR);
	RotAsPQPRot(Rother, pqpRother);
	
	PQP_ToleranceResult res;
	PQP_Tolerance(&res, pqpR, pqpT, GetPQPModel(),
		      pqpRother, pqpTother, other->GetPQPModel(), tol);
	
	if(res.CloserThanTolerance())
	{
	    if(d)
		*d = res.Distance();
	    if(p1)
	    {
		p1[0] = res.P1()[0];
		p1[1] = res.P1()[1];
		p1[2] = res.P1()[2];		

		Algebra3D::RotMultPoint(R, p1, p1);
		Algebra3D::TransMultPoint(T, p1, p1);	    
	    }
	    if(p2)
	    {
		p2[0] = res.P2()[0];
		p2[1] = res.P2()[1];
		p2[2] = res.P2()[2];		

		Algebra3D::RotMultPoint(Rother, p2, p2);
		Algebra3D::TransMultPoint(Tother, p2, p2);	    
	    }
	    
	    return true;	    
	}
	
	return false;	
    }
    
    
    double PQPTriMesh::Distance(const double T[], 
				const double R[],
				TriMesh * const other_tmesh, 
				const double Tother[],
				const double Rother[],
				double p1[3],
				double p2[3])
    {
	PQPTriMesh *other = dynamic_cast<PQPTriMesh *>(other_tmesh);
	
	if(other == NULL)
	    return false;
	if(GetNrVertices() == 0 || other_tmesh->GetNrVertices() == 0)
	    return HUGE_VAL;
	

	double pqpR[3][3];
	double pqpT[3] = {T ? T[0] : 0, T ? T[1] : 0, T ? T[2] : 0};
	
	double pqpRother[3][3];
	double pqpTother[3] = {Tother ? Tother[0] : 0, Tother ? Tother[1] : 0, Tother ? Tother[2] : 0};
	
	RotAsPQPRot(R, pqpR);
	RotAsPQPRot(Rother, pqpRother);
	
	PQP_DistanceResult res;
	const double rel_err = 0;
	const double abs_err = 0;
	
	PQP_Distance(&res, pqpR, pqpT, GetPQPModel(),
		     pqpRother, pqpTother, other->GetPQPModel(), 
		     rel_err, abs_err);
	
	if(p1)
	{
	    p1[0] = res.P1()[0];
	    p1[1] = res.P1()[1];
	    p1[2] = res.P1()[2];		

	    Algebra3D::RotMultPoint(R, p1, p1);
	    Algebra3D::TransMultPoint(T, p1, p1);	    
	}
	if(p2)
	{
	    p2[0] = res.P2()[0];
	    p2[1] = res.P2()[1];
	    p2[2] = res.P2()[2];		

	    Algebra3D::RotMultPoint(Rother, p2, p2);
	    Algebra3D::TransMultPoint(Tother, p2, p2);	    
	}
	
	return res.Distance();	
    }

}

    
    
    
    
    

