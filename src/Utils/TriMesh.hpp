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

#ifndef Antipatrea__TriMesh_HPP_
#define Antipatrea__TriMesh_HPP_

#include "Utils/Polygon2D.hpp"
#include "Utils/Constants.hpp"
#include "Utils/Grid.hpp"
#include "Utils/HeightField.hpp"
#include "Utils/GMaterial.hpp"
#include "Utils/GTexture.hpp"
#include <vector>
#include <cstdlib>

namespace Antipatrea
{
class TriMesh
{
  public:
	TriMesh(void)
	{
		m_gMainTexture = NULL;
		m_gMainMaterial = NULL;
		m_manualTexCoords = false;
		Clear();
	}

	virtual ~TriMesh(void)
	{
		Clear();
	}

	struct Vertex
	{
		double m_pos[3];
		double m_normal[3];
		std::vector<int> m_fids;
		bool m_needsUpdateNormal;
	};

	struct TexCoord
	{
		double m_u;
		double m_v;
	};

	struct Face
	{
		Face(void)
		{
			m_normal[0] = 1;
			m_normal[1] = 0;
			m_normal[2] = 0;
			m_unnormalized = 1;
			m_gid = Constants::ID_UNDEFINED;
			m_mid = Constants::ID_UNDEFINED;
			m_tid = Constants::ID_UNDEFINED;
			m_needsUpdate = false;
		}

		std::vector<int> m_vids;
		std::vector<int> m_vtids;
		double m_normal[3];
		double m_unnormalized;
		double m_centroid[3];
		int m_gid;
		int m_mid;
		int m_tid;
		bool m_needsUpdate;
	};

	struct Group
	{
		Group(void) : m_mid(Constants::ID_UNDEFINED),
					  m_tid(Constants::ID_UNDEFINED)
		{
		}

		int m_mid;
		int m_tid;
		std::vector<int> m_fids;
	};

	virtual void Clear(void);

	virtual int GetNrVertices(void) const
	{
		return m_vertices.size();
	}
	virtual const double *GetVertex(const int i) const
	{
		return m_vertices[i]->m_pos;
	}
	virtual const double *GetVertexNormal(const int i)
	{
		if (m_vertices[i]->m_needsUpdateNormal)
			UpdateVertexNormal(i);
		return m_vertices[i]->m_normal;
	}

	virtual int GetNrTexCoords(void) const
	{
		return m_texCoords.size();
	}

	virtual TexCoord GetTexCoord(const int i) const
	{
		return m_texCoords[i];
	}

	virtual void AddTexCoord(const double u, const double v)
	{
		TexCoord t;
		t.m_u = u;
		t.m_v = v;
		m_texCoords.push_back(t);
	}
	virtual void AddTexCoord(TexCoord t)
	{
		m_texCoords.push_back(t);
	}

	virtual int GetNrFaces(void) const
	{
		return m_faces.size();
	}

	virtual const Face *GetFace(const int i)
	{
		if (m_faces[i]->m_needsUpdate)
			UpdateFace(i);
		return m_faces[i];
	}

	virtual int GetNrGroups(void) const
	{
		return m_groups.size();
	}
	virtual const Group *GetGroup(const int i) const
	{
		return m_groups[i];
	}
	virtual Group *GetGroup(const int i)
	{
		return m_groups[i];
	}

	virtual int GetNrMaterials(void) const
	{
		return m_materials.size();
	}
	virtual const GMaterial *GetMaterial(const int i) const
	{
		return m_materials[i];
	}
	virtual GMaterial *GetMaterial(const int i)
	{
		return m_materials[i];
	}
	virtual int GetCurrentMaterialId(void) const
	{
		return m_midCurr;
	}

	virtual int GetNrTextures(void) const
	{
		return m_textures.size();
	}
	virtual const GTexture *GetTexture(const int i) const
	{
		return m_textures[i];
	}
	virtual GTexture *GetTexture(const int i)
	{
		return m_textures[i];
	}
	virtual int GetCurrentTextureId(void) const
	{
		return m_tidCurr;
	}

	virtual void AddVertex(const double v[],
						   const double normal[] = NULL);

	virtual void AddVertex(const double vx,
						   const double vy,
						   const double vz,
						   const double nx = 0,
						   const double ny = 0,
						   const double nz = 1);

	virtual void AddVertices(const int n,
							 const double vert[],
							 const double normals[] = NULL);

	virtual void SetVertex(const int i, const double v[]);

	virtual void SetVertex(const int i,
						   const double vx,
						   const double vy,
						   const double vz);

	virtual void SetVertexNormal(const int i, const double normal[]);

	virtual void SetVertexNormal(const int i,
								 const double nx,
								 const double ny,
								 const double nz);

	virtual void AddGroup(void);

	virtual void SetCurrentGroup(const int gcurr);

	virtual void AddMaterial(GMaterial *const gMaterial);

	virtual void SetCurrentMaterial(const int mcurr);

	virtual void AddTexture(GTexture *const gTexture);

	virtual void SetCurrentTexture(const int idCurrTexture);

	virtual void AddFace(Face *const f);

	virtual void AddTriangle(const int tri[]);

	virtual void AddTriangle(const int vt1, const int vt2, const int vt3);

	virtual void AddTriangles(const int n, const int tri[]);

	virtual void AddTriangle(const double v1[], const double v2[], const double v3[]);

	virtual void AddTriangle(const double vtri[]);

	virtual void AddTriangles(const int n, const double tris[]);

	virtual void AddTriMesh(TriMesh &tmesh);

	virtual void AddQuad(const int vid1, const int vid2, const int vid3, const int vid4, const bool asTriangles = false);

	virtual void AddQuad(const int vids[], const bool asTriangles = false);

	virtual void AddQuads(const int n, const int vids[], const bool asTriangles = false);

	virtual void AddQuad(const double v1[],
						 const double v2[],
						 const double v3[],
						 const double v4[], const bool asTriangles = false);

	virtual void AddQuad(const double quad[], const bool asTriangles = false);

	virtual void AddQuads(const int n, const double quads[], const bool asTriangles = false);

	virtual void AddConvexPolygon(const int n, const double vertices[], const bool asTriangles = false);

	virtual void AddRegularPolygon(const double cx,
								   const double cy,
								   const double cz,
								   const double r,
								   const int nsides,
								   const bool asTriangles = false);

	virtual void AddPolygon(Polygon2D & poly);

	virtual void AddExtrudedPolygon(Polygon2D & poly,
									const double zmin, const double zmax,
									const bool useHeightsAtMin = false,
									const bool useHeightsAtMax = false);

	virtual void AddCapCapStripCCW(const int n,
								   const int start_base,
								   const int start_top,
								   const bool close);

	virtual void AddCapCapStripCW(const int n,
								  const int start_base,
								  const int start_top,
								  const bool close);

	virtual void AddCapTopStripCCW(const int n,
								   const int start_base,
								   const int start_top,
								   const bool close);

	virtual void AddCapTopStripCW(const int n,
								  const int start_base,
								  const int start_top,
								  const bool close);

	virtual void AddBox2D(const double xmin, const double ymin,
						  const double xmax, const double ymax);

	virtual void AddBox(const double xmin,
						const double ymin,
						const double zmin,
						const double xmax,
						const double ymax,
						const double zmax);

	virtual void AddBoundaries(const double xmin,
							   const double ymin,
							   const double zmin,
							   const double xmax,
							   const double ymax,
							   const double zmax,
							   const double thick);

	virtual void AddBoundaries2D(const double xmin,
								 const double ymin,
								 const double xmax,
								 const double ymax,
								 const double thick);

	virtual void ApplyTrans(const int vstart, const int vend,
							const double x, const double y, const double z);

	virtual void ApplyTrans(const int vstart, const int vend, const double T[]);

	virtual void ApplyRot(const int vstart, const int vend, const double R[]);

	virtual void ApplyTransRot(const int vstart, const int vend, const double TR[]);

	virtual void ApplyQuat(const int vstart, const int vend, const double Q[]);

	virtual void ApplyTransQuat(const int vstart, const int vend, const double TQ[]);

	virtual void ApplyScaling(const int vstart, const int vend,
							  const double sx, const double sy, const double sz);

	virtual void ApplyScaling(const int vstart, const int vend, const double s[]);

	virtual void GetBoundingBoxMinMax(const int vstart, const int vend,
									  double min[], double max[]) const;

	const double *GetBoundingBoxMin(void);

	const double *GetBoundingBoxMax(void);

	virtual void AdjustToFitBoundingBoxMinMax(const int vstart, const int vend,
											  const double xmin, const double ymin, const double zmin,
											  const double xmax, const double ymax, const double zmax);

	virtual void AdjustToFitBoundingBoxMinMax(const int vstart, const int vend,
											  const double min[], const double max[]);

	virtual bool Collision(const double T[], const double R[],
						   TriMesh *const other_tmesh,
						   const double other_T[], const double other_R[])
	{
		return false;
	}

	virtual bool DistanceThreshold(const double T[], const double R[],
								   TriMesh *const other_tmesh,
								   const double other_T[], const double other_R[],
								   const double tol,
								   double *const d = NULL,
								   double p1[] = NULL,
								   double p2[] = NULL)
	{
		return true;
	}

	virtual double Distance(const double T[], const double R[],
							TriMesh *const other_tmesh,
							const double other_T[],
							const double other_R[],
							double p1[] = NULL,
							double p2[] = NULL)
	{
		return 0;
	}

	virtual const GTexture *GetMainTexture(void) const
	{
		return m_gMainTexture;
	}

	virtual GTexture *GetMainTexture(void)
	{
		return m_gMainTexture;
	}

	virtual const GMaterial *GetMainMaterial(void) const
	{
		return m_gMainMaterial;
	}

	virtual GMaterial *GetMainMaterial(void)
	{
		return m_gMainMaterial;
	}

	virtual void SetMainTexture(GTexture *gTex)
	{
		m_gMainTexture = gTex;
	}

	virtual void SetMainMaterial(GMaterial *gMat)
	{
		m_gMainMaterial = gMat;
	}

	virtual void GenerateRandomTextureCoords(void);

	virtual void Draw(void);

	virtual void GetFacesInsideSphere(const double center[3], const double r, std::vector<int> *fids);

	virtual int ClosestVertex(const double p[2]);

	virtual void UseManualTexCoords(const bool use)
	{
		m_manualTexCoords = use;
	}

	virtual void AddHeightField(const HeightField & hf);

	virtual void ReadOgreMesh(const char meshName[], const char gtexName[]);

	virtual std::vector<int> *GetGroupsNotForDrawing(void)
	{
		return &m_gidsNotForDrawing;
	}

  protected:
	virtual void OnVertexChange(const int vid);
	virtual void OnFaceChange(const int fid);
	virtual void UpdateBoundingBox(void);
	virtual void UpdateFace(const int fid);
	virtual void UpdateVertexNormal(const int vid);

	bool m_updateBBox;
	double m_bboxMin[3];
	double m_bboxMax[3];

	std::vector<Face *> m_faces;
	std::vector<Group *> m_groups;
	int m_gidCurr;
	std::vector<GMaterial *> m_materials;
	int m_midCurr;
	std::vector<GTexture *> m_textures;
	int m_tidCurr;

	GTexture *m_gMainTexture;
	GMaterial *m_gMainMaterial;
	bool m_manualTexCoords;

	std::vector<Vertex *> m_vertices;
	std::vector<TexCoord> m_texCoords;

	std::vector<int> m_gidsNotForDrawing;
};
}

#endif
