#ifndef NX_MESH_H
#define NX_MESH_H

#include <vcg/space/point3.h>
#include <vcg/space/color4.h>
#include "trianglesoup.h"
// #include <QString>

// stuff to define the mesh
#include <vcg/complex/complex.h>
#include <wrap/io_trimesh/export.h> //DEBUG

#include <vcg/math/quadric.h>

// update
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>

// local optimization
#include <vcg/complex/algorithms/local_optimization.h>
#include <vcg/complex/algorithms/local_optimization/tri_edge_collapse_quadric.h>


/*
Nexus

Copyright(C) 2012 - Federico Ponchio
ISTI - Italian National Research Council - Visual Computing Lab

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License (http://www.gnu.org/licenses/gpl.txt)
for more details.
*/
#include "../common/nexus.h"
#include "../common/cone.h"

//Quadrics simplification structures.
class AVertex;
class AEdge;
class AFace;

struct AUsedTypes: public vcg::UsedTypes<vcg::Use<AVertex>::AsVertexType,vcg::Use<AEdge>::AsEdgeType,vcg::Use<AFace>::AsFaceType>{};

class AVertex  : public vcg::Vertex<
		AUsedTypes,
		vcg::vertex::VFAdj,
		vcg::vertex::Coord3f,
		vcg::vertex::Normal3f,
		vcg::vertex::TexCoord2f,
		vcg::vertex::Color4b,
		vcg::vertex::Mark,
		vcg::vertex::BitFlags> {
public:
	unsigned int node;
	AVertex() { q.SetZero(); } //TODO remove this, it's only because of a stupid assert is valid on copy
	//int position;
	vcg::math::Quadric<double> &Qd() {return q;}

private:
	vcg::math::Quadric<double> q;

};

class VertexNodeCompare {
public:
	bool operator()(const AVertex &va, const AVertex &vb) const {
		return va.node < vb.node;
	}
};

class VertexCompare {
public:
	bool operator()(const AVertex &va, const AVertex &vb) const {
		const vcg::Point3f &a = va.cP();
		const vcg::Point3f &b = vb.cP();
		if(a[2] < b[2]) return true;
		if(a[2] > b[2]) return false;
		if(a[1] < b[1]) return true;
		if(a[1] > b[1]) return false;
		return a[0] < b[0];
	}
};

class AEdge : public vcg::Edge< AUsedTypes> {};

class AFace: public vcg::Face<
		AUsedTypes,
		vcg::face::VFAdj,
		vcg::face::VertexRef,
		vcg::face::BitFlags> {
public:
	unsigned int node;
	bool operator<(const AFace &t) const {
		return node < t.node;
	}
};

class Mesh: public vcg::tri::TriMesh<std::vector<AVertex>, std::vector<AFace> > {
public:
	enum Simplification { QUADRICS, EDGE, CLUSTER, RANDOM };
	void load(Soup &soup);
	void load(Cloud &soup);
	//void lock(std::vector<bool> &locked);
	void lockVertices();
	void save(Soup &soup, unsigned int node);
	void getTriangles(Triangle *triangles, unsigned int node);
	void getVertices(Splat *vertices, unsigned int node);

	float simplify(unsigned short target_faces, Simplification method);
	std::vector<AVertex> simplifyCloud(unsigned short target_vertices); //return removed vertices
	float averageDistance();

	void savePly(std::string filename);
	nx::Node getNode();
	unsigned int serializedSize(const nx::Signature &sig);
	//appends nodes found in the mesh
	void serialize(unsigned char *buffer, nx::Signature &sig, std::vector<nx::Patch> &patches);

	vcg::Sphere3f boundingSphere();
	nx::Cone3s normalsCone();

	float randomSimplify(unsigned short target_faces);
	void quadricInit();
	float quadricSimplify(unsigned short target_faces);

	float edgeLengthError();

protected:
	vcg::LocalOptimization<Mesh> *deciSession = nullptr;
	vcg::tri::TriEdgeCollapseQuadricParameter *qparams = nullptr;
};

typedef vcg::tri::BasicVertexPair<AVertex> VertexPair;

class TriEdgeCollapse:
		public vcg::tri::TriEdgeCollapseQuadric< Mesh, VertexPair, TriEdgeCollapse, vcg::tri::QInfoStandard<AVertex>  > {
public:
	typedef  vcg::tri::TriEdgeCollapseQuadric< Mesh,  VertexPair, TriEdgeCollapse, vcg::tri::QInfoStandard<AVertex>  > TECQ;
	typedef  Mesh::VertexType::EdgeType EdgeType;
	inline TriEdgeCollapse(  const VertexPair &p, int i, vcg::BaseParameterClass *pp) :TECQ(p,i,pp){}
};

#endif // NX_MESH_H
