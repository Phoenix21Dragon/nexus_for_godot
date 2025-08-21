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
#include <vcg/space/color4.h>
#include <wrap/system/multithreading/util.h>
#include <wrap/system/time/clock.h>

#include "renderer.h"
#include "nexus.h"
#include "token.h"
#include "controller.h"

#include <QDebug>

#ifdef WIN32
//microsoft compiler does not provide it.
/*double log2( double n )
{
	// log(n)/log(2) is log2.
	return log( n )* 3.32192809;
}*/
#endif

#include <godot_cpp/classes/camera3d.hpp>
#include <godot_cpp/classes/viewport.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/projection.hpp>

#include <godot_cpp/classes/array_mesh.hpp>
#include <godot_cpp/classes/mesh_instance3d.hpp>
#include <godot_cpp/classes/standard_material3d.hpp>
#include <godot_cpp/classes/image_texture.hpp>
#include <godot_cpp/classes/material.hpp>
#include <godot_cpp/classes/image.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/array.hpp>
#include <godot_cpp/variant/vector3.hpp>

extern int current_texture;

using namespace nx;
using namespace godot;

void Renderer::update_camera_data(const godot::Camera3D *cam) {
    if (!cam) return;

    // 1) Projektionsmatrix (Godot → float[16])
    godot::Projection proj_godot = cam->get_camera_projection();
    float proj[16];
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 4; c++) {
            proj[c*4 + r] = proj_godot.columns[c][r]; // Column-major -> wird in setView() transponiert -> Matrix44f arbeitet mit row-major
        }
    }

    // 2) ModelView-Matrix
    godot::Transform3D cam_xform = cam->get_global_transform();
	// UtilityFunctions::print("cam_origin: ", cam_xform.origin);
    cam_xform = cam_xform.affine_inverse(); // Welt → Kamera
    float model[16];
    // Basis (Rotation + Scale)
    for (int c = 0; c < 3; c++) {
        godot::Vector3 col = cam_xform.basis.get_column(c);
        model[c * 4 + 0] = col.x;
        model[c * 4 + 1] = col.y;
        model[c * 4 + 2] = col.z;
        model[c * 4 + 3] = 0.0f;
    }
    model[12] = cam_xform.origin.x;
    model[13] = cam_xform.origin.y;
    model[14] = cam_xform.origin.z;
    model[15] = 1.0f;

    // 3) Viewport
    godot::Viewport *vp = cam->get_viewport();
    // int viewport[4] = {0, 0, vp->get_visible_rect().size.x, vp->get_visible_rect().size.y};
    int viewport[4] = {0, 0, 800, 659};


    // 4) Frustum setzen
    metric.frustum.setView(proj, model, viewport);
}


void Stats::resetAll() {
	//memset(this, 0, sizeof(Stats)); //fast hack.
	rendered = patch_rendered = node_rendered = instance_rendered = 0;
	frustum_culled = cone_culled = occlusion_culled = 0;
	error = instance_error = 0;
}

void Stats::resetCurrent() {
	instance_rendered = 0;
	instance_error = 0.0f;
}


void Renderer::startFrame() {
	stats.resetAll();
	frame++;
}


void Renderer::getView(const float *proj, const float *modelview, const int *viewport) {
	metric.getView(proj, modelview, viewport);
}

void Renderer::nearFar(Nexus *nexus, float &neard, float &fard) {
	
	if(!nexus->isReady()) return;
	
	Frustum &frustum = metric.frustum;
	vcg::Sphere3f s = nexus->header.sphere;
	
	float fd = 0;
	float nd = 1e20;
	frustum.updateNearFar(nd, fd, s.Center(), s.Radius());
	
	//if nd gets too small unnecessarily we lose precision in depth buffer.
	//needs to recourse to find closest but we cheat and just look at the closes vertex in the highest level
	if(nd <= frustum.scale()*s.Radius()/4.0f)
		nd = nexus->nodes[nexus->header.n_nodes-2].error * frustum.scale();
	
	/*        float nnd = 1e20;
		nx::Token *token = nexus->getToken(0);
		if(token->lock()) {
			Node &node = nexus->nodes[0];
			NodeData &data = nexus->data[0];
			vcg::Point3f *points = data.coords();
			for(uint i = 0; i < node.nvert; i++)
				frustum.updateNear(nnd, points[i]);
			token->unlock();
		}
		
		float min_distance = nexus->nodes[0].error * frustum.scale();
		
		if(nd == 1e20)
			nd = min_distance;
		else {
			//if we get closest than the approximation error of the first node we are very close :)
			float approximation = nexus->nodes[0].error * frustum.scale();
			nd -= approximation;
			if(nd < min_distance) nd = min_distance;
		}
	} */
	
	if(nd < neard) neard = nd;
	if(fd > fard) fard = fd;
}

Ref<ArrayMesh> Renderer::render(Nexus *nexus, bool get_view, int wait ) {
	UtilityFunctions::print("\n\nRender Frame ", frame);
	controller = nexus->controller;
	if(!nexus->isReady()) return NULL;
	
	// if(get_view) getView();
	
	
	locked.clear();
	last_node = 0;
	
	// mt::Clock time = mt::Clock::currentTime();
	// time.start();
	// if(stats.time.isValid()) {
	// 	int elapsed = stats.time.elapsed();
	// 	if(elapsed > 0) {
	// 		float fps = 1000.0f/elapsed;
	// 		stats.fps = 0.1*fps + 0.9*stats.fps;
	// 	}
	// }
	
	stats.resetCurrent();
	// stats.time = time;
	
	// if(wait) {
	// 	traverse(nexus);
		
	// 	while(1) {
	// 		if(nexus->controller->isWaiting())
	// 			break;
			
	// 		if(time.elapsed() > wait)
	// 			break;
			
	// 		mt::sleep_ms(10);
	// 	}
	// }
	errors.clear();
	errors.resize(nexus->header.n_nodes);
	traverse(nexus);
	stats.instance_rendered = 0;
	
	Ref<ArrayMesh> mesh = renderSelected(nexus);	
	
	for(unsigned int i = 0; i < locked.size(); i++)
		locked[i]->unlock();
	locked.clear();
	
	stats.rendered += stats.instance_rendered;
	if(stats.instance_error > stats.error) stats.error = stats.instance_error;
	return mesh;
}

void Renderer::endFrame() {
	if(controller)
		controller->endFrame();
	//current_error = stats.total_error;
	//current_rendered = stats.total_rendered;
	
	//porca trottola l'udpdate priorities.
	//non costa molto ma se ci sono un sacco di istance e' uno spreco.
	
	//tenere traccia dei controllers?
}

void Renderer::setMode(Renderer::Mode m, bool on) {
	if(on) mode |= m;
	else mode &= ~m;
}

Ref<ArrayMesh> Renderer::renderSelected(Nexus *nexus) {
	// std::cout << "RENDER SELECTED" << std::endl;

	Ref<ArrayMesh> mesh;
	mesh.instantiate();	

	// int GPU_loaded = 0;
	uint32_t last_texture = 0xffffffff;
	for(uint32_t i = 0; i <= last_node; i++) {
		if(!selected[i]) continue;
		
		Node &node = nexus->nodes[i];
		
		if(nexus->header.signature.face.hasIndex() && skipNode(i)) continue;
		
		stats.node_rendered++;
		
	// 	//TODO cleanup frustum..
	// 	vcg::Sphere3f sphere = node.tightSphere();
	// 	if(frustum_culling && metric.frustum.isOutside(sphere.Center(), sphere.Radius())) {
	// 		stats.frustum_culled++;
	// 		continue;
	// 	}
	// 	if(cone_culling && node.cone.Backface(sphere, metric.frustum.viewPoint())) {
	// 		stats.cone_culled++;
	// 		continue;
	// 	}
		
		NodeData &data = nexus->nodedata[i];
		assert(data.memory);

		if (!data.mesh.is_valid())
		{
			nexus->loadGpu(i);
		} else
		{
			UtilityFunctions::print("DONT NEAD TO LOAD NODE ", i, " IN GPU. surface count: ", data.mesh->get_surface_count());
			for (size_t i = 0; i < data.mesh->get_surface_count(); i++)
			{
				UtilityFunctions::print("surface ", i, ": ", data.mesh->surface_get_name(i));
			}
			
		}
		// Mappe Patch Nummer zu Surface Nummer 
		std::unordered_map<int, int> patch_to_surface;
		for (int i = 0; i < data.mesh->get_surface_count(); i++) {
			String name = data.mesh->surface_get_name(i);
			int patch_num = name.substr(7).to_int(); // "patch: " = 7 Zeichen
			patch_to_surface[patch_num] = i;
		}


		for(uint32_t k = node.first_patch; k < node.last_patch(); k++) {
			Patch &patch = nexus->patches[k];

			auto it = patch_to_surface.find(k);
			if(it != patch_to_surface.end()) {
				int surface_idx = it->second;
				mesh->add_surface_from_arrays(
					data.mesh->surface_get_primitive_type(surface_idx),
					data.mesh->surface_get_arrays(surface_idx)
				);
				UtilityFunctions::print("ADDED ", data.mesh->surface_get_name(surface_idx), " for Rendering");
			}

			// // Wenn child Node nicht ausgewählt ist muss dieser Patch gerendert werden
			// if(!selected[patch.node]) { 
			// 	// Iteriere über alle Surfaces der Node bis der passende Patch gefunden ist
			// 	for (int i = 0; i < data.mesh->get_surface_count(); i++)
			// 	{
			// 		if (data.mesh->surface_get_name(i).contains(String::num_int64(k)))
			// 		{
			// 			mesh->add_surface_from_arrays(data.mesh->surface_get_primitive_type(i), data.mesh->surface_get_arrays(i));
			// 		}
					
			// 	}				
			// }
		}

		// // Loading Mesh here
		// nx::Signature& sig = nexus->header.signature;

		// uint32_t nvert = node.nvert;
		// uint16_t* indices = data.faces(sig, nvert);
		// vcg::Point3f* coords = data.coords();
		// vcg::Point2f* uvs = data.texCoords(sig, nvert);
		// vcg::Point3s* normals = data.normals(sig, nvert);

		// // UtilityFunctions::print("\nLOAD GEOMETRY of node: ", i);	
		// uint32_t offset = node.offset;
		// for(uint32_t k = node.first_patch; k < node.last_patch(); k++) {
		// 	Patch &patch = nexus->patches[k];			
			
		// 	if(!selected[patch.node]) { //we need to draw this
		// 		uint32_t tex_index = patch.texture;
		// 		uint32_t next_offset = patch.triangle_offset;
		// 		uint32_t face_count = next_offset - offset;
	
		// 		// UtilityFunctions::print("Patch ", k, ": offset=", offset, ", next_offset=", next_offset, ", face_count=", face_count);
				
		// 		PackedVector3Array godot_vertices;
		// 		PackedVector3Array godot_normals;
		// 		PackedVector2Array godot_uvs;
		// 		PackedInt32Array godot_indices;

		// 		std::map<uint16_t, int> index_map;
		// 		int local_index = 0;


		// 		// Nur betroffene Vertices und Faces sammeln
		// 		for (uint32_t f = offset; f < next_offset; ++f) {
		// 			for (int v = 0; v < 3; ++v) {
		// 				uint16_t global_idx = indices[f * 3 + v];

		// 				if (index_map.find(global_idx) == index_map.end()) {
		// 					Vector3 pos = Vector3(coords[global_idx].X(), coords[global_idx].Y(), coords[global_idx].Z());
		// 					godot_vertices.push_back(pos);

		// 					if (normals)
		// 						godot_normals.push_back(Vector3(normals[global_idx].X(), normals[global_idx].Y(), normals[global_idx].Z()));
		// 					if (uvs)
		// 						godot_uvs.push_back(Vector2(uvs[global_idx].X(), uvs[global_idx].Y()));

		// 					index_map[global_idx] = local_index++;
		// 				}
		// 				godot_indices.push_back(index_map[global_idx]);
		// 			}
		// 		}

		// 		Array arrays;
		// 		arrays.resize(Mesh::ARRAY_MAX);
		// 		arrays[Mesh::ARRAY_VERTEX] = godot_vertices;
		// 		arrays[Mesh::ARRAY_NORMAL] = godot_normals;
		// 		arrays[Mesh::ARRAY_TEX_UV] = godot_uvs;
		// 		arrays[Mesh::ARRAY_INDEX] = godot_indices;

		// 		int surface_index = mesh->get_surface_count();
		// 		mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
		// 		mesh->surface_set_name(surface_index, String("node: {0} patch: {1}").format(Array::make(i, k)));
				
		// 		offset = next_offset;
		// 	}
			
		// }
	}

	return mesh;
}

Traversal::Action Renderer::expand(HeapNode h) {
	// UtilityFunctions::print("  Expand Node: ", h.node, "?");
	if(h.node > last_node) last_node = h.node;
	
	Nexus *nx = (Nexus *)nexus;
	nx::Token &token = nx->tokens[h.node];
	
	Priority newprio = Priority(h.error, frame);
	Priority oldprio = token.getPriority();
	if(oldprio < newprio)
		token.setPriority(newprio);
	
	nx->controller->addToken(&token);
	
	// UtilityFunctions::print("      h.error=", h.error, " < target_error=", target_error); 
	if(h.node != 0 && (h.error < target_error ||
					   (max_rendered && stats.instance_rendered > max_rendered))) {
		// UtilityFunctions::print("      BLOCK");
		return BLOCK;
	}
	
	Node &node = nx->nodes[h.node];
	if(token.lock()) {
		stats.instance_error = h.error;
		errors[h.node] = h.error;
		if(h.visible) { //TODO check proper working on large models
			bool visible;
			vcg::Sphere3f sphere = node.tightSphere();
			// UtilityFunctions::print("      metric.getError() with tightSphere of node ", h.node);
			metric.getError(sphere, node.error, visible);
			if(visible)
				stats.instance_rendered += node.nvert/2;
		}
		locked.push_back(&token);
		// UtilityFunctions::print("      EXPAND");
		return EXPAND;
		
	} else {
		// UtilityFunctions::print("      BLOCK because tocken.lock() is false");
		return BLOCK;
	}
}


float Renderer::nodeError(uint32_t n, bool &visible) {
	Node &node = ((Nexus *)nexus)->nodes[n];
	// UtilityFunctions::print("    metric.getError() with node.sphere of node ", n);
	return metric.getError(node.sphere, node.error, visible); //here we must use the saturated radius.
	//vcg::Sphere3f sphere = node.tightSphere();
	//return metric.getError(sphere, node.error, visible); //here we must use the saturated radius.
}


