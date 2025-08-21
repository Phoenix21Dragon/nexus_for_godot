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
#define _FILE_OFFSET_BITS 64

#include "nexus.h"
#include "controller.h"
#include "globalgl.h"
#include "qtnexusfile.h"

#include <QGLWidget>


using namespace nx;
using namespace godot;


// void _glCheckError(const char *file, int line) {
// #ifndef NDEBUG
// 	GLenum err (glGetError());
	
// 	while(err != GL_NO_ERROR) {
// 		std::string error;
		
// 		switch(err) {
// 		case GL_INVALID_OPERATION:      error="INVALID_OPERATION";      break;
// 		case GL_INVALID_ENUM:           error="INVALID_ENUM";           break;
// 		case GL_INVALID_VALUE:          error="INVALID_VALUE";          break;
// 		case GL_OUT_OF_MEMORY:          error="OUT_OF_MEMORY";          break;
// 		case GL_INVALID_FRAMEBUFFER_OPERATION:  error="INVALID_FRAMEBUFFER_OPERATION";  break;
// 		}
		
// 		std::cerr << "GL_" << error.c_str() <<" - "<<file<<":"<<line<<std::endl;
// 		err=glGetError();
// 	}
// #endif
// }


Nexus::Nexus(Controller *control): controller(control), loaded(false), http_stream(false) {
	file = new QTNexusFile();
}

Nexus::~Nexus() {
	close();
	delete file;
}

bool Nexus::open(const char *_uri) {
	filename = _uri;

	url = std::string(_uri);
	std::cout << "url: " << url << std::endl;

	if(url.compare(0, 7, "http://") == 0) {
		if(!controller)
			throw "No controller, no http streaming";
		http_stream = true;
	}

	if(url.compare(0, 7, "file://") == 0)
		url = url.substr(7, url.size());

	if(!isStreaming()) {
		file->setFileName(url.c_str());
		if(!file->open(NexusFile::ReadWrite))
			//file = fopen(_uri, "rb+");
			//if(!file)
			return false;
	}
	if(!controller) {
		try {
			loadHeader();
			loadIndex();
		} catch(const char *error) {
			return false;
		}
	} else {
		std::cout << "controller->load(this);" << std::endl;
		controller->load(this);
	}
	return true;
}

void Nexus::flush() {
	if(controller) {
		controller->flush(this);
		return;
	}
	NexusData::flush();
	delete []tokens;
}

void Nexus::initIndex() {
	NexusData::initIndex();
	tokens = new Token[header.n_nodes];
	for(uint i = 0; i < header.n_nodes; i++)
		tokens[i] = Token(this, i);
}

void Nexus::loadIndex(char *buffer) {
	NexusData::loadIndex(buffer);

	loaded = true;
}
void nx::Nexus::loadImageFromData(nx::TextureData& data, int t)
{
	Texture& texture = textures[t];
	data.memory = (char*)file->map(texture.getBeginOffset(), texture.getSize());
	if (!data.memory) {
		std::cerr << "Failed mapping texture data" << std::endl;
		exit(0);
	}
	QImage img;
	bool success = img.loadFromData((uchar*)data.memory, texture.getSize());
	file->unmap((uchar*)data.memory);

	if (!success) {
		std::cerr << "Failed loading texture" << std::endl;
		exit(0);
	}

	img = img.convertToFormat(QImage::Format_RGBA8888);
	data.width = img.width();
	data.height = img.height();

	int imgsize = data.width * data.height * 4;
	data.memory = new char[imgsize];

	//flip memory for texture
	int linesize = img.width() * 4;
	char* mem = data.memory + linesize * (img.height() - 1);
	for (int i = 0; i < img.height(); i++) {
		memcpy(mem, img.scanLine(i), linesize);
		mem -= linesize;
	}
}
void Nexus::loadIndex() {
	// std::cout << "Nexus:loadIndex()" << std::endl;
	NexusData::loadIndex();

	loaded = true;
}


bool Nexus::isReady() {
	return loaded;
}

int nextPowerOf2(int n) {
	unsigned count = 0;
	
	if (n && !(n & (n - 1)))
		return n;
	
	while( n != 0) {
		n >>= 1;
		count += 1;
	}
	return 1 << count;
}

bool isPowerOfTwo(unsigned int x) {
	return (x & (x - 1)) == 0;
}

uint64_t Nexus::loadGpu(uint32_t n) {
	UtilityFunctions::print("\nLOAD NODE ", n, " into data.mesh");	
	NodeData &data = nodedata[n];
	assert(data.memory);
	
	data.mesh.instantiate();	

	nx::Node& node = nodes[n];
	nx::Signature& sig = header.signature;

	uint32_t nvert = node.nvert;
	uint16_t* indices = data.faces(sig, nvert);
	vcg::Point3f* coords = data.coords();
	vcg::Point2f* uvs = data.texCoords(sig, nvert);
	vcg::Point3s* normals = data.normals(sig, nvert);


	uint32_t offset = node.offset;
	for (uint32_t k = node.first_patch; k < node.last_patch(); k++) {
		nx::Patch& patch = patches[k];
		uint32_t tex_index = patch.texture;
		uint32_t next_offset = patch.triangle_offset;
		uint32_t face_count = next_offset - offset;

		UtilityFunctions::print("Patch ", k, ": offset=", offset, ", next_offset=", next_offset, ", face_count=", face_count);

		// if (face_count == 0) {
		// 	continue; // Überspringen
		// }

		PackedVector3Array godot_vertices;
		PackedVector3Array godot_normals;
		PackedVector2Array godot_uvs;
		PackedInt32Array godot_indices;

		std::map<uint16_t, int> index_map;
		int local_index = 0;


		// Nur betroffene Vertices und Faces sammeln
		for (uint32_t f = offset; f < next_offset; ++f) {
			for (int v = 0; v < 3; ++v) {
				uint16_t global_idx = indices[f * 3 + v];

				if (index_map.find(global_idx) == index_map.end()) {
					Vector3 pos = Vector3(coords[global_idx].X(), coords[global_idx].Y(), coords[global_idx].Z());
					godot_vertices.push_back(pos);

					if (normals)
						godot_normals.push_back(Vector3(normals[global_idx].X(), normals[global_idx].Y(), normals[global_idx].Z()));
					if (uvs)
						godot_uvs.push_back(Vector2(uvs[global_idx].X(), uvs[global_idx].Y()));

					index_map[global_idx] = local_index++;
				}
				godot_indices.push_back(index_map[global_idx]);
			}
		}

		Array arrays;
		arrays.resize(Mesh::ARRAY_MAX);
		arrays[Mesh::ARRAY_VERTEX] = godot_vertices;
		arrays[Mesh::ARRAY_NORMAL] = godot_normals;
		arrays[Mesh::ARRAY_TEX_UV] = godot_uvs;
		arrays[Mesh::ARRAY_INDEX] = godot_indices;

		int surface_index = data.mesh->get_surface_count();
		data.mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);
		data.mesh->surface_set_name(surface_index, "patch: " + itos(k));
	}

	uint32_t vertex_size = node.nvert*sig.vertex.size();
	uint32_t face_size = node.nface*sig.face.size();
	int size = vertex_size + face_size;
	return size;
}

uint64_t Nexus::dropGpu(uint32_t n) {
	NodeData &data = nodedata[n];
	Node &node = nodes[n];

	data.mesh.unref();

	Signature &sig = header.signature;
	uint32_t vertex_size = node.nvert*sig.vertex.size();
	uint32_t face_size = node.nface*sig.face.size();
	int size = vertex_size + face_size;

	return size;
}

