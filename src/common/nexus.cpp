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
// #include "controller.h"
// #include "globalgl.h"
#include "qtnexusfile.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <iostream>

// #include <QGLWidget>


using namespace nx;

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


// Nexus::Nexus(Controller *control): controller(control), loaded(false), http_stream(false) {
// 	file = new QtNexusFile();

// 	// UtilityFunctions::print("Nexus Constructor");
// }
Nexus::Nexus(): loaded(false), http_stream(false) {
	file = new QTNexusFile();

	// UtilityFunctions::print("Nexus Constructor");
}

Nexus::~Nexus() {
	// close();
	delete file;
}

void Nexus::initIndex() {
	NexusData::initIndex();
	// tokens = new Token[header.n_nodes];
	// for(uint i = 0; i < header.n_nodes; i++)
		// tokens[i] = Token(this, i);
}

void Nexus::loadIndex(char *buffer) {
	NexusData::loadIndex(buffer);

	loaded = true;
}

void nx::Nexus::loadImageFromData(nx::TextureData& data, int t)
{
    Texture& texture = textures[t];

    // mmap zum Lesen
    data.memory = (char *)file->map(texture.getBeginOffset(), texture.getSize());
    if (!data.memory) {
        std::cerr << "Failed mapping texture data" << std::endl;
        exit(1);
    }

    int width, height, channels;
    unsigned char* image_data = stbi_load_from_memory(
        reinterpret_cast<const stbi_uc*>(data.memory),
        static_cast<int>(texture.getSize()),
        &width,
        &height,
        &channels,
        4 // force RGBA
    );

    // unmap direkt danach
    file->unmap((unsigned char*)data.memory, texture.getSize());

    if (!image_data) {
        std::cerr << "Failed loading texture (stb_image)" << std::endl;
        exit(1);
    }

    data.width = width;
    data.height = height;

    int imgsize = width * height * 4;
    // data.memory = new char[imgsize];

    // Bild vertikal flippen
    // int linesize = width * 4;
    // for (int i = 0; i < height; ++i) {
    //     memcpy(
    //         data.memory + (height - 1 - i) * linesize,
    //         image_data + i * linesize,
    //         linesize
    //     );
    // }

	std::cout 
	<< "    tex_index: " << t
	<< " | data.width: " << data.width
	<< " | data.height: " << data.height
	<< " | imgsize: " << imgsize
	<< std::endl;

    // free stb buffer
    stbi_image_free(image_data);
}


void Nexus::loadIndex() {
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

// uint64_t Nexus::loadGpu(uint32_t n) {
// 	NodeData &data = nodedata[n];
// 	assert(data.memory);
// 	assert(data.vbo == 0);

// 	Node &node = nodes[n];

// 	Signature &sig = header.signature;
// 	uint32_t vertex_size = node.nvert*sig.vertex.size();
// 	uint32_t face_size = node.nface*sig.face.size();

// 	char *vertex_start = data.memory;
// 	char *face_start = vertex_start + vertex_size;

// 	glCheckError();

// 	glGenBuffers(1, (GLuint *)(&(data.vbo)));
// 	glBindBuffer(GL_ARRAY_BUFFER, data.vbo);
// 	glBufferData(GL_ARRAY_BUFFER, vertex_size, vertex_start, GL_STATIC_DRAW);

// 	if(node.nface) {
// 		glGenBuffers(1, (GLuint *)(&(data.fbo)));
// 		glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, data.fbo);
// 		glBufferData(GL_ELEMENT_ARRAY_BUFFER, face_size, face_start, GL_STATIC_DRAW);
// 	}

// 	int size = vertex_size + face_size;
// 	if(header.n_textures) {
// 		//be sure to load images
// 		for(uint32_t p = node.first_patch; p < node.last_patch(); p++) {
// 			Patch &patch = patches[p];
// 			uint32_t t = patch.texture;
// 			if(t == 0xffffffff) continue;

// 			TextureData &data = texturedata[t];
// 			data.count_gpu++;
// 			if(texturedata[t].tex) continue;
			
// 			glCheckError();
// 			glGenTextures(1, &data.tex);
// 			glBindTexture(GL_TEXTURE_2D, data.tex);

// 			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, data.width, data.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data.memory);
// 			glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
// 			if(isPowerOfTwo(data.width) && isPowerOfTwo(data.height)) {
// 				glGenerateMipmap(GL_TEXTURE_2D);
// 				glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
// 			} else
// 				glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
			

// 			//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
// 			//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

// 			glCheckError();
// 			size += data.width*data.height*3;
// 			//careful with cache... might create problems to return different sizes in get drop and size
// 			//glGenerateMipmap(GL_TEXTURE_2D);  //Generate mipmaps now!!!
// /*			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAX_LEVEL, 4);

// 			int red = 0xff0000ff;
// 			glTexImage2D(GL_TEXTURE_2D, 1, GL_RGB, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, (char *)&red);
			
// 			int violet = 0x00ffffff;
			
// 			glTexImage2D(GL_TEXTURE_2D, 2, GL_RGB, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, (char *)&violet);

// 			int blue = 0x0000ffff;
			
// 			glTexImage2D(GL_TEXTURE_2D, 3, GL_RGB, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, (char *)&blue);

// 			int green = 0x00ff00ff;
			
			
// 			glTexImage2D(GL_TEXTURE_2D, 4, GL_RGB, 1, 1, 0, GL_RGBA, GL_UNSIGNED_BYTE, (char *)&green); */
			
			

			
// 			glCheckError();
// 		}
// 	}

// 	return size;
// }

// uint64_t Nexus::dropGpu(uint32_t n) {
// 	NodeData &data = nodedata[n];
// 	Node &node = nodes[n];

// #ifndef NO_OPENGL
// 	glDeleteBuffers(1, (GLuint *)(&(data.vbo)));
// 	if(node.nface)
// 		glDeleteBuffers(1, (GLuint *)(&(data.fbo)));
// #endif
// 	data.vbo = data.fbo = 0;

// 	Signature &sig = header.signature;
// 	uint32_t vertex_size = node.nvert*sig.vertex.size();
// 	uint32_t face_size = node.nface*sig.face.size();
// 	int size = vertex_size + face_size;

// 	if(header.n_textures) {
// 		//be sure to load images
// 		for(uint32_t p = node.first_patch; p < node.last_patch(); p++) {
// 			uint32_t t = patches[p].texture;
// 			if(t == 0xffffffff) continue;

// 			TextureData &tdata = texturedata[t];
// 			tdata.count_gpu--;
// 			if(tdata.count_gpu != 0) continue;

// 			glDeleteTextures(1, &tdata.tex);
// 			tdata.tex = 0;
// 			size += tdata.width*tdata.height*3;
// 		}
// 	}
// 	return size;
// }

