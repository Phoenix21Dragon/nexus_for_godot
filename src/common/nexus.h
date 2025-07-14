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
#ifndef NX_NEXUS_H
#define NX_NEXUS_H

#include "nexusdata.h"


typedef void CURL;

namespace nx {

class Nexus: public NexusData {
public:
	Nexus();
	~Nexus();
	
	bool isReady();
	bool isStreaming() { return http_stream; }

	void initIndex();
	void loadIndex();
	void loadIndex(char *buffer);

	bool loaded;
	bool http_stream;

	//tempoarary texture loading from file
	std::string filename;
};

} //namespace
#endif // NX_NEXUS_H
