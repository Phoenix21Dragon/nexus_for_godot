#ifndef NX_QTNEXUSFILE_H
#define NX_QTNEXUSFILE_H

#include "nexusfile.h"
#include <string>

namespace nx {
	class QTNexusFile 
		: public NexusFile {
	
	private:
	// QFile file; 
	std::string filename;
	int fd = -1;
	size_t file_size = 0;
	
	public:
		void setFileName(const char* uri) override;
		bool open(OpenMode openmode) override;
		long long int read(char* where, size_t length) override;
		long long int write(char* from, size_t length) override;
		size_t size() override;
		void* map(size_t from, size_t size) override;
		bool unmap(void* mapped, size_t size) override;
		bool seek(size_t to) override;
	};
}

#endif // NX_QTNEXUSFILE_H

// #ifndef NX_NONQTNEXUSFILE_H
// #define NX_NONQTNEXUSFILE_H

// #include "nexusfile.h"
// #include <fstream>
// #include <string>

// namespace nx {
// 	class QtNexusFile : public NexusFile {
// 	private:
// 		std::fstream file;
// 		std::string filename;
// 		std::ios::openmode current_mode;
// 		int fd = -1;

// 	public:
// 		void setFileName(const char* uri) override;
// 		bool open(OpenMode openmode) override;
// 		long long int read(char* where, size_t length) override;
// 		long long int write(char* from, size_t length) override;
// 		size_t size() override;
// 		void* map(size_t from, size_t size) override; 
// 		bool unmap(void* mapped) override { return false; } // Optional
// 		bool seek(size_t to) override;
// 	};
// }

// #endif // NX_NONQTNEXUSFILE_H
