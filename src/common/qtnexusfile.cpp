// #include "qtnexusfile.h"

// namespace nx {
// 	void nx::QTNexusFile::setFileName(const char* uri)
// 	{
// 		file.setFileName(uri);
// 	}

// 	bool nx::QTNexusFile::open(OpenMode openmode)
// 	{
// 		QIODevice::OpenMode mode;
// 		if (openmode & OpenMode::Read) mode |= QIODevice::ReadOnly;
// 		if (openmode & OpenMode::Write) mode |= QIODevice::WriteOnly;
// 		if (openmode & OpenMode::Append) mode |= QIODevice::Append;

// 		return file.open(mode);
// 	}

// 	long long int nx::QTNexusFile::read(char* where, size_t length)
// 	{
// 		return file.read(where, length);
// 	}


// 	long long int nx::QTNexusFile::write(char* from, size_t length)
// 	{
// 		return file.write(from, length);
// 	}

// 	size_t QTNexusFile::size()
// 	{
// 		return file.size();
// 	}

// 	void* nx::QTNexusFile::map(size_t from, size_t size)
// 	{
// 		return file.map(from, size);
// 	}

// 	bool nx::QTNexusFile::unmap(void* mapped)
// 	{
// 		return file.unmap((uchar*)mapped);
// 	}

// 	bool nx::QTNexusFile::seek(size_t to)
// 	{
// 		return file.seek(to);
// 	}
// }


// ######### Version 1 #########

// #include "qtnexusfile.h"
// #include <sys/stat.h>  // Für Datei-Größe
// #include <cstring>
// #include <unistd.h>
// #include <iostream>

// #include <fcntl.h>
// #include <sys/mman.h>
// #include <sys/stat.h>
// #include <unistd.h>
// #include <cstring>

// namespace nx {

// void QtNexusFile::setFileName(const char* uri) {
// 	filename = std::string(uri);
// 	}

// bool QtNexusFile::open(OpenMode openmode) {
// 	current_mode = std::ios::binary;

// 	if (openmode & Read)
// 		current_mode |= std::ios::in;
// 	if (openmode & Write)
// 		current_mode |= std::ios::out;
// 	if (openmode & Append)
// 		current_mode |= std::ios::app;

// 	file.open(filename, current_mode);

// 	// Zusätzlich Dateideskriptor für mmap
// 	fd = ::open(filename.c_str(), O_RDONLY); // Nur zum Lesen für mmap
// 	if (fd == -1) {
// 		std::cout << "Failed to open fd for mmap: " << filename << std::endl;
// 	} else {
// 		std::cout << "fd: " << fd << std::endl;
// 	}

// 	return file.is_open() && fd != -1;
// }

// long long int QtNexusFile::read(char* where, size_t length) {
// 	if (!file.is_open() || !(current_mode & std::ios::in))
// 		return -1;

// 	file.read(where, length);
// 	return file.gcount();
// }

// long long int QtNexusFile::write(char* from, size_t length) {
// 	if (!file.is_open() || !(current_mode & std::ios::out))
// 		return -1;

// 	file.write(from, length);
// 	return file ? length : -1;
// }

// size_t QtNexusFile::size() {
// 	struct stat stat_buf;
// 	if (stat(filename.c_str(), &stat_buf) != 0)
// 		return 0;
// 	return stat_buf.st_size;
// }

// bool QtNexusFile::seek(size_t to) {
// 	if (!file.is_open())
// 		return false;

// 	file.seekg(to, std::ios::beg);
// 	file.seekp(to, std::ios::beg);
// 	return true;
// }

// void* QtNexusFile::map(size_t from, size_t size)
// {
// 	void* mappeddata = ::mmap(nullptr, size, PROT_READ, MAP_PRIVATE, fd, from);
// 	if (mappeddata == MAP_FAILED) {
// 		std::cout << "MAP_FAILED" << std::endl;
// 		perror("mmap");
// 	}
// 	// return file.map(from, size);
// 	return mappeddata;
// } 

// } // namespace nx


// ######### Version 2 #########

#include "qtnexusfile.h"
#include <string>
#include <fstream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>

namespace nx {

void QTNexusFile::setFileName(const char* uri) {
	filename = std::string(uri);
}

bool QTNexusFile::open(OpenMode openmode) {
	int flags = 0;
	if (openmode & Read) flags |= O_RDONLY;
	if (openmode & Write) flags |= O_WRONLY;
	if (openmode & Append) flags |= O_APPEND;

	fd = ::open(filename.c_str(), flags);
	if (fd < 0) return false;

	struct stat st;
	if (fstat(fd, &st) == 0) {
		file_size = st.st_size;
	}

	return true;
}

long long int QTNexusFile::read(char* where, size_t length) {
	if (fd < 0) return -1;
	return ::read(fd, where, length);
}

long long int QTNexusFile::write(char* from, size_t length) {
	if (fd < 0) return -1;
	return ::write(fd, from, length);
}

size_t QTNexusFile::size() {
	return file_size;
}

void* QTNexusFile::map(size_t from, size_t size) {
    size_t page_size = sysconf(_SC_PAGE_SIZE);
    size_t page_offset = from % page_size;
    size_t aligned_offset = from - page_offset;

    struct stat st;
    fstat(fd, &st);
    if (from + size > (size_t)st.st_size) {
        std::cerr << "Mapping range exceeds file size!" << std::endl;
        return nullptr;
    }

    void* mapped = ::mmap(nullptr, size + page_offset, PROT_READ, MAP_PRIVATE, fd, aligned_offset);
    if (mapped == MAP_FAILED) {
        perror("mmap");
        return nullptr;
    }

    return static_cast<char*>(mapped) + page_offset;
}


bool QTNexusFile::unmap(void* mapped, size_t size) {
	return ::munmap(mapped, size) == 0;
}

bool QTNexusFile::seek(size_t to) {
	if (fd < 0) return false;
	return ::lseek(fd, to, SEEK_SET) >= 0;
}

// void QTNexusFile::close() {
// 	if (fd >= 0) {
// 		::close(fd);
// 		fd = -1;
// 	}
// }

// ~QTNexusFile() {
// 	close();
// }
// };

} // namespace nx
