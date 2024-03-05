#ifndef SDCARDMANAGER_H
#define SDCARDMANAGER_H

#include <FS.h>
#include <SD.h>

class SDCardManager {
public:
    SDCardManager();  // Constructor
    void initialize();  // Method to initialize SD card
    void createDirectory(const char* path);  // Method to create a directory on the SD card
    void removeDirectory(const char* path);  // Method to remove a directory from the SD card
    void readFile(const char* path);  // Method to read a file from the SD card
    void writeFile(const char* path, const char* message);  // Method to write to a file on the SD card
    void appendFile(const char* path, const char* message);  // Method to append to a file on the SD card
    void renameFile(const char* path1, const char* path2);  // Method to rename a file on the SD card
    void deleteFile(const char* path);  // Method to delete a file from the SD card
};

#endif
