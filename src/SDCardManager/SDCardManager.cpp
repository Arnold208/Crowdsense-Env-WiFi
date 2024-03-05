#include "SDCardManager/SDCardManager.h"

SDCardManager::SDCardManager() {
    // Constructor
}

void SDCardManager::initialize() {
    if (!SD.begin(5)) {
        Serial.println("Card Mount Failed");
    }
}

void SDCardManager::createDirectory(const char* path) {
    Serial.printf("Creating Dir: %s\n", path);
    if (SD.mkdir(path)) {
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void SDCardManager::removeDirectory(const char* path) {
    Serial.printf("Removing Dir: %s\n", path);
    if (SD.rmdir(path)) {
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

void SDCardManager::readFile(const char* path) {
    Serial.printf("Reading file: %s\n", path);
    File file = SD.open(path);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }
    Serial.print("Read from file: ");
    while (file.available()) {
        Serial.write(file.read());
    }
    file.close();
}

void SDCardManager::writeFile(const char* path, const char* message) {
    Serial.printf("Writing file: %s\n", path);
    File file = SD.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (file.print(message)) {
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void SDCardManager::appendFile(const char* path, const char* message) {
    Serial.printf("Appending to file: %s\n", path);
    File file = SD.open(path, FILE_APPEND);
    if (!file) {
        Serial.println("Failed to open file for appending");
        return;
    }
    if (file.print(message) && file.println()) {
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void SDCardManager::renameFile(const char* path1, const char* path2) {
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (SD.rename(path1, path2)) {
        Serial.println("File renamed");
    } else {
        Serial.println("Rename failed");
    }
}

void SDCardManager::deleteFile(const char* path) {
    Serial.printf("Deleting file: %s\n", path);
    if (SD.remove(path)) {
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}