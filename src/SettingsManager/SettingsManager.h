#ifndef SETTINGS_MANAGER_H
#define SETTINGS_MANAGER_H

#include <Arduino.h>
#include <FS.h>
#include <ArduinoJson.h>

class SettingsManager {
public:
    SettingsManager(fs::FS &fs);
    void readSettings(const char *path, JsonDocument &doc);
    void saveSettings(const char *path, const JsonDocument &doc);
    void updateSettings(const char *path, int newFrequency, float tempOffset, float humiOffset, float pressOffset, float p1Offset, float p2Offset, float p0Offset, float sounOffset, float luxOffset, float uvOffset);
    void updateWiFi(const char *path, String ssid, String password);
    const char* getJsonTemplate() const;

private:
    // Private member variable for JSON template
    static const char* _jsonTemplate;
    fs::FS &_fs;
};

#endif
