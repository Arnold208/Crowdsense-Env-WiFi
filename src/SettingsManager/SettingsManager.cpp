#include "SettingsManager/SettingsManager.h"

const char* SettingsManager::_jsonTemplate = 
    "{"
    "    \"frequency\": 0,"
    "    \"temp-offset\": 0.0,"
    "    \"humi-offset\": 0.0,"
    "    \"press-offset\": 0.0,"
    "    \"p1-offset\": 0.0,"
    "    \"p2-offset\": 0.0,"
    "    \"p0-offset\": 0.0,"
    "    \"soun-offset\": 0.0,"
    "    \"lux-offset\": 0.0,"
    "    \"uv-offset\": 0.0"
    "}";

SettingsManager::SettingsManager(fs::FS &fs) : _fs(fs) {}

void SettingsManager::readSettings(const char *path, JsonDocument &doc) {
    File file = _fs.open(path);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }
    deserializeJson(doc, file);
    file.close();
}

void SettingsManager::saveSettings(const char *path, const JsonDocument &doc) {
    File file = _fs.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    serializeJson(doc, file);
    file.close();
}

void SettingsManager::updateSettings(const char *path, int newFrequency, float tempOffset, float humiOffset, float pressOffset, float p1Offset, float p2Offset, float p0Offset, float sounOffset, float luxOffset, float uvOffset) {
    JsonDocument doc;
    readSettings(path, doc);

    // Update values
    doc["frequency"] = newFrequency;
    doc["temp-offset"] = tempOffset;
    doc["humi-offset"] = humiOffset;
    doc["press-offset"] = pressOffset;
    doc["p1-offset"] = p1Offset;
    doc["p2-offset"] = p2Offset;
    doc["p0-offset"] = p0Offset;
    doc["soun-offset"] = sounOffset;
    doc["lux-offset"] = luxOffset;
    doc["uv-offset"] = uvOffset;


    // Save the updated document
    saveSettings(path, doc);
}

void SettingsManager::updateWiFi(const char *path, String ssid, String password) {
    JsonDocument doc;
    readSettings(path, doc);

    // Update values
    doc["ssid"] = ssid;
    doc["password"] = password;
   ;

    // Save the updated document
    saveSettings(path, doc);
}


const char* SettingsManager::getJsonTemplate() const {
    return _jsonTemplate;
}


