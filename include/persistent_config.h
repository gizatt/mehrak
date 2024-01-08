/*
    Utility for managing a persistent configuration library backed into nonvoltile storage,
    with an BLE frontend for modification.
*/

#pragma once

#include <InternalFileSystem.h>
#include <bluefruit.h>
#include <unordered_map>
#include <string>

class FloatConfigValue
{

public:
    FloatConfigValue(std::string name, float value, BLEUuid uuid) : _name(name),
                                                      _file(InternalFS)
    {
        // Get value from existing file if it exists; otherwise
        // start that file.
        _file.open(_name.c_str(), Adafruit_LittleFS_Namespace::FILE_O_READ);
        if (_file)
        {
            // Data existed, read in our value.
            _file.read(&_value_synced_to_file, sizeof(float));
            // Assume success.
             _file.close();
        } else
        {
            _write_value(value);
        }

        // Create GATT config.
        _ble_characteristic = new BLECharacteristic(uuid);
        _ble_characteristic->setProperties(
            CHR_PROPS_READ | CHR_PROPS_WRITE);
        _ble_characteristic->setPermission(SECMODE_OPEN, SECMODE_OPEN);
        _ble_characteristic->setUserDescriptor(_name.c_str());
        _ble_characteristic->setFixedLen(sizeof(float));
        _ble_characteristic->begin();
        _ble_characteristic->writeFloat(_value_synced_to_file);
    }

    float get_value()
    {
        float new_value = _ble_characteristic->readFloat();
        if (new_value != _value_synced_to_file)
        {
            _write_value(new_value);
        }
        return new_value;
    }

    void set_value(float value)
    {
        _write_value(value);
        _ble_characteristic->writeFloat(value);
    }

private:
    bool _write_value(float value)
    {
        bool wrote = false;
        if (_file.open(_name.c_str(), Adafruit_LittleFS_Namespace::FILE_O_WRITE))
        {
            _file.seek(0);
            _file.write((const char *)&value, sizeof(float));
           _value_synced_to_file = value;
            _file.close();
            wrote = true;
        }
        return wrote;
    }

    std::string _name;
    BLECharacteristic *_ble_characteristic;
    float _value_synced_to_file;
    Adafruit_LittleFS_Namespace::File _file;
};

class PersistentConfigManager
{
public:
    PersistentConfigManager(std::string service_name, BLEUuid uuid = 0x1234) : _ble_service(uuid)
    {
        _ble_service.begin();
    }

    float get_value(std::string name, float default_value = 0.0)
    {
        auto config_entry = _config_entries.find(name);
        if (config_entry == _config_entries.end())
        {
            return _setup_config_entry(name, default_value);
        }
        else
        {
            return config_entry->second->get_value();
        }
    }

    void set_value(std::string name, float value)
    {
        auto config_entry = _config_entries.find(name);
        if (config_entry == _config_entries.end())
        {
            _setup_config_entry(name, value);
        }
        else
        {
            config_entry->second->set_value(value);
        }
    }

private:
    float _setup_config_entry(std::string name, float value)
    {
        // Add an entry, including setting up a BLE characteristic,
        // for a value of the given name and initial value.
        _config_entries[name] = new FloatConfigValue(name, value, _ble_service.uuid._uuid.uuid + _config_entries.size());
        return _config_entries[name]->get_value();
    }

    BLEService _ble_service;
    std::unordered_map<std::string, FloatConfigValue *> _config_entries;
};