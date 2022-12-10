/*
 * EMS-ESP - https://github.com/emsesp/EMS-ESP
 * Copyright 2020  Paul Derbyshire
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "heatpump.h"

namespace emsesp {

REGISTER_FACTORY(Heatpump, EMSdevice::DeviceType::HEATPUMP);

uuid::log::Logger Heatpump::logger_{F_(heatpump), uuid::log::Facility::CONSOLE};

Heatpump::Heatpump(uint8_t device_type, uint8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand)
    : EMSdevice(device_type, device_id, product_id, version, name, flags, brand) {
    LOG_DEBUG(F("Adding new Heat Pump module with device ID 0x%02X"), device_id);

    if (EMSbus::tx_mode() <= EMS_TXMODE_HW) {
        // telegram handlers
        register_telegram_type(0x042B, F("HP1"), true, [&](std::shared_ptr<const Telegram> t) { process_HPMonitor1(t); });
        register_telegram_type(0x047B, F("HP2"), true, [&](std::shared_ptr<const Telegram> t) { process_HPMonitor2(t); });
    }
}

// creates JSON doc from values
// returns false if empty
bool Heatpump::export_values(JsonObject & json, int8_t id) {
    if (Helpers::hasValue(airHumidity_)) {
        json["airHumidity"] = (float)airHumidity_ / 2;
    }

    if (Helpers::hasValue(dewTemperature_)) {
        json["dewTemperature"] = dewTemperature_;
    }

    return json.size();
}

void Heatpump::device_info_web(JsonArray & root, uint8_t & part) {
    // fetch the values into a JSON document
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_SMALL);
    //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_SMALL> doc;
    JsonObject                                     json = doc.to<JsonObject>();
    if (!export_values(json)) {
        return; // empty
    }

    create_value_json(root, F("airHumidity"), nullptr, F_(airHumidity), F_(percent), json);
    create_value_json(root, F("dewTemperature"), nullptr, F_(dewTemperature), F_(degrees), json);
}

// publish values via MQTT
void Heatpump::publish_values(JsonObject & json, bool force) {
    // handle HA first
    if (Mqtt::mqtt_format() == Mqtt::Format::HA) {
        if (!mqtt_ha_config_ || force) {
            register_mqtt_ha_config();
            return;
        }
    }

    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_SMALL);
    //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_SMALL> doc;
    JsonObject                                     json_data = doc.to<JsonObject>();
    if (export_values(json_data)) {
        Mqtt::publish(F("heatpump_data"), doc.as<JsonObject>());
    }
}

void Heatpump::register_mqtt_ha_config() {
    if (!Mqtt::connected()) {
        return;
    }

    // Create the Master device
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_HA_CONFIG);
    //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_HA_CONFIG> doc;
    doc["name"]    = F_(EMSESP);
    doc["uniq_id"] = F_(heatpump);
    doc["ic"]      = F_(iconpump);

    char temp[128];
    snprintf_P(temp, sizeof(temp), PSTR("%s/heatpump_data"), Mqtt::base().c_str());
    doc["stat_t"] = temp;

    doc["val_tpl"] = FJSON("{{value_json.airHumidity}}");

    JsonObject dev = doc.createNestedObject("dev");
    snprintf_P(temp, sizeof(temp), PSTR("%s Heat Pump"), Mqtt::base().c_str());
    dev["name"]    = temp;
    dev["sw"]      = EMSESP_APP_VERSION;
    dev["mf"]      = brand_to_string();
    dev["mdl"]     = this->name();
    JsonArray ids  = dev.createNestedArray("ids");
    snprintf_P(temp, sizeof(temp), PSTR("%s-heatpump"), Mqtt::base().c_str());
    ids.add(temp);
    snprintf_P(temp, sizeof(temp), PSTR("homeassistant/sensor/%s/heatpump/config"), Mqtt::base().c_str());
    Mqtt::publish_ha(temp, doc.as<JsonObject>()); // publish the config payload with retain flag

    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(airHumidity), device_type(), "airHumidity", F_(percent), nullptr);
    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(dewTemperature), device_type(), "dewTemperature", F_(degrees), nullptr);

    mqtt_ha_config_ = true; // done
}

// check to see if values have been updated
bool Heatpump::updated_values() {
    if (changed_) {
        changed_ = false;
        return true;
    }
    return false;
}

/*
 * Type 0x47B - HeatPump Monitor 2
 * e.g. "38 10 FF 00 03 7B 08 24 00 4B"
 */
void Heatpump::process_HPMonitor2(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(dewTemperature_, 0);
    changed_ |= telegram->read_value(airHumidity_, 1);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

/*
 * Type 0x42B- HeatPump Monitor 1
 * e.g. "38 10 FF 00 03 2B 00 D1 08 2A 01"
 */
void Heatpump::process_HPMonitor1(std::shared_ptr<const Telegram> telegram) {
    // still to implement
}

#pragma GCC diagnostic pop

} // namespace emsesp