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

#include "mixer.h"

namespace emsesp {

REGISTER_FACTORY(Mixer, EMSdevice::DeviceType::MIXER);

uuid::log::Logger Mixer::logger_{F_(mixer), uuid::log::Facility::CONSOLE};

Mixer::Mixer(uint8_t device_type, uint8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand)
    : EMSdevice(device_type, device_id, product_id, version, name, flags, brand) {
    LOG_DEBUG(F("Adding new Mixer with device ID 0x%02X"), device_id);

    if (EMSbus::tx_mode() <= EMS_TXMODE_HW) {
        if (flags == EMSdevice::EMS_DEVICE_FLAG_MMPLUS) {
            if (device_id <= 0x27) {
                // telegram handlers 0x20 - 0x27 for HC
                register_telegram_type(device_id - 0x20 + 0x02D7, F("MMPLUSStatusMessage_HC"), true, [&](std::shared_ptr<const Telegram> t) {
                    process_MMPLUSStatusMessage_HC(t);
                });
            } else {
                // telegram handlers for warm water/DHW 0x28, 0x29
                register_telegram_type(device_id - 0x28 + 0x0331, F("MMPLUSStatusMessage_WWC"), true, [&](std::shared_ptr<const Telegram> t) {
                    process_MMPLUSStatusMessage_WWC(t);
                });
            }
        }

        // EMS 1.0
        if (flags == EMSdevice::EMS_DEVICE_FLAG_MM10) {
            // register_telegram_type(0x00AA, F("MMConfigMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_MMConfigMessage(t); });
            register_telegram_type(0x00AB, F("MMStatusMessage"), true, [&](std::shared_ptr<const Telegram> t) { process_MMStatusMessage(t); });
            // register_telegram_type(0x00AC, F("MMSetMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_MMSetMessage(t); });
        }

        // HT3
        if (flags == EMSdevice::EMS_DEVICE_FLAG_IPM) {
            register_telegram_type(0x010C, F("IPMSetMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_IPMStatusMessage(t); });
        }
    }
}

// output json to web UI
void Mixer::device_info_web(JsonArray & root, uint8_t & part) {
    if (type() == Type::NONE) {
        return; // don't have any values yet
    }

    // fetch the values into a JSON document
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_SMALL);
    //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_SMALL> doc;
    JsonObject                                     json = doc.to<JsonObject>();

    if (!export_values_format(Mqtt::Format::SINGLE, json)) {
        return; // empty
    }

    char prefix_str[10];
    if (type() == Type::HC) {
        snprintf_P(prefix_str, sizeof(prefix_str), PSTR("(hc %d) "), hc_);
        // create_value_json(root, F("flowTempLowLoss"), FPSTR(prefix_str), F_(flowTempLowLoss), F_(degrees), json);
        create_value_json(root, F("flowSetTemp"), FPSTR(prefix_str), F_(flowSetTemp), F_(degrees), json);
        create_value_json(root, F("flowTempHc"), FPSTR(prefix_str), F_(flowTempHc), F_(degrees), json);
        create_value_json(root, F("pumpStatus"), FPSTR(prefix_str), F_(pumpStatus), nullptr, json);
        create_value_json(root, F("valveStatus"), FPSTR(prefix_str), F_(valveStatus), F_(percent), json);
    } else {
        snprintf_P(prefix_str, sizeof(prefix_str), PSTR("(wwc %d) "), hc_);
        create_value_json(root, F("wWTemp"), FPSTR(prefix_str), F_(wWTemp), F_(degrees), json);
        create_value_json(root, F("pumpStatus"), FPSTR(prefix_str), F_(pumpStatus), nullptr, json);
        create_value_json(root, F("tempStatus"), FPSTR(prefix_str), F_(tempStatus), nullptr, json);
    }
}

// check to see if values have been updated
bool Mixer::updated_values() {
    if (changed_) {
        changed_ = false;
        return true;
    }
    return false;
}

// publish values via MQTT
// topic is mixer_data<id>
void Mixer::publish_values(JsonObject & json, bool force) {
    // handle HA first
    if (Mqtt::mqtt_format() == Mqtt::Format::HA) {
        if (!mqtt_ha_config_ || force) {
            register_mqtt_ha_config();
            return;
        }
    }

    if (Mqtt::mqtt_format() == Mqtt::Format::SINGLE) {
        DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_SMALL);
        //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_SMALL> doc;
        JsonObject                                     json_data = doc.to<JsonObject>();
        if (export_values_format(Mqtt::mqtt_format(), json_data)) {
            char topic[30];
            if (type() == Type::HC) {
                snprintf_P(topic, 30, PSTR("mixer_data_hc%d"), hc_);
            } else {
                snprintf_P(topic, 30, PSTR("mixer_data_wwc%d"), hc_);
            }
            Mqtt::publish(topic, doc.as<JsonObject>());
        }
    } else {
        // format is HA or Nested. This is bundled together and sent in emsesp.cpp
        export_values_format(Mqtt::mqtt_format(), json);
    }
}

// publish config topic for HA MQTT Discovery
void Mixer::register_mqtt_ha_config() {
    if (!Mqtt::connected()) {
        return;
    }

    // if we don't have valid values for this HC don't add it ever again
    if (!Helpers::hasValue(pumpStatus_)) {
        return;
    }

    // Create the Master device
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_HA_CONFIG);
    //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_HA_CONFIG> doc;

    char temp[128];
    snprintf_P(temp, sizeof(temp), PSTR("%s Mixer %02X"), Mqtt::base().c_str(), device_id() - 0x20 + 1);
    doc["name"] = temp;

    snprintf_P(temp, sizeof(temp), PSTR("%s_mixer%02X"), Mqtt::base().c_str(), device_id() - 0x20 + 1);
    doc["uniq_id"] = temp;

    doc["ic"] = FJSON("mdi:home-thermometer-outline");

    char stat_t[128];
    snprintf_P(stat_t, sizeof(stat_t), PSTR("%s/mixer_data"), Mqtt::base().c_str());
    doc["stat_t"] = stat_t;

    doc["val_tpl"] = FJSON("{{value_json.type}}"); // HA needs a single value. We take the type which is wwc or hc

    JsonObject dev = doc.createNestedObject("dev");
    snprintf_P(temp, sizeof(temp), PSTR("%s Mixer"), Mqtt::base().c_str());
    dev["name"]    = temp;
    dev["sw"]      = EMSESP_APP_VERSION;
    dev["mf"]      = brand_to_string();
    dev["mdl"]     = this->name();
    JsonArray ids  = dev.createNestedArray("ids");
    snprintf_P(temp, sizeof(temp), PSTR("%s-mixer"), Mqtt::base().c_str());
    ids.add(temp);

    std::string topic(100, '\0');
    if (type() == Type::HC) {
        snprintf_P(&topic[0], topic.capacity() + 1, PSTR("homeassistant/sensor/%s/mixer_hc%d/config"), Mqtt::base().c_str(), hc_);
        Mqtt::publish_ha(topic, doc.as<JsonObject>()); // publish the config payload with retain flag
        char hc_name[10];
        snprintf_P(hc_name, sizeof(hc_name), PSTR("hc%d"), hc_);
        // Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(flowTempLowLoss), device_type(), "flowTempLowLoss", F_(degrees), nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(flowSetTemp), device_type(), "flowSetTemp", F_(degrees), nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(flowTempHc), device_type(), "flowTempHc", F_(degrees), nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(pumpStatus), device_type(), "pumpStatus", nullptr, F_(iconpump));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(valveStatus), device_type(), "valveStatus", F_(percent), F_(iconpercent));
    } else {
        // WWC
        snprintf_P(&topic[0], topic.capacity() + 1, PSTR("homeassistant/sensor/%s/mixer_wwc%d/config"), Mqtt::base().c_str(), hc_);
        Mqtt::publish_ha(topic, doc.as<JsonObject>()); // publish the config payload with retain flag
        char wwc_name[10];
        snprintf_P(wwc_name, sizeof(wwc_name), PSTR("wwc%d"), hc_);
        Mqtt::register_mqtt_ha_sensor(wwc_name, nullptr, F_(wWTemp), device_type(), "wWTemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(wwc_name, nullptr, F_(pumpStatus), device_type(), "pumpStatus", nullptr, F_(iconpump));
        Mqtt::register_mqtt_ha_sensor(wwc_name, nullptr, F_(tempStatus), device_type(), "tempStatus", nullptr, nullptr);
    }

    mqtt_ha_config_ = true; // done
}

bool Mixer::export_values(JsonObject & json, int8_t id) {
    if ((id <= 0) || (type() == Type::HC && id == hc_) || (type() == Type::WWC && id == hc_ + 8)) {
        return export_values_format(Mqtt::Format::NESTED, json);
    }
    return false;
}

// creates JSON doc from values
// returns false if empty
bool Mixer::export_values_format(uint8_t mqtt_format, JsonObject & json) {
    // check if there is data for the mixer unit
    if (type() == Type::NONE) {
        return 0;
    }

    JsonObject json_hc;
    char       hc_name[10]; // hc{1-4}

    if (type() == Type::HC) {
        snprintf_P(hc_name, sizeof(hc_name), PSTR("hc%d"), hc_);
        if (mqtt_format == Mqtt::Format::SINGLE) {
            json_hc      = json;
            json["type"] = FJSON("hc");
        } else if (mqtt_format == Mqtt::Format::HA) {
            json_hc         = json.createNestedObject(hc_name);
            json_hc["type"] = FJSON("hc");
        } else {
            json_hc = json.createNestedObject(hc_name);
        }
        // T0: flow temperature on the low loss header
        // if (Helpers::hasValue(flowTempLowLoss_)) {
        //     json_hc["flowTempLowLoss"] = flowTempLowLoss_;
        // }
        // Setpoint for heating circuit
        if (Helpers::hasValue(flowSetTemp_)) {
            json_hc["flowSetTemp"] = flowSetTemp_;
        }
        // TC1: flow temperature in assigned hc or tank temperature in assigned tank primary circuit
        if (Helpers::hasValue(flowTempHc_)) {
            json_hc["flowTempHc"] = (float)flowTempHc_ / 10;
        }
        // PC1: heating pump in assigned hc -or- PW1: tank primary pump in assigned tank primary circuit (code switch 9 or 10)
        Helpers::json_boolean(json_hc, "pumpStatus", pumpStatus_);
        // VC1: mixing valve actuator in the assigned hc with mixer -or- PW2: DHW circulation pump with connection to module (code switch 9 or 10)
        if (Helpers::hasValue(status_)) {
            json_hc["valveStatus"] = status_;
        }

        return json_hc.size();
    }

    // WWC
    snprintf_P(hc_name, sizeof(hc_name), PSTR("wwc%d"), hc_);
    if (mqtt_format == Mqtt::Format::SINGLE) {
        json_hc      = json;
        json["type"] = FJSON("wwc");
    } else if (mqtt_format == Mqtt::Format::HA) {
        json_hc         = json.createNestedObject(hc_name);
        json_hc["type"] = FJSON("wwc");
    } else {
        json_hc = json.createNestedObject(hc_name);
    }
    if (Helpers::hasValue(flowTempHc_)) {
        json_hc["wWTemp"] = (float)flowTempHc_ / 10;
    }
    Helpers::json_boolean(json_hc, "pumpStatus", pumpStatus_);
    if (Helpers::hasValue(status_)) {
        json_hc["tempStatus"] = status_;
    }

    return json_hc.size();
}

// heating circuits 0x02D7, 0x02D8 etc...
// e.g.  A0 00 FF 00 01 D7 00 00 00 80 00 00 00 00 03 C5
//       A0 0B FF 00 01 D7 00 00 00 80 00 00 00 00 03 80
void Mixer::process_MMPLUSStatusMessage_HC(std::shared_ptr<const Telegram> telegram) {
    type(Type::HC);
    hc_ = telegram->type_id - 0x02D7 + 1;              // determine which circuit this is
    changed_ |= telegram->read_value(flowSetTemp_, 5); // Requested Flow temperature (see Norberts list)
    changed_ |= telegram->read_value(flowTempHc_, 3);  // TC1, is * 10
    changed_ |= telegram->read_bitvalue(pumpStatus_, 0, 0);
    changed_ |= telegram->read_value(status_, 2); // valve status
}

// Mixer warm water loading/DHW - 0x0331, 0x0332
// e.g. A9 00 FF 00 02 32 02 6C 00 3C 00 3C 3C 46 02 03 03 00 3C // on 0x28
//      A8 00 FF 00 02 31 02 35 00 3C 00 3C 3C 46 02 03 03 00 3C // in 0x29
void Mixer::process_MMPLUSStatusMessage_WWC(std::shared_ptr<const Telegram> telegram) {
    type(Type::WWC);
    hc_ = telegram->type_id - 0x0331 + 1;             // determine which circuit this is. There are max 2.
    changed_ |= telegram->read_value(flowTempHc_, 0); // TC1, is * 10
    changed_ |= telegram->read_bitvalue(pumpStatus_, 2, 0);
    changed_ |= telegram->read_value(status_, 11); // temp status
}

// Mixer IMP - 0x010C
// e.g.  A0 00 FF 00 00 0C 01 00 00 00 00 00 54
//       A1 00 FF 00 00 0C 02 04 00 01 1D 00 82
void Mixer::process_IPMStatusMessage(std::shared_ptr<const Telegram> telegram) {
    type(Type::HC);
    hc_ = device_id() - 0x20 + 1;

    // check if circuit is active, 0-off, 1-unmixed, 2-mixed
    uint8_t ismixed = 0;
    telegram->read_value(ismixed, 0);
    if (ismixed == 0) {
        return;
    }

    // do we have a mixed circuit
    if (ismixed == 2) {
        changed_ |= telegram->read_value(flowTempHc_, 3); // TC1, is * 10
        changed_ |= telegram->read_value(status_, 2);     // valve status
    }

    changed_ |= telegram->read_bitvalue(pumpStatus_, 1, 0); // pump is also in unmixed circuits
    changed_ |= telegram->read_value(flowSetTemp_, 5);      // is also in unmixed circuits, see #711
}

// Mixer on a MM10 - 0xAB
// e.g. Mixer Module -> All, type 0xAB, telegram: 21 00 AB 00 2D 01 BE 64 04 01 00 (CRC=15) #data=7
// see also https://github.com/emsesp/EMS-ESP/issues/386
void Mixer::process_MMStatusMessage(std::shared_ptr<const Telegram> telegram) {
    type(Type::HC);

    // the heating circuit is determine by which device_id it is, 0x20 - 0x23
    // 0x21 is position 2. 0x20 is typically reserved for the WM10 switch module
    // see https://github.com/emsesp/EMS-ESP/issues/270 and https://github.com/emsesp/EMS-ESP/issues/386#issuecomment-629610918
    hc_ = device_id() - 0x20 + 1;
    changed_ |= telegram->read_value(flowSetTemp_, 0);      // Setpoint from MMSetMessage
    changed_ |= telegram->read_value(flowTempHc_, 1);       // FV, is * 10
    changed_ |= telegram->read_bitvalue(pumpStatus_, 3, 2); // is 0 or 0x64 (100%), check only bit 2
    changed_ |= telegram->read_value(status_, 4);           // valve status -100 to 100
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// Mixer on a MM10 - 0xAA
// e.g. Thermostat -> Mixer Module, type 0xAA, telegram: 10 21 AA 00 FF 0C 0A 11 0A 32 xx
void Mixer::process_MMConfigMessage(std::shared_ptr<const Telegram> telegram) {
    hc_ = device_id() - 0x20 + 1;
    // pos 0: active FF = on
    // pos 1: valve runtime 0C = 120 sec in units of 10 sec
}

// Mixer on a MM10 - 0xAC
// e.g. Thermostat -> Mixer Module, type 0xAC, telegram: 10 21 AC 00 1E 64 01 AB
void Mixer::process_MMSetMessage(std::shared_ptr<const Telegram> telegram) {
    hc_ = device_id() - 0x20 + 1;
    // pos 0: flowtemp setpoint 1E = 30°C
    // pos 1: position in %
}

#pragma GCC diagnostic pop

} // namespace emsesp
