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

#include "thermostat.h"
namespace emsesp {

REGISTER_FACTORY(Thermostat, EMSdevice::DeviceType::THERMOSTAT);

uuid::log::Logger Thermostat::logger_{F_(thermostat), uuid::log::Facility::CONSOLE};

Thermostat::Thermostat(uint8_t device_type, uint8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand)
    : EMSdevice(device_type, device_id, product_id, version, name, flags, brand) {
    uint8_t actual_master_thermostat = EMSESP::actual_master_thermostat(); // what we're actually using
    uint8_t master_thermostat        = EMSESP_DEFAULT_MASTER_THERMOSTAT;
    EMSESP::webSettingsService.read([&](WebSettings & settings) {
        master_thermostat = settings.master_thermostat; // what the user has defined
    });

    uint8_t model = this->model();

    if (EMSbus::tx_mode() <= EMS_TXMODE_HW) {
        // if we're on auto mode, register this thermostat if it has a device id of 0x10, 0x17 or 0x18
        // or if its the master thermostat we defined
        // see https://github.com/emsesp/EMS-ESP/issues/362#issuecomment-629628161
        if ((master_thermostat == device_id)
            || ((master_thermostat == EMSESP_DEFAULT_MASTER_THERMOSTAT) && (device_id < 0x19)
                && ((actual_master_thermostat == EMSESP_DEFAULT_MASTER_THERMOSTAT) || (device_id < actual_master_thermostat)))) {
            EMSESP::actual_master_thermostat(device_id);
            actual_master_thermostat = device_id;
            reserve_mem(15); // reserve some space for the telegram registries, to avoid memory fragmentation

            // common telegram handlers
            register_telegram_type(EMS_TYPE_RCOutdoorTemp, F("RCOutdoorTemp"), false, [&](std::shared_ptr<const Telegram> t) { process_RCOutdoorTemp(t); });
            register_telegram_type(EMS_TYPE_RCTime, F("RCTime"), false, [&](std::shared_ptr<const Telegram> t) { process_RCTime(t); });
            register_telegram_type(0xA2, F("RCError"), false, [&](std::shared_ptr<const Telegram> t) { process_RCError(t); });
            register_telegram_type(0x12, F("RCErrorMessage"), false, [&](std::shared_ptr<const Telegram> t) { process_RCErrorMessage(t); });
        }
        // RC10
        if (model == EMSdevice::EMS_DEVICE_FLAG_RC10) {
            monitor_typeids = {0xB1};
            set_typeids     = {0xB0};
            for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
                register_telegram_type(monitor_typeids[i], F("RC10Monitor"), false, [&](std::shared_ptr<const Telegram> t) { process_RC10Monitor(t); });
                register_telegram_type(set_typeids[i], F("RC10Set"), false, [&](std::shared_ptr<const Telegram> t) { process_RC10Set(t); });
            }

            // RC35
        } else if ((model == EMSdevice::EMS_DEVICE_FLAG_RC35) || (model == EMSdevice::EMS_DEVICE_FLAG_RC30_1)) {
            monitor_typeids = {0x3E, 0x48, 0x52, 0x5C};
            set_typeids     = {0x3D, 0x47, 0x51, 0x5B};
            timer_typeids   = {0x3F, 0x49, 0x53, 0x5D};
            for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
                register_telegram_type(monitor_typeids[i], F("RC35Monitor"), false, [&](std::shared_ptr<const Telegram> t) { process_RC35Monitor(t); });
                register_telegram_type(set_typeids[i], F("RC35Set"), false, [&](std::shared_ptr<const Telegram> t) { process_RC35Set(t); });
                register_telegram_type(timer_typeids[i], F("RC35Timer"), false, [&](std::shared_ptr<const Telegram> t) { process_RC35Timer(t); });
            }
            register_telegram_type(EMS_TYPE_IBASettings, F("IBASettings"), true, [&](std::shared_ptr<const Telegram> t) { process_IBASettings(t); });
            register_telegram_type(EMS_TYPE_wwSettings, F("WWSettings"), true, [&](std::shared_ptr<const Telegram> t) { process_RC35wwSettings(t); });

            // RC20
        } else if (model == EMSdevice::EMS_DEVICE_FLAG_RC20) {
            monitor_typeids = {0x91};
            set_typeids     = {0xA8};
            if (actual_master_thermostat == device_id) {
                for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
                    register_telegram_type(monitor_typeids[i], F("RC20Monitor"), false, [&](std::shared_ptr<const Telegram> t) { process_RC20Monitor(t); });
                    register_telegram_type(set_typeids[i], F("RC20Set"), false, [&](std::shared_ptr<const Telegram> t) { process_RC20Set(t); });
                }
            } else {
                register_telegram_type(0xAF, F("RC20Remote"), false, [&](std::shared_ptr<const Telegram> t) { process_RC20Remote(t); });
            }
            // RC20 newer
        } else if (model == EMSdevice::EMS_DEVICE_FLAG_RC20_2) {
            monitor_typeids = {0xAE};
            set_typeids     = {0xAD};
            if (actual_master_thermostat == device_id) {
                for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
                    register_telegram_type(monitor_typeids[i], F("RC20Monitor"), false, [&](std::shared_ptr<const Telegram> t) { process_RC20Monitor_2(t); });
                    register_telegram_type(set_typeids[i], F("RC20Set"), false, [&](std::shared_ptr<const Telegram> t) { process_RC20Set_2(t); });
                }
            } else {
                register_telegram_type(0xAF, F("RC20Remote"), false, [&](std::shared_ptr<const Telegram> t) { process_RC20Remote(t); });
            }
            // RC30
        } else if (model == EMSdevice::EMS_DEVICE_FLAG_RC30) {
            monitor_typeids = {0x41};
            set_typeids     = {0xA7};
            for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
                register_telegram_type(monitor_typeids[i], F("RC30Monitor"), false, [&](std::shared_ptr<const Telegram> t) { process_RC30Monitor(t); });
                register_telegram_type(set_typeids[i], F("RC30Set"), false, [&](std::shared_ptr<const Telegram> t) { process_RC30Set(t); });
            }

            // EASY
        } else if (model == EMSdevice::EMS_DEVICE_FLAG_EASY) {
            monitor_typeids = {0x0A};
            set_typeids     = {};
            register_telegram_type(monitor_typeids[0], F("EasyMonitor"), true, [&](std::shared_ptr<const Telegram> t) { process_EasyMonitor(t); });

        } else if (model == EMSdevice::EMS_DEVICE_FLAG_CRF) {
            monitor_typeids = {0x02A5, 0x02A6, 0x02A7, 0x02A8};
            set_typeids     = {};
            for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
                register_telegram_type(monitor_typeids[i], F("CRFMonitor"), false, [&](std::shared_ptr<const Telegram> t) { process_CRFMonitor(t); });
            }

            // RC300/RC100
        } else if ((model == EMSdevice::EMS_DEVICE_FLAG_RC300) || (model == EMSdevice::EMS_DEVICE_FLAG_RC100)) {
            monitor_typeids = {0x02A5, 0x02A6, 0x02A7, 0x02A8};
            set_typeids     = {0x02B9, 0x02BA, 0x02BB, 0x02BC};
            summer_typeids  = {0x02AF, 0x02B0, 0x02B1, 0x02B2};
            curve_typeids   = {0x029B, 0x029C, 0x029D, 0x029E};
            for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
                register_telegram_type(monitor_typeids[i], F("RC300Monitor"), false, [&](std::shared_ptr<const Telegram> t) { process_RC300Monitor(t); });
                register_telegram_type(set_typeids[i], F("RC300Set"), false, [&](std::shared_ptr<const Telegram> t) { process_RC300Set(t); });
                register_telegram_type(summer_typeids[i], F("RC300Summer"), false, [&](std::shared_ptr<const Telegram> t) { process_RC300Summer(t); });
                register_telegram_type(curve_typeids[i], F("RC300Curves"), false, [&](std::shared_ptr<const Telegram> t) { process_RC300Curve(t); });
            }
            register_telegram_type(0x2F5, F("RC300WWmode"), true, [&](std::shared_ptr<const Telegram> t) { process_RC300WWmode(t); });
            register_telegram_type(0x31B, F("RC300WWtemp"), true, [&](std::shared_ptr<const Telegram> t) { process_RC300WWtemp(t); });
            register_telegram_type(0x31D, F("RC300WWmode2"), false, [&](std::shared_ptr<const Telegram> t) { process_RC300WWmode2(t); });
            register_telegram_type(0x31E, F("RC300WWmode2"), false, [&](std::shared_ptr<const Telegram> t) { process_RC300WWmode2(t); });
            register_telegram_type(0x23A, F("RC300OutdoorTemp"), true, [&](std::shared_ptr<const Telegram> t) { process_RC300OutdoorTemp(t); });
            register_telegram_type(0x267, F("RC300Floordry"), false, [&](std::shared_ptr<const Telegram> t) { process_RC300Floordry(t); });
            register_telegram_type(0x240, F("RC300Settings"), true, [&](std::shared_ptr<const Telegram> t) { process_RC300Settings(t); });

            // JUNKERS/HT3
        } else if (model == EMSdevice::EMS_DEVICE_FLAG_JUNKERS) {
            monitor_typeids = {0x016F, 0x0170, 0x0171, 0x0172};
            for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
                register_telegram_type(monitor_typeids[i], F("JunkersMonitor"), false, [&](std::shared_ptr<const Telegram> t) { process_JunkersMonitor(t); });
            }

            if (has_flags(EMS_DEVICE_FLAG_JUNKERS_OLD)) {
                // FR120, FR100
                set_typeids = {0x0179, 0x017A, 0x017B, 0x017C};
                for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
                    register_telegram_type(set_typeids[i], F("JunkersSet"), false, [&](std::shared_ptr<const Telegram> t) { process_JunkersSet2(t); });
                }
            } else {
                set_typeids = {0x0165, 0x0166, 0x0167, 0x0168};
                for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
                    register_telegram_type(set_typeids[i], F("JunkersSet"), false, [&](std::shared_ptr<const Telegram> t) { process_JunkersSet(t); });
                }
            }
        }

        if (actual_master_thermostat != device_id) {
            LOG_DEBUG(F("Adding new thermostat with device ID 0x%02X"), device_id);
            return; // don't fetch data if more than 1 thermostat
        }
        LOG_DEBUG(F("Adding new thermostat with device ID 0x%02X (as master)"), device_id);
        add_commands();

        // reserve some memory for the heating circuits (max 4 to start with)
        heating_circuits_.reserve(4);

        // only for for the master-thermostat, go a query all the heating circuits. This is only done once.
        // The automatic fetch will from now on only update the active heating circuits
        for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
            EMSESP::send_read_request(monitor_typeids[i], device_id);
        }

        for (uint8_t i = 0; i < set_typeids.size(); i++) {
            EMSESP::send_read_request(set_typeids[i], device_id);
        }

        /* do not flood tx-queue now, these values are fetched later by toggle fetch
        for (uint8_t i = 0; i < summer_typeids.size(); i++) {
            EMSESP::send_read_request(summer_typeids[i], device_id);
        }
        for (uint8_t i = 0; i < curve_typeids.size(); i++) {
            EMSESP::send_read_request(curve_typeids[i], device_id);
        }
        for (uint8_t i = 0; i < timer_typeids.size(); i++) {
            EMSESP::send_read_request(timer_typeids[i], device_id);
        }
        */
        EMSESP::send_read_request(0x12, device_id); // read last error (only published on errors)
        EMSESP::send_read_request(0xA2, device_id); // actual errorCode (published on errors and very rarely)
    }
}

// prepare data for Web UI
void Thermostat::device_info_web(JsonArray & root, uint8_t & part) {
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_LARGE_DYN);
    //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_LARGE> doc;
    JsonObject                                     json = doc.to<JsonObject>();
    if (part == 0) {
        if (export_values_main(json)) {
            create_value_json(root, F("dateTime"), nullptr, F_(time), nullptr, json);
            create_value_json(root, F("errorcode"), nullptr, F_(error), nullptr, json);
            create_value_json(root, F("lastcode"), nullptr, F_(lastCode), nullptr, json);
            create_value_json(root, F("display"), nullptr, F_(display), nullptr, json);
            create_value_json(root, F("language"), nullptr, F_(language), nullptr, json);
            create_value_json(root, F("offsetclock"), nullptr, F_(offsetclock), nullptr, json);
            create_value_json(root, F("dampedoutdoortemp"), nullptr, F_(dampedoutdoortemp), F_(degrees), json);
            create_value_json(root, F("inttemp1"), nullptr, F_(inttemp1), F_(degrees), json);
            create_value_json(root, F("inttemp2"), nullptr, F_(inttemp2), F_(degrees), json);
            create_value_json(root, F("intoffset"), nullptr, F_(intoffset), nullptr, json);
            create_value_json(root, F("minexttemp"), nullptr, F_(minexttemp), F_(degrees), json);
            create_value_json(root, F("building"), nullptr, F_(building), nullptr, json);
            create_value_json(root, F("floordry"), nullptr, F_(floordry), nullptr, json);
            create_value_json(root, F("floordrytemp"), nullptr, F_(floordrytemp), F_(degrees), json);
            create_value_json(root, F("wwmode"), nullptr, F_(wwmode), nullptr, json);
            create_value_json(root, F("wwsettemp"), nullptr, F_(wwsettemp), nullptr, json);
            create_value_json(root, F("wwsettemplow"), nullptr, F_(wwsettemplow), nullptr, json);
            create_value_json(root, F("wwextra1"), nullptr, F_(wwextra1), nullptr, json);
            create_value_json(root, F("wwcircmode"), nullptr, F_(wwcircmode), nullptr, json);
        }
        if (heating_circuits_.size() > 0) {
            part = 1;
        }
    } else {
        std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuits_[part - 1];
        if (export_values_hc(hc, json)) {
            // display for each active heating circuit
            char prefix_str[10];
            snprintf_P(prefix_str, sizeof(prefix_str), PSTR("(hc %d) "), hc->hc_num());

            create_value_json(root, F("seltemp"), FPSTR(prefix_str), F_(seltemp), F_(degrees), json);
            create_value_json(root, F("currtemp"), FPSTR(prefix_str), F_(currtemp), F_(degrees), json);
            create_value_json(root, F("heattemp"), FPSTR(prefix_str), F_(heattemp), F_(degrees), json);
            create_value_json(root, F("comforttemp"), FPSTR(prefix_str), F_(comforttemp), F_(degrees), json);
            create_value_json(root, F("daytemp"), FPSTR(prefix_str), F_(daytemp), F_(degrees), json);
            create_value_json(root, F("ecotemp"), FPSTR(prefix_str), F_(ecotemp), F_(degrees), json);
            create_value_json(root, F("nighttemp"), FPSTR(prefix_str), F_(nighttemp), F_(degrees), json);
            create_value_json(root, F("manualtemp"), FPSTR(prefix_str), F_(manualtemp), F_(degrees), json);
            create_value_json(root, F("holidaytemp"), FPSTR(prefix_str), F_(holidaytemp), F_(degrees), json);
            create_value_json(root, F("nofrosttemp"), FPSTR(prefix_str), F_(nofrosttemp), F_(degrees), json);
            create_value_json(root, F("heatingtype"), FPSTR(prefix_str), F_(heatingtype), nullptr, json);
            create_value_json(root, F("targetflowtemp"), FPSTR(prefix_str), F_(targetflowtemp), F_(degrees), json);
            create_value_json(root, F("offsettemp"), FPSTR(prefix_str), F_(offsettemp), F_(degrees), json);
            create_value_json(root, F("designtemp"), FPSTR(prefix_str), F_(designtemp), F_(degrees), json);
            create_value_json(root, F("roominfluence"), FPSTR(prefix_str), F_(roominfluence), F_(degrees), json);
            create_value_json(root, F("flowtempoffset"), FPSTR(prefix_str), F_(flowtempoffset), F_(degrees), json);
            create_value_json(root, F("minflowtemp"), FPSTR(prefix_str), F_(minflowtemp), F_(degrees), json);
            create_value_json(root, F("maxflowtemp"), FPSTR(prefix_str), F_(maxflowtemp), F_(degrees), json);
            create_value_json(root, F("summertemp"), FPSTR(prefix_str), F_(summertemp), F_(degrees), json);
            create_value_json(root, F("summermode"), FPSTR(prefix_str), F_(summermode), nullptr, json);
            create_value_json(root, F("reducemode"), FPSTR(prefix_str), F_(reducemode), nullptr, json);
            create_value_json(root, F("program"), FPSTR(prefix_str), F_(program), nullptr, json);
            create_value_json(root, F("controlmode"), FPSTR(prefix_str), F_(controlmode), nullptr, json);
            create_value_json(root, F("mode"), FPSTR(prefix_str), F_(mode), nullptr, json);
            create_value_json(root, F("modetype"), FPSTR(prefix_str), F_(modetype), nullptr, json);
        }
        if (++part > heating_circuits_.size()) {
            part = 0; // no more parts
        }
    }
}

// this function is called post the telegram handler function has been executed
// we check if any of the thermostat values have changed and then republish if necessary
bool Thermostat::updated_values() {
    // only publish on the master thermostat
    if (EMSESP::actual_master_thermostat() != device_id()) {
        return false;
    }
    if (changed_) {
        changed_ = false;
        return true;
    }
    return false;
}

bool Thermostat::export_values(JsonObject & json, int8_t id) {
    if (id > 0) {
        std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(id);
        if (hc != nullptr) {
            JsonObject json_hc;
            char       hc_name[10]; // hc{1-4}
            snprintf_P(hc_name, 10, PSTR("hc%d"), hc->hc_num());
            json_hc = json.createNestedObject(hc_name);
            return export_values_hc(hc, json_hc);
        }
        return false;
    }
    bool has_value = export_values_main(json);
    for (const auto & hc : heating_circuits_) {
        JsonObject json_hc;
        char       hc_name[10]; // hc{1-4}
        snprintf_P(hc_name, 10, PSTR("hc%d"), hc->hc_num());
        json_hc = json.createNestedObject(hc_name);
        has_value |= export_values_hc(hc, json_hc);
    }
    return has_value;
}

// publish values via MQTT
void Thermostat::publish_values(JsonObject & json, bool force) {
    if (EMSESP::actual_master_thermostat() != device_id()) {
        return;
    }

    // if MQTT is in single mode send out the main data to the thermostat_data topic
    if (Mqtt::mqtt_format() == Mqtt::Format::SINGLE) {
        DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_MEDIUM);
        //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_MEDIUM> doc;
        JsonObject                                      json_data = doc.to<JsonObject>();
        if (export_values_main(json_data)) {
            Mqtt::publish(F("thermostat_data"), json_data);
            doc.clear();
        }
        for (const auto & hc : heating_circuits_) {
            if (export_values_hc(hc, json_data)) {
                char topic[30];
                snprintf_P(topic, 30, PSTR("thermostat_data_hc%d"), hc->hc_num());
                Mqtt::publish(topic, json_data);
            }
            doc.clear();
        }
        return;
    }

    // see if we have already registered this with HA MQTT Discovery, if not send the config first
    if (Mqtt::mqtt_format() == Mqtt::Format::HA) {
        if (!ha_config(force)) {
            return;
        }
    }

    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_LARGE_DYN);
    //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_LARGE> doc;
    JsonObject                                     json_data = doc.to<JsonObject>();

    // get the thermostat data.
    // we're in HA or CUSTOM, send out the complete topic with all the data
    if (export_values(json_data)) {
        Mqtt::publish(F("thermostat_data"), json_data);
    }
}

bool Thermostat::export_values_main(JsonObject & rootThermostat) {
    uint8_t model = this->model();

    // Clock time
    if (datetime_.size()) {
        rootThermostat["dateTime"] = datetime_;
    }

    if (Helpers::hasValue(errorNumber_)) {
        rootThermostat["errorcode"] = errorCode_;
    }

    if (lastCode_[0] != '\0') {
        rootThermostat["lastcode"] = lastCode_;
    }

    if (model == EMSdevice::EMS_DEVICE_FLAG_RC30_1) {
        // Display
        if (Helpers::hasValue(ibaMainDisplay_)) {
            if (ibaMainDisplay_ == 0) {
                rootThermostat["display"] = FJSON("internal temperature");
            } else if (ibaMainDisplay_ == 1) {
                rootThermostat["display"] = FJSON("internal setpoint");
            } else if (ibaMainDisplay_ == 2) {
                rootThermostat["display"] = FJSON("external temperature");
            } else if (ibaMainDisplay_ == 3) {
                rootThermostat["display"] = FJSON("burner temperature");
            } else if (ibaMainDisplay_ == 4) {
                rootThermostat["display"] = FJSON("WW temperature");
            } else if (ibaMainDisplay_ == 5) {
                rootThermostat["display"] = FJSON("functioning mode");
            } else if (ibaMainDisplay_ == 6) {
                rootThermostat["display"] = FJSON("time");
            } else if (ibaMainDisplay_ == 7) {
                rootThermostat["display"] = FJSON("date");
            } else if (ibaMainDisplay_ == 8) {
                rootThermostat["display"] = FJSON("smoke temperature");
            }
        }

        // Language
        Helpers::json_enum(rootThermostat, "language", {F("German"), F("Dutch"), F("French"), F("Italian")}, ibaLanguage_);

        // Offset clock
        if (Helpers::hasValue(ibaClockOffset_)) {
            rootThermostat["offsetclock"] = ibaClockOffset_; // offset (in sec) to clock, 0xff=-1s, 0x02=2s
        }
    }

    // Damped outdoor temperature (RC35)
    if (Helpers::hasValue(dampedoutdoortemp_)) {
        if (model == EMS_DEVICE_FLAG_RC35 || model == EMS_DEVICE_FLAG_RC30_1) {
            rootThermostat["dampedoutdoortemp"] = dampedoutdoortemp_;
        }
    }

    // Damped outdoor temperature (RC300)
    if (Helpers::hasValue(dampedoutdoortemp2_)) {
        rootThermostat["dampedoutdoortemp"] = (float)dampedoutdoortemp2_ / 10;
    }

    // Floordry
    Helpers::json_enum(rootThermostat, "floordry", {F("off"), F("start"), F("heat"), F("hold"), F("cool"), F("end")}, floordrystatus_);

    if (Helpers::hasValue(floordrytemp_) && (floordrytemp_ > 0)) {
        rootThermostat["floordrytemp"] = floordrytemp_;
    }

    // Temp sensor 1
    if (Helpers::hasValue(tempsensor1_)) {
        rootThermostat["inttemp1"] = (float)tempsensor1_ / 10;
    }

    // Temp sensor 2
    if (Helpers::hasValue(tempsensor2_)) {
        rootThermostat["inttemp2"] = (float)tempsensor2_ / 10;
    }

    // Offset int. temperature
    if (Helpers::hasValue(ibaCalIntTemperature_)) {
        rootThermostat["intoffset"] = (float)ibaCalIntTemperature_ / 2;
    }

    // Min ext. temperature
    if (Helpers::hasValue(ibaMinExtTemperature_)) {
        rootThermostat["minexttemp"] = (float)ibaMinExtTemperature_; // min ext temp for heating curve, in deg.
    }

    // Building
    if (model == EMS_DEVICE_FLAG_RC300 || model == EMS_DEVICE_FLAG_RC100) {
        Helpers::json_enum(rootThermostat, "building", {F("light"), F("medium"), F("heavy")}, ibaBuildingType_ - 1);
    } else {
        Helpers::json_enum(rootThermostat, "building", {F("light"), F("medium"), F("heavy")}, ibaBuildingType_);
    }

    // Warm water mode, "high" is normal setting
    if (model == EMS_DEVICE_FLAG_RC300 || model == EMS_DEVICE_FLAG_RC100) {
        Helpers::json_enum(rootThermostat, "wwmode", {F("off"), F("low"), F("high"), F("auto"), F("own_prog")}, wwMode_);
    } else {
        Helpers::json_enum(rootThermostat, "wwmode", {F("off"), F("on"), F("auto")}, wwMode_);
    }

    // Warm water set temp
    if (Helpers::hasValue(wwSetTemp_)) {
        rootThermostat["wwsettemp"] = wwSetTemp_;
    }

    // Warm water set temp low
    if (Helpers::hasValue(wwSetTempLow_)) {
        rootThermostat["wwsettemplow"] = wwSetTempLow_;
    }

    // Warm water extra1
    if (Helpers::hasValue(wwExtra1_)) {
        rootThermostat["wwextra1"] = wwExtra1_;
    }

    // Warm water extra2
    if (Helpers::hasValue(wwExtra2_)) {
        rootThermostat["wwextra2"] = wwExtra2_;
    }

    // Warm Water circulation mode
    if (model == EMS_DEVICE_FLAG_RC300 || model == EMS_DEVICE_FLAG_RC100) {
        Helpers::json_enum(rootThermostat, "wwcircmode", {F("off"), F("on"), F("auto"), F("own_prog")}, wwCircMode_);
    } else {
        Helpers::json_enum(rootThermostat, "wwcircmode", {F("off"), F("on"), F("auto")}, wwCircMode_);
    }

    return (rootThermostat.size());
}

// creates JSON doc from values, for each heating circuit
// returns false if empty
bool Thermostat::export_values_hc(std::shared_ptr<Thermostat::HeatingCircuit> hc, JsonObject & dataThermostat) {
    uint8_t model = this->model();

    if (!hc->is_active()) {
        return false;
    }
    // different logic on how temperature values are stored, depending on model
    uint8_t setpoint_temp_divider;
    uint8_t curr_temp_divider;
    if (model == EMS_DEVICE_FLAG_EASY) {
        setpoint_temp_divider = 100;
        curr_temp_divider     = 100;
    } else if (model == EMS_DEVICE_FLAG_JUNKERS) {
        setpoint_temp_divider = 10;
        curr_temp_divider     = 10;
    } else {
        setpoint_temp_divider = 2;
        curr_temp_divider     = 10;
    }

    // Setpoint room temperature
    if (Helpers::hasValue(hc->setpoint_roomTemp)) {
        dataThermostat["seltemp"] = Helpers::round2((float)hc->setpoint_roomTemp / setpoint_temp_divider);
    }

    // Current room temperature
    if (Helpers::hasValue(hc->curr_roomTemp)) {
        dataThermostat["currtemp"] = Helpers::round2((float)hc->curr_roomTemp / curr_temp_divider);
    }

    if (Mqtt::mqtt_format() == Mqtt::Format::HA) {
        if (Helpers::hasValue(hc->ha_temp)) {
            dataThermostat["hatemp"] = Helpers::round2((float)hc->ha_temp / 10);
        } else if (Helpers::hasValue(hc->curr_roomTemp)) {
            dataThermostat["hatemp"] = Helpers::round2((float)hc->curr_roomTemp / curr_temp_divider);
        } else {
            dataThermostat["hatemp"] = Helpers::round2((float)hc->setpoint_roomTemp / setpoint_temp_divider);
        }
    }

    if (Helpers::hasValue(hc->daytemp)) {
        if (model == EMSdevice::EMS_DEVICE_FLAG_JUNKERS) {
            // Heat temperature
            dataThermostat["heattemp"] = (float)hc->daytemp / 2;
        } else if (model == EMSdevice::EMS_DEVICE_FLAG_RC300 || model == EMSdevice::EMS_DEVICE_FLAG_RC100) {
            // Comfort temperature
            dataThermostat["comforttemp"] = (float)hc->daytemp / 2;
        } else {
            // Day temperature
            dataThermostat["daytemp"] = (float)hc->daytemp / 2;
        }
    }

    if (Helpers::hasValue(hc->nighttemp)) {
        if (model == EMSdevice::EMS_DEVICE_FLAG_JUNKERS || model == EMSdevice::EMS_DEVICE_FLAG_RC300 || model == EMSdevice::EMS_DEVICE_FLAG_RC100) {
            // Eco temperature
            dataThermostat["ecotemp"] = (float)hc->nighttemp / 2;
        } else {
            // Night temperature
            dataThermostat["nighttemp"] = (float)hc->nighttemp / 2;
        }
    }

    // Manual temperature
    if (Helpers::hasValue(hc->manualtemp)) {
        dataThermostat["manualtemp"] = (float)hc->manualtemp / 2;
    }

    // Holiday temperature
    if (Helpers::hasValue(hc->holidaytemp)) {
        dataThermostat["holidaytemp"] = (float)hc->holidaytemp / 2;
    }

    // Nofrost temperature
    if (Helpers::hasValue(hc->nofrosttemp)) {
        if (model == EMSdevice::EMS_DEVICE_FLAG_JUNKERS) {
            dataThermostat["nofrosttemp"] = (float)hc->nofrosttemp / 2;
        } else {
            dataThermostat["nofrosttemp"] = hc->nofrosttemp;
        }
    }

    // Heating Type
    Helpers::json_enum(dataThermostat, "heatingtype", {F("off"), F("radiator"), F("convector"), F("floor")}, hc->heatingtype);

    // Target flow temperature
    if (Helpers::hasValue(hc->targetflowtemp)) {
        dataThermostat["targetflowtemp"] = hc->targetflowtemp;
    }

    // Offset temperature
    if (Helpers::hasValue(hc->offsettemp)) {
        if (model == EMSdevice::EMS_DEVICE_FLAG_RC300 || model == EMSdevice::EMS_DEVICE_FLAG_RC100) {
            dataThermostat["offsettemp"] = hc->offsettemp;
        } else {
            dataThermostat["offsettemp"] = hc->offsettemp / 2;
        }
    }

    // Design temperature
    if (Helpers::hasValue(hc->designtemp)) {
        dataThermostat["designtemp"] = hc->designtemp;
    }

    // Room influence
    if (Helpers::hasValue(hc->roominfluence)) {
        dataThermostat["roominfluence"] = hc->roominfluence;
    }

    // Flow temperature offset
    if (Helpers::hasValue(hc->flowtempoffset)) {
        dataThermostat["flowtempoffset"] = hc->flowtempoffset;
    }

    // Min Flow temperature offset
    if (Helpers::hasValue(hc->minflowtemp)) {
        dataThermostat["minflowtemp"] = hc->minflowtemp;
    }

    // Max Flow temperature offset
    if (Helpers::hasValue(hc->maxflowtemp)) {
        dataThermostat["maxflowtemp"] = hc->maxflowtemp;
    }

    // Summer temperature
    if (Helpers::hasValue(hc->summertemp)) {
        dataThermostat["summertemp"] = hc->summertemp;
    }

    // Summer mode
    Helpers::json_enum(dataThermostat, "summermode", {F("summer"), F("auto"), F("winter")}, hc->summer_setmode);

    // Reduce mode
    Helpers::json_enum(dataThermostat, "reducemode", {F("nofrost"), F("reduce"), F("room"), F("outdoor")}, hc->reducemode);

    // Control mode room or outdoor
    if (model == EMS_DEVICE_FLAG_RC35 || model == EMS_DEVICE_FLAG_RC30_1) {
        Helpers::json_enum(dataThermostat, "controlmode", {F("outdoor"), F("room")}, hc->controlmode);
    } else if (model == EMS_DEVICE_FLAG_RC300 || model == EMS_DEVICE_FLAG_RC100) {
        Helpers::json_enum(dataThermostat, "controlmode", {F("off"), F("outdoor"), F("simple"), F("MPC"), F("room"), F("power"), F("const.")}, hc->controlmode);
    }

    // program no.
    if (Helpers::hasValue(hc->program)) {
        dataThermostat["program"] = hc->program;
    }

    // mode - always force showing this when in HA so not to break HA's climate component
    if ((Helpers::hasValue(hc->mode)) || (Mqtt::mqtt_format() == Mqtt::Format::HA)) {
        uint8_t hc_mode = hc->get_mode(model);
        // if we're sending to HA the only valid mode types are heat, auto and off
        if (Mqtt::mqtt_format() == Mqtt::Format::HA) {
            if ((hc_mode == HeatingCircuit::Mode::MANUAL) || (hc_mode == HeatingCircuit::Mode::DAY)) {
                hc_mode = HeatingCircuit::Mode::HEAT;
            } else if ((hc_mode == HeatingCircuit::Mode::NIGHT) || (hc_mode == HeatingCircuit::Mode::OFF)) {
                hc_mode = HeatingCircuit::Mode::OFF;
            } else {
                hc_mode = HeatingCircuit::Mode::AUTO;
            }
        }
        // Mode
        dataThermostat["mode"] = mode_tostring(hc_mode);
    }

    // special handling of mode type, for the RC35 replace with summer/holiday if set
    // https://github.com/emsesp/EMS-ESP/issues/373#issuecomment-619810209
    // Mode Type
    if (Helpers::hasValue(hc->summer_mode) && hc->summer_mode) {
        dataThermostat["modetype"] = FJSON("summer");
    } else if (Helpers::hasValue(hc->holiday_mode) && hc->holiday_mode) {
        dataThermostat["modetype"] = FJSON("holiday");
    } else if (Helpers::hasValue(hc->mode_type)) {
        dataThermostat["modetype"] = mode_tostring(hc->get_mode_type(model));
    }

    return (dataThermostat.size());
}

// set up HA MQTT Discovery
bool Thermostat::ha_config(bool force) {
    if (!Mqtt::connected()) {
        return false;
    }

    // if force, reset registered flag for main controller and all heating circuits
    if (force) {
        for (const auto & hc : heating_circuits_) {
            hc->ha_registered(false);
        }
        ha_registered(false);
    }

    // set up the main controller
    if (!ha_registered() && uuid::get_uptime_sec() > (EMSESP::tx_delay() + 60u)) {
        register_mqtt_ha_config();
        ha_registered(true);
        // return false; // heating circuits in next cycle
    }

    // check to see which heating circuits need to be added as HA climate components
    // but only if it's active and there is a real value for the current room temperature (https://github.com/emsesp/EMS-ESP/issues/582)
    // no check for room temperature, we have fallback ha_temp now.
    for (const auto & hc : heating_circuits_) {
        if (hc->is_active() && !hc->ha_registered()) {
            register_mqtt_ha_config(hc->hc_num());
            hc->ha_registered(true);
        }
    }

    return true;
}

// returns the heating circuit object based on the hc number
// of nullptr if it doesn't exist yet
std::shared_ptr<Thermostat::HeatingCircuit> Thermostat::heating_circuit(const uint8_t hc_num) {
    // if hc_num is 0 then return the first existing hc in the list
    if (hc_num == AUTO_HEATING_CIRCUIT) {
        for (const auto & heating_circuit : heating_circuits_) {
            if (heating_circuit->is_active()) {
                return heating_circuit;
            }
        }
    }

    // otherwise find a match
    for (const auto & heating_circuit : heating_circuits_) {
        if ((heating_circuit->hc_num() == hc_num) && heating_circuit->is_active()) {
            return heating_circuit;
        }
    }

    return nullptr; // not found
}

// determine which heating circuit the type ID is referring too
// returns pointer to the HeatingCircuit or nullptr if it can't be found
// if its a new one, the object will be created and also the fetch flags set
std::shared_ptr<Thermostat::HeatingCircuit> Thermostat::heating_circuit(std::shared_ptr<const Telegram> telegram) {
    if (device_id() != EMSESP::actual_master_thermostat()) {
        return nullptr;
    }
    // look through the Monitor and Set arrays to see if there is a match
    uint8_t hc_num  = 0;
    bool    toggle_ = false;
    // search set message types
    for (uint8_t i = 0; i < monitor_typeids.size(); i++) {
        if (monitor_typeids[i] == telegram->type_id) {
            hc_num  = i + 1;
            toggle_ = true;
            break;
        }
    }

    // not found, search status message types
    if (hc_num == 0) {
        for (uint8_t i = 0; i < set_typeids.size(); i++) {
            if (set_typeids[i] == telegram->type_id) {
                hc_num = i + 1;
                break;
            }
        }
    }

    // not found, search summer message types
    if (hc_num == 0) {
        for (uint8_t i = 0; i < summer_typeids.size(); i++) {
            if (summer_typeids[i] == telegram->type_id) {
                hc_num = i + 1;
                break;
            }
        }
    }

    // not found, search heating_curve message types
    if (hc_num == 0) {
        for (uint8_t i = 0; i < curve_typeids.size(); i++) {
            if (curve_typeids[i] == telegram->type_id) {
                hc_num = i + 1;
                break;
            }
        }
    }

    // not found, search timer message types
    if (hc_num == 0) {
        for (uint8_t i = 0; i < timer_typeids.size(); i++) {
            if (timer_typeids[i] == telegram->type_id) {
                hc_num = i + 1;
                break;
            }
        }
    }

    // not found, search device-id types for remote thermostats
    if (telegram->src >= 0x18 && telegram->src <= 0x1B) {
        hc_num = telegram->src - 0x17;
    }

    // still didn't recognize it, ignore it
    if (hc_num == 0) {
        return nullptr;
    }

    // if we have the heating circuit already present, returns its object
    // otherwise create a new object and add it
    for (const auto & heating_circuit : heating_circuits_) {
        if (heating_circuit->hc_num() == hc_num) {
            return heating_circuit;
        }
    }

    // register new heatingcircuits only on active monitor telegrams
    if (!toggle_) {
        return nullptr;
    }

    // create a new heating circuit object
    auto new_hc = std::make_shared<Thermostat::HeatingCircuit>(hc_num);
    heating_circuits_.push_back(new_hc);

    std::sort(heating_circuits_.begin(), heating_circuits_.end()); // sort based on hc number

    // set the flag saying we want its data during the next auto fetch
    toggle_fetch(monitor_typeids[hc_num - 1], toggle_);

    if (set_typeids.size()) {
        toggle_fetch(set_typeids[hc_num - 1], toggle_);
    }
    if (summer_typeids.size()) {
        toggle_fetch(summer_typeids[hc_num - 1], toggle_);
    }
    if (curve_typeids.size()) {
        toggle_fetch(curve_typeids[hc_num - 1], toggle_);
    }
    if (timer_typeids.size()) {
        toggle_fetch(timer_typeids[hc_num - 1], toggle_);
    }

    return heating_circuits_.back(); // even after sorting, this should still point back to the newly created HC
}

// publish config topic for HA MQTT Discovery for main thermostat values
// homeassistant/sensor/ems-esp/thermostat/config
void Thermostat::register_mqtt_ha_config() {
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_HA_CONFIG);
    //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_HA_CONFIG> doc;
    doc["uniq_id"] = FJSON("thermostat");
    doc["ic"]      = FJSON("mdi:home-thermometer-outline");

    char temp[128];
    snprintf_P(temp, sizeof(temp), PSTR("%s/thermostat_data"), Mqtt::base().c_str());
    doc["stat_t"] = temp;

    doc["name"]    = FJSON("Thermostat Status");
    doc["val_tpl"] = FJSON("{{value_json.dateTime}}"); // default value - must have one, so we use dateTime
    JsonObject dev = doc.createNestedObject("dev");
    snprintf_P(temp, sizeof(temp), PSTR("%s Thermostat"), Mqtt::base().c_str());
    dev["name"]    = temp;
    dev["sw"]      = EMSESP_APP_VERSION;
    dev["mf"]      = brand_to_string();
    dev["mdl"]     = name();
    JsonArray ids  = dev.createNestedArray("ids");
    snprintf_P(temp, sizeof(temp), PSTR("%s-thermostat"), Mqtt::base().c_str());
    ids.add(temp);
    snprintf_P(temp, sizeof(temp), PSTR("homeassistant/sensor/%s/thermostat/config"), Mqtt::base().c_str());
    Mqtt::publish_ha(temp, doc.as<JsonObject>()); // publish the config payload with retain flag

    Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(time), device_type(), "dateTime", nullptr, nullptr);

    if (Helpers::hasValue(errorNumber_)) {
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(error), device_type(), "errorcode", nullptr, nullptr);
    }

    uint8_t model = this->model();

    if (model == EMS_DEVICE_FLAG_RC30_1) {
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(display), device_type(), "display", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(language), device_type(), "language", nullptr, nullptr);
    }

    if (model == EMS_DEVICE_FLAG_RC300 || model == EMS_DEVICE_FLAG_RC100) {
        if (Helpers::hasValue(dampedoutdoortemp2_)) {
            Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(dampedoutdoortemp), device_type(), "dampedoutdoortemp", F_(degrees), nullptr);
        }
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(building), device_type(), "building", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(minexttemp), device_type(), "minexttemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(floordry), device_type(), "floordry", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(floordrytemp), device_type(), "floordrytemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(wwmode), device_type(), "wwmode", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(wwsettemp), device_type(), "wwsettemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(wwcircmode), device_type(), "wwcircmode", nullptr, nullptr);
    }

    if (model == EMS_DEVICE_FLAG_RC35 || model == EMS_DEVICE_FLAG_RC30_1) {
        // excluding inttemp1, inttemp2, intoffset, minexttemp
        if (Helpers::hasValue(dampedoutdoortemp_)) {
            Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(dampedoutdoortemp), device_type(), "dampedoutdoortemp", F_(degrees), nullptr);
        }
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(building), device_type(), "building", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(minexttemp), device_type(), "minexttemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(wwmode), device_type(), "wwmode", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(nullptr, nullptr, F_(wwcircmode), device_type(), "wwcircmode", nullptr, nullptr);
    }
}

// publish config topic for HA MQTT Discovery for each of the heating circuit
// e.g. homeassistant/climate/ems-esp/thermostat_hc1/config
void Thermostat::register_mqtt_ha_config(uint8_t hc_num) {
    DynamicJsonDocument doc(EMSESP_MAX_JSON_SIZE_MEDIUM);
    //StaticJsonDocument<EMSESP_MAX_JSON_SIZE_MEDIUM> doc;

    char str1[20];
    snprintf_P(str1, sizeof(str1), PSTR("Thermostat hc%d"), hc_num);

    char str2[20];
    snprintf_P(str2, sizeof(str2), PSTR("thermostat_hc%d"), hc_num);

    char str3[25];
    snprintf_P(str3, sizeof(str3), PSTR("~/%s"), str2);
    doc["mode_cmd_t"]  = str3;
    doc["temp_cmd_t"]  = str3;
    doc["name"]        = str1;
    doc["uniq_id"]     = str2;
    doc["mode_cmd_t"]  = str3;
    doc["temp_cmd_t"]  = str3;
    doc["~"]           = Mqtt::base(); // ems-esp
    doc["mode_stat_t"] = FJSON("~/thermostat_data");
    doc["temp_stat_t"] = FJSON("~/thermostat_data");
    doc["curr_temp_t"] = FJSON("~/thermostat_data");

    char mode_str[30];
    snprintf_P(mode_str, sizeof(mode_str), PSTR("{{value_json.hc%d.mode}}"), hc_num);
    doc["mode_stat_tpl"] = mode_str;

    char temp[128];
    snprintf_P(temp, sizeof(temp), PSTR("{{value_json.hc%d.seltemp}}"), hc_num);
    doc["temp_stat_tpl"] = temp;

    char currtemp_str[30];
    snprintf_P(currtemp_str, sizeof(currtemp_str), PSTR("{{value_json.hc%d.hatemp}}"), hc_num);
    doc["curr_temp_tpl"] = currtemp_str;

    doc["min_temp"]  = FJSON("5");
    doc["max_temp"]  = FJSON("30");
    doc["temp_step"] = FJSON("0.5");

    // the HA climate component only responds to auto, heat and off
    JsonArray modes = doc.createNestedArray("modes");
    modes.add("auto");
    modes.add("heat");
    modes.add("off");

    JsonObject dev = doc.createNestedObject("dev");
    snprintf_P(temp, sizeof(temp), PSTR("%s Thermostat"), Mqtt::base().c_str());
    dev["name"]    = temp;
    dev["sw"]      = EMSESP_APP_VERSION;
    dev["mf"]      = brand_to_string();
    dev["mdl"]     = name();
    JsonArray ids  = dev.createNestedArray("ids");
    snprintf_P(temp, sizeof(temp), PSTR("%s-thermostat"), Mqtt::base().c_str());
    ids.add(temp);

    std::string topic(100, '\0');
    snprintf_P(&topic[0], topic.capacity() + 1, PSTR("homeassistant/climate/%s/thermostat_hc%d/config"), Mqtt::base().c_str(), hc_num);
    Mqtt::publish_ha(topic, doc.as<JsonObject>()); // publish the config payload with retain flag

    // enable the a special "thermostat_hc<n>" topic to take both mode strings and floats for each of the heating circuits
    std::string topic2(100, '\0');
    snprintf_P(&topic2[0], topic2.capacity() + 1, PSTR("thermostat_hc%d"), hc_num);
    register_mqtt_topic(topic2, [=](const char * m) { return thermostat_ha_cmd(m, hc_num); });

    char hc_name[10]; // hc{1-4}
    strlcpy(hc_name, "hc", 10);
    char s[3];
    strlcat(hc_name, Helpers::itoa(s, hc_num), 10);

    Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(mode), device_type(), "mode", nullptr, nullptr);

    Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(seltemp), device_type(), "seltemp", F_(degrees), F_(iconwatertemp));
    Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(currtemp), device_type(), "hatemp", F_(degrees), F_(iconwatertemp));

    switch (model()) {
    case EMS_DEVICE_FLAG_RC100:
    case EMS_DEVICE_FLAG_RC300:
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(modetype), device_type(), "modetype", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(ecotemp), device_type(), "ecotemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(manualtemp), device_type(), "manualtemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(comforttemp), device_type(), "comforttemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(summertemp), device_type(), "summertemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(designtemp), device_type(), "designtemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(offsettemp), device_type(), "offsettemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(minflowtemp), device_type(), "minflowtemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(maxflowtemp), device_type(), "maxflowtemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(roominfluence), device_type(), "roominfluence", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(nofrosttemp), device_type(), "nofrosttemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(targetflowtemp), device_type(), "targetflowtemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(controlmode), device_type(), "controlmode", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(program), device_type(), "program", nullptr, nullptr);
        break;
    case EMS_DEVICE_FLAG_RC20_2:
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(daytemp), device_type(), "daytemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(nighttemp), device_type(), "nighttemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(program), device_type(), "program", nullptr, nullptr);
        break;
    case EMS_DEVICE_FLAG_RC30_1:
    case EMS_DEVICE_FLAG_RC35:
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(modetype), device_type(), "modetype", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(nighttemp), device_type(), "nighttemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(daytemp), device_type(), "daytemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(designtemp), device_type(), "designtemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(offsettemp), device_type(), "offsettemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(holidaytemp), device_type(), "holidaytemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(targetflowtemp), device_type(), "targetflowtemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(summertemp), device_type(), "summertemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(nofrosttemp), device_type(), "nofrosttemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(roominfluence), device_type(), "roominfluence", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(minflowtemp), device_type(), "minflowtemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(maxflowtemp), device_type(), "maxflowtemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(reducemode), device_type(), "reducemode", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(controlmode), device_type(), "controlmode", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(program), device_type(), "program", nullptr, nullptr);
        break;
    case EMS_DEVICE_FLAG_JUNKERS:
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(modetype), device_type(), "modetype", nullptr, nullptr);
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(heattemp), device_type(), "heattemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(ecotemp), device_type(), "ecotemp", F_(degrees), F_(iconwatertemp));
        Mqtt::register_mqtt_ha_sensor(hc_name, nullptr, F_(nofrosttemp), device_type(), "nofrosttemp", F_(degrees), F_(iconwatertemp));
        break;
    default:
        break;
    }
}

// for HA specifically when receiving over MQTT in the thermostat topic
// e.g. thermostat_hc1
// it could be either a 'mode' or a float value for a temperature. we try brute force both and see which one works.
// return true if it parses the message correctly
bool Thermostat::thermostat_ha_cmd(const char * message, uint8_t hc_num) {
    // check if it's json. We know the message isn't empty
    if (message[0] == '{') {
        return false;
    }

    // check for mode first
    if (!set_mode(message, hc_num)) {
        // handle as a numerical temperature value
        float f = strtof((char *)message, 0);
        set_temperature(f, HeatingCircuit::Mode::AUTO, hc_num);
    }

    return true;
}

// decodes the thermostat mode for the heating circuit based on the thermostat type
// modes are off, manual, auto, day and night
uint8_t Thermostat::HeatingCircuit::get_mode(uint8_t model) const {
    if (!Helpers::hasValue(mode)) {
        return HeatingCircuit::Mode::UNKNOWN;
    }

    if (model == EMSdevice::EMS_DEVICE_FLAG_RC20) {
        if (mode == 0) {
            return HeatingCircuit::Mode::OFF;
        } else if (mode == 1) {
            return HeatingCircuit::Mode::MANUAL;
        } else if (mode == 2) {
            return HeatingCircuit::Mode::AUTO;
        }
    } else if (model == EMSdevice::EMS_DEVICE_FLAG_CRF) {
        if (mode == 0) {
            return HeatingCircuit::Mode::AUTO;
        } else if (mode == 1) {
            return HeatingCircuit::Mode::OFF;
        }
    } else if ((model == EMSdevice::EMS_DEVICE_FLAG_RC300) || (model == EMSdevice::EMS_DEVICE_FLAG_RC100)) {
        if (mode == 0) {
            return HeatingCircuit::Mode::MANUAL;
        } else if (mode == 1) {
            return HeatingCircuit::Mode::AUTO;
        }
    } else if (model == EMSdevice::EMS_DEVICE_FLAG_JUNKERS) {
        if (mode == 1) {
            return HeatingCircuit::Mode::MANUAL;
        } else if (mode == 2) {
            return HeatingCircuit::Mode::AUTO;
        } else if (mode == 3) {
            return HeatingCircuit::Mode::HOLIDAY;
        }
    } else { // default for all other thermostats
        if (mode == 0) {
            return HeatingCircuit::Mode::NIGHT;
        } else if (mode == 1) {
            return HeatingCircuit::Mode::DAY;
        } else if (mode == 2) {
            return HeatingCircuit::Mode::AUTO;
        }
    }

    return HeatingCircuit::Mode::UNKNOWN;
}

// figures out the thermostat day/night mode depending on the thermostat type
// mode types are day, night, eco, comfort
uint8_t Thermostat::HeatingCircuit::get_mode_type(uint8_t model) const {
    if (model == EMS_DEVICE_FLAG_JUNKERS) {
        if (mode_type == 3) {
            return HeatingCircuit::Mode::HEAT;
        } else if (mode_type == 2) {
            return HeatingCircuit::Mode::ECO;
        } else if (mode_type == 1) {
            return HeatingCircuit::Mode::NOFROST;
        }
    } else if ((model == EMS_DEVICE_FLAG_RC35) || (model == EMS_DEVICE_FLAG_RC30_1)) {
        if (mode_type == 0) {
            return HeatingCircuit::Mode::NIGHT;
        } else if (mode_type == 1) {
            return HeatingCircuit::Mode::DAY;
        }
    } else if (model == EMS_DEVICE_FLAG_CRF) {
        if (mode_type == 0) {
            return HeatingCircuit::Mode::OFF;
        } else if (mode_type == 1) {
            return HeatingCircuit::Mode::ON;
        }
    } else if (model == EMS_DEVICE_FLAG_RC300) {
        if (mode_type == 0) {
            return HeatingCircuit::Mode::ECO;
        } else if (mode_type == 1) {
            return HeatingCircuit::Mode::COMFORT;
        }
    } else if (model == EMS_DEVICE_FLAG_RC100) {
        return HeatingCircuit::Mode::DAY; // no modes on these devices
    }

    return HeatingCircuit::Mode::DAY;
}

// decodes the thermostat mode based on the thermostat type
// works with both modes and mode_types
std::string Thermostat::mode_tostring(uint8_t mode) {
    switch (mode) {
    case HeatingCircuit::Mode::OFF:
        return read_flash_string(F_(off));
        break;
    case HeatingCircuit::Mode::MANUAL:
        return read_flash_string(F("manual"));
        break;
    case HeatingCircuit::Mode::DAY:
        return read_flash_string(F("day"));
        break;
    case HeatingCircuit::Mode::NIGHT:
        return read_flash_string(F("night"));
        break;
    case HeatingCircuit::Mode::ECO:
        return read_flash_string(F("eco"));
        break;
    case HeatingCircuit::Mode::COMFORT:
        return read_flash_string(F("comfort"));
        break;
    case HeatingCircuit::Mode::HEAT:
        return read_flash_string(F("heat"));
        break;
    case HeatingCircuit::Mode::HOLIDAY:
        return read_flash_string(F("holiday"));
        break;
    case HeatingCircuit::Mode::NOFROST:
        return read_flash_string(F("nofrost"));
        break;
    case HeatingCircuit::Mode::AUTO:
        return read_flash_string(F("auto"));
        break;
    case HeatingCircuit::Mode::SUMMER:
        return read_flash_string(F("summer"));
        break;
    case HeatingCircuit::Mode::OFFSET:
        return read_flash_string(F("offset"));
        break;
    case HeatingCircuit::Mode::DESIGN:
        return read_flash_string(F("design"));
        break;
    case HeatingCircuit::Mode::MINFLOW:
        return read_flash_string(F("minflow"));
        break;
    case HeatingCircuit::Mode::MAXFLOW:
        return read_flash_string(F("maxflow"));
        break;
    case HeatingCircuit::Mode::ROOMINFLUENCE:
        return read_flash_string(F("roominfluence"));
        break;
    case HeatingCircuit::Mode::ON:
        return read_flash_string(F_(on));
        break;
    default:
    case HeatingCircuit::Mode::UNKNOWN:
        return read_flash_string(F("unknown"));
        break;
    }
}

// 0xA8 - for reading the mode from the RC20 thermostat (0x17)
void Thermostat::process_RC20Set(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->mode, 23);
}

// type 0xAE - data from the RC20 thermostat (0x17)
// 17 00 AE 00 80 12 2E 00 D0 00 00 64 (#data=8)
void Thermostat::process_RC20Monitor_2(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_bitvalue(hc->mode_type, 0, 7);      // day/night MSB 7th bit is day
    changed_ |= telegram->read_value(hc->setpoint_roomTemp, 2, 1); // is * 2, force as single byte
    changed_ |= telegram->read_value(hc->curr_roomTemp, 3);        // is * 10
}

// 0xAD - for reading the mode from the RC20/ES72 thermostat (0x17)
// see https://github.com/emsesp/EMS-ESP/issues/334#issuecomment-611698259
// offset: 01-nighttemp, 02-daytemp, 03-mode, 0B-program(1-9), 0D-setpoint_roomtemp(temporary)
void Thermostat::process_RC20Set_2(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->nighttemp, 1); // is * 2,
    changed_ |= telegram->read_value(hc->daytemp, 2);   // is * 2,
    changed_ |= telegram->read_value(hc->mode, 3);
    changed_ |= telegram->read_value(hc->program, 11); // 1 .. 9 predefined programs
}

// 0xAF - for reading the roomtemperature from the RC20/ES72 thermostat (0x18, 0x19, ..)
void Thermostat::process_RC20Remote(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->curr_roomTemp, 0);
}

// type 0xB1 - data from the RC10 thermostat (0x17)
void Thermostat::process_RC10Monitor(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->setpoint_roomTemp, 1, 1); // is * 2, force as single byte
    changed_ |= telegram->read_value(hc->curr_roomTemp, 2);        // is * 10
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"

// type 0xB0 - for reading the mode from the RC10 thermostat (0x17)
void Thermostat::process_RC10Set(std::shared_ptr<const Telegram> telegram) {
    // mode not implemented yet
}
#pragma GCC diagnostic pop

// type 0x0165, ff
void Thermostat::process_JunkersSet(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->daytemp, 17);     // is * 2
    changed_ |= telegram->read_value(hc->nighttemp, 16);   // is * 2
    changed_ |= telegram->read_value(hc->nofrosttemp, 15); // is * 2
}

// type 0x0179, ff
void Thermostat::process_JunkersSet2(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->daytemp, 7);     // is * 2
    changed_ |= telegram->read_value(hc->nighttemp, 6);   // is * 2
    changed_ |= telegram->read_value(hc->nofrosttemp, 5); // is * 2
}

// type 0xA3 - for external temp settings from the the RC* thermostats (e.g. RC35)
void Thermostat::process_RCOutdoorTemp(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(dampedoutdoortemp_, 0);
    changed_ |= telegram->read_value(tempsensor1_, 3); // sensor 1 - is * 10
    changed_ |= telegram->read_value(tempsensor2_, 5); // sensor 2 - is * 10
}

// 0x91 - data from the RC20 thermostat (0x17) - 15 bytes long
void Thermostat::process_RC20Monitor(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->setpoint_roomTemp, 1, 1); // is * 2, force as single byte
    changed_ |= telegram->read_value(hc->curr_roomTemp, 2);        // is * 10
}

// type 0x0A - data from the Nefit Easy/TC100 thermostat (0x18) - 31 bytes long
void Thermostat::process_EasyMonitor(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->curr_roomTemp, 8);      // is * 100
    changed_ |= telegram->read_value(hc->setpoint_roomTemp, 10); // is * 100
}

// Settings Parameters - 0xA5 - RC30_1
void Thermostat::process_IBASettings(std::shared_ptr<const Telegram> telegram) {
    // 22 - display line on RC35
    changed_ |=
        telegram->read_value(ibaMainDisplay_,
                             0); // display on Thermostat: 0 int. temp, 1 int. setpoint, 2 ext. temp., 3 burner temp., 4 ww temp, 5 functioning mode, 6 time, 7 data, 8 smoke temp
    changed_ |= telegram->read_value(ibaLanguage_, 1);          // language on Thermostat: 0 german, 1 dutch, 2 french, 3 italian
    changed_ |= telegram->read_value(ibaCalIntTemperature_, 2); // offset int. temperature sensor, by * 0.1 Kelvin
    changed_ |= telegram->read_value(ibaBuildingType_, 6);      // building type: 0 = light, 1 = medium, 2 = heavy
    changed_ |= telegram->read_value(ibaMinExtTemperature_, 5); // min ext temp for heating curve, in deg., 0xF6=-10, 0x0 = 0, 0xFF=-1
    changed_ |= telegram->read_value(ibaClockOffset_, 12);      // offset (in sec) to clock, 0xff = -1 s, 0x02 = 2 s
}

// Settings WW 0x37 - RC35
void Thermostat::process_RC35wwSettings(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(wwMode_, 2);     // 0 off, 1-on, 2-auto
    changed_ |= telegram->read_value(wwCircMode_, 3); // 0 off, 1-on, 2-auto
}

// type 0x6F - FR10/FR50/FR100/FR110/FR120 Junkers
void Thermostat::process_JunkersMonitor(std::shared_ptr<const Telegram> telegram) {
    // ignore single byte telegram messages
    if (telegram->message_length <= 1) {
        return;
    }

    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->curr_roomTemp, 4);     // value is * 10
    changed_ |= telegram->read_value(hc->setpoint_roomTemp, 2); // value is * 10

    changed_ |= telegram->read_value(hc->mode_type, 0); // 1 = nofrost, 2 = eco, 3 = heat
    changed_ |= telegram->read_value(hc->mode, 1);      // 1 = manual, 2 = auto
}

// type 0x02A5 - data from Worchester CRF200
void Thermostat::process_CRFMonitor(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->curr_roomTemp, 0); // is * 10
    changed_ |= telegram->read_bitvalue(hc->mode_type, 2, 0);
    changed_ |= telegram->read_bitvalue(hc->mode, 2, 4); // bit 4, mode (auto=0 or off=1)
    changed_ |= telegram->read_value(hc->setpoint_roomTemp, 6, 1); // is * 2, force as single byte
    changed_ |= telegram->read_value(hc->targetflowtemp, 4);
}

// type 0x02A5 - data from the Nefit RC1010/3000 thermostat (0x18) and RC300/310s on 0x10
void Thermostat::process_RC300Monitor(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->curr_roomTemp, 0); // is * 10

    changed_ |= telegram->read_bitvalue(hc->mode_type, 10, 1);
    changed_ |= telegram->read_bitvalue(hc->mode, 10, 0); // bit 1, mode (auto=1 or manual=0)

    // if manual, take the current setpoint temp at pos 6
    // if auto, take the next setpoint temp at pos 7
    // pos 3 is the current target temp and sometimes can be 0
    // see https://github.com/emsesp/EMS-ESP/issues/256#issuecomment-585171426
    // pos 3 actual setpoint (optimized), i.e. changes with temporary change, summer/holiday-modes
    // pos 6 actual setpoint according to programmed changes eco/comfort
    // pos 7 next setpoint in the future, time to next setpoint in pos 8/9
    changed_ |= telegram->read_value(hc->setpoint_roomTemp, 3, 1); // is * 2, force as single byte
    changed_ |= telegram->read_bitvalue(hc->summer_mode, 2, 4);
    changed_ |= telegram->read_value(hc->targetflowtemp, 4);
}

// type 0x02B9 EMS+ for reading from RC300/RC310 thermostat
void Thermostat::process_RC300Set(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    // NOTE when setting the room temp we pick from two values, hopefully one is correct!
    // manual is position 10
    // comfort is position 2, there are 3 levels in pos 3, 2, 1
    // eco is position 4
    // auto is position 8, temporary until next switch
    // actual setpoint taken from RC300Monitor (Michael 12.06.2020)
    // changed_ |= telegram->read_value(hc->setpoint_roomTemp, 8, 1);  // single byte conversion, value is * 2 - auto?
    // changed_ |= telegram->read_value(hc->setpoint_roomTemp, 10, 1); // single byte conversion, value is * 2 - manual

    // check why mode is both in the Monitor and Set for the RC300. It'll be read twice!
    // changed_ |= telegram->read_value(hc->mode, 0); // Auto = xFF, Manual = x00 eg. 10 00 FF 08 01 B9 FF
    changed_ |= telegram->read_value(hc->daytemp, 2);     // is * 2
    changed_ |= telegram->read_value(hc->nighttemp, 4);   // is * 2
    changed_ |= telegram->read_value(hc->manualtemp, 10); // is * 2
    changed_ |= telegram->read_value(hc->program, 11);    // timer program 1 or 2
}

// types 0x2AF ff
void Thermostat::process_RC300Summer(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->roominfluence, 0);
    changed_ |= telegram->read_value(hc->offsettemp, 2);
    changed_ |= telegram->read_value(hc->summertemp, 6);
    changed_ |= telegram->read_value(hc->summer_setmode, 7);
    if (hc->heatingtype < 3) {
        changed_ |= telegram->read_value(hc->designtemp, 4);
    } else {
        changed_ |= telegram->read_value(hc->designtemp, 5);
    }
    changed_ |= telegram->read_value(hc->minflowtemp, 8);
}

// types 0x29B ff
void Thermostat::process_RC300Curve(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->controlmode, 0); // 1-outdoor, 2-simple, 3-MPC, 4-room, 5-power, 6-const
    changed_ |= telegram->read_value(hc->heatingtype, 1); // 1=radiator, 2=convector, 3=floor
    changed_ |= telegram->read_value(hc->nofrosttemp, 6);
    if (hc->heatingtype < 3) {
        changed_ |= telegram->read_value(hc->maxflowtemp, 8);
    } else {
        changed_ |= telegram->read_value(hc->maxflowtemp, 7);
    }
}

// types 0x31B (and 0x31C?)
void Thermostat::process_RC300WWtemp(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(wwSetTemp_, 0);
    changed_ |= telegram->read_value(wwSetTempLow_, 1);
}

// type 02F5
void Thermostat::process_RC300WWmode(std::shared_ptr<const Telegram> telegram) {
    // circulation pump see: https://github.com/Th3M3/buderus_ems-wiki/blob/master/Einstellungen%20der%20Bedieneinheit%20RC310.md
    changed_ |= telegram->read_value(wwCircPump_, 1); // FF=off, 0=on ?
    changed_ |= telegram->read_value(wwMode_, 2);     // 0=off, 1=low, 2=high, 3=auto, 4=own prog
    changed_ |= telegram->read_value(wwCircMode_, 3); // 0=off, 1=on, 2=auto, 4=own?
}

// types 0x31D and 0x31E
void Thermostat::process_RC300WWmode2(std::shared_ptr<const Telegram> telegram) {
    // 0x31D for WW system 1, 0x31E for WW system 2
    if (telegram->type_id == 0x031D) {
        changed_ |= telegram->read_value(wwExtra1_, 0); // 0=no, 1=yes
    } else {
        changed_ |= telegram->read_value(wwExtra2_, 0); // 0=no, 1=yes
    }
    // pos 1 = holiday mode
    // pos 2 = current status of DHW setpoint
    // pos 3 = current status of DHW circulation pump
}

// 0x23A damped outdoor temp
void Thermostat::process_RC300OutdoorTemp(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(dampedoutdoortemp2_, 0); // is *10
}

// 0x240 RC300 parameter
void Thermostat::process_RC300Settings(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(ibaBuildingType_, 9); // 1=light, 2=medium, 3=heavy
    changed_ |= telegram->read_value(ibaMinExtTemperature_, 10);
}

// 0x267 RC300 floordrying
void Thermostat::process_RC300Floordry(std::shared_ptr<const Telegram> telegram) {
    changed_ |= telegram->read_value(floordrystatus_, 0);
    changed_ |= telegram->read_value(floordrytemp_, 1);
}

// type 0x41 - data from the RC30 thermostat(0x10) - 14 bytes long
void Thermostat::process_RC30Monitor(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->setpoint_roomTemp, 1, 1); // is * 2, force as single byte
    changed_ |= telegram->read_value(hc->curr_roomTemp, 2);
}

// type 0xA7 - for reading the mode from the RC30 thermostat (0x10)
void Thermostat::process_RC30Set(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->mode, 23);
}

// type 0x3E (HC1), 0x48 (HC2), 0x52 (HC3), 0x5C (HC4) - data from the RC35 thermostat (0x10) - 16 bytes
void Thermostat::process_RC35Monitor(std::shared_ptr<const Telegram> telegram) {
    // exit if the 15th byte (second from last) is 0x00, which I think is calculated flow setpoint temperature
    // with weather controlled RC35s this value is >=5, otherwise can be zero and our setpoint temps will be incorrect
    // see https://github.com/emsesp/EMS-ESP/issues/373#issuecomment-627907301
    if (telegram->offset > 0 || telegram->message_length < 15) {
        return;
    }
    if (telegram->message_data[14] == 0x00) {
        return;
    }

    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->setpoint_roomTemp, 2, 1); // is * 2, force to single byte, is 0 in summermode
    changed_ |= telegram->read_value(hc->curr_roomTemp, 3);        // is * 10 - or 0x7D00 if thermostat is mounted on boiler

    changed_ |= telegram->read_bitvalue(hc->mode_type, 1, 1);
    changed_ |= telegram->read_bitvalue(hc->summer_mode, 1, 0);
    changed_ |= telegram->read_bitvalue(hc->holiday_mode, 0, 5);

    changed_ |= telegram->read_value(hc->targetflowtemp, 14);
}

// type 0x3D (HC1), 0x47 (HC2), 0x51 (HC3), 0x5B (HC4) - Working Mode Heating - for reading the mode from the RC35 thermostat (0x10)
void Thermostat::process_RC35Set(std::shared_ptr<const Telegram> telegram) {
    // check to see we have a valid type. heating: 1 radiator, 2 convectors, 3 floors, 4 room supply
    if (telegram->offset == 0 && telegram->message_data[0] == 0x00) {
        return;
    }

    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->heatingtype, 0);   // 0- off, 1-radiator, 2-convector, 3-floor
    changed_ |= telegram->read_value(hc->nighttemp, 1);     // is * 2
    changed_ |= telegram->read_value(hc->daytemp, 2);       // is * 2
    changed_ |= telegram->read_value(hc->holidaytemp, 3);   // is * 2
    changed_ |= telegram->read_value(hc->roominfluence, 4); // is * 1
    changed_ |= telegram->read_value(hc->offsettemp, 6);    // is * 2
    changed_ |= telegram->read_value(hc->mode, 7);          // night, day, auto

    changed_ |= telegram->read_value(hc->summertemp, 22);     // is * 1
    changed_ |= telegram->read_value(hc->nofrosttemp, 23);    // is * 1
    changed_ |= telegram->read_value(hc->flowtempoffset, 24); // is * 1, only in mixed circuits
    changed_ |= telegram->read_value(hc->reducemode, 25);     // 0-nofrost, 1-reduce, 2-roomhold, 3-outdoorhold
    changed_ |= telegram->read_value(hc->controlmode, 33);    // 0-outdoortemp, 1-roomtemp
    // changed_ |= telegram->read_value(hc->noreducetemp, 38);    // outdoor temperature for no reduce
    changed_ |= telegram->read_value(hc->minflowtemp, 16);
    if (hc->heatingtype == 3) {
        changed_ |= telegram->read_value(hc->designtemp, 36);  // is * 1
        changed_ |= telegram->read_value(hc->maxflowtemp, 35); // is * 1
    } else {
        changed_ |= telegram->read_value(hc->designtemp, 17);  // is * 1
        changed_ |= telegram->read_value(hc->maxflowtemp, 15); // is * 1
    }
}

// type 0x3F (HC1), 0x49 (HC2), 0x53 (HC3), 0x5D (HC4) - timer setting
void Thermostat::process_RC35Timer(std::shared_ptr<const Telegram> telegram) {
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(telegram);
    if (hc == nullptr) {
        return;
    }
    changed_ |= telegram->read_value(hc->program, 84); // 0 .. 10, 0-userprogram 1, 10-userprogram 2
}

// process_RCTime - type 0x06 - date and time from a thermostat - 14 bytes long
void Thermostat::process_RCTime(std::shared_ptr<const Telegram> telegram) {
    if (flags() == EMS_DEVICE_FLAG_EASY) {
        return; // not supported
    }
    if (telegram->offset > 0 || telegram->message_length < 8) {
        return;
    }
    if (telegram->message_data[7] & 0x0C) { // date and time not valid
        set_datetime("ntp", -1);            // set from NTP
        return;
    }
    if (datetime_.empty()) {
        datetime_.resize(25, '\0');
    }
    auto timeold = datetime_;
    // render time to HH:MM:SS DD/MM/YYYY
    // had to create separate buffers because of how printf works
    char buf1[6];
    char buf2[6];
    char buf3[6];
    char buf4[6];
    char buf5[6];
    char buf6[6];
    snprintf_P(&datetime_[0],
               datetime_.capacity() + 1,
               PSTR("%s:%s:%s %s/%s/%s"),
               Helpers::smallitoa(buf1, telegram->message_data[2]),  // hour
               Helpers::smallitoa(buf2, telegram->message_data[4]),  // minute
               Helpers::smallitoa(buf3, telegram->message_data[5]),  // second
               Helpers::smallitoa(buf4, telegram->message_data[3]),  // day
               Helpers::smallitoa(buf5, telegram->message_data[1]),  // month
               Helpers::itoa(buf6, telegram->message_data[0] + 2000) // year
    );
    if (timeold != datetime_) {
        changed_ = true;
    }
}

// process_RCError - type 0xA2 - error message - 14 bytes long
// 10 00 A2 00 41 32 32 03 30 00 02 00 00 00 00 00 00 02 CRC
//              A  2  2  816
void Thermostat::process_RCError(std::shared_ptr<const Telegram> telegram) {
    if (telegram->offset > 0 || telegram->message_length < 5) {
        return;
    }
    if (errorCode_.empty()) {
        errorCode_.resize(10, '\0');
    }
    char buf[4];
    buf[0] = telegram->message_data[0];
    buf[1] = telegram->message_data[1];
    buf[2] = telegram->message_data[2];
    buf[3] = 0;
    changed_ |= telegram->read_value(errorNumber_, 3);

    snprintf_P(&errorCode_[0], errorCode_.capacity() + 1, PSTR("%s(%d)"), buf, errorNumber_);
}

// 0x12 error log
void Thermostat::process_RCErrorMessage(std::shared_ptr<const Telegram> telegram) {
    if (telegram->offset > 0 || telegram->message_length < 12) {
        return;
    }
    // data: displaycode(2), errornumber(2), year, month, hour, day, minute, duration(2), src-addr
    if (telegram->message_data[4] & 0x80) { // valid date
        char     code[3];
        uint16_t codeNo __attribute__ ((aligned (4)));
        code[0] = telegram->message_data[0];
        code[1] = telegram->message_data[1];
        code[2] = 0;
        telegram->read_value(codeNo, 2);
        uint16_t year  __attribute__ ((aligned (4))) = (telegram->message_data[4] & 0x7F) + 2000;
        uint8_t  month = telegram->message_data[5];
        uint8_t  day   = telegram->message_data[7];
        uint8_t  hour  = telegram->message_data[6];
        uint8_t  min   = telegram->message_data[8];
        snprintf_P(lastCode_, sizeof(lastCode_), PSTR("%s(%d) %02d.%02d.%d %02d:%02d"), code, codeNo, day, month, year, hour, min);
    }
}

// 0xA5 - Set minimum external temperature
bool Thermostat::set_minexttemp(const char * value, const int8_t id) {
    int mt = 0;
    if (!Helpers::value2number(value, mt)) {
        LOG_WARNING(F("Set min external temperature: Invalid value"));
        return false;
    }

    LOG_INFO(F("Setting min external temperature to %d C"), mt);
    if ((model() == EMS_DEVICE_FLAG_RC300) || (model() == EMS_DEVICE_FLAG_RC100)) {
        write_command(0x240, 10, mt, 0x240);
    } else {
        write_command(EMS_TYPE_IBASettings, 5, mt, EMS_TYPE_IBASettings);
    }

    return true;
}

// 0xA5 - Clock offset
bool Thermostat::set_clockoffset(const char * value, const int8_t id) {
    int co = 0;
    if (!Helpers::value2number(value, co)) {
        LOG_WARNING(F("Set clock offset: Invalid value"));
        return false;
    }

    LOG_INFO(F("Setting clock offset to %d"), co);
    write_command(EMS_TYPE_IBASettings, 12, co, EMS_TYPE_IBASettings);

    return true;
}

// 0xA5 - Calibrate internal temperature
bool Thermostat::set_calinttemp(const char * value, const int8_t id) {
    int ct = 0;
    if (!Helpers::value2number(value, ct)) {
        LOG_WARNING(F("Cal internal temperature: Invalid value"));
        return false;
    }

    LOG_INFO(F("Calibrating internal temperature to %d.%d C"), ct / 10, ct < 0 ? -ct % 10 : ct % 10);
    write_command(EMS_TYPE_IBASettings, 2, ct, EMS_TYPE_IBASettings);

    return true;
}

// 0xA5 - Set the display settings
bool Thermostat::set_display(const char * value, const int8_t id) {
    int ds = 0;
    if (!Helpers::value2number(value, ds)) {
        LOG_WARNING(F("Set display: Invalid value"));
        return false;
    }

    LOG_INFO(F("Setting display to %d"), ds);
    write_command(EMS_TYPE_IBASettings, 0, ds, EMS_TYPE_IBASettings);

    return true;
}

// set roomtemp for HA-thermostat
bool Thermostat::set_roomtemp(const char * value, const int8_t id) {
    float f = 0;
    if (!Helpers::value2float(value, f)) {
        LOG_WARNING(F("Set roomtemperature: Invalid value"));
        return false;
    }

    uint8_t                                     hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    std::shared_ptr<Thermostat::HeatingCircuit> hc     = heating_circuit(hc_num);
    if (hc == nullptr) {
        return false;
    }

    if (f > 100 || f < 0) {
        hc->ha_temp = EMS_VALUE_SHORT_NOTSET;
    } else {
        hc->ha_temp = (int16_t)(f * 10);
    }

    return true;
}

// set remotetemp for RC20 remote simulation on RC35 master
bool Thermostat::set_remotetemp(const char * value, const int8_t id) {
    float f = 0;
    if (!Helpers::value2float(value, f)) {
        LOG_WARNING(F("Set remote temperature: Invalid value"));
        return false;
    }

    uint8_t                                     hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    std::shared_ptr<Thermostat::HeatingCircuit> hc     = heating_circuit(hc_num);
    if (hc == nullptr) {
        return false;
    }

    if (f > 100 || f < 0) {
        Roomctrl::set_remotetemp(hc->hc_num() - 1, EMS_VALUE_SHORT_NOTSET);
    } else {
        Roomctrl::set_remotetemp(hc->hc_num() - 1, (int16_t)(f * 10));
    }

    return true;
}

// 0xA5 - Set the building settings
bool Thermostat::set_building(const char * value, const int8_t id) {
    uint8_t bd = 0;
    if (!Helpers::value2enum(value, bd, {F("light"), F("medium"), F("heavy")})) {
        LOG_WARNING(F("Set building: Invalid value"));
        return false;
    }

    LOG_INFO(F("Setting building to %s"), value);

    if ((model() == EMS_DEVICE_FLAG_RC300) || (model() == EMS_DEVICE_FLAG_RC100)) {
        write_command(0x240, 9, bd + 1, 0x240);
    } else {
        write_command(EMS_TYPE_IBASettings, 6, bd, EMS_TYPE_IBASettings);
    }

    return true;
}

// 0xA5 Set the language settings
bool Thermostat::set_language(const char * value, const int8_t id) {
    uint8_t lg = 0;
    if (!Helpers::value2enum(value, lg, {F("german"), F("dutch"), F("french"), F("italian")})) {
        LOG_WARNING(F("Set language: Invalid value"));
        return false;
    }

    LOG_INFO(F("Setting language to %d"), lg);
    write_command(EMS_TYPE_IBASettings, 1, lg, EMS_TYPE_IBASettings);

    return true;
}

// Set the control-mode for hc 0-off, 1-RC20, 2-RC3x
bool Thermostat::set_control(const char * value, const int8_t id) {
    uint8_t ctrl = 0;
    if (!Helpers::value2enum(value, ctrl, {F("off"), F("rc20"), F("rc3x")})) {
        LOG_WARNING(F("Set control: Invalid value"));
        return false;
    }

    uint8_t                                     hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    std::shared_ptr<Thermostat::HeatingCircuit> hc     = heating_circuit(hc_num);
    if (hc == nullptr) {
        return false;
    }

    LOG_INFO(F("Setting circuit-control for hc%d to %d"), hc_num, ctrl);
    write_command(set_typeids[hc->hc_num() - 1], 26, ctrl);

    return true;
}

// sets the thermostat ww working mode, where mode is a string, ems and ems+
bool Thermostat::set_wwmode(const char * value, const int8_t id) {
    uint8_t set = 0xFF;

    if ((model() == EMS_DEVICE_FLAG_RC300) || (model() == EMS_DEVICE_FLAG_RC100)) {
        if (!Helpers::value2enum(value, set, {F("off"), F("low"), F("high"), F("auto"), F("own")})) {
            LOG_WARNING(F("Set warm water mode: Invalid mode"));
            return false;
        }
        LOG_INFO(F("Setting warm water mode to %s"), value);
        write_command(0x02F5, 2, set, 0x02F5);
    } else {
        if (!Helpers::value2enum(value, set, {F("off"), F("on"), F("auto")})) {
            LOG_WARNING(F("Set warm water mode: Invalid mode"));
            return false;
        }
        LOG_INFO(F("Setting warm water mode to %s"), value);
        write_command(EMS_TYPE_wwSettings, 2, set, EMS_TYPE_wwSettings);
    }
    return true;
}

// Set ww high temperature, ems+
bool Thermostat::set_wwtemp(const char * value, const int8_t id) {
    int t = 0;
    if (!Helpers::value2number(value, t)) {
        LOG_WARNING(F("Set warm water temperature: Invalid value"));
        return false;
    }
    LOG_INFO(F("Setting warm water temperature to %d C"), t);
    write_command(0x031B, 0, t, 0x031B);
    return true;
}

// Set ww low temperature, ems+
bool Thermostat::set_wwtemplow(const char * value, const int8_t id) {
    int t = 0;
    if (!Helpers::value2number(value, t)) {
        LOG_WARNING(F("Set warm water low temperature: Invalid value"));
        return false;
    }
    LOG_INFO(F("Setting warm water low temperature to %d C"), t);
    write_command(0x031B, 1, t, 0x031B);
    return true;
}

// Set ww onetime RC300, ems+
bool Thermostat::set_wwonetime(const char * value, const int8_t id) {
    bool b = false;
    if (!Helpers::value2bool(value, b)) {
        LOG_WARNING(F("Set warm water onetime: Invalid value"));
        return false;
    }
    char s[7];
    LOG_INFO(F("Setting warm water onetime to %s"), Helpers::render_boolean(s, b));
    write_command(0x02F5, 11, b ? 0xFF : 0x00, 0x031D);
    return true;
}

// sets the thermostat ww circulation working mode, where mode is a string
bool Thermostat::set_wwcircmode(const char * value, const int8_t id) {
    uint8_t set = 0xFF;

    if ((model() == EMS_DEVICE_FLAG_RC300) || (model() == EMS_DEVICE_FLAG_RC100)) {
        if (!Helpers::value2enum(value, set, {F("off"), F("on"), F("auto"), F("own")})) {
            LOG_WARNING(F("Set warm water circulation mode: Invalid mode"));
            return false;
        }
        LOG_INFO(F("Setting warm water circulation mode to %s"), value);
        write_command(0x02F5, 3, set, 0x02F5);
        return true;
    }
    if (!Helpers::value2enum(value, set, {F("off"), F("on"), F("auto")})) {
        LOG_WARNING(F("Set warm water circulation mode: Invalid mode"));
        return false;
    }
    LOG_INFO(F("Setting warm water circulation mode to %s"), value);
    write_command(EMS_TYPE_wwSettings, 3, set, EMS_TYPE_wwSettings);
    return true;
}

// set the holiday as string dd.mm.yyyy-dd.mm.yyyy
bool Thermostat::set_holiday(const char * value, const int8_t id) {
    std::string hd(30, '\0');
    if (!Helpers::value2string(value, hd)) {
        LOG_WARNING(F("Set holiday: Invalid value"));
        return false;
    }
    uint8_t                                     hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    std::shared_ptr<Thermostat::HeatingCircuit> hc     = heating_circuit(hc_num);
    if (hc == nullptr) {
        LOG_WARNING(F("Set holiday: Heating Circuit %d not found or activated for device ID 0x%02X"), hc_num, device_id());
        return false;
    }
    if (hd.length() != 21 || hd[6] != '2' || hd[17] != '2') {
        LOG_WARNING(F("Set holiday: Invalid value"));
        return false;
    }
    uint8_t data[6];
    data[0] = (hd[0] - '0') * 10 + (hd[1] - '0');
    data[1] = (hd[3] - '0') * 10 + (hd[4] - '0');
    data[2] = (hd[7] - '0') * 100 + (hd[8] - '0') * 10 + (hd[9] - '0');
    data[3] = (hd[11] - '0') * 10 + (hd[12] - '0');
    data[4] = (hd[14] - '0') * 10 + (hd[15] - '0');
    data[5] = (hd[18] - '0') * 100 + (hd[19] - '0') * 10 + (hd[20] - '0');

    if (hd[10] == '-') {
        LOG_INFO(F("Setting holiday away from home for hc %d"), hc->hc_num());
        write_command(timer_typeids[hc->hc_num() - 1], 87, data, 6, 0);
    } else if (hd[10] == '+') {
        LOG_INFO(F("Setting holiday at home for hc %d"), hc->hc_num());
        write_command(timer_typeids[hc->hc_num() - 1], 93, data, 6, 0);
    } else {
        LOG_WARNING(F("Set holiday: Invalid"));
        return false;
    }

    return true;
}

// set pause in hours
bool Thermostat::set_pause(const char * value, const int8_t id) {
    int hrs = 0;
    if (!Helpers::value2number(value, hrs)) {
        LOG_WARNING(F("Set pause: Invalid value"));
        return false;
    }

    uint8_t                                     hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    std::shared_ptr<Thermostat::HeatingCircuit> hc     = heating_circuit(hc_num);
    if (hc == nullptr) {
        LOG_WARNING(F("Set pause: Heating Circuit %d not found or activated for device ID 0x%02X"), hc_num, device_id());
        return false;
    }

    LOG_INFO(F("Setting pause: %d hours, hc: %d"), hrs, hc->hc_num());
    write_command(timer_typeids[hc->hc_num() - 1], 85, hrs);

    return true;
}

// set partymode in hours
bool Thermostat::set_party(const char * value, const int8_t id) {
    int hrs = 0;
    if (!Helpers::value2number(value, hrs)) {
        LOG_WARNING(F("Set party: Invalid value"));
        return false;
    }
    uint8_t hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;

    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(hc_num);
    if (hc == nullptr) {
        LOG_WARNING(F("Set party: Heating Circuit %d not found or activated for device ID 0x%02X"), hc_num, device_id());
        return false;
    }
    LOG_INFO(F("Setting party: %d hours, hc: %d"), hrs, hc->hc_num());
    write_command(timer_typeids[hc->hc_num() - 1], 86, hrs);

    return true;
}

// set date&time as string hh:mm:ss-dd.mm.yyyy-dw-dst or "NTP" for setting to internet-time
// dw - day of week (0..6), dst- summertime (0/1)
// id is ignored
bool Thermostat::set_datetime(const char * value, const int8_t id) {
    std::string dt(30, '\0');
    if (!Helpers::value2string(value, dt)) {
        LOG_WARNING(F("Set date: Invalid value"));
        return false;
    }

    uint8_t data[9];
    if (dt == "ntp") {
        time_t now = time(nullptr);
        tm *   tm_ = localtime(&now);
        if (tm_->tm_year < 110) { // no NTP time
            LOG_WARNING(F("No NTP time. Cannot set RCtime"));
            return false;
        }

        data[0] = tm_->tm_year - 100; // Bosch counts from 2000
        data[1] = tm_->tm_mon + 1;
        data[2] = tm_->tm_hour;
        data[3] = tm_->tm_mday;
        data[4] = tm_->tm_min;
        data[5] = tm_->tm_sec;
        data[6] = (tm_->tm_wday + 6) % 7; // Bosch counts from Mo, time from Su
        data[7] = tm_->tm_isdst + 2;      // set DST and flag for ext. clock
        char time_string[25];
        strftime(time_string, 25, "%FT%T%z", tm_);
        LOG_INFO(F("Date and time: %s"), time_string);
    } else {
        data[0] = (dt[16] - '0') * 100 + (dt[17] - '0') * 10 + (dt[18] - '0'); // year
        data[1] = (dt[12] - '0') * 10 + (dt[13] - '0');                        // month
        data[2] = (dt[0] - '0') * 10 + (dt[1] - '0');                          // hour
        data[3] = (dt[9] - '0') * 10 + (dt[10] - '0');                         // day
        data[4] = (dt[3] - '0') * 10 + (dt[4] - '0');                          // min
        data[5] = (dt[6] - '0') * 10 + (dt[7] - '0');                          // sec
        data[6] = (dt[20] - '0');                                              // day of week
        data[7] = (dt[22] - '0') + 2;                                          // DST and flag
    }

    LOG_INFO(F("Setting date and time"));
    write_command(EMS_TYPE_time, 0, data, 8, EMS_TYPE_time);

    return true;
}

// sets the thermostat working mode, where mode is a string
// converts string mode to HeatingCircuit::Mode
bool Thermostat::set_mode(const char * value, const int8_t id) {
    // quit if its numerical, as it could be mistaken as a temperature value
    if (value[0] < 'A') {
        return false;
    }

    std::string mode(10, '\0');
    if (!Helpers::value2string(value, mode)) {
        LOG_WARNING(F("Set mode: Invalid mode"));
        return false;
    }

    uint8_t hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    if (mode_tostring(HeatingCircuit::Mode::OFF) == mode) {
        return set_mode_n(HeatingCircuit::Mode::OFF, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::MANUAL) == mode) {
        return set_mode_n(HeatingCircuit::Mode::MANUAL, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::AUTO) == mode) {
        return set_mode_n(HeatingCircuit::Mode::AUTO, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::DAY) == mode) {
        return set_mode_n(HeatingCircuit::Mode::DAY, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::NIGHT) == mode) {
        return set_mode_n(HeatingCircuit::Mode::NIGHT, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::HEAT) == mode) {
        return set_mode_n(HeatingCircuit::Mode::HEAT, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::NOFROST) == mode) {
        return set_mode_n(HeatingCircuit::Mode::NOFROST, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::ECO) == mode) {
        return set_mode_n(HeatingCircuit::Mode::ECO, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::HOLIDAY) == mode) {
        return set_mode_n(HeatingCircuit::Mode::HOLIDAY, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::COMFORT) == mode) {
        return set_mode_n(HeatingCircuit::Mode::COMFORT, hc_num);
    }

    LOG_WARNING(F("Set mode: Invalid mode %s"), value);
    return false;
}

// Set the thermostat working mode
// mode is HeatingCircuit::Mode
bool Thermostat::set_mode_n(const uint8_t mode, const uint8_t hc_num) {
    // get hc based on number
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(hc_num);
    if (hc == nullptr) {
        LOG_WARNING(F("Set mode: Heating Circuit %d not found or activated"), hc_num);
        return false;
    }

    uint8_t  set_mode_value, offset;
    uint16_t validate_typeid __attribute__ ((aligned (4))) = 0;
    uint8_t  hc_p            = hc->hc_num() - 1;

    // set the value to send via EMS depending on the mode type
    switch (mode) {
    case HeatingCircuit::Mode::NIGHT:
    case HeatingCircuit::Mode::OFF:
        set_mode_value = 0;
        break;

    case HeatingCircuit::Mode::DAY:
    case HeatingCircuit::Mode::HEAT:
    case HeatingCircuit::Mode::MANUAL:
    case HeatingCircuit::Mode::NOFROST:
        set_mode_value = 1;
        break;

    default:
    case HeatingCircuit::Mode::AUTO:
    case HeatingCircuit::Mode::ECO:
        set_mode_value = 2;
        break;
    }

    switch (model()) {
    case EMSdevice::EMS_DEVICE_FLAG_RC20:
        offset          = EMS_OFFSET_RC20Set_mode;
        validate_typeid = set_typeids[hc_p];
        break;
    case EMSdevice::EMS_DEVICE_FLAG_RC20_2: // ES72
        offset          = EMS_OFFSET_RC20_2_Set_mode;
        validate_typeid = set_typeids[hc_p];
        break;
    case EMSdevice::EMS_DEVICE_FLAG_RC30:
        offset          = EMS_OFFSET_RC30Set_mode;
        validate_typeid = set_typeids[hc_p];
        break;
    case EMSdevice::EMS_DEVICE_FLAG_RC35:
    case EMSdevice::EMS_DEVICE_FLAG_RC30_1:
        offset          = EMS_OFFSET_RC35Set_mode;
        validate_typeid = set_typeids[hc_p];
        break;
    case EMSdevice::EMS_DEVICE_FLAG_RC300:
    case EMSdevice::EMS_DEVICE_FLAG_RC100:
        offset          = EMS_OFFSET_RCPLUSSet_mode;
        validate_typeid = monitor_typeids[hc_p];
        if (mode == HeatingCircuit::Mode::AUTO) {
            set_mode_value = 0xFF; // special value for auto
        } else {
            set_mode_value = 0; // everything else, like manual/day etc..
        }
        break;
    case EMSdevice::EMS_DEVICE_FLAG_JUNKERS:
        if (has_flags(EMS_DEVICE_FLAG_JUNKERS_OLD)) {
            offset = EMS_OFFSET_JunkersSetMessage2_set_mode;
        } else {
            offset = EMS_OFFSET_JunkersSetMessage_set_mode;
        }
        validate_typeid = monitor_typeids[hc_p];
        if (mode == HeatingCircuit::Mode::NOFROST) {
            set_mode_value = 0x01;
        } else if (mode == HeatingCircuit::Mode::ECO || (mode == HeatingCircuit::Mode::NIGHT)) {
            set_mode_value = 0x02;
        } else if ((mode == HeatingCircuit::Mode::DAY) || (mode == HeatingCircuit::Mode::HEAT)) {
            set_mode_value = 0x03; // comfort
        } else if (mode == HeatingCircuit::Mode::AUTO) {
            set_mode_value = 0x04;
        }
        break;
    default:
        offset = 0;
        break;
    }

    LOG_INFO(F("Setting thermostat mode to %s for heating circuit %d"), mode_tostring(mode).c_str(), hc->hc_num());

    // add the write command to the Tx queue
    // post validate is the corresponding monitor or set type IDs as they can differ per model
    write_command(set_typeids[hc->hc_num() - 1], offset, set_mode_value, validate_typeid);

    return true;
}

// sets the thermostat summermode for RC300
bool Thermostat::set_summermode(const char * value, const int8_t id) {
    uint8_t                                     hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    std::shared_ptr<Thermostat::HeatingCircuit> hc     = heating_circuit(hc_num);
    if (hc == nullptr) {
        LOG_WARNING(F("Setting summer mode: Heating Circuit %d not found or activated"), hc_num);
        return false;
    }
    uint8_t set = 0xFF;
    if (!Helpers::value2enum(value, set, {F("summer"), F("auto"), F("winter")})) {
        LOG_WARNING(F("Setting summer mode: Invalid mode"));
        return false;
    }
    LOG_INFO(F("Setting summer mode to %s for heating circuit %d"), value, hc->hc_num());
    write_command(summer_typeids[hc->hc_num() - 1], 7, set, summer_typeids[hc->hc_num() - 1]);
    return true;
}

// sets the thermostat reducemode for RC35
bool Thermostat::set_reducemode(const char * value, const int8_t id) {
    uint8_t                                     hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    std::shared_ptr<Thermostat::HeatingCircuit> hc     = heating_circuit(hc_num);
    if (hc == nullptr) {
        LOG_WARNING(F("Setting reduce mode: Heating Circuit %d not found or activated"), hc_num);
        return false;
    }
    uint8_t set = 0xFF;
    if (!Helpers::value2enum(value, set, {F("nofrost"), F("reduce"), F("room"), F("outdoor")})) {
        LOG_WARNING(F("Setting reduce mode: Invalid mode"));
        return false;
    }
    LOG_INFO(F("Setting reduce mode to %s for heating circuit %d"), value, hc->hc_num());
    write_command(set_typeids[hc->hc_num() - 1], 25, set, set_typeids[hc->hc_num() - 1]);
    return true;
}

// sets the thermostat controlmode for RC35, RC300
bool Thermostat::set_controlmode(const char * value, const int8_t id) {
    uint8_t                                     hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    std::shared_ptr<Thermostat::HeatingCircuit> hc     = heating_circuit(hc_num);
    if (hc == nullptr) {
        LOG_WARNING(F("Setting control mode: Heating Circuit %d not found or activated"), hc_num);
        return false;
    }
    uint8_t set = 0xFF;
    if (model() == EMS_DEVICE_FLAG_RC300 || model() == EMS_DEVICE_FLAG_RC100) {
        if (Helpers::value2enum(value, set, {F("off"), F("outdoor"), F("simple"), F("MPC"), F("room"), F("power"), F("const")})) {
            LOG_INFO(F("Setting control mode to %d for heating circuit %d"), set, hc->hc_num());
            write_command(curve_typeids[hc->hc_num() - 1], 0, set, curve_typeids[hc->hc_num() - 1]);
            return true;
        }
    } else if (model() == EMS_DEVICE_FLAG_RC35 || model() == EMS_DEVICE_FLAG_RC30_1) {
        if (Helpers::value2enum(value, set, {F("outdoor"), F("room")})) {
            LOG_INFO(F("Setting control mode to %d for heating circuit %d"), set, hc->hc_num());
            write_command(set_typeids[hc->hc_num() - 1], 33, set, set_typeids[hc->hc_num() - 1]);
            return true;
        }
    }
    LOG_WARNING(F("Setting control mode: Invalid mode"));
    return false;
}

// sets the thermostat program for RC35 and RC20
bool Thermostat::set_program(const char * value, const int8_t id) {
    uint8_t                                     hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    std::shared_ptr<Thermostat::HeatingCircuit> hc     = heating_circuit(hc_num);
    if (hc == nullptr) {
        LOG_WARNING(F("Setting program: Heating Circuit %d not found or activated"), hc_num);
        return false;
    }
    int set = 0xFF;
    if (!Helpers::value2number(value, set)) {
        LOG_WARNING(F("Setting program: Invalid number"));
        return false;
    }

    if (model() == EMS_DEVICE_FLAG_RC20_2 && set > 0 && set < 10) {
        write_command(set_typeids[hc->hc_num() - 1], 11, set, set_typeids[hc->hc_num() - 1]);
    } else if ((model() == EMS_DEVICE_FLAG_RC35 || model() == EMS_DEVICE_FLAG_RC30_1) && set < 11) {
        write_command(timer_typeids[hc->hc_num() - 1], 84, set, timer_typeids[hc->hc_num() - 1]);
    } else if ((model() == EMS_DEVICE_FLAG_RC300 || model() == EMS_DEVICE_FLAG_RC100) && (set == 1 || set == 2)) {
        write_command(set_typeids[hc->hc_num() - 1], 11, set, set_typeids[hc->hc_num() - 1]);
    } else {
        LOG_WARNING(F("Setting program: Invalid number"));
        return false;
    }
    LOG_INFO(F("Setting program to %d for heating circuit %d"), set, hc->hc_num());
    return true;
}

// sets the thermostat temp, where mode is a string
bool Thermostat::set_temperature(const float temperature, const std::string & mode, const uint8_t hc_num) {
    if (mode_tostring(HeatingCircuit::Mode::MANUAL) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::MANUAL, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::AUTO) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::AUTO, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::DAY) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::DAY, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::NIGHT) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::NIGHT, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::COMFORT) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::COMFORT, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::HEAT) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::HEAT, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::ECO) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::ECO, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::NOFROST) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::NOFROST, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::SUMMER) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::SUMMER, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::HOLIDAY) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::HOLIDAY, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::OFFSET) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::OFFSET, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::DESIGN) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::DESIGN, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::MINFLOW) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::MINFLOW, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::MAXFLOW) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::MAXFLOW, hc_num);
    }
    if (mode_tostring(HeatingCircuit::Mode::ROOMINFLUENCE) == mode) {
        return set_temperature(temperature, HeatingCircuit::Mode::ROOMINFLUENCE, hc_num);
    }

    LOG_WARNING(F("Set temperature: Invalid mode"));
    return false;
}

// Set the temperature of the thermostat
// the id passed into this function is the heating circuit number
bool Thermostat::set_temperature(const float temperature, const uint8_t mode, const uint8_t hc_num) {
    // get hc based on number
    std::shared_ptr<Thermostat::HeatingCircuit> hc = heating_circuit(hc_num);
    if (hc == nullptr) {
        LOG_WARNING(F("Set temperature: Heating Circuit %d not found or activated for device ID 0x%02X"), hc_num, device_id());
        return false;
    }

    uint8_t  model           = this->model();
    int8_t   offset          = -1; // we use -1 to check if there is a value
    uint8_t  factor          = 2;  // some temperatures only use 1
    uint16_t validate_typeid __attribute__ ((aligned (4))) = monitor_typeids[hc->hc_num() - 1];
    uint16_t set_typeid      __attribute__ ((aligned (4))) = set_typeids[hc->hc_num() - 1];

    if (model == EMS_DEVICE_FLAG_RC10) {
        offset = EMS_OFFSET_RC10Set_temp;

    } else if (model == EMS_DEVICE_FLAG_RC20) {
        offset = EMS_OFFSET_RC20Set_temp;

    } else if (model == EMS_DEVICE_FLAG_RC30) {
        offset = EMS_OFFSET_RC30Set_temp;

    } else if ((model == EMS_DEVICE_FLAG_RC300) || (model == EMS_DEVICE_FLAG_RC100)) {
        validate_typeid = set_typeids[hc->hc_num() - 1];
        switch (mode) {
        case HeatingCircuit::Mode::SUMMER:
            offset          = 0x06;
            set_typeid      = summer_typeids[hc->hc_num() - 1];
            validate_typeid = set_typeid;
            factor          = 1;
            break;
        case HeatingCircuit::Mode::MANUAL:
            offset = 0x0A; // manual offset
            break;
        case HeatingCircuit::Mode::COMFORT:
            offset = 0x02; // comfort offset level 2
            break;
        case HeatingCircuit::Mode::ECO:
            offset = 0x04; // eco offset
            break;
        case HeatingCircuit::Mode::OFFSET:
            offset          = 2;
            set_typeid      = summer_typeids[hc->hc_num() - 1];
            validate_typeid = set_typeid;
            factor          = 1;
            break;
        case HeatingCircuit::Mode::DESIGN:
            set_typeid      = summer_typeids[hc->hc_num() - 1];
            validate_typeid = set_typeid;
            if (hc->heatingtype == 3) {
                offset = 5;
            } else {
                offset = 4;
            }
            factor = 1;
            break;
        case HeatingCircuit::Mode::MINFLOW:
            set_typeid      = summer_typeids[hc->hc_num() - 1];
            validate_typeid = set_typeid;
            offset          = 8;
            factor          = 1;
            break;
        case HeatingCircuit::Mode::MAXFLOW:
            set_typeid      = curve_typeids[hc->hc_num() - 1];
            validate_typeid = set_typeid;
            if (hc->heatingtype == 3) {
                offset = 7;
            } else {
                offset = 8;
            }
            factor = 1;
            break;
        case HeatingCircuit::Mode::NOFROST:
            set_typeid      = curve_typeids[hc->hc_num() - 1];
            validate_typeid = set_typeid;
            offset          = 6;
            factor          = 1;
            break;
        case HeatingCircuit::Mode::ROOMINFLUENCE:
            set_typeid      = summer_typeids[hc->hc_num() - 1];
            validate_typeid = set_typeid;
            offset          = 0;
            factor          = 1;
            break;
        default:
        case HeatingCircuit::Mode::AUTO:
            uint8_t mode_ = hc->get_mode(flags());
            if (mode_ == HeatingCircuit::Mode::MANUAL) {
                offset = 0x0A; // manual offset
            } else {
                offset = 0x08; // auto offset
            }
            validate_typeid = monitor_typeids[hc->hc_num() - 1]; // get setpoint roomtemp back
            break;
        }

    } else if (model == EMS_DEVICE_FLAG_RC20_2) {
        switch (mode) {
        case HeatingCircuit::Mode::NIGHT: // change the night temp
            offset = EMS_OFFSET_RC20_2_Set_temp_night;
            break;
        case HeatingCircuit::Mode::DAY: // change the day temp
            offset = EMS_OFFSET_RC20_2_Set_temp_day;
            break;
        default:
        case HeatingCircuit::Mode::AUTO: // automatic selection, if no type is defined, we use the standard code
            uint8_t mode_type = hc->get_mode_type(flags());
            offset            = (mode_type == HeatingCircuit::Mode::NIGHT) ? EMS_OFFSET_RC20_2_Set_temp_night : EMS_OFFSET_RC20_2_Set_temp_day;
            break;
        }

    } else if ((model == EMS_DEVICE_FLAG_RC35) || (model == EMS_DEVICE_FLAG_RC30_1)) {
        validate_typeid = set_typeids[hc->hc_num() - 1];
        switch (mode) {
        case HeatingCircuit::Mode::NIGHT: // change the night temp
            offset = EMS_OFFSET_RC35Set_temp_night;
            break;
        case HeatingCircuit::Mode::DAY: // change the day temp
            offset = EMS_OFFSET_RC35Set_temp_day;
            break;
        case HeatingCircuit::Mode::HOLIDAY: // change the holiday temp
            offset = EMS_OFFSET_RC35Set_temp_holiday;
            break;
        case HeatingCircuit::Mode::OFFSET: // change the offset temp
            offset = EMS_OFFSET_RC35Set_temp_offset;
            break;
        case HeatingCircuit::Mode::FLOWOFFSET: // change the offset of flowtemp
            offset = EMS_OFFSET_RC35Set_temp_flowoffset;
            factor = 1;
            break;
        case HeatingCircuit::Mode::DESIGN:
            if (hc->heatingtype == 3) {
                offset = EMS_OFFSET_RC35Set_temp_design_floor;
            } else {
                offset = EMS_OFFSET_RC35Set_temp_design;
            }
            factor = 1;
            break;
        case HeatingCircuit::Mode::SUMMER:
            offset = EMS_OFFSET_RC35Set_temp_summer;
            factor = 1;
            break;
        case HeatingCircuit::Mode::NOFROST:
            offset = EMS_OFFSET_RC35Set_temp_nofrost;
            factor = 1;
            break;
        case HeatingCircuit::Mode::ROOMINFLUENCE:
            offset = 4;
            factor = 1;
            break;
        case HeatingCircuit::Mode::MINFLOW:
            offset = 16;
            factor = 1;
            break;
        case HeatingCircuit::Mode::MAXFLOW:
            if (hc->heatingtype == 3) {
                offset = 35;
            } else {
                offset = 15;
            }
            factor = 1;
            break;
        default:
        case HeatingCircuit::Mode::AUTO:                         // automatic selection, if no type is defined, we use the standard code
            validate_typeid = monitor_typeids[hc->hc_num() - 1]; //get setpoint roomtemp back
            if (model == EMS_DEVICE_FLAG_RC35) {
                uint8_t mode_ = hc->get_mode(flags());
                if (mode_ == HeatingCircuit::Mode::NIGHT) {
                    offset = EMS_OFFSET_RC35Set_temp_night;
                } else if (mode_ == HeatingCircuit::Mode::DAY) {
                    offset = EMS_OFFSET_RC35Set_temp_day;
                } else {
                    offset = EMS_OFFSET_RC35Set_seltemp; // https://github.com/emsesp/EMS-ESP/issues/310
                }
            } else {
                uint8_t mode_type = hc->get_mode_type(flags());
                offset            = (mode_type == HeatingCircuit::Mode::NIGHT) ? EMS_OFFSET_RC35Set_temp_night : EMS_OFFSET_RC35Set_temp_day;
            }
            break;
        }

    } else if (model == EMS_DEVICE_FLAG_JUNKERS) {
        // figure out if we have older or new thermostats, Heating Circuits on 0x65 or 0x79
        // see https://github.com/emsesp/EMS-ESP/issues/335#issuecomment-593324716)
        bool old_junkers = (has_flags(EMS_DEVICE_FLAG_JUNKERS_OLD));
        if (!old_junkers) {
            switch (mode) {
            case HeatingCircuit::Mode::NOFROST:
                offset = EMS_OFFSET_JunkersSetMessage_no_frost_temp;
                break;
            case HeatingCircuit::Mode::NIGHT:
            case HeatingCircuit::Mode::ECO:
                offset = EMS_OFFSET_JunkersSetMessage_night_temp;
                break;
            case HeatingCircuit::Mode::HEAT:
            case HeatingCircuit::Mode::DAY:
                offset = EMS_OFFSET_JunkersSetMessage_day_temp;
                break;
            default:
            case HeatingCircuit::Mode::AUTO: // automatic selection, if no type is defined, we use the standard code
                uint8_t mode_type = hc->get_mode_type(flags());
                if (mode_type == HeatingCircuit::Mode::NIGHT || mode_type == HeatingCircuit::Mode::ECO) {
                    offset = EMS_OFFSET_JunkersSetMessage_night_temp;
                } else if (mode_type == HeatingCircuit::Mode::DAY || mode_type == HeatingCircuit::Mode::HEAT) {
                    offset = EMS_OFFSET_JunkersSetMessage_day_temp;
                } else {
                    offset = EMS_OFFSET_JunkersSetMessage_no_frost_temp;
                }
                break;
            }

        } else {
            // older, like the FR100
            switch (mode) {
            case HeatingCircuit::Mode::NOFROST:
                offset = EMS_OFFSET_JunkersSetMessage2_no_frost_temp;
                break;
            case HeatingCircuit::Mode::ECO:
            case HeatingCircuit::Mode::NIGHT:
                offset = EMS_OFFSET_JunkersSetMessage2_eco_temp;
                break;
            case HeatingCircuit::Mode::HEAT:
            case HeatingCircuit::Mode::DAY:
                offset = EMS_OFFSET_JunkersSetMessage2_heat_temp;
                break;
            default:
            case HeatingCircuit::Mode::AUTO: // automatic selection, if no type is defined, we use the standard code
                uint8_t mode_type = hc->get_mode_type(flags());
                if (mode_type == HeatingCircuit::Mode::NIGHT || mode_type == HeatingCircuit::Mode::ECO) {
                    offset = EMS_OFFSET_JunkersSetMessage2_eco_temp;
                } else if (mode_type == HeatingCircuit::Mode::DAY || mode_type == HeatingCircuit::Mode::HEAT) {
                    offset = EMS_OFFSET_JunkersSetMessage2_heat_temp;
                } else {
                    offset = EMS_OFFSET_JunkersSetMessage2_no_frost_temp;
                }
                break;
            }
        }
    }

    // if we know what to send and to where, go and do it
    if (offset != -1) {
        char s[10];
        LOG_INFO(F("Setting thermostat temperature to %s for heating circuit %d, mode %s"),
                 Helpers::render_value(s, temperature, 2),
                 hc->hc_num(),
                 mode_tostring(mode).c_str());

        // add the write command to the Tx queue. value is *2
        // post validate is the corresponding monitor or set type IDs as they can differ per model
        write_command(set_typeid, offset, (uint8_t)((float)temperature * (float)factor), validate_typeid);
        return true;
    }

    LOG_WARNING(F("Set temperature: Invalid value"));
    return false;
}

bool Thermostat::set_temperature_value(const char * value, const int8_t id, const uint8_t mode) {
    float   f      = 0;
    uint8_t hc_num = (id == -1) ? AUTO_HEATING_CIRCUIT : id;
    if (Helpers::value2float(value, f)) {
        return set_temperature(f, mode, hc_num);
    } else {
        LOG_WARNING(F("Set temperature: Invalid value"));
        return false;
    }
}

bool Thermostat::set_temp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::AUTO);
}

bool Thermostat::set_nighttemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::NIGHT);
}

bool Thermostat::set_daytemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::DAY);
}

bool Thermostat::set_comforttemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::COMFORT);
}

bool Thermostat::set_nofrosttemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::NOFROST);
}

bool Thermostat::set_ecotemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::ECO);
}

bool Thermostat::set_heattemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::HEAT);
}

bool Thermostat::set_summertemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::SUMMER);
}

bool Thermostat::set_designtemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::DESIGN);
}

bool Thermostat::set_offsettemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::OFFSET);
}

bool Thermostat::set_holidaytemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::HOLIDAY);
}

bool Thermostat::set_manualtemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::MANUAL);
}

bool Thermostat::set_flowtempoffset(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::FLOWOFFSET);
}

bool Thermostat::set_maxflowtemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::MAXFLOW);
}

bool Thermostat::set_minflowtemp(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::MINFLOW);
}

bool Thermostat::set_roominfluence(const char * value, const int8_t id) {
    return set_temperature_value(value, id, HeatingCircuit::Mode::ROOMINFLUENCE);
}

// API commands for MQTT and Console
void Thermostat::add_commands() {
    // if this thermostat doesn't support write, don't add the commands
    if (has_flags(EMS_DEVICE_FLAG_NO_WRITE)) {
        return;
    }

    // common to all thermostats
    register_mqtt_cmd(F("temp"), [&](const char * value, const int8_t id) { return set_temp(value, id); });
    register_mqtt_cmd(F("mode"), [&](const char * value, const int8_t id) { return set_mode(value, id); });
    register_mqtt_cmd(F("datetime"), [&](const char * value, const int8_t id) { return set_datetime(value, id); });
    register_mqtt_cmd(F("roomtemp"), [&](const char * value, const int8_t id) { return set_roomtemp(value, id); });

    switch (model()) {
    case EMS_DEVICE_FLAG_RC100:
    case EMS_DEVICE_FLAG_RC300:
        register_mqtt_cmd(F("manualtemp"), [&](const char * value, const int8_t id) { return set_manualtemp(value, id); });
        register_mqtt_cmd(F("ecotemp"), [&](const char * value, const int8_t id) { return set_ecotemp(value, id); });
        register_mqtt_cmd(F("comforttemp"), [&](const char * value, const int8_t id) { return set_comforttemp(value, id); });
        register_mqtt_cmd(F("summermode"), [&](const char * value, const int8_t id) { return set_summermode(value, id); });
        register_mqtt_cmd(F("summertemp"), [&](const char * value, const int8_t id) { return set_summertemp(value, id); });
        register_mqtt_cmd(F("wwmode"), [&](const char * value, const int8_t id) { return set_wwmode(value, id); });
        register_mqtt_cmd(F("wwsettemp"), [&](const char * value, const int8_t id) { return set_wwtemp(value, id); });
        register_mqtt_cmd(F("wwsettemplow"), [&](const char * value, const int8_t id) { return set_wwtemplow(value, id); });
        register_mqtt_cmd(F("wwonetime"), [&](const char * value, const int8_t id) { return set_wwonetime(value, id); });
        register_mqtt_cmd(F("wwcircmode"), [&](const char * value, const int8_t id) { return set_wwcircmode(value, id); });
        register_mqtt_cmd(F("building"), [&](const char * value, const int8_t id) { return set_building(value, id); });
        register_mqtt_cmd(F("nofrosttemp"), [&](const char * value, const int8_t id) { return set_nofrosttemp(value, id); });
        register_mqtt_cmd(F("designtemp"), [&](const char * value, const int8_t id) { return set_designtemp(value, id); });
        register_mqtt_cmd(F("offsettemp"), [&](const char * value, const int8_t id) { return set_offsettemp(value, id); });
        register_mqtt_cmd(F("minflowtemp"), [&](const char * value, const int8_t id) { return set_minflowtemp(value, id); });
        register_mqtt_cmd(F("maxflowtemp"), [&](const char * value, const int8_t id) { return set_maxflowtemp(value, id); });
        register_mqtt_cmd(F("minexttemp"), [&](const char * value, const int8_t id) { return set_minexttemp(value, id); });
        register_mqtt_cmd(F("roominfluence"), [&](const char * value, const int8_t id) { return set_roominfluence(value, id); });
        register_mqtt_cmd(F("program"), [&](const char * value, const int8_t id) { return set_program(value, id); });
        register_mqtt_cmd(F("controlmode"), [&](const char * value, const int8_t id) { return set_controlmode(value, id); });
        break;
    case EMS_DEVICE_FLAG_RC20_2:
        register_mqtt_cmd(F("nighttemp"), [&](const char * value, const int8_t id) { return set_nighttemp(value, id); });
        register_mqtt_cmd(F("daytemp"), [&](const char * value, const int8_t id) { return set_daytemp(value, id); });
        register_mqtt_cmd(F("program"), [&](const char * value, const int8_t id) { return set_program(value, id); });
        break;
    case EMS_DEVICE_FLAG_RC30_1: // only RC30_1
        register_mqtt_cmd(F("clockoffset"), [&](const char * value, const int8_t id) { return set_clockoffset(value, id); });
        register_mqtt_cmd(F("language"), [&](const char * value, const int8_t id) { return set_language(value, id); });
        register_mqtt_cmd(F("display"), [&](const char * value, const int8_t id) { return set_display(value, id); });
    case EMS_DEVICE_FLAG_RC35: // RC30 and RC35
        register_mqtt_cmd(F("nighttemp"), [&](const char * value, const int8_t id) { return set_nighttemp(value, id); });
        register_mqtt_cmd(F("daytemp"), [&](const char * value, const int8_t id) { return set_daytemp(value, id); });
        register_mqtt_cmd(F("nofrosttemp"), [&](const char * value, const int8_t id) { return set_nofrosttemp(value, id); });
        register_mqtt_cmd(F("remotetemp"), [&](const char * value, const int8_t id) { return set_remotetemp(value, id); });
        register_mqtt_cmd(F("minexttemp"), [&](const char * value, const int8_t id) { return set_minexttemp(value, id); });
        register_mqtt_cmd(F("calinttemp"), [&](const char * value, const int8_t id) { return set_calinttemp(value, id); });
        register_mqtt_cmd(F("building"), [&](const char * value, const int8_t id) { return set_building(value, id); });
        register_mqtt_cmd(F("control"), [&](const char * value, const int8_t id) { return set_control(value, id); });
        register_mqtt_cmd(F("pause"), [&](const char * value, const int8_t id) { return set_pause(value, id); });
        register_mqtt_cmd(F("party"), [&](const char * value, const int8_t id) { return set_party(value, id); });
        register_mqtt_cmd(F("holiday"), [&](const char * value, const int8_t id) { return set_holiday(value, id); });
        register_mqtt_cmd(F("summertemp"), [&](const char * value, const int8_t id) { return set_summertemp(value, id); });
        register_mqtt_cmd(F("designtemp"), [&](const char * value, const int8_t id) { return set_designtemp(value, id); });
        register_mqtt_cmd(F("offsettemp"), [&](const char * value, const int8_t id) { return set_offsettemp(value, id); });
        register_mqtt_cmd(F("holidaytemp"), [&](const char * value, const int8_t id) { return set_holidaytemp(value, id); });
        register_mqtt_cmd(F("wwmode"), [&](const char * value, const int8_t id) { return set_wwmode(value, id); });
        register_mqtt_cmd(F("wwcircmode"), [&](const char * value, const int8_t id) { return set_wwcircmode(value, id); });
        register_mqtt_cmd(F("roominfluence"), [&](const char * value, const int8_t id) { return set_roominfluence(value, id); });
        register_mqtt_cmd(F("flowtempoffset"), [&](const char * value, const int8_t id) { return set_flowtempoffset(value, id); });
        register_mqtt_cmd(F("minflowtemp"), [&](const char * value, const int8_t id) { return set_minflowtemp(value, id); });
        register_mqtt_cmd(F("maxflowtemp"), [&](const char * value, const int8_t id) { return set_maxflowtemp(value, id); });
        register_mqtt_cmd(F("reducemode"), [&](const char * value, const int8_t id) { return set_reducemode(value, id); });
        register_mqtt_cmd(F("program"), [&](const char * value, const int8_t id) { return set_program(value, id); });
        register_mqtt_cmd(F("controlmode"), [&](const char * value, const int8_t id) { return set_controlmode(value, id); });
        break;
    case EMS_DEVICE_FLAG_JUNKERS:
        register_mqtt_cmd(F("nofrosttemp"), [&](const char * value, const int8_t id) { return set_nofrosttemp(value, id); });
        register_mqtt_cmd(F("ecotemp"), [&](const char * value, const int8_t id) { return set_ecotemp(value, id); });
        register_mqtt_cmd(F("heattemp"), [&](const char * value, const int8_t id) { return set_heattemp(value, id); });
        break;
    default:
        break;
    }
}

} // namespace emsesp
