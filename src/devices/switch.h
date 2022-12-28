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

#ifndef EMSESP_SWITCH_H
#define EMSESP_SWITCH_H

#include <Arduino.h>
#include <ArduinoJson.h>

#include <uuid/log.h>

#include "emsdevice.h"
#include "emsesp.h"
#include "telegram.h"
#include "helpers.h"
#include "mqtt.h"

namespace emsesp {

class Switch : public EMSdevice {
  public:
    Switch(uint8_t device_type, uint8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand);

    virtual void publish_values(JsonObject & json, bool force);
    virtual bool export_values(JsonObject & json, int8_t id = -1);
    virtual void device_info_web(JsonArray & root, uint8_t & part);
    virtual bool updated_values();

  private:
    static uuid::log::Logger logger_;

    void process_WM10SetMessage(std::shared_ptr<const Telegram> telegram);
    void process_WM10MonitorMessage(std::shared_ptr<const Telegram> telegram);
    void process_WM10TempMessage(std::shared_ptr<const Telegram> telegram);
    void register_mqtt_ha_config();

    uint16_t flowTempHc_     __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET;
    uint8_t  status_         = EMS_VALUE_UINT_NOTSET;
    uint8_t  activated_      = EMS_VALUE_BOOL_NOTSET;
    bool     changed_        = false;
    bool     mqtt_ha_config_ = false; // for HA MQTT Discovery
};

} // namespace emsesp

#endif
