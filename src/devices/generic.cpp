/*
 * EMS-ESP - https://github.com/proddy/EMS-ESP
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

#include "generic.h"

namespace emsesp {

REGISTER_FACTORY(Generic, EMSdevice::DeviceType::GENERIC);

uuid::log::Logger Generic::logger_{F_(generic), uuid::log::Facility::CONSOLE};

Generic::Generic(uint8_t device_type, uint8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand)
    : EMSdevice(device_type, device_id, product_id, version, name, flags, brand) {
}

void Generic::device_info_web(JsonArray & root) {
}

// publish values via MQTT
void Generic::publish_values(JsonObject & json, bool force) {
}

// export values to JSON
bool Generic::export_values(JsonObject & json) {
    return true;
}

// check to see if values have been updated
bool Generic::updated_values() {
    return false;
}

} // namespace emsesp