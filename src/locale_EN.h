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

// common words
MAKE_PSTR_WORD(exit)
MAKE_PSTR_WORD(help)
MAKE_PSTR_WORD(log)
MAKE_PSTR_WORD(logout)
MAKE_PSTR_WORD(enabled)
MAKE_PSTR_WORD(disabled)
MAKE_PSTR_WORD(set)
MAKE_PSTR_WORD(show)
MAKE_PSTR_WORD(on)
MAKE_PSTR_WORD(off)
MAKE_PSTR_WORD(su)
MAKE_PSTR_WORD(name)
MAKE_PSTR_WORD(auto)
MAKE_PSTR_WORD(scan)
MAKE_PSTR_WORD(password)
MAKE_PSTR_WORD(read)
MAKE_PSTR_WORD(version)
MAKE_PSTR_WORD(values)
MAKE_PSTR_WORD(system)
MAKE_PSTR_WORD(fetch)
MAKE_PSTR_WORD(restart)
MAKE_PSTR_WORD(format)
MAKE_PSTR_WORD(raw)
MAKE_PSTR_WORD(watch)
MAKE_PSTR_WORD(send)
MAKE_PSTR_WORD(telegram)
MAKE_PSTR_WORD(bus_id)
MAKE_PSTR_WORD(tx_mode)
MAKE_PSTR_WORD(ems)
MAKE_PSTR_WORD(devices)
MAKE_PSTR_WORD(shower)
MAKE_PSTR_WORD(mqtt)
MAKE_PSTR_WORD(emsesp)
MAKE_PSTR_WORD(connected)
MAKE_PSTR_WORD(disconnected)
MAKE_PSTR_WORD(passwd)
MAKE_PSTR_WORD(hostname)
MAKE_PSTR_WORD(wifi)
MAKE_PSTR_WORD(reconnect)
MAKE_PSTR_WORD(ssid)
MAKE_PSTR_WORD(heartbeat)
MAKE_PSTR_WORD(users)
MAKE_PSTR_WORD(master)
MAKE_PSTR_WORD(pin)
MAKE_PSTR_WORD(publish)
MAKE_PSTR_WORD(bar)
MAKE_PSTR_WORD(min)
MAKE_PSTR_WORD(hours)
MAKE_PSTR_WORD(uA)
MAKE_PSTR_WORD(timeout)

// for commands
MAKE_PSTR_WORD(call)
MAKE_PSTR_WORD(cmd)
MAKE_PSTR_WORD(id)
MAKE_PSTR_WORD(device)
MAKE_PSTR_WORD(data)
MAKE_PSTR_WORD(command)
MAKE_PSTR_WORD(commands)
MAKE_PSTR_WORD(info)
MAKE_PSTR_WORD(test)
MAKE_PSTR_WORD(settings)

// devices
MAKE_PSTR_WORD(boiler)
MAKE_PSTR_WORD(thermostat)
MAKE_PSTR_WORD(switch)
MAKE_PSTR_WORD(solar)
MAKE_PSTR_WORD(mixer)
MAKE_PSTR_WORD(gateway)
MAKE_PSTR_WORD(controller)
MAKE_PSTR_WORD(connect)
MAKE_PSTR_WORD(heatpump)
MAKE_PSTR_WORD(generic)
MAKE_PSTR_WORD(dallassensor)
MAKE_PSTR_WORD(unknown)

MAKE_PSTR(1space, " ")
MAKE_PSTR(2spaces, "  ")
MAKE_PSTR(kwh, "kWh")
MAKE_PSTR(wh, "Wh")
MAKE_PSTR(rssi, "rssi")
MAKE_PSTR(EMSESP, "EMS-ESP")
MAKE_PSTR(master_thermostat_fmt, "Master Thermostat Device ID = %s")
MAKE_PSTR(host_fmt, "Host = %s")
MAKE_PSTR(hostname_fmt, "WiFi Hostname = %s")
MAKE_PSTR(mark_interval_fmt, "Mark interval = %lus")
MAKE_PSTR(wifi_ssid_fmt, "WiFi SSID = %s")
MAKE_PSTR(wifi_password_fmt, "WiFi Password = %S")
MAKE_PSTR(mqtt_heartbeat_fmt, "MQTT Heartbeat is %s")
MAKE_PSTR(cmd_optional, "[cmd]")
MAKE_PSTR(ha_optional, "[ha]")
MAKE_PSTR(deep_optional, "[deep]")
MAKE_PSTR(tx_mode_fmt, "Tx mode = %d")
MAKE_PSTR(bus_id_fmt, "Bus ID = %02X")
MAKE_PSTR(watchid_optional, "[ID]")
MAKE_PSTR(watch_format_optional, "[off | on | raw | unknown]")
MAKE_PSTR(invalid_watch, "Invalid watch type")
MAKE_PSTR(data_mandatory, "\"XX XX ...\"")
MAKE_PSTR(percent, "%")
MAKE_PSTR(degrees, "°C")
MAKE_PSTR(meter3, "m³")
MAKE_PSTR(asterisks, "********")
MAKE_PSTR(n_mandatory, "<n>")
MAKE_PSTR(id_optional, "[id|hc]")
MAKE_PSTR(data_optional, "[data]")
MAKE_PSTR(offset_optional, "[offset]")
MAKE_PSTR(typeid_mandatory, "<type ID>")
MAKE_PSTR(deviceid_mandatory, "<device ID>")
MAKE_PSTR(device_type_optional, "[device]")
MAKE_PSTR(invalid_log_level, "Invalid log level")
MAKE_PSTR(log_level_fmt, "Log level = %s")
MAKE_PSTR(log_level_optional, "[level]")
MAKE_PSTR(name_mandatory, "<name>")
MAKE_PSTR(name_optional, "[name]")
MAKE_PSTR(new_password_prompt1, "Enter new password: ")
MAKE_PSTR(new_password_prompt2, "Retype new password: ")
MAKE_PSTR(password_prompt, "Password: ")
MAKE_PSTR(unset, "<unset>")
MAKE_PSTR(usr_brand_fmt, "Boiler brand = %d (%s)")
MAKE_PSTR(usr_type_fmt, "Boiler type = %s")
MAKE_PSTR(min_boiler_wh_fmt, "Boiler minimum power = %dWh")
MAKE_PSTR(max_boiler_wh_fmt, "Boiler maximum power = %dWh")
MAKE_PSTR(gas_meter_reading_fmt, "Gas meter = %dm³")
MAKE_PSTR(conv_factor_fmt, "Conversion factor m³<-> Wh = %d")

// status
MAKE_PSTR(wifirssi, "WiFi RSSI")
MAKE_PSTR(uptime, "Uptime")
MAKE_PSTR(uptimesec, "Uptime (sec)")
MAKE_PSTR(mqttfails, "MQTT fails")
MAKE_PSTR(txwrites, "Tx writes")
MAKE_PSTR(txreads, "Tx reads")
MAKE_PSTR(txfails, "Tx fails")
MAKE_PSTR(rxrec, "Rx received")
MAKE_PSTR(rxfails, "Rx fails")
MAKE_PSTR(dallasfails, "Dallas fails")
MAKE_PSTR(freemem, "Free memory")
MAKE_PSTR(heapfrag, "Heap fragmentation")
MAKE_PSTR(maxfreeblock, "Max. free block size")
MAKE_PSTR(freestack, "Free cont. stack")
MAKE_PSTR(analog, "Analog input")


// boiler
MAKE_PSTR(heatingActive, "Heating active")
MAKE_PSTR(tapwaterActive, "Warm water/DHW active")
MAKE_PSTR(serviceCode, "Service code")
MAKE_PSTR(serviceCodeNumber, "Service code number")
MAKE_PSTR(lastCode, "Last error")
MAKE_PSTR(wWSelTemp, "Warm water selected temperature")
MAKE_PSTR(wWSetTemp, "Warm water set temperature")
MAKE_PSTR(wWDisinfectionTemp, "Warm water disinfection temperature")
MAKE_PSTR(selFlowTemp, "Selected flow temperature")
MAKE_PSTR(selBurnPow, "Burner selected max power")
MAKE_PSTR(curBurnPow, "Burner current power")
MAKE_PSTR(heatingPumpMod, "Heating pump modulation")
MAKE_PSTR(heatingPump2Mod, "Heating pump 2 modulation")
MAKE_PSTR(wWType, "Warm water type")
MAKE_PSTR(wWChargeType, "Warm water charging type")
MAKE_PSTR(wWCircPump, "Warm water circulation pump available")
MAKE_PSTR(wWCircPumpMode, "Warm water circulation pump freq")
MAKE_PSTR(wWCirc, "Warm water circulation active")
MAKE_PSTR(outdoorTemp, "Outside temperature")
MAKE_PSTR(wWCurTemp, "Warm water current temperature (intern)")
MAKE_PSTR(wWCurTemp2, "Warm water current temperature (extern)")
MAKE_PSTR(wWCurFlow, "Warm water current tap water flow")
MAKE_PSTR(curFlowTemp, "Current flow temperature")
MAKE_PSTR(retTemp, "Return temperature")
MAKE_PSTR(switchTemp, "Mixer switch temperature")
MAKE_PSTR(sysPress, "System pressure")
MAKE_PSTR(boilTemp, "Max temperature")
MAKE_PSTR(wWStorageTemp1, "Warm water storage temperature (intern)")
MAKE_PSTR(wWStorageTemp2, "Warm water storage temperature (extern)")
MAKE_PSTR(exhaustTemp, "Exhaust temperature")
MAKE_PSTR(wWActivated, "Warm water activated")
MAKE_PSTR(wWOneTime, "Warm water one time charging")
MAKE_PSTR(wWDisinfecting, "Warm water disinfecting")
MAKE_PSTR(wWCharging, "Warm water charging")
MAKE_PSTR(wWRecharging, "Warm water recharging")
MAKE_PSTR(wWTempOK, "Warm water temperature ok")
MAKE_PSTR(wWActive, "Warm water active")
MAKE_PSTR(burnGas, "Gas")
MAKE_PSTR(flameCurr, "Flame current")
MAKE_PSTR(heatingPump, "Heating Pump")
MAKE_PSTR(fanWork, "Fan")
MAKE_PSTR(ignWork, "Ignition")
MAKE_PSTR(ww3wayValve, "3-way valve active")
MAKE_PSTR(heatingActivated, "Heating activated")
MAKE_PSTR(heatingTemp, "Heating temperature setting")
MAKE_PSTR(pumpModMax, "Circuit pump modulation max power")
MAKE_PSTR(pumpModMin, "Circuit pump modulation min power")
MAKE_PSTR(pumpDelay, "Circuit pump delay time")
MAKE_PSTR(burnMinPeriod, "Burner min period")
MAKE_PSTR(burnMinPower, "Burner min power")
MAKE_PSTR(burnMaxPower, "Burner max power")
MAKE_PSTR(boilHystOn, "Temperature hysteresis on")
MAKE_PSTR(boilHystOff, "Temperature hysteresis off")
MAKE_PSTR(setFlowTemp, "Set flow temperature")
MAKE_PSTR(wWSetPumpPower, "Warm water pump set power")
MAKE_PSTR(mixerTemp, "Mixer temperature")
MAKE_PSTR(tankMiddleTemp, "Tank middle temperature (TS3)")
MAKE_PSTR(wWStarts, "Warm water starts")
MAKE_PSTR(wWWorkM, "Warm water active time")
MAKE_PSTR(setBurnPow, "Burner set power")
MAKE_PSTR(burnStarts, "Burner starts")
MAKE_PSTR(burnWorkMin, "Burner active time")
MAKE_PSTR(heatWorkMin, "Heating active time")
MAKE_PSTR(UBAuptime, "Boiler total uptime")
MAKE_PSTR(wWMaxPower, "Warm water max power")
MAKE_PSTR(gasReading, "Gas meter reading")

MAKE_PSTR(upTimeControl, "Operating time control")
MAKE_PSTR(upTimeCompHeating, "Operating time compressor heating")
MAKE_PSTR(upTimeCompCooling, "Operating time compressor cooling")
MAKE_PSTR(upTimeCompWw, "Operating time compressor warm water")
MAKE_PSTR(heatingStarts, "Heating starts (control)")
MAKE_PSTR(coolingStarts, "Cooling starts (control)")
MAKE_PSTR(wWStarts2, "Warm water starts (control)")
MAKE_PSTR(nrgConsTotal, "Energy consumption total")
MAKE_PSTR(auxElecHeatNrgConsTotal, "Auxiliary electrical heater energy consumption total")
MAKE_PSTR(auxElecHeatNrgConsHeating, "Auxiliary electrical heater energy consumption heating")
MAKE_PSTR(auxElecHeatNrgConsDHW, "Auxiliary electrical heater energy consumption DHW")
MAKE_PSTR(nrgConsCompTotal, "Energy consumption compressor total")
MAKE_PSTR(nrgConsCompHeating, "Energy consumption compressor heating")
MAKE_PSTR(nrgConsCompWw, "Energy consumption compressor warm water")
MAKE_PSTR(nrgConsCompCooling, "Energy consumption compressor total")
MAKE_PSTR(nrgSuppTotal, "Energy supplied total")
MAKE_PSTR(nrgSuppHeating, "Energy supplied heating")
MAKE_PSTR(nrgSuppWw, "Energy supplied warm water")
MAKE_PSTR(nrgSuppCooling, "Energy supplied cooling")
MAKE_PSTR(maintenanceMessage, "Maintenance message")
MAKE_PSTR(maintenance, "Scheduled maintenance")
MAKE_PSTR(maintenanceTime, "Next maintenance in")
MAKE_PSTR(maintenanceDate, "Next maintenance on")

// solar
MAKE_PSTR(collectorTemp, "Collector temperature (TS1)")
MAKE_PSTR(tankBottomTemp, "Tank bottom temperature (TS2)")
// MAKE_PSTR(tankMiddleTemp, "Tank middle temperature (TS3)")
MAKE_PSTR(tank2BottomTemp, "Second Tank bottom temperature (TS5)")
MAKE_PSTR(heatExchangerTemp, "Heat exchanger temperature (TS6)")
MAKE_PSTR(solarPumpModulation, "Pump modulation (PS1)")
MAKE_PSTR(cylinderPumpModulation, "Cylinder pump modulation (PS5)")
MAKE_PSTR(collectorMaxTemp, "Maximum collector temperature")
MAKE_PSTR(tankBottomMaxTemp, "Maximum Bottom Tank temperature")
MAKE_PSTR(collectorMinTemp, "Minimum collector temperature")
MAKE_PSTR(pumpWorkTime, "Pump working time (min)")
MAKE_PSTR(pumpWorkTimeText, "Pump working time")
MAKE_PSTR(energyLastHour, "Energy last hour")
MAKE_PSTR(energyToday, "Energy today")
MAKE_PSTR(energyTotal, "Energy total")
MAKE_PSTR(solarPump, "Pump (PS1)")
MAKE_PSTR(valveStatus, "Valve status")
MAKE_PSTR(tankHeated, "Tank heated")
MAKE_PSTR(collectorShutdown, "Collector shutdown")

// mixer
MAKE_PSTR(ww_hc, "  Warm water circuit %d:")
MAKE_PSTR(wWTemp, "Current warm water temperature")
MAKE_PSTR(tempStatus, "Current temperature status")
MAKE_PSTR(hc, "  Heating circuit %d:")
MAKE_PSTR(tempSwitch, "Temperature switch in assigned hc (MC1)")
MAKE_PSTR(pumpStatus, "Pump status in assigned hc (PC1)")
// MAKE_PSTR(flowTempLowLoss, "Flow temperature low loss header (T0)")
MAKE_PSTR(flowSetTemp, "Setpoint flow temperature in assigned hc")
MAKE_PSTR(flowTempHc, "Flow temperature in assigned hc (TC1)")
MAKE_PSTR(mixValve, "Mixing valve actuator in assigned hc (VC1)")


// thermostat
MAKE_PSTR(time, "Time")
MAKE_PSTR(error, "Error code")
MAKE_PSTR(display, "Display")
MAKE_PSTR(language, "Language")
MAKE_PSTR(offsetclock, "Offset clock")
MAKE_PSTR(dampedoutdoortemp, "Damped outdoor temperature")
MAKE_PSTR(inttemp1, "Temperature sensor 1")
MAKE_PSTR(inttemp2, "Temperature sensor 2")
MAKE_PSTR(intoffset, "Offset int. temperature")
MAKE_PSTR(minexttemp, "Min ext. temperature")
MAKE_PSTR(building, "Building")
MAKE_PSTR(floordry, "Floordrying")
MAKE_PSTR(floordrytemp, "Floordrying temperature")

MAKE_PSTR(wwmode, "Warm water mode")
MAKE_PSTR(wwsettemp, "Warm water set temperature")
MAKE_PSTR(wwsettemplow, "Warm water set temperature low")
MAKE_PSTR(wwextra1, "Warm water circuit 1 extra")
MAKE_PSTR(wwextra2, "Warm water circuit 2 extra")
MAKE_PSTR(wwcircmode, "Warm water circulation mode")

// thermostat - per heating circuit
MAKE_PSTR(seltemp, "Setpoint room temperature")
MAKE_PSTR(currtemp, "Current room temperature")
MAKE_PSTR(heattemp, "Heat temperature")
MAKE_PSTR(comforttemp, "Comfort temperature")
MAKE_PSTR(daytemp, "Day temperature")
MAKE_PSTR(ecotemp, "Eco temperature")
MAKE_PSTR(nighttemp, "Night temperature")
MAKE_PSTR(manualtemp, "Manual temperature")
MAKE_PSTR(holidaytemp, "Holiday temperature")
MAKE_PSTR(nofrosttemp, "Nofrost temperature")
MAKE_PSTR(heatingtype, "Heating type")
MAKE_PSTR(targetflowtemp, "Target flow temperature")
MAKE_PSTR(offsettemp, "Offset temperature")
MAKE_PSTR(designtemp, "Design temperature")
MAKE_PSTR(summertemp, "Summer temperature")
MAKE_PSTR(summermode, "Summer mode")
MAKE_PSTR(roominfluence, "Room influence")
MAKE_PSTR(flowtempoffset, "Flow temperature offset")
MAKE_PSTR(minflowtemp, "Min. flow temperature")
MAKE_PSTR(maxflowtemp, "Max. flow temperature")
MAKE_PSTR(reducemode, "Reduce mode")
MAKE_PSTR(program, "Timer program")
MAKE_PSTR(controlmode, "Control mode")
MAKE_PSTR(mode, "Mode")
MAKE_PSTR(modetype, "Mode type")

// heat pump
MAKE_PSTR(airHumidity, "Relative air humidity")
MAKE_PSTR(dewTemperature, "Dew point temperature")

// other
MAKE_PSTR(activated, "Switch activated")
MAKE_PSTR(status, "Switch status")


// Home Assistant icons
MAKE_PSTR(iconwatertemp, "mdi:coolant-temperature")
MAKE_PSTR(iconpercent, "mdi:sine-wave")
MAKE_PSTR(iconfire, "mdi:fire")
MAKE_PSTR(iconfan, "mdi:fan")
MAKE_PSTR(iconflash, "mdi:flash")
MAKE_PSTR(iconwaterpump, "mdi:water-pump")
MAKE_PSTR(iconexport, "mdi:home-export-outline")
MAKE_PSTR(iconimport, "mdi:home-import-outline")
MAKE_PSTR(iconcruise, "mdi:car-cruise-control")
MAKE_PSTR(iconvalve, "mdi:valve")
MAKE_PSTR(iconpower, "mdi:power-cycle")
MAKE_PSTR(iconthermostat, "mdi:home-thermometer-outline")
MAKE_PSTR(iconpump, "mdi:pump")
MAKE_PSTR(iconwifi, "mdi:wifi")
MAKE_PSTR(iconcounter, "mdi:counter")
MAKE_PSTR(iconlockout, "mdi:clock-outline")
MAKE_PSTR(iconmemory, "mdi:memory")
MAKE_PSTR(icongasmeter, "mdi:meter-gas-outline")


// MQTT topic suffix
MAKE_PSTR(mqtt_suffix_ww, "_ww")
MAKE_PSTR(mqtt_suffix_info, "_info")
