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

#ifndef EMSESP_BOILER_H
#define EMSESP_BOILER_H

#include <Arduino.h>
#include <ArduinoJson.h>

#include <uuid/log.h>

#include <string>

#include "emsdevice.h"
#include "telegram.h"
#include "emsesp.h"
#include "helpers.h"
#include "mqtt.h"

namespace emsesp {

class Boiler : public EMSdevice {
  public:
    Boiler(uint8_t device_type, int8_t device_id, uint8_t product_id, const std::string & version, const std::string & name, uint8_t flags, uint8_t brand);

    virtual void publish_values(JsonObject & json, bool force);
    virtual bool export_values(JsonObject & json, int8_t id = -1);
    virtual void device_info_web(JsonArray & root, uint8_t & part);
    virtual bool updated_values();
    uint8_t  public_retTemp_        = EMS_VALUE_UINT_NOTSET;   // Burner current power %
    //uint8_t  public_curBurnPow_        = EMS_VALUE_UINT_NOTSET;   // Burner current power %

  private:
    static uuid::log::Logger logger_;

    // specific boiler characteristics, stripping the top 4 bits
    inline uint8_t model() const {
        return (flags() & 0x0F);
    }

    void register_mqtt_ha_config();
    void register_mqtt_ha_config_ww();
    void register_mqtt_ha_config_dbg();
    void check_active(const bool force = false);
    bool export_values_main(JsonObject & doc, const bool textformat = false);
    bool export_values_ww(JsonObject & doc, const bool textformat = false);
    bool export_values_info(JsonObject & doc, const bool textformat = false);

    bool changed_           = false;
    uint8_t mqtt_ha_config_    = 0; // HA MQTT Discovery
    uint8_t mqtt_ha_config_ww_ = 0; // HA MQTT Discovery
    uint8_t mqtt_ha_config_dbg_= 0; // HA MQTT Discovery
    
    static constexpr uint8_t  EMS_TYPE_UBAParameterWW     = 0x33;
    static constexpr uint8_t  EMS_TYPE_UBAFunctionTest    = 0x1D;
    static constexpr uint8_t  EMS_TYPE_UBAFlags           = 0x35;
    static constexpr uint8_t  EMS_TYPE_UBASetPoints       = 0x1A;
    static constexpr uint8_t  EMS_TYPE_UBAParameters      = 0x16;
    static constexpr uint8_t  EMS_TYPE_UBAParametersPlus  = 0xE6;
    static constexpr uint8_t  EMS_TYPE_UBAParameterWWPlus = 0xEA;
    static constexpr uint16_t EMS_TYPE_UBAInfomration     __attribute__ ((aligned (4))) = 0x495;
    static constexpr uint16_t EMS_TYPE_UBAEnergySupplied  __attribute__ ((aligned (4))) = 0x494;

    static constexpr uint8_t EMS_BOILER_SELFLOWTEMP_HEATING = 20; // was originally 70, changed to 30 for issue #193, then to 20 with issue #344
    
    // iRT specific?
    uint8_t periodTimer_        = EMS_VALUE_UINT_NOTSET; // 5 minute timer
    // UBAParameterWW
    uint8_t wWActivated_        = 1; // Warm Water activated
    uint8_t wWActivated_raw     = 1; // Warm Water activated
    uint8_t wWSelTemp_          = EMS_VALUE_UINT_NOTSET; // Warm Water selected temperature
    uint8_t wWCircPump_         = EMS_VALUE_BOOL_NOTSET; // Warm Water circulation pump available
    uint8_t wWCircPumpMode_     = EMS_VALUE_UINT_NOTSET; // Warm Water circulation pump mode
    uint8_t wWChargeType_       = EMS_VALUE_BOOL_NOTSET; // Warm Water charge type (pump or 3-way-valve)
    uint8_t wWDisinfectionTemp_ = EMS_VALUE_UINT_NOTSET; // Warm Water disinfection temperature to prevent infection
    uint8_t wWComfort_          = EMS_VALUE_UINT_NOTSET; // WW comfort mode

    // MC10Status
    uint16_t mixerTemp_      __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // mixer temperature
    uint16_t tankMiddleTemp_ __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // tank middle temperature (TS3)

    // UBAMonitorFast - 0x18 on EMS1
    uint16_t curFlowTemp_       __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // Current flow temperature
    uint16_t fineFlowTemp_      __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // Current flow temperature
    uint16_t wWStorageTemp1_    __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // warm water storage temp 1
    uint16_t wWStorageTemp2_    __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // warm water storage temp 2
    uint16_t retTemp_           __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // Return temperature
    uint16_t fineRetTemp_       __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // Return temperature
    uint16_t flameCurr_         __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // Flame current in micro amps
    uint16_t serviceCodeNumber_ __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // error/service code
    uint32_t lastCodeDate_      __attribute__ ((aligned (4))) = 0;
    char     serviceCode_[4]    = {'\0'};                  // 3 character status/service code
    char     lastCode_[30]      = {'\0'};
    uint8_t  selFlowTemp_       = EMS_VALUE_UINT_NOTSET;   // Selected flow temperature
    uint8_t  curFlowTemp_raw    = EMS_VALUE_UINT_NOTSET;   // Current flow temperature
    uint8_t  fineFlowTemp1_raw  = EMS_VALUE_UINT_NOTSET;   // Current flow temperature
    uint8_t  fineFlowTemp2_raw  = EMS_VALUE_UINT_NOTSET;   // Current flow temperature
    uint8_t  fineRetTemp1_raw   = EMS_VALUE_UINT_NOTSET;   // Current return temperature
    uint8_t  fineRetTemp2_raw   = EMS_VALUE_UINT_NOTSET;   // Current return temperature
    uint8_t  retTemp_raw        = EMS_VALUE_UINT_NOTSET;   // Return temperature
    uint8_t  burnGas_           = EMS_VALUE_BOOL_NOTSET;   // Gas on/off
    uint8_t  fanWork_           = EMS_VALUE_BOOL_NOTSET;   // Fan on/off
    uint8_t  ignWork_           = EMS_VALUE_BOOL_NOTSET;   // Ignition on/off
    uint8_t  heatingPump_       = EMS_VALUE_BOOL_NOTSET;   // Boiler heating pump on/off
    uint8_t  heatingPump_raw    = EMS_VALUE_BOOL_NOTSET;   // Boiler heating pump on/off
    uint8_t  ww3wayValve_       = EMS_VALUE_BOOL_NOTSET;   // 3-way valve on WW
    uint8_t  wWCirc_            = EMS_VALUE_BOOL_NOTSET;   // Circulation on/off
    uint8_t  selBurnPow_        = EMS_VALUE_UINT_NOTSET;   // Burner max power %
    uint8_t  curBurnPow_        = EMS_VALUE_UINT_NOTSET;   // Burner current power %
    uint8_t  curBurnPow_raw     = EMS_VALUE_UINT_NOTSET;   // Burner current power %
    uint8_t  curBurnPow_2_      = EMS_VALUE_UINT_NOTSET;   // Burner current power %
    uint8_t  curBurnPow_2_raw   = EMS_VALUE_UINT_NOTSET;   // Burner current power %
    uint8_t  sysPress_          = EMS_VALUE_UINT_NOTSET;   // System pressure
    uint8_t  serviceCodeNumber_raw = EMS_VALUE_UINT_NOTSET; // error/service code
    uint8_t  boilerState_       = EMS_VALUE_UINT_NOTSET;   // Boiler state flag
    uint8_t  command91_raw      = EMS_VALUE_UINT_NOTSET;   // Debug-value
    uint8_t  command95_raw      = EMS_VALUE_UINT_NOTSET;   // Debug-value
    uint8_t  command9B_raw      = EMS_VALUE_UINT_NOTSET;   // Debug-value
    uint8_t  command9C_raw      = EMS_VALUE_UINT_NOTSET;   // Debug-value
    uint8_t  command9E_raw      = EMS_VALUE_UINT_NOTSET;   // Debug-value
    uint8_t  commandA1_raw      = EMS_VALUE_UINT_NOTSET;   // Debug-value
    uint8_t  commandA2_raw      = EMS_VALUE_UINT_NOTSET;   // Debug-value

    // UBAMonitorSlow - 0x19 on EMS1
    int16_t  outdoorTemp_    __attribute__ ((aligned (4))) = EMS_VALUE_SHORT_NOTSET;  // Outside temperature
    uint16_t boilTemp_       __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // Boiler temperature
    uint16_t exhaustTemp_    __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // Exhaust temperature
    uint32_t burnStarts_     __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET;  // # burner restarts
    uint32_t burnWorkMin_    __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET;  // Total burner operating time
    uint32_t heatWorkMin_    __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET;  // Total heat operating time
    uint16_t switchTemp_     __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // Switch temperature
    uint8_t  boilTemp_raw    = EMS_VALUE_UINT_NOTSET;  // Boiler temperature
    uint8_t  burnStarts_raw  = EMS_VALUE_UINT_NOTSET;  // # burner restarts
    uint8_t  outdoorTemp_raw = EMS_VALUE_UINT_NOTSET;   // Outside temperature
    uint8_t  heatingPumpMod_ = EMS_VALUE_UINT_NOTSET;   // Heating pump modulation %
    uint8_t  burnWorkMin_raw = EMS_VALUE_UINT_NOTSET;   // Total burner operating time

    // UBAMonitorWW
    uint8_t  wWSetTemp_      = EMS_VALUE_UINT_NOTSET;   // Warm Water set temperature
    uint16_t wWCurTemp_      __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // Warm Water current temperature
    uint16_t wWCurTemp2_     __attribute__ ((aligned (4))) = EMS_VALUE_USHORT_NOTSET; // Warm Water current temperature storage
    uint32_t wWStarts_       __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET;  // Warm Water # starts
    uint32_t wWWorkM_        __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET;  // Warm Water # minutes
    uint8_t  wWCurTemp_raw   = EMS_VALUE_UINT_NOTSET;   // Warm Water current temperature
    uint8_t  wWCurTemp2_raw  = EMS_VALUE_UINT_NOTSET;   // Warm Water current temperature storage
    uint8_t  wWOneTime_      = EMS_VALUE_BOOL_NOTSET;   // Warm Water one time function on/off
    uint8_t  wWDisinfecting_ = EMS_VALUE_BOOL_NOTSET;   // Warm Water disinfection on/off
    uint8_t  wWCharging_     = EMS_VALUE_BOOL_NOTSET;   // Warm Water charging on/off
    uint8_t  wWRecharging_   = EMS_VALUE_BOOL_NOTSET;   // Warm Water recharge on/off
    uint8_t  wWTempOK_       = EMS_VALUE_BOOL_NOTSET;   // Warm Water temperature ok on/off
    uint8_t  wWCurFlow_      = EMS_VALUE_UINT_NOTSET;   // Warm Water current flow temp in l/min
    uint8_t  wWType_         = EMS_VALUE_UINT_NOTSET;   // 0-off, 1-flow, 2-flowbuffer, 3-buffer, 4-layered buffer
    uint8_t  wWActive_       = EMS_VALUE_BOOL_NOTSET;
    uint8_t  wWMaxPower_     = EMS_VALUE_UINT_NOTSET;   // Warm Water maximum power

    // UBATotalUptime
    uint32_t UBAuptime_ __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Total UBA working hours

    // UBAParameters
    uint8_t heatingActivated_ = 1; // Heating activated on the boiler
    uint8_t heatingTemp_      = EMS_VALUE_UINT_NOTSET; // Heating temperature setting on the boiler
    uint8_t heatingTemp_raw   = EMS_VALUE_UINT_NOTSET; // Heating temperature setting on the boiler
    uint8_t pumpModMax_       = EMS_VALUE_UINT_NOTSET; // Boiler circuit pump modulation max. power %
    uint8_t pumpModMin_       = EMS_VALUE_UINT_NOTSET; // Boiler circuit pump modulation min. power
    uint8_t burnMinPower_     = EMS_VALUE_UINT_NOTSET;
    uint8_t burnMaxPower_     = EMS_VALUE_UINT_NOTSET;
    uint8_t burnMaxPower_raw  = EMS_VALUE_UINT_NOTSET;
    int8_t  boilHystOff_      = EMS_VALUE_INT_NOTSET;
    int8_t  boilHystOn_       = EMS_VALUE_INT_NOTSET;
    uint8_t burnMinPeriod_    = EMS_VALUE_UINT_NOTSET;
    uint8_t pumpDelay_        = EMS_VALUE_UINT_NOTSET;

    // UBASetPoint
    uint8_t setFlowTemp_    = EMS_VALUE_UINT_NOTSET; // boiler setpoint temp
    uint8_t setFlowTemp_raw = EMS_VALUE_UINT_NOTSET; // boiler setpoint temp
    uint8_t setBurnPow_     = EMS_VALUE_UINT_NOTSET; // max output power in %
    uint8_t setBurnPow_raw  = EMS_VALUE_UINT_NOTSET; // max output power in %
    uint8_t wWSetPumpPower_ = EMS_VALUE_UINT_NOTSET; // ww pump speed/power?

    // other internal calculated params
    uint8_t tapwaterActive_  = EMS_VALUE_BOOL_NOTSET; // Hot tap water is on/off
    uint8_t heatingActive_   = EMS_VALUE_BOOL_NOTSET; // Central heating is on/off
    uint8_t heatingPump2Mod_ = EMS_VALUE_UINT_NOTSET; // heating pump 2 modulation from 0xE3 (heating pumps)
    uint16_t debugvalues[128] = {EMS_VALUE_USHORT_NOTSET};
    uint16_t debugtemp[128]   = {EMS_VALUE_USHORT_NOTSET};
    uint16_t debugpercent[128]= {EMS_VALUE_USHORT_NOTSET};

    // UBAInformation
    uint32_t upTimeControl_             __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Operating time control
    uint32_t upTimeCompHeating_         __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Operating time compressor heating
    uint32_t upTimeCompCooling_         __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Operating time compressor cooling
    uint32_t upTimeCompWw_              __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Operating time compressor warm water
    uint32_t heatingStarts_             __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Heating starts (control)
    uint32_t coolingStarts_             __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Cooling  starts (control)
    uint32_t wWStarts2_                 __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Warm water starts (control)
    uint32_t nrgConsTotal_              __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Energy consumption total
    uint32_t auxElecHeatNrgConsTotal_   __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Auxiliary electrical heater energy consumption total
    uint32_t auxElecHeatNrgConsHeating_ __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Auxiliary electrical heater energy consumption heating
    uint32_t auxElecHeatNrgConsDHW_     __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Auxiliary electrical heater energ consumption DHW
    uint32_t nrgConsCompTotal_          __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Energy consumption compressor total
    uint32_t nrgConsCompHeating_        __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Energy consumption compressor heating
    uint32_t nrgConsCompWw_             __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Energy consumption compressor warm water
    uint32_t nrgConsCompCooling_        __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Energy consumption compressor cooling

    // UBAEnergySupplied
    uint32_t nrgSuppTotal_   __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Energy supplied total
    uint32_t nrgSuppHeating_ __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Energy supplied heating
    uint32_t nrgSuppWw_      __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Energy supplied warm water
    uint32_t nrgSuppCooling_ __attribute__ ((aligned (4))) = EMS_VALUE_ULONG_NOTSET; // Energy supplied cooling

    // _UBAMaintenanceData
    uint8_t maintenanceMessage_  = EMS_VALUE_UINT_NOTSET;
    uint8_t maintenanceType_     = EMS_VALUE_UINT_NOTSET;
    uint8_t maintenanceTime_     = EMS_VALUE_UINT_NOTSET;
    char    maintenanceDate_[12]  __attribute__ ((aligned (4))) = {'\0'};

    void process_UBAParameterWW(std::shared_ptr<const Telegram> telegram);
    void process_UBAMonitorFast(std::shared_ptr<const Telegram> telegram);
    void process_UBATotalUptime(std::shared_ptr<const Telegram> telegram);
    void process_UBAParameters(std::shared_ptr<const Telegram> telegram);
    void process_UBAMonitorWW(std::shared_ptr<const Telegram> telegram);
    void process_UBAMonitorFastPlus(std::shared_ptr<const Telegram> telegram);
    void process_UBAMonitorSlow(std::shared_ptr<const Telegram> telegram);
    void process_UBAMonitorSlowPlus(std::shared_ptr<const Telegram> telegram);
    void process_UBAMonitorSlowPlus2(std::shared_ptr<const Telegram> telegram);
    void process_UBAParametersPlus(std::shared_ptr<const Telegram> telegram);
    void process_UBAParameterWWPlus(std::shared_ptr<const Telegram> telegram);
    void process_UBAOutdoorTemp(std::shared_ptr<const Telegram> telegram);
    void process_UBASetPoints(std::shared_ptr<const Telegram> telegram);
    void process_UBAFlags(std::shared_ptr<const Telegram> telegram);
    void process_MC10Status(std::shared_ptr<const Telegram> telegram);
    void process_UBAMaintenanceStatus(std::shared_ptr<const Telegram> telegram);
    void process_UBAMaintenanceData(std::shared_ptr<const Telegram> telegram);
    void process_UBAErrorMessage(std::shared_ptr<const Telegram> telegram);
    void process_UBAMonitorWWPlus(std::shared_ptr<const Telegram> telegram);
    void process_UBAInformation(std::shared_ptr<const Telegram> telegram);
    void process_UBAEnergySupplied(std::shared_ptr<const Telegram> telegram);
    void process_UBASettingsWW(std::shared_ptr<const Telegram> telegram);
//IRT-Bus
    void process_IRTSetFlowTemp(std::shared_ptr<const Telegram> telegram);  // 0x01
    void process_IRTSetWeatherControl(std::shared_ptr<const Telegram> telegram);  // 0x04
    void process_IRTSetWwActivated(std::shared_ptr<const Telegram> telegram);  // 0x05
    void process_IRTSetBurnerPower(std::shared_ptr<const Telegram> telegram);  // 0x07
    void process_IRTGetMaxWarmWater(std::shared_ptr<const Telegram> telegram);  // 0x81
    void process_IRTGetBoilerFlags(std::shared_ptr<const Telegram> telegram);  // 0x82
    void process_IRTGetActBurnerPower(std::shared_ptr<const Telegram> telegram);  // 0x83
    void process_IRTGetMaxBurnerPower(std::shared_ptr<const Telegram> telegram);  // 0x86
    void process_IRTGetOutdoorTemp(std::shared_ptr<const Telegram> telegram);  // 0x8A
    void process_IRTGetWwTemp2(std::shared_ptr<const Telegram> telegram);  // 0x8B
    void process_IRTGetBoilTemp(std::shared_ptr<const Telegram> telegram);  // 0x8C
    void process_IRTGetMaxFlowTemp(std::shared_ptr<const Telegram> telegram);  // 0x90
    void process_IRTGetPumpStatus(std::shared_ptr<const Telegram> telegram);  // 0x93
    void process_IRTGetDisplayCode(std::shared_ptr<const Telegram> telegram);  // 0xA3
    void process_IRTGetWwTemp(std::shared_ptr<const Telegram> telegram);  // 0xA4
    void process_IRTGetRetTemp(std::shared_ptr<const Telegram> telegram);  // 0xA6
    void process_IRTGetFlowTemp(std::shared_ptr<const Telegram> telegram);  // 0xA8
    void process_IRTGetBurnerRuntime(std::shared_ptr<const Telegram> telegram);  // 0xAA
    void process_IRTGetBurnerStarts(std::shared_ptr<const Telegram> telegram);  // 0xAB
    void process_IRTGetMaxBurnerPowerSetting(std::shared_ptr<const Telegram> telegram);  // 0xDE
    void process_IRTHandlerUnknownFunktion(std::shared_ptr<const Telegram> telegram);  // unknown funktions
    void process_IRTGetPeriodTimer(std::shared_ptr<const Telegram> telegram);  // 0xC9
    void process_IRTCommand91(std::shared_ptr<const Telegram> telegram);  // 0x91
    void process_IRTGetActBurnerPower_2(std::shared_ptr<const Telegram> telegram);  // 0x95
    void process_IRTCommand9B(std::shared_ptr<const Telegram> telegram);  // 0x9B
    void process_IRTCommand9C(std::shared_ptr<const Telegram> telegram);  // 0x9C
    void process_IRTFineFlow1(std::shared_ptr<const Telegram> telegram);  // 0x9D
    void process_IRTFineFlow2(std::shared_ptr<const Telegram> telegram);  // 0x9E
    void process_IRTFineRet1(std::shared_ptr<const Telegram> telegram);  // 0x9F
    void process_IRTFineRet2(std::shared_ptr<const Telegram> telegram);  // 0xA0
    void process_IRTCommandA1(std::shared_ptr<const Telegram> telegram);  // 0xA1
    void process_IRTCommandA2(std::shared_ptr<const Telegram> telegram);  // 0xA2
    
    // commands - none of these use the additional id parameter
    bool set_warmwater_mode(const char * value, const int8_t id);
    bool set_warmwater_activated(const char * value, const int8_t id);
    bool set_tapwarmwater_activated(const char * value, const int8_t id);
    bool set_warmwater_onetime(const char * value, const int8_t id);
    bool set_warmwater_circulation(const char * value, const int8_t id);
    bool set_warmwater_circulation_pump(const char * value, const int8_t id);
    bool set_warmwater_circulation_mode(const char * value, const int8_t id);
    bool set_warmwater_temp(const char * value, const int8_t id);
    bool set_flow_temp(const char * value, const int8_t id);
    bool set_heating_activated(const char * value, const int8_t id);
    bool set_heating_temp(const char * value, const int8_t id);
    bool set_min_power(const char * value, const int8_t id);
    bool set_max_power(const char * value, const int8_t id);
    bool set_min_pump(const char * value, const int8_t id);
    bool set_max_pump(const char * value, const int8_t id);
    bool set_hyst_on(const char * value, const int8_t id);
    bool set_hyst_off(const char * value, const int8_t id);
    bool set_burn_period(const char * value, const int8_t id);
    bool set_pump_delay(const char * value, const int8_t id);
    // bool set_reset(const char * value, const int8_t id);
    bool set_maintenance(const char * value, const int8_t id);
};

} // namespace emsesp

#endif
