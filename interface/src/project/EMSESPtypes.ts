export interface EMSESPSettings {
  tx_mode: number;
  tx_delay: number;
  ems_bus_id: number;
  syslog_enabled: boolean;
  syslog_level: number;
  syslog_mark_interval: number;
  syslog_host: string;
  master_thermostat: number;
  shower_timer: boolean;
  shower_alert: boolean;
  rx_gpio: number;
  tx_gpio: number;
  dallas_gpio: number;
  dallas_parasite: boolean;
  led_gpio: number;
  hide_led: boolean;
  api_enabled: boolean;
  bool_format: number;
  analog_enabled: boolean;
  usr_brand: number;
  usr_type: string;
  min_boiler_wh: number;
  max_boiler_wh: number;
  gas_meter_reading: number;
  conv_factor: number;
  trace_raw: boolean;
}

export enum busConnectionStatus {
  BUS_STATUS_EMS_CONNECTED = 0,
  BUS_STATUS_IRT_CONNECTED = 1,
  BUS_STATUS_TX_ERRORS = 2,
  BUS_STATUS_OFFLINE = 3
}

export interface EMSESPStatus {
  status: busConnectionStatus;
  rx_received: number;
  tx_sent: number;
  rx_quality: number;
  tx_quality: number;
}

export interface Device {
  id: number;
  type: string;
  brand: string;
  name: string;
  deviceid: number;
  productid: number;
  version: string;
}

export interface Sensor {
  no: number;
  id: string;
  temp: string;
}

export interface EMSESPDevices {
  devices: Device[];
  sensors: Sensor[];
}

export interface EMSESPDeviceData {
  name: string;
  data: string[];
}
