# ![logo](media/EMS-ESP_logo_dark.png)

**EMS-ESP** is an open-source firmware for the Espressif ESP8266 and ESP32 microcontroller that communicates with **EMS** (Energy Management System) based equipment from manufacturers like Bosch, Buderus, Nefit, Junkers, Worcester and Sieger.

This is the firmware for the ESP8266, known as version 2. We've since moved to the ESP32 platform with v3 which opens up many more possibilities for additional features. Make sure you check out <https://github.com/emsesp/EMS-ESP32> and look at the [demo](https://ems-esp.derbyshire.nl).

## **Important: This version will be restricted to maintenance releases only!**
<br/>

[![version](https://img.shields.io/github/release/emsesp/EMS-ESP.svg?label=Latest%20Release)](https://github.com/emsesp/EMS-ESP/blob/main/CHANGELOG.md)
[![release-date](https://img.shields.io/github/release-date/emsesp/EMS-ESP.svg?label=Released)](https://github.com/emsesp/EMS-ESP/commits/main)
[![license](https://img.shields.io/github/license/emsesp/EMS-ESP.svg)](LICENSE)
[![downloads](https://img.shields.io/github/downloads/emsesp/EMS-ESP/total.svg)](https://github.com/emsesp/EMS-ESP/releases)
[![Average time to resolve an issue](http://isitmaintained.com/badge/resolution/emsesp/EMS-ESP.svg)](http://isitmaintained.com/project/emsesp/EMS-ESP "Average time to resolve an issue")
[![Percentage of issues still open](http://isitmaintained.com/badge/open/emsesp/EMS-ESP.svg)](http://isitmaintained.com/project/emsesp/EMS-ESP "Percentage of issues still open")
<br/>
[![chat](https://img.shields.io/discord/816637840644505620.svg?style=flat-square&color=blueviolet)](https://discord.gg/3J3GgnzpyT)

If you like **EMS-ESP**, please give it a star, or fork it and contribute!

[![GitHub stars](https://img.shields.io/github/stars/emsesp/EMS-ESP.svg?style=social&label=Star)](https://github.com/emsesp/EMS-ESP/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/emsesp/EMS-ESP.svg?style=social&label=Fork)](https://github.com/emsesp/EMS-ES32P/network)
[![donate](https://img.shields.io/badge/donate-PayPal-blue.svg)](https://www.paypal.com/paypalme/prderbyshire/2)

Note, EMS-ESP requires a small hardware circuit that can convert the EMS bus data to be read by the microcontroller. These can be ordered at <https://bbqkees-electronics.nl>.

<img src="media/gateway-integration.jpg" width=40%>

---

## **Features**

- Compatible with both ESP8266 and ESP32
- A multi-user secure web interface to change settings and monitor the data
- A console, accessible via Serial and Telnet for more monitoring
- Native support for Home Assistant via [MQTT Discovery](https://www.home-assistant.io/docs/mqtt/discovery/)
- Can run standalone as an independent WiFi Access Point or join an existing WiFi network
- Easy first-time configuration via a web Captive Portal
- Support for more than [70 EMS devices](https://emsesp.github.io/docs/#/Supported-EMS-Devices) (boilers, thermostats, solar modules, mixer modules, heat pumps, gateways)
  
## **Screenshots**

### Web Interface

| | |
| --- | --- |
| <img src="media/web_settings.PNG"> | <img src="media/web_status.PNG"> |
| <img src="media/web_devices.PNG"> | <img src="media/web_mqtt.PNG"> |

### Telnet Console

<img src="media/console.PNG" width=80% height=80%>

### In Home Assistant

<img src="media/ha_lovelace.PNG" width=80% height=80%>
  
## **Installing**

Refer to the [official documentation](https://emsesp.github.io/docs) to how to install the firmware and configure it. The documentation is being constantly updated as new features and settings are added.

You can choose to use an pre-built firmware image or compile the code yourself:

- [Uploading a pre-built firmware build](https://emsesp.github.io/docs/#/Uploading-firmware)
- [Building the firmware from source code and flashing manually](https://emsesp.github.io/docs/#/Building-firmware)

## **Support Information**

If you're looking for support on **EMS-ESP** there are some options available:

### Documentation

- [Official EMS-ESP Documentation](https://emsesp.github.io/docs): For information on how to build and upload the firmware
- [FAQ and Troubleshooting](https://emsesp.github.io/docs/#/Troubleshooting): For information on common problems and solutions. See also [BBQKees's wiki](https://bbqkees-electronics.nl/wiki/gateway/troubleshooting.html)

### Support Community

- [Discord Server](https://discord.gg/3J3GgnzpyT): For support, troubleshooting and general questions. You have better chances to get fast answers from members of the community
- [Search in Issues](https://github.com/emsesp/EMS-ESP/issues): You might find an answer to your question by searching current or closed issues

### Developer's Community

- [Bug Report](https://github.com/emsesp/EMS-ESP/issues/new?template=bug_report.md): For reporting Bugs
- [Feature Request](https://github.com/emsesp/EMS-ESP/issues/new?template=feature_request.md): For requesting features/functions
- [Troubleshooting](https://github.com/emsesp/EMS-ESP/issues/new?template=questions---troubleshooting.md): As a last resort, you can open new *Troubleshooting & Question* issue on GitHub if the solution could not be found using the other channels. Just remember: the more info you provide the more chances you'll have to get an accurate answer

# **Contributors ✨**

EMS-ESP is a project originally created by [proddy](https://github.com/proddy) with the main contributors and owners:

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
    <td align="center">
    <a href="https://derbyshire.nl"><img src="https://avatars.githubusercontent.com/u/1230712?v=3?s=100" width="100px;" alt=""/>
    <br/><sub><b>proddy</b></sub></a>
    <br/>
    </a> <a href="https://github.com/emsesp/EMS-ESP/commits?author=proddy" title="v2 Commits">v2</a>
    <a href="https://github.com/emsesp/EMS-ESP32/commits?author=proddy" title="v3 Commits">v3</a>
    </td>
    <td align="center">
    <a href="https://github.com/MichaelDvP"><img src="https://avatars.githubusercontent.com/u/59284019?v=3?s=100" width="100px;" alt=""/><br /><sub><b>MichaelDvP</b></sub></a><br /></a> <a href="https://github.com/emsesp/EMS-ESP/commits?author=MichaelDvP" title="v2 Commits">v2</a>
    <a href="https://github.com/emsesp/EMS-ESP32/commits?author=MichaelDvP" title="v3 Commits">v3</a>
    </td>

  </tr>
</table>
<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

You can also contribute to EMS-ESP by

- providing Pull Requests (Features, Fixes, suggestions)
- testing new released features and report issues on your EMS equipment
- contributing to missing [Documentation](https://emsesp.github.io/docs)

# **Libraries used**

- [esp8266-react](https://github.com/rjwats/esp8266-react) by @rjwats for the framework that provides the  Web UI
- [uuid-\*](https://github.com/nomis/mcu-uuid-console) from @nomis. The console, syslog, telnet and logging is based on these libraries
- [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
- [AsyncMqttClient](https://github.com/marvinroger/async-mqtt-client) for MQTT, with modifications from @bertmelis
- ESPAsyncWebServer and ESPAsyncTCP for the Web and TCP backends, with custom modifications for performance

# **License**

This program is licensed under GPL-3.0
