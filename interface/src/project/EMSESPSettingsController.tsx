import React, { Component } from 'react';
import { ValidatorForm, TextValidator, SelectValidator } from 'react-material-ui-form-validator';

import { Checkbox, TextField, Typography, Box, Link } from '@material-ui/core';
import SaveIcon from '@material-ui/icons/Save';
import MenuItem from '@material-ui/core/MenuItem';

import { ENDPOINT_ROOT } from '../api';
import { restController, RestControllerProps, RestFormLoader, RestFormProps, FormActions, FormButton, BlockFormControlLabel, SectionContent } from '../components';

import { isIP, isHostname, or, optional } from '../validators';

import { EMSESPSettings } from './EMSESPtypes';

export const EMSESP_SETTINGS_ENDPOINT = ENDPOINT_ROOT + "emsespSettings";

type EMSESPSettingsControllerProps = RestControllerProps<EMSESPSettings>;

class EMSESPSettingsController extends Component<EMSESPSettingsControllerProps> {

    componentDidMount() {
        ValidatorForm.addValidationRule('isIPOrHostname', optional(or(isIP, isHostname)));
        this.props.loadData();
    }

    render() {
        return (
            <SectionContent title='EMS-ESP Settings' titleGutter>
                <RestFormLoader
                    {...this.props}
                    render={props => (
                        <EMSESPSettingsControllerForm {...props} />
                    )}
                />
            </SectionContent>
        )
    }

}

export default restController(EMSESP_SETTINGS_ENDPOINT, EMSESPSettingsController);

type EMSESPSettingsControllerFormProps = RestFormProps<EMSESPSettings>;

function EMSESPSettingsControllerForm(props: EMSESPSettingsControllerFormProps) {
    const { data, saveData, handleValueChange } = props;
    return (
        <ValidatorForm onSubmit={saveData}>
            <Box bgcolor="info.main" p={2} mt={2} mb={2}>
                <Typography variant="body1">
                    Change the default settings on this page. For help click <Link target="_blank" href="https://emsesp.github.io/docs/#/Configure-firmware?id=settings" color="primary">{'here'}</Link>.
                </Typography>
            </Box>
            <br></br>
            <Typography variant="h6" color="primary" >
                EMS Bus
            </Typography>
            <SelectValidator name="tx_mode"
                label="Tx Mode"
                value={data.tx_mode}
                fullWidth
                variant="outlined"
                onChange={handleValueChange('tx_mode')}
                margin="normal">
                <MenuItem value={0}>0 - Off</MenuItem>
                <MenuItem value={1}>1 - Default</MenuItem>
                <MenuItem value={2}>2 - EMS+</MenuItem>
                <MenuItem value={3}>3 - HT3</MenuItem>
                <MenuItem value={4}>4 - Hardware</MenuItem>
                <MenuItem value={5}>5 - iRT passive</MenuItem>
                <MenuItem value={6}>6 - iRT active</MenuItem>
            </SelectValidator>
            <SelectValidator name="ems_bus_id"
                label="Bus ID"
                value={data.ems_bus_id}
                fullWidth
                variant="outlined"
                onChange={handleValueChange('ems_bus_id')}
                margin="normal">
                <MenuItem value={0x0B}>Service Key (0x0B)</MenuItem>
                <MenuItem value={0x0D}>Modem (0x0D)</MenuItem>
                <MenuItem value={0x0A}>Terminal (0x0A)</MenuItem>
                <MenuItem value={0x0F}>Time Module (0x0F)</MenuItem>
                <MenuItem value={0x12}>Alarm Module (0x12)</MenuItem>
            </SelectValidator>
            <TextValidator
                validators={['required', 'isNumber', 'minNumber:0', 'maxNumber:40']}
                errorMessages={['Rx GPIO is required', "Must be a number", "Must be 0 or higher", "Max value is 40"]}
                name="rx_gpio"
                label="Rx GPIO pin"
                fullWidth
                variant="outlined"
                value={data.rx_gpio}
                type="number"
                onChange={handleValueChange('rx_gpio')}
                margin="normal"
            />
            <TextValidator
                validators={['required', 'isNumber', 'minNumber:0', 'maxNumber:40']}
                errorMessages={['Tx GPIO is required', "Must be a number", "Must be 0 or higher", "Max value is 40"]}
                name="tx_gpio"
                label="Tx GPIO pin"
                fullWidth
                variant="outlined"
                value={data.tx_gpio}
                type="number"
                onChange={handleValueChange('tx_gpio')}
                margin="normal"
            />
            <TextValidator
                validators={['required', 'isNumber', 'minNumber:0', 'maxNumber:120']}
                errorMessages={['Tx delay is required', "Must be a number", "Must be 0 or higher", "Max value is 120"]}
                name="tx_delay"
                label="Tx delayed start (seconds)"
                fullWidth
                variant="outlined"
                value={data.tx_delay}
                type="number"
                onChange={handleValueChange('tx_delay')}
                margin="normal"
            />
            <br></br>
            <Typography variant="h6" color="primary" >
                Dallas Sensor
            </Typography>
            <TextValidator
                validators={['required', 'isNumber', 'minNumber:0', 'maxNumber:40']}
                errorMessages={['Dallas GPIO is required', "Must be a number", "Must be 0 or higher", "Max value is 40"]}
                name="dallas_gpio"
                label="Dallas GPIO pin (0=none)"
                fullWidth
                variant="outlined"
                value={data.dallas_gpio}
                type="number"
                onChange={handleValueChange('dallas_gpio')}
                margin="normal"
            />
            <BlockFormControlLabel
                control={
                    <Checkbox
                        checked={data.dallas_parasite}
                        onChange={handleValueChange('dallas_parasite')}
                        value="dallas_parasite"
                    />
                }
                label="Dallas Parasite Mode"
            />
            <br></br>
            <Typography variant="h6" color="primary" >
                LED
            </Typography>
            <TextValidator
                validators={['required', 'isNumber', 'minNumber:0', 'maxNumber:40']}
                errorMessages={['LED GPIO is required', "Must be a number", "Must be 0 or higher", "Max value is 40"]}
                name="led_gpio"
                label="LED GPIO pin (0=none)"
                fullWidth
                variant="outlined"
                value={data.led_gpio}
                type="number"
                onChange={handleValueChange('led_gpio')}
                margin="normal"
            />
            <BlockFormControlLabel
                control={
                    <Checkbox
                        checked={data.hide_led}
                        onChange={handleValueChange('hide_led')}
                        value="hide_led"
                    />
                }
                label="Invert/Hide LED"
            />
            <br></br>
            <Typography variant="h6" color="primary" >
                Shower
            </Typography>
            <BlockFormControlLabel
                control={
                    <Checkbox
                        checked={data.shower_timer}
                        onChange={handleValueChange('shower_timer')}
                        value="shower_timer"
                    />
                }
                label="Shower Timer"
            />
            <BlockFormControlLabel
                control={
                    <Checkbox
                        checked={data.shower_alert}
                        onChange={handleValueChange('shower_alert')}
                        value="shower_alert"
                    />
                }
                label="Shower Alert"
            />
            <br></br>
            <Typography variant="h6" color="primary" >
                API
            </Typography>
            <BlockFormControlLabel
                control={
                    <Checkbox
                        checked={data.api_enabled}
                        onChange={handleValueChange('api_enabled')}
                        value="api_enabled"
                    />
                }
                label="Allow WEB API to write commands"
            />
            <br></br>
            <Typography variant="h6" color="primary" >
                Syslog
            </Typography>
            <BlockFormControlLabel
                control={
                    <Checkbox
                        checked={data.syslog_enabled}
                        onChange={handleValueChange('syslog_enabled')}
                        value="syslog_enabled"
                    />
                }
                label="Enable Syslog"
            />
            <TextValidator
                validators={['isIPOrHostname']}
                errorMessages={["Not a valid IP address or hostname"]}
                name="syslog_host"
                label="Syslog IP/Host"
                fullWidth
                variant="outlined"
                value={data.syslog_host}
                onChange={handleValueChange('syslog_host')}
                margin="normal"
            />
            <SelectValidator name="syslog_level"
                label="Syslog Log Level"
                value={data.syslog_level}
                fullWidth
                variant="outlined"
                onChange={handleValueChange('syslog_level')}
                margin="normal">
                <MenuItem value={-1}>OFF</MenuItem>
                <MenuItem value={3}>ERR</MenuItem>
                <MenuItem value={5}>NOTICE</MenuItem>
                <MenuItem value={6}>INFO</MenuItem>
                <MenuItem value={7}>DEBUG</MenuItem>
                <MenuItem value={8}>ALL</MenuItem>
            </SelectValidator>
            <TextValidator
                validators={['required', 'isNumber', 'minNumber:0', 'maxNumber:65535']}
                errorMessages={['Syslog Mark is required', "Must be a number", "Must be 0 or higher", "Max value is 10"]}
                name="syslog_mark_interval"
                label="Syslog Mark Interval (seconds, 0=off)"
                fullWidth
                variant="outlined"
                value={data.syslog_mark_interval}
                type="number"
                onChange={handleValueChange('syslog_mark_interval')}
                margin="normal"
            />
            <BlockFormControlLabel
                control={
                    <Checkbox
                        checked={data.trace_raw}
                        onChange={handleValueChange('trace_raw')}
                        value="trace_raw"
                    />
                }
                label="Trace EMS telegrams in raw format"
            />

            <br></br>
            <Typography variant="h6" color="primary" >
                Analog Input
            </Typography>
            <BlockFormControlLabel
                control={
                    <Checkbox
                        checked={data.analog_enabled}
                        onChange={handleValueChange('analog_enabled')}
                        value="analog_enabled"
                    />
                }
                label="Enable ADC"
            />
            <br></br>
            <Typography variant="h6" color="primary" >
                Other
            </Typography>
            <SelectValidator name="bool_format"
                label="Boolean Format"
                value={data.bool_format}
                fullWidth
                variant="outlined"
                onChange={handleValueChange('bool_format')}
                margin="normal">
                <MenuItem value={1}>"on"/"off"</MenuItem>
                <MenuItem value={2}>true/false</MenuItem>
                <MenuItem value={3}>1/0</MenuItem>
                <MenuItem value={4}>"ON""/"OFF""</MenuItem>
            </SelectValidator>
            <br></br>
             <Typography variant="h6" color="primary" >
                Boiler-Settings
            </Typography>
            <SelectValidator name="usr_brand"
                label="Boiler brand"
                value={data.usr_brand}
                fullWidth
                variant="outlined"
                onChange={handleValueChange('usr_brand')}
                margin="normal">
                <MenuItem value={0}>0 - not set/other</MenuItem>
                <MenuItem value={1}>1 - Bosch</MenuItem>
                <MenuItem value={2}>2 - Junkers</MenuItem>
                <MenuItem value={3}>3 - Buderus</MenuItem>
                <MenuItem value={4}>4 - Nefit</MenuItem>
                <MenuItem value={5}>5 - Sieger</MenuItem>
                <MenuItem value={11}>11 - Worcester</MenuItem>
            </SelectValidator>
            <TextField
                name="usr_type"
                label="Type of boiler (optional)"
                fullWidth
                variant="outlined"
                value={data.usr_type}
                onChange={handleValueChange('usr_type')}
                margin="normal"
            />
           <TextValidator
                validators={['required', 'isNumber', 'minNumber:0', 'maxNumber:65535']}
                errorMessages={['Min boiler power(Wh) is required', "Must be a number", "Must be 0 or higher", "Max value is 65000 (65kWh)"]}
                name="min_boiler_wh"
                label="Min boiler power (Wh, 0=not set)"
                fullWidth
                variant="outlined"
                value={data.min_boiler_wh}
                type="number"
                onChange={handleValueChange('min_boiler_wh')}
                margin="normal"
            />
            <TextValidator
                validators={['required', 'isNumber', 'minNumber:0', 'maxNumber:65535']}
                errorMessages={['Max boiler power(Wh) is required', "Must be a number", "Must be 0 or higher", "Max value is 65000 (65kWh)"]}
                name="max_boiler_wh"
                label="Max boiler power (Wh, 0=not set)"
                fullWidth
                variant="outlined"
                value={data.max_boiler_wh}
                type="number"
                onChange={handleValueChange('max_boiler_wh')}
                margin="normal"
            />
            <TextValidator
                validators={['required','isNumber', 'minNumber:0', 'maxNumber:99999']}
                errorMessages={['Value is required',"Must be a number", "Must be 0 or higher", "Max value is 99999 (99999 m³)"]}
                name="gas_meter_reading"
                label="Gas meter reading (m³, 0=not set)"
                fullWidth
                variant="outlined"
                value={data.gas_meter_reading}
                type="number"
                onChange={handleValueChange('gas_meter_reading')}
                margin="normal"
            />
            <TextValidator
                validators={['required','isNumber', 'minNumber:0', 'maxNumber:12000']}
                errorMessages={['Value is required',"Must be a number", "Must be 0 or higher", "Max value is 12000"]}
                name="conv_factor"
                label="Conversion factor m³<-> Wh"
                fullWidth
                variant="outlined"
                value={data.conv_factor}
                type="number"
                onChange={handleValueChange('conv_factor')}
                margin="normal"
            />
            <br></br>
             <FormActions>
                <FormButton startIcon={<SaveIcon />} variant="contained" color="primary" type="submit">
                    Save
                </FormButton>
            </FormActions>
        </ValidatorForm>
    );
}
