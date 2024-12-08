import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_DISTANCE,
    CONF_LATITUDE,
    CONF_LONGITUDE,
    CONF_TEMPERATURE,
    CONF_VOLTAGE,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_EMPTY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_THERMOMETER,
    ICON_TIMELAPSE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_DEGREES,
    UNIT_EMPTY,
    UNIT_METER,
    UNIT_PERCENT,
    UNIT_VOLT,
)
from . import UWB_COMPONENT, CONF_UWB_ID, CONF_UWB_DEVICE_ID

AUTO_LOAD = ["uwb"]

CONF_UWB_POSITION_ERROR_ESTIMATE = "error_estimate"
CONF_UWB_ANCHORS_IN_USE = "anchors_in_use"
CONF_ANTDELAY_PROGRESS = "antenna_calibration_progress"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_UWB_ID): cv.use_id(UWB_COMPONENT),
        cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_METER,
            icon="mdi:map-marker-distance",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ).extend(
            {
                cv.Required(CONF_UWB_DEVICE_ID): cv.hex_int_range(min=0, max=254),
            }
        ),
        cv.Optional(CONF_LATITUDE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon="mdi:map-marker",
            accuracy_decimals=8,
        ),
        cv.Optional(CONF_LONGITUDE): sensor.sensor_schema(
            unit_of_measurement=UNIT_DEGREES,
            icon="mdi:map-marker",
            accuracy_decimals=8,
        ),
        cv.Optional(CONF_UWB_POSITION_ERROR_ESTIMATE): sensor.sensor_schema(
            unit_of_measurement=UNIT_METER,
            icon="mdi:map-marker-radius-outline",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_DISTANCE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_UWB_ANCHORS_IN_USE): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon="mdi:numeric",
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon="mdi:sine-wave",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_ANTDELAY_PROGRESS): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon=ICON_TIMELAPSE,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_EMPTY,
            state_class=STATE_CLASS_MEASUREMENT, # not STATE_CLASS_TOTAL_INCREASING because progress may decrease
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ).extend(
            {
                cv.Optional(CONF_DISABLED_BY_DEFAULT, default=True): cv.boolean,
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    uwb = await cg.get_variable(config[CONF_UWB_ID])

    if distance_config := config.get(CONF_DISTANCE):
        sens = await sensor.new_sensor(distance_config)
        cg.add(uwb.addDistanceSensor(distance_config[CONF_UWB_DEVICE_ID], sens))
    if latitude_config := config.get(CONF_LATITUDE):
        sens = await sensor.new_sensor(latitude_config)
        cg.add(uwb.addLatitudeSensor(sens))
    if longitude_config := config.get(CONF_LONGITUDE):
        sens = await sensor.new_sensor(longitude_config)
        cg.add(uwb.addLongitudeSensor(sens))
    if err_est_config := config.get(CONF_UWB_POSITION_ERROR_ESTIMATE):
        sens = await sensor.new_sensor(err_est_config)
        cg.add(uwb.addErrorEstimateSensor(sens))
    if anchors_in_use_config := config.get(CONF_UWB_ANCHORS_IN_USE):
        sens = await sensor.new_sensor(anchors_in_use_config)
        cg.add(uwb.addAnchorsInUseSensor(sens))
    if voltage_config := config.get(CONF_VOLTAGE):
        sens = await sensor.new_sensor(voltage_config)
        cg.add(uwb.setVoltageSensor(sens))
    if temperature_config := config.get(CONF_TEMPERATURE):
        sens = await sensor.new_sensor(temperature_config)
        cg.add(uwb.setTemperatureSensor(sens))
    if antdelay_progress := config.get(CONF_ANTDELAY_PROGRESS):
        sens = await sensor.new_sensor(antdelay_progress)
        cg.add(uwb.setAntennaCalibrationProgress(sens))
