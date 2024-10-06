import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_DISTANCE,
    CONF_LATITUDE,
    CONF_LONGITUDE,
    DEVICE_CLASS_DISTANCE,
    STATE_CLASS_MEASUREMENT,
    UNIT_DEGREES,
    UNIT_METER,

)
from . import UWB_COMPONENT, CONF_UWB_ID, CONF_UWB_DEVICE_ID

AUTO_LOAD = ["uwb"]

CONF_UWB_POSITION_ERROR_ESTIMATE = "error_estimate"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_UWB_ID): cv.use_id(UWB_COMPONENT),
        cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_METER,
            icon="mdi:map-marker-radius-outline",
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
