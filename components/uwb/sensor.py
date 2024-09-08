import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_DISTANCE,
    ICON_SIGNAL_DISTANCE_VARIANT,
    UNIT_METER
)
from . import UWB_COMPONENT, CONF_UWB_ID, CONF_UWB_DEVICE_ID

AUTO_LOAD = ["uwb"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_UWB_ID): cv.use_id(UWB_COMPONENT),
        cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_METER,
            icon=ICON_SIGNAL_DISTANCE_VARIANT,
            accuracy_decimals=2,
        ).extend(
            {
                cv.Required(CONF_UWB_DEVICE_ID): cv.hex_int_range(min=0, max=254),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    uwb = await cg.get_variable(config[CONF_UWB_ID])

    if distance_config := config.get(CONF_DISTANCE):
        sens = await sensor.new_sensor(distance_config)
        cg.add(uwb.addDistanceSensor(distance_config[CONF_UWB_DEVICE_ID], sens))
