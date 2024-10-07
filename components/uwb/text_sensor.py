import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import (
    DEVICE_CLASS_EMPTY,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from . import UWB_COMPONENT, CONF_UWB_ID

AUTO_LOAD = ["uwb"]

CONF_UWB_DIAGNOSTIC_STATUS = "status"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_UWB_ID): cv.use_id(UWB_COMPONENT),
        cv.Optional(CONF_UWB_DIAGNOSTIC_STATUS): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:information-outline",
            device_class=DEVICE_CLASS_EMPTY,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    uwb = await cg.get_variable(config[CONF_UWB_ID])

    if diag_status_text_sensor := config.get(CONF_UWB_DIAGNOSTIC_STATUS):
        sens = await text_sensor.new_text_sensor(diag_status_text_sensor)
        cg.add(uwb.setDiagnosticStatusSensor(sens))
