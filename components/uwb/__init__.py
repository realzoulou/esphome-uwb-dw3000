import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import CONF_ID
from esphome.core import CORE

DEPENDENCIES = ["logger"]

UwbComponent = cg.global_ns.class_("UwbComponent", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UwbComponent),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
    )
    await cg.register_component(var, config)
    
    if CORE.is_esp32:
        cg.add_build_flag("-DHARDWARE_SERIAL_WITH_PINS")

    if CORE.is_esp8266:
        cg.add_build_flag("-DHARDWARE_SERIAL")
