import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_ID,
)

DEPENDENCIES = ["logger"]

uwb_ns = cg.esphome_ns.namespace("uwb")

UwbComponent = uwb_ns.class_("UwbComponent",
                             # add constructor params here
                             cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UwbComponent),
    }
).extend(cv.COMPONENT_SCHEMA)

FINAL_VALIDATE_SCHEMA = cv.All(

)

async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
    )
    await cg.register_component(var, config)
