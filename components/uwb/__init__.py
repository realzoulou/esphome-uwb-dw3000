import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.const import (
    CONF_ID,
)
from enum import IntEnum, Enum

DEPENDENCIES = ["logger", "spi"]

uwb_ns = cg.esphome_ns.namespace("uwb")

UwbComponent = uwb_ns.class_("UwbComponent",
                             # add constructor params here
                             cg.Component)

CONF_UWB_ROLE = "role"

eUwbRole = uwb_ns.enum("eUwbRole")
UWB_ROLE = {
    "anchor_controller": eUwbRole.UWB_ROLE_ANCHOR_CONTROLLER,
    "anchor_peripheral": eUwbRole.UWB_ROLE_ANCHOR_PERIPHERAL,
    "tag":               eUwbRole.UWB_ROLE_TAG
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UwbComponent),
        cv.Required(CONF_UWB_ROLE): cv.enum(UWB_ROLE, lower=True),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    var = cg.new_Pvariable(
        config[CONF_ID],
    )
    cg.add(var.setRole(config[CONF_UWB_ROLE]))

    # treat warnings as error, abort compilation on the first error, check printf format and arguments
    cg.add_build_flag("-Werror -Wfatal-errors -Wformat=2")

    # optimize for speed
    cg.add_build_flag("-O2")

    # src/esphome/core/time.cpp: In member function 'size_t esphome::ESPTime::strftime(char*, size_t, const char*)':
    # src/esphome/core/time.cpp:20:54: error: format not a string literal, format string not checked [-Werror=format-nonliteral]
    cg.add_build_flag("-Wno-format-nonliteral")

    await cg.register_component(var, config)
