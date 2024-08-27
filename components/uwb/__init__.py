import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["spi"]

MULTI_CONF = True

CONF_UWB_ID = "uwb_id"

uwb_ns = cg.esphome_ns.namespace("uwb")
UWB_COMPONENT = uwb_ns.class_("UwbComponent", cg.Component)

CONF_UWB_ROLE = "role"

eUwbRole = uwb_ns.enum("eUwbRole")
UWB_ROLE = {
    "anchor": eUwbRole.UWB_ROLE_ANCHOR,
    "tag":    eUwbRole.UWB_ROLE_TAG
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UWB_COMPONENT),
        cv.Required(CONF_UWB_ROLE): cv.enum(UWB_ROLE, lower=True),
    }
).extend(cv.COMPONENT_SCHEMA)
cv.only_with_arduino


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)

    cg.add(var.setRole(config[CONF_UWB_ROLE]))

    # ----- General compiler settings
    # treat warnings as error, abort compilation on the first error, check printf format and arguments
    cg.add_build_flag("-Werror -Wfatal-errors -Wformat=2")
    # optimize for speed
    cg.add_build_flag("-O2")
    # src/esphome/core/time.cpp: In member function 'size_t esphome::ESPTime::strftime(char*, size_t, const char*)':
    # src/esphome/core/time.cpp:20:54: error: format not a string literal, format string not checked [-Werror=format-nonliteral]
    cg.add_build_flag("-Wno-format-nonliteral")
