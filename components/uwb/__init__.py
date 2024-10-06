import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_LATITUDE,
    CONF_LONGITUDE,
)

AUTO_LOAD = ["spi", "sensor"]

MULTI_CONF = True

CONF_UWB_ID = "uwb_id"

uwb_ns = cg.esphome_ns.namespace("uwb")
UWB_COMPONENT = uwb_ns.class_("UwbComponent", cg.Component)

CONF_UWB_DEVICE_ID = "device_id"
CONF_UWB_ROLE = "role"
CONF_UWB_ROLE_ANCHOR = "anchor"
CONF_UWB_ROLE_TAG = "tag"
CONF_UWB_LED_OFF_AFTER = "led_off_after_ms"
CONF_TAG_ANCHORS = "anchors"
CONF_TAG_RANGING_INTERVAL = "ranging_interval_ms"
CONF_TAG_ANCHOR_AWAY_DURATION = "anchor_away_after_ms"
CONF_TAG_MIN_DISTANCE_CHANGE = "min_distance_change"
CONF_TAG_MAX_SPEED = "max_speed"

eUwbRole = uwb_ns.enum("eUwbRole")
UWB_ROLE = {
    CONF_UWB_ROLE_ANCHOR: eUwbRole.UWB_ROLE_ANCHOR,
    CONF_UWB_ROLE_TAG:    eUwbRole.UWB_ROLE_TAG
}
MIN_DISTANCE_CHANGE_DEFAULT : float = uwb_ns.MIN_DISTANCE_CHANGE_DEFAULT
MAX_SPEED_DEFAULT           : float = uwb_ns.MAX_SPEED_DEFAULT

# BEGIN: parse_latlon and LAT_LON_REGEX copied from esphome/components/sun/__init__.py
# Parses sexagesimal values like 22°57′7″S
LAT_LON_REGEX = re.compile(
    r"([+\-])?\s*"
    r"(?:([0-9]+)\s*°)?\s*"
    r"(?:([0-9]+)\s*[′\'])?\s*"
    r'(?:([0-9]+)\s*[″"])?\s*'
    r"([NESW])?"
)

def parse_latlon(value):
    if isinstance(value, str) and value.endswith("°"):
        # strip trailing degree character
        value = value[:-1]
    try:
        return cv.float_(value)
    except cv.Invalid:
        pass

    value = cv.string_strict(value)
    m = LAT_LON_REGEX.match(value)

    if m is None:
        raise cv.Invalid("Invalid format for latitude/longitude")
    sign = m.group(1)
    deg = m.group(2)
    minute = m.group(3)
    second = m.group(4)
    d = m.group(5)

    val = float(deg or 0) + float(minute or 0) / 60 + float(second or 0) / 3600
    if sign == "-":
        val *= -1
    if d and d in "SW":
        val *= -1
    return val
# END: parse_latlon and LAT_LON_REGEX copied from esphome/components/sun/__init__.py

TAG_ANCHORS_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_UWB_DEVICE_ID): cv.hex_int_range(min=0, max=254),
        cv.Required(CONF_LATITUDE): cv.All(parse_latlon, cv.float_range(min=-90, max=90)),
        cv.Required(CONF_LONGITUDE): cv.All(parse_latlon, cv.float_range(min=-180, max=180)),
    }
)
ANCHOR_TAGS_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_UWB_DEVICE_ID): cv.hex_int_range(min=0, max=254),
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(UWB_COMPONENT),
        cv.Required(CONF_UWB_ROLE): cv.enum(UWB_ROLE, lower=True),
        cv.Required(CONF_UWB_DEVICE_ID): cv.hex_int_range(min=0, max=254),
        # options for role=anchor
        cv.Optional(CONF_LATITUDE): cv.All(parse_latlon, cv.float_range(min=-90, max=90)),
        cv.Optional(CONF_LONGITUDE): cv.All(parse_latlon, cv.float_range(min=-180, max=180)),
        # options for role=tag
        cv.Optional(CONF_TAG_ANCHORS): cv.All(
            cv.ensure_list(TAG_ANCHORS_SCHEMA),
            cv.Length(min=1, msg=f"{CONF_UWB_ROLE} '{CONF_UWB_ROLE_TAG}' must define at least 1 {CONF_TAG_ANCHORS}")
        ),
        cv.Optional(CONF_TAG_RANGING_INTERVAL): cv.int_range(min=1000),
        cv.Optional(CONF_TAG_ANCHOR_AWAY_DURATION): cv.int_range(min=1000),
        cv.Optional(CONF_TAG_MIN_DISTANCE_CHANGE): cv.float_range(min=0.01),
        cv.Optional(CONF_TAG_MAX_SPEED): cv.float_range(min=0.01),
        cv.Optional(CONF_UWB_LED_OFF_AFTER): cv.int_range(min=0),
    }
).extend(cv.COMPONENT_SCHEMA)
cv.only_with_arduino

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)

    cg.add(var.setDeviceId(config[CONF_UWB_DEVICE_ID]))

    role = config[CONF_UWB_ROLE]
    cg.add(var.setRole(role))

    # optional key: CONF_LED_OFF_AFTER
    try:
        if ledOffDuration := config[CONF_UWB_LED_OFF_AFTER]:
            cg.add(var.setLedsOffAfter(ledOffDuration))
    except KeyError:
        True

    if role == CONF_UWB_ROLE_TAG:
        minDistanceChange = MIN_DISTANCE_CHANGE_DEFAULT
        maxSpeed = MAX_SPEED_DEFAULT
        # optional role keys
        try:
            if config[CONF_TAG_MIN_DISTANCE_CHANGE]:
                minDistanceChange = config[CONF_TAG_MIN_DISTANCE_CHANGE]
            if config[CONF_TAG_MAX_SPEED]:
                maxSpeed = config[CONF_TAG_MAX_SPEED]
            if ranging_interval := config[CONF_TAG_RANGING_INTERVAL]:
                cg.add(var.setRangingInterval(ranging_interval))
            if anchor_away_duration := config[CONF_TAG_ANCHOR_AWAY_DURATION]:
                cg.add(var.setMaxAgeAnchorDistance(anchor_away_duration))
        except KeyError:
            True
        for anchor in config[CONF_TAG_ANCHORS]:
            cg.add(var.addAnchor(anchor[CONF_UWB_DEVICE_ID], anchor[CONF_LATITUDE], anchor[CONF_LONGITUDE],
                                 minDistanceChange, maxSpeed))

    if role == CONF_UWB_ROLE_ANCHOR:
        try:
            latitude = config[CONF_LATITUDE]
            longitude = config[CONF_LONGITUDE]
            if latitude and longitude:
                cg.add(var.setAnchorPosition(latitude, longitude))
        except KeyError:
            True

    # ----- General compiler settings
    # treat warnings as error, abort compilation on the first error, check printf format and arguments
    cg.add_build_flag("-Werror -Wfatal-errors -Wformat=2")
    # optimize for speed
    cg.add_build_flag("-O2")
    # src/esphome/core/time.cpp: In member function 'size_t esphome::ESPTime::strftime(char*, size_t, const char*)':
    # src/esphome/core/time.cpp:20:54: error: format not a string literal, format string not checked [-Werror=format-nonliteral]
    cg.add_build_flag("-Wno-format-nonliteral")
