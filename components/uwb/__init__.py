#
# Copyright 2024 realzoulou
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

# pylint: disable=line-too-long, invalid-name, missing-function-docstring, missing-module-docstring, too-many-branches

import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_LATITUDE,
    CONF_LONGITUDE,
)

AUTO_LOAD = ["spi", "sensor", "text_sensor", "number", "select", "button"]

MULTI_CONF = True

CONF_UWB_ID = "uwb_id"

uwb_ns = cg.esphome_ns.namespace("uwb")
UWB_COMPONENT = uwb_ns.class_("UwbComponent", cg.Component)

CONF_UWB_DEVICE_ID = "device_id"
CONF_UWB_ROLE = "role"
CONF_UWB_ROLE_ANCHOR = "anchor"
CONF_UWB_ROLE_TAG = "tag"
CONF_UWB_LED_OFF_AFTER = "led_off_after_ms"
CONF_UWB_ANT_DELAY = "antenna_delay"
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
RANGING_INTERVAL_DEFAULT    : float = uwb_ns.RANGING_INTERVAL_TIME_DEFAULT
ANCHOR_AWAY_DURATION_DEFAULT: float = uwb_ns.MAX_AGE_ANCHOR_DISTANCE_DEFAULT
ANT_DELAY_DEFAULT           : int   = uwb_ns.ANT_DLY_DEFAULT

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
        cv.Optional(CONF_UWB_ANT_DELAY): cv.int_range(min=1,max=32767),
    }
).extend(cv.COMPONENT_SCHEMA)
cv.only_with_arduino # pylint: disable=pointless-statement

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
        True # pylint: disable=pointless-statement

    # optional key: CONF_UWB_ANT_DELAY
    try:
        if ant_delay := config[CONF_UWB_ANT_DELAY]:
            cg.add(var.setAntennaDelay(ant_delay))
    except KeyError:
        True # pylint: disable=pointless-statement

    if role == CONF_UWB_ROLE_TAG:
        minDistanceChange = MIN_DISTANCE_CHANGE_DEFAULT
        maxSpeed = MAX_SPEED_DEFAULT
        ranging_interval = RANGING_INTERVAL_DEFAULT
        anchor_away_duration = ANCHOR_AWAY_DURATION_DEFAULT
        # optional tag keys
        try:
            if config[CONF_TAG_MIN_DISTANCE_CHANGE]:
                minDistanceChange = config[CONF_TAG_MIN_DISTANCE_CHANGE]
        except KeyError:
            True # pylint: disable=pointless-statement
        try:
            if config[CONF_TAG_MAX_SPEED]:
                maxSpeed = config[CONF_TAG_MAX_SPEED]
        except KeyError:
            True # pylint: disable=pointless-statement
        try:
            if ranging_interval := config[CONF_TAG_RANGING_INTERVAL]:
                cg.add(var.setRangingInterval(ranging_interval))
        except KeyError:
            True # pylint: disable=pointless-statement
        try:
            if anchor_away_duration := config[CONF_TAG_ANCHOR_AWAY_DURATION]:
                cg.add(var.setMaxAgeAnchorDistance(anchor_away_duration))
        except KeyError:
            True # pylint: disable=pointless-statement

        for anchor in config[CONF_TAG_ANCHORS]:
            cg.add(var.addAnchor(anchor[CONF_UWB_DEVICE_ID], anchor[CONF_LATITUDE], anchor[CONF_LONGITUDE],
                                 minDistanceChange, maxSpeed))

    if role == CONF_UWB_ROLE_ANCHOR:
        try: # both latitude and longitude, or none
            latitude = config[CONF_LATITUDE]
            longitude = config[CONF_LONGITUDE]
            if latitude and longitude:
                cg.add(var.setAnchorPosition(latitude, longitude))
        except KeyError:
            True # pylint: disable=pointless-statement

    # ----- General compiler settings
    # treat warnings as error, abort compilation on the first error, check printf format and arguments
    cg.add_build_flag("-Werror -Wfatal-errors -Wformat=2")
    # optimize for speed
    cg.add_build_flag("-O2")
    # src/esphome/core/time.cpp: In member function 'size_t esphome::ESPTime::strftime(char*, size_t, const char*)':
    # src/esphome/core/time.cpp:20:54: error: format not a string literal, format string not checked [-Werror=format-nonliteral]
    cg.add_build_flag("-Wno-format-nonliteral")
    # sanitizers
    #cg.add_build_flag("-fsanitize=undefined -fno-sanitize=shift-base")
    cg.add_build_flag("-fstack-protector-all")
