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

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.components.number import Number
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_INTERNAL,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    CONF_OPTIMISTIC,
    CONF_STEP,
    DEVICE_CLASS_DISTANCE,
    ENTITY_CATEGORY_CONFIG,
    UNIT_METER,
)
from . import CONF_UWB_ID, UWB_COMPONENT, uwb_ns

AUTO_LOAD = ["uwb"]

AntDelayCalibDistanceNumber = uwb_ns.class_("AntDelayCalibDistanceNumber", Number)

CONF_ANTDELAY_CALIB_DIST = "antenna_delay_calibration_distance"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_UWB_ID): cv.use_id(UWB_COMPONENT),
        cv.Optional(CONF_ANTDELAY_CALIB_DIST): number.number_schema(AntDelayCalibDistanceNumber,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:map-marker-distance",
            device_class=DEVICE_CLASS_DISTANCE,
            unit_of_measurement=UNIT_METER,
        ).extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=1): cv.positive_float,
                cv.Optional(CONF_MAX_VALUE, default=500): cv.positive_float,
                cv.Optional(CONF_DISABLED_BY_DEFAULT, default=True): cv.boolean,
                cv.Optional(CONF_STEP, default=1): cv.positive_float, # 1m steps
                cv.Optional(CONF_INTERNAL, default=False): cv.boolean,
                cv.Optional(CONF_OPTIMISTIC, default=False): cv.boolean,
            }
        )
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    uwb = await cg.get_variable(config[CONF_UWB_ID])

    if antdelay_dist := config.get(CONF_ANTDELAY_CALIB_DIST):
        var = await number.new_number(
            antdelay_dist,
            min_value=antdelay_dist[CONF_MIN_VALUE],
            max_value=antdelay_dist[CONF_MAX_VALUE],
            step=antdelay_dist[CONF_STEP]
        )
        cg.add(uwb.setAntennaCalibrationDistance(var))
