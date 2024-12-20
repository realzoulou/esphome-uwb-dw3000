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
from esphome.components import button
from esphome.components.button import Button
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    DEVICE_CLASS_EMPTY,
    ENTITY_CATEGORY_CONFIG,
)
from . import CONF_UWB_ID, UWB_COMPONENT, uwb_ns

AUTO_LOAD = ["uwb"]

AntDelayCalibStartButton = uwb_ns.class_("AntDelayCalibStartButton", Button)

CONF_ANTDELAY_START_BUTTON = "antenna_delay_calibration_start"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_UWB_ID): cv.use_id(UWB_COMPONENT),
        cv.Optional(CONF_ANTDELAY_START_BUTTON): button.button_schema(AntDelayCalibStartButton,
            device_class=DEVICE_CLASS_EMPTY,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:button-cursor",
        ).extend(
            {
                cv.Optional(CONF_DISABLED_BY_DEFAULT, default=True): cv.boolean,
            }
        )
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    uwb = await cg.get_variable(config[CONF_UWB_ID])

    if antdelay_start := config.get(CONF_ANTDELAY_START_BUTTON):
        var = await button.new_button(antdelay_start)
        cg.add(uwb.setAntennaCalibrationStartButton(var))
