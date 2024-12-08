import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.components.select import Select
from esphome.const import (
    CONF_DISABLED_BY_DEFAULT,
    CONF_OPTIMISTIC,
    ENTITY_CATEGORY_CONFIG,
)
from . import CONF_UWB_ID, UWB_COMPONENT, uwb_ns

AUTO_LOAD = ["uwb"]

AntDelayCalibDeviceSelect = uwb_ns.class_("AntDelayCalibDeviceSelect", Select)

CONF_ANTDELAY_CALIB_DEVICE = "antenna_delay_calibration_device"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_UWB_ID): cv.use_id(UWB_COMPONENT),
        cv.Optional(CONF_ANTDELAY_CALIB_DEVICE): select.select_schema(AntDelayCalibDeviceSelect,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:form-dropdown",
        ).extend(
            {
                cv.Optional(CONF_DISABLED_BY_DEFAULT, default=True): cv.boolean,
                cv.Optional(CONF_OPTIMISTIC, default=False): cv.boolean,
            }
        )
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    uwb = await cg.get_variable(config[CONF_UWB_ID])

    if antdelay_device := config.get(CONF_ANTDELAY_CALIB_DEVICE):
        var = await select.new_select(antdelay_device,
                                      options=[])
        cg.add(uwb.setAntennaCalibrationDevice(var))
