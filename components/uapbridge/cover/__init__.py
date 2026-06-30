from esphome.components import cover
import esphome.config_validation as cv
import esphome.codegen as cg
from .. import uapbridge_ns, CONF_UAPBRIDGE_ID, UAPBridge
from esphome.const import (
    CONF_ID,
    CONF_OPEN_DURATION,
    CONF_CLOSE_DURATION,
)

DEPENDENCIES = ["uapbridge"]

CONF_AUTO_CALIBRATION = "auto_calibration"

UAPBridgeCover = uapbridge_ns.class_("UAPBridgeCover", cover.Cover, cg.Component)

CONFIG_SCHEMA = cv.All(
    cover.cover_schema(UAPBridgeCover)
    .extend(
        {
            cv.GenerateID(): cv.declare_id(UAPBridgeCover),
            cv.GenerateID(CONF_UAPBRIDGE_ID): cv.use_id(UAPBridge),
            # Optional travel times. When set they enable an estimated position
            # (0-100 %) and best-effort "go to position". With auto_calibration
            # they are only the initial seed and get refined automatically.
            cv.Optional(CONF_OPEN_DURATION): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_CLOSE_DURATION): cv.positive_time_period_milliseconds,
            # Self-learn the travel times from full open/close cycles and persist
            # them to flash. Enables position estimation even without durations.
            cv.Optional(CONF_AUTO_CALIBRATION, default=False): cv.boolean,
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    # If you set one duration you must set the other, otherwise the estimate
    # would be one-sided.
    cv.has_none_or_all_keys(CONF_OPEN_DURATION, CONF_CLOSE_DURATION),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cover.register_cover(var, config)

    parent = await cg.get_variable(config[CONF_UAPBRIDGE_ID])
    cg.add(var.set_uapbridge_parent(parent))

    if CONF_OPEN_DURATION in config:
        cg.add(var.set_open_duration(config[CONF_OPEN_DURATION]))
    if CONF_CLOSE_DURATION in config:
        cg.add(var.set_close_duration(config[CONF_CLOSE_DURATION]))
    cg.add(var.set_auto_calibration(config[CONF_AUTO_CALIBRATION]))
