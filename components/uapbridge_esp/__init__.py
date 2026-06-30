import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from ..uapbridge import to_code_base, CONFIG_SCHEMA_BASE, UAPBridge
from esphome.const import CONF_ID

AUTO_LOAD = ["uapbridge"]
MULTI_CONF = True

# Create UAPBridge_esp namespace
uapbridge_esp_ns = cg.esphome_ns.namespace("uapbridge_esp")
UAPBridge_esp = uapbridge_esp_ns.class_("UAPBridge_esp", UAPBridge)

# This component talks to the ESP-IDF UART driver directly. Since ESPHome 2026.6
# the ESP32 UART always uses the IDF driver under both the esp-idf and Arduino
# frameworks, so requiring ESP32 (rather than the now-removed cv.only_with_esp_idf)
# is the correct and forward-compatible constraint.
CONFIG_SCHEMA = cv.All(
    CONFIG_SCHEMA_BASE.extend(
        {
            cv.GenerateID(): cv.declare_id(UAPBridge_esp),
        }
    ).add_extra(cv.only_on_esp32)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "uapbridge_uart",
    require_tx=True,
    require_rx=True,
    baud_rate=19200
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await to_code_base(var, config)
    await uart.register_uart_device(var, config)
