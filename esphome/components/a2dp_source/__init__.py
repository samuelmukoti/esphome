from esphome import automation
import esphome.codegen as cg
from esphome.components.const import CONF_RECONNECT_INTERVAL
from esphome.components.esp32 import (
    VARIANT_ESP32,
    add_idf_sdkconfig_option,
    only_on_variant,
    request_bluetooth,
    request_software_coexistence,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_MAC_ADDRESS,
    CONF_NAME,
    CONF_ON_CONNECT,
    CONF_ON_DISCONNECT,
    CONF_PIN,
)
from esphome.core import CORE
from esphome.types import ConfigType

CODEOWNERS = ["@samuelmukoti"]
# esp32_ble runs the controller in BLE-only mode and releases the Classic Bluetooth memory
CONFLICTS_WITH = ["esp32_ble"]
DEPENDENCIES = ["esp32"]
DOMAIN = "a2dp_source"

CONF_A2DP_SOURCE_ID = "a2dp_source_id"
CONF_AUTO_CONNECT = "auto_connect"
CONF_REMOTE_NAME = "remote_name"
CONF_SUSPEND_TIMEOUT = "suspend_timeout"

a2dp_source_ns = cg.esphome_ns.namespace("a2dp_source")
A2DPSource = a2dp_source_ns.class_("A2DPSource", cg.Component)


def _validate_pin(value: str) -> str:
    value = cv.string_strict(value)
    if not value.isdigit() or not 4 <= len(value) <= 16:
        raise cv.Invalid("The PIN must be 4 to 16 digits")
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(A2DPSource),
            cv.Optional(CONF_NAME): cv.All(cv.string_strict, cv.Length(min=1, max=32)),
            cv.Optional(CONF_MAC_ADDRESS): cv.mac_address,
            cv.Optional(CONF_REMOTE_NAME): cv.All(
                cv.string_strict, cv.Length(min=1, max=64)
            ),
            cv.Optional(CONF_PIN, default="0000"): _validate_pin,
            cv.Optional(CONF_AUTO_CONNECT, default=True): cv.boolean,
            cv.Optional(
                CONF_RECONNECT_INTERVAL, default="15s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_SUSPEND_TIMEOUT, default="10s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(max=cv.TimePeriod(hours=1)),
            ),
            cv.Optional(CONF_ON_CONNECT): automation.validate_automation({}),
            cv.Optional(CONF_ON_DISCONNECT): automation.validate_automation({}),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.has_exactly_one_key(CONF_MAC_ADDRESS, CONF_REMOTE_NAME),
    only_on_variant(
        supported=[VARIANT_ESP32],
        msg_prefix="Classic Bluetooth (A2DP)",
    ),
)


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_device_name(config.get(CONF_NAME, CORE.friendly_name or CORE.name)))
    if (mac := config.get(CONF_MAC_ADDRESS)) is not None:
        cg.add(var.set_remote_address(mac.as_hex))
    cg.add_optional(var.set_remote_name, config.get(CONF_REMOTE_NAME))
    cg.add(var.set_pin(config[CONF_PIN]))
    cg.add(var.set_auto_connect(config[CONF_AUTO_CONNECT]))
    cg.add(var.set_reconnect_interval(config[CONF_RECONNECT_INTERVAL]))
    cg.add(var.set_suspend_timeout(config[CONF_SUSPEND_TIMEOUT]))

    for conf in config.get(CONF_ON_CONNECT, []):
        await automation.build_callback_automation(
            var, "add_on_connect_callback", [], conf
        )
    for conf in config.get(CONF_ON_DISCONNECT, []):
        await automation.build_callback_automation(
            var, "add_on_disconnect_callback", [], conf
        )

    request_bluetooth()
    # Classic Bluetooth and WiFi share the radio, which needs the software coexistence arbiter
    if "wifi" in CORE.loaded_integrations:
        request_software_coexistence()

    add_idf_sdkconfig_option("CONFIG_BT_BLUEDROID_ENABLED", True)
    add_idf_sdkconfig_option("CONFIG_BT_CLASSIC_ENABLED", True)
    add_idf_sdkconfig_option("CONFIG_BT_A2DP_ENABLE", True)
    add_idf_sdkconfig_option("CONFIG_BT_SSP_ENABLED", True)
    add_idf_sdkconfig_option("CONFIG_BT_BLE_ENABLED", False)
    add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_MODE_BR_EDR_ONLY", True)
    add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_MODE_BLE_ONLY", False)
    add_idf_sdkconfig_option("CONFIG_BTDM_CTRL_MODE_BTDM", False)
    if "psram" in CORE.loaded_integrations:
        add_idf_sdkconfig_option("CONFIG_BT_ALLOCATION_FROM_SPIRAM_FIRST", True)

    cg.add_define("USE_A2DP_SOURCE")


A2DP_SOURCE_AUTOMATION_SCHEMA = automation.maybe_simple_id(
    {cv.GenerateID(): cv.use_id(A2DPSource)}
)

for _name, _call in (
    ("a2dp_source.connect", "connect()"),
    ("a2dp_source.disconnect", "disconnect()"),
):
    automation.register_apply_action(
        _name, A2DP_SOURCE_AUTOMATION_SCHEMA, automation.ApplyCall(_call)
    )

automation.register_apply_condition(
    "a2dp_source.is_connected", A2DP_SOURCE_AUTOMATION_SCHEMA, "is_connected()"
)
