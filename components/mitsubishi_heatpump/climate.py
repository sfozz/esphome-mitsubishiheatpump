import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, climate, select, sensor

from esphome.components.logger import HARDWARE_UART_TO_SERIAL
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_INTERNAL,
    CONF_HARDWARE_UART,
    CONF_BAUD_RATE,
    CONF_RX_PIN,
    CONF_TX_PIN,
    CONF_UPDATE_INTERVAL,
    CONF_MODE,
    CONF_FAN_MODE,
    CONF_SWING_MODE,
    PLATFORM_ESP8266
)
from esphome.core import CORE, coroutine

AUTO_LOAD = ["climate", "select", "binary_sensor"]

CONF_SUPPORTS = "supports"

CONF_INTERNAL_POWER_ON = "internal_power_on"
CONF_DEVICE_STATE_CONNECTED = "device_state_connected"
CONF_DEVICE_STATE_ACTIVE = "device_state_active"
CONF_DEVICE_STATE_LAST_UPDATED = "device_state_last_updated"
CONF_DEVICE_STATUS_OPERATING = "device_status_operating"
CONF_DEVICE_STATUS_CURRENT_TEMPERATURE = "device_status_current_temperature"
CONF_DEVICE_STATUS_COMPRESSOR_FREQUENCY = "device_status_compressor_frequency"
CONF_DEVICE_STATUS_LAST_UPDATED = "device_status_last_updated"
CONF_DEVICE_SET_POINT = "device_set_point"

CONF_HORIZONTAL_SWING_SELECT = "horizontal_vane_select"
CONF_VERTICAL_SWING_SELECT = "vertical_vane_select"
DEFAULT_CLIMATE_MODES = ["HEAT_COOL", "COOL", "HEAT", "DRY", "FAN_ONLY"]
DEFAULT_FAN_MODES = ["AUTO", "DIFFUSE", "LOW", "MEDIUM", "MIDDLE", "HIGH"]
DEFAULT_SWING_MODES = ["OFF", "VERTICAL"]
HORIZONTAL_SWING_OPTIONS = [
    "auto",
    "swing",
    "left",
    "left_center",
    "center",
    "right_center",
    "right",
]
VERTICAL_SWING_OPTIONS = ["swing", "auto", "up", "up_center", "center", "down_center", "down"]

# Remote temperature timeout configuration
CONF_REMOTE_OPERATING_TIMEOUT = "remote_temperature_operating_timeout_minutes"
CONF_REMOTE_IDLE_TIMEOUT = "remote_temperature_idle_timeout_minutes"
CONF_REMOTE_PING_TIMEOUT = "remote_temperature_ping_timeout_minutes"

MitsubishiHeatPump = cg.global_ns.class_(
    "MitsubishiHeatPump", climate.Climate, cg.PollingComponent
)

MitsubishiACSelect = cg.global_ns.class_(
    "MitsubishiACSelect", select.Select, cg.Component
)

def valid_uart(uart):
    if CORE.is_esp8266:
        uarts = ["UART0"]  # UART1 is tx-only
    elif CORE.is_esp32:
        uarts = ["UART0", "UART1", "UART2"]
    elif CORE.is_rp2040:
        uarts = ["UART0", "UART1"]
    else:
        raise NotImplementedError

    return cv.one_of(*uarts, upper=True)(uart)


SELECT_SCHEMA = select.SELECT_SCHEMA.extend(
    {cv.GenerateID(CONF_ID): cv.declare_id(MitsubishiACSelect)}
)

INTERNAL_POWER_ON_SCHEMA = binary_sensor.binary_sensor_schema(binary_sensor.BinarySensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC
)

DEVICE_STATE_CONNECTED_SCHEMA = binary_sensor.binary_sensor_schema(binary_sensor.BinarySensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC
)

DEVICE_STATE_ACTIVE_SCHEMA = binary_sensor.binary_sensor_schema(binary_sensor.BinarySensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC
)

DEVICE_STATE_LAST_UPDATED_SCHEMA = sensor.sensor_schema(sensor.Sensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC
)

DEVICE_STATUS_OPERATING_SCHEMA = binary_sensor.binary_sensor_schema(binary_sensor.BinarySensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC
)

DEVICE_STATUS_CURRENT_TEMPERATURE_SCHEMA = sensor.sensor_schema(sensor.Sensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC
)

DEVICE_STATUS_COMPRESSOR_FREQUENCY_SCHEMA = sensor.sensor_schema(sensor.Sensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC
)

DEVICE_STATUS_LAST_UPDATED_SCHEMA = sensor.sensor_schema(sensor.Sensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC
)

DEVICE_SET_POINT_SCHEMA = sensor.sensor_schema(sensor.Sensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC
)

INTERNAL_POWER_ON_DEFAULT = INTERNAL_POWER_ON_SCHEMA({
    CONF_ID: "internal_power_on",
    CONF_NAME: "Internal power on"
})

DEVICE_STATE_CONNECTED_DEFAULT = DEVICE_STATE_CONNECTED_SCHEMA({
    CONF_ID: "device_state_connected",
    CONF_NAME: "Device state connected"
})

DEVICE_STATE_ACTIVE_DEFAULT = DEVICE_STATE_ACTIVE_SCHEMA({
    CONF_ID: "device_state_active",
    CONF_NAME: "Device state active"
})

DEVICE_STATE_LAST_UPDATED_DEFAULT = DEVICE_STATE_LAST_UPDATED_SCHEMA({
    CONF_ID: "device_state_last_updated",
    CONF_NAME: "Device state last updated"
})

DEVICE_STATUS_OPERATING_DEFAULT = DEVICE_STATUS_OPERATING_SCHEMA({
    CONF_ID: "device_status_operating",
    CONF_NAME: "Device status operating"
})

DEVICE_STATUS_CURRENT_TEMPERATURE_DEFAULT = DEVICE_STATUS_CURRENT_TEMPERATURE_SCHEMA({
    CONF_ID: "device_status_current_temperature",
    CONF_NAME: "Device current temperature"
})

DEVICE_STATUS_COMPRESSOR_FREQUENCY_DEFAULT = DEVICE_STATUS_COMPRESSOR_FREQUENCY_SCHEMA({
    CONF_ID: "device_status_compressor_frequency",
    CONF_NAME: "Device status compressor frequency"
})

DEVICE_STATUS_LAST_UPDATED_DEFAULT = DEVICE_STATUS_LAST_UPDATED_SCHEMA({
    CONF_ID: "device_status_last_updated",
    CONF_NAME: "Device status last updated"
})

DEVICE_SET_POINT_DEFAULT = DEVICE_SET_POINT_SCHEMA({
    CONF_ID: "device_set_point",
    CONF_NAME: "Device Set Point"
})

INTERNAL_POWER_ON_DEFAULT[CONF_INTERNAL] = False
DEVICE_STATE_CONNECTED_DEFAULT[CONF_INTERNAL] = False
DEVICE_STATE_ACTIVE_DEFAULT[CONF_INTERNAL] = False
DEVICE_STATE_LAST_UPDATED_DEFAULT[CONF_INTERNAL] = False
DEVICE_STATUS_OPERATING_DEFAULT[CONF_INTERNAL] = False
DEVICE_STATUS_CURRENT_TEMPERATURE_DEFAULT[CONF_INTERNAL] = False
DEVICE_STATUS_COMPRESSOR_FREQUENCY_DEFAULT[CONF_INTERNAL] = False
DEVICE_STATUS_LAST_UPDATED_DEFAULT[CONF_INTERNAL] = False
DEVICE_SET_POINT_DEFAULT[CONF_INTERNAL] = False

CONFIG_SCHEMA = climate.CLIMATE_SCHEMA.extend(
    {
        cv.GenerateID(): cv.declare_id(MitsubishiHeatPump),
        cv.Optional(CONF_HARDWARE_UART, default="UART0"): valid_uart,
        cv.Optional(CONF_BAUD_RATE): cv.positive_int,
        cv.Optional(CONF_REMOTE_OPERATING_TIMEOUT): cv.positive_int,
        cv.Optional(CONF_REMOTE_IDLE_TIMEOUT): cv.positive_int,
        cv.Optional(CONF_REMOTE_PING_TIMEOUT): cv.positive_int,
        cv.Optional(CONF_RX_PIN): cv.positive_int,
        cv.Optional(CONF_TX_PIN): cv.positive_int,
        # If polling interval is greater than 9 seconds, the HeatPump library
        # reconnects, but doesn't then follow up with our data request.
        cv.Optional(CONF_UPDATE_INTERVAL, default="2000ms"): cv.All(
            cv.update_interval, cv.Range(max=cv.TimePeriod(milliseconds=8500))
        ),
        # Add selects for vertical and horizontal vane positions
        cv.Optional(CONF_HORIZONTAL_SWING_SELECT): SELECT_SCHEMA,
        cv.Optional(CONF_VERTICAL_SWING_SELECT): SELECT_SCHEMA,
        cv.Optional(CONF_INTERNAL_POWER_ON, default=INTERNAL_POWER_ON_DEFAULT): INTERNAL_POWER_ON_SCHEMA,
        cv.Optional(CONF_DEVICE_STATE_CONNECTED, default=DEVICE_STATE_CONNECTED_DEFAULT): DEVICE_STATE_CONNECTED_SCHEMA,
        cv.Optional(CONF_DEVICE_STATE_ACTIVE, default=DEVICE_STATE_ACTIVE_DEFAULT): DEVICE_STATE_ACTIVE_SCHEMA,
        cv.Optional(CONF_DEVICE_STATE_LAST_UPDATED, default=DEVICE_STATE_LAST_UPDATED_DEFAULT): DEVICE_STATE_LAST_UPDATED_SCHEMA,
        cv.Optional(CONF_DEVICE_STATUS_OPERATING, default=DEVICE_STATUS_OPERATING_DEFAULT): DEVICE_STATUS_OPERATING_SCHEMA,
        cv.Optional(CONF_DEVICE_STATUS_CURRENT_TEMPERATURE, default=DEVICE_STATUS_CURRENT_TEMPERATURE_DEFAULT): DEVICE_STATUS_CURRENT_TEMPERATURE_SCHEMA,
        cv.Optional(CONF_DEVICE_STATUS_COMPRESSOR_FREQUENCY, default=DEVICE_STATUS_COMPRESSOR_FREQUENCY_DEFAULT): DEVICE_STATUS_COMPRESSOR_FREQUENCY_SCHEMA,
        cv.Optional(CONF_DEVICE_STATUS_LAST_UPDATED, default=DEVICE_STATUS_LAST_UPDATED_DEFAULT): DEVICE_STATUS_LAST_UPDATED_SCHEMA,
        cv.Optional(CONF_DEVICE_SET_POINT, default=DEVICE_SET_POINT_DEFAULT): DEVICE_SET_POINT_SCHEMA,

        # Optionally override the supported ClimateTraits.
        cv.Optional(CONF_SUPPORTS, default={}): cv.Schema(
            {
                cv.Optional(CONF_MODE, default=DEFAULT_CLIMATE_MODES):
                    cv.ensure_list(climate.validate_climate_mode),
                cv.Optional(CONF_FAN_MODE, default=DEFAULT_FAN_MODES):
                    cv.ensure_list(climate.validate_climate_fan_mode),
                cv.Optional(CONF_SWING_MODE, default=DEFAULT_SWING_MODES):
                    cv.ensure_list(climate.validate_climate_swing_mode),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


@coroutine
def to_code(config):
    serial = HARDWARE_UART_TO_SERIAL[PLATFORM_ESP8266][config[CONF_HARDWARE_UART]]
    var = cg.new_Pvariable(config[CONF_ID], cg.RawExpression(f"&{serial}"))

    if CONF_BAUD_RATE in config:
        cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))

    if CONF_RX_PIN in config:
        cg.add(var.set_rx_pin(config[CONF_RX_PIN]))

    if CONF_TX_PIN in config:
        cg.add(var.set_tx_pin(config[CONF_TX_PIN]))

    if CONF_REMOTE_OPERATING_TIMEOUT in config:
        cg.add(var.set_remote_operating_timeout_minutes(config[CONF_REMOTE_OPERATING_TIMEOUT]))

    if CONF_REMOTE_IDLE_TIMEOUT in config:
        cg.add(var.set_remote_idle_timeout_minutes(config[CONF_REMOTE_IDLE_TIMEOUT]))

    if CONF_REMOTE_PING_TIMEOUT in config:
        cg.add(var.set_remote_ping_timeout_minutes(config[CONF_REMOTE_PING_TIMEOUT]))


    supports = config[CONF_SUPPORTS]
    traits = var.config_traits()

    for mode in supports[CONF_MODE]:
        if mode == "OFF":
            continue
        cg.add(traits.add_supported_mode(climate.CLIMATE_MODES[mode]))

    for mode in supports[CONF_FAN_MODE]:
        cg.add(traits.add_supported_fan_mode(climate.CLIMATE_FAN_MODES[mode]))

    for mode in supports[CONF_SWING_MODE]:
        cg.add(traits.add_supported_swing_mode(
            climate.CLIMATE_SWING_MODES[mode]
        ))

    if CONF_HORIZONTAL_SWING_SELECT in config:
        conf = config[CONF_HORIZONTAL_SWING_SELECT]
        swing_select = yield select.new_select(conf, options=HORIZONTAL_SWING_OPTIONS)
        yield cg.register_component(swing_select, conf)
        cg.add(var.set_horizontal_vane_select(swing_select))

    if CONF_VERTICAL_SWING_SELECT in config:
        conf = config[CONF_VERTICAL_SWING_SELECT]
        swing_select = yield select.new_select(conf, options=VERTICAL_SWING_OPTIONS)
        yield cg.register_component(swing_select, conf)
        cg.add(var.set_vertical_vane_select(swing_select))

    yield cg.register_component(var, config)
    yield climate.register_climate(var, config)
    yield binary_sensor.register_binary_sensor(var.internal_power_on, config[CONF_INTERNAL_POWER_ON])
    yield binary_sensor.register_binary_sensor(var.device_state_connected, config[CONF_DEVICE_STATE_CONNECTED])
    yield binary_sensor.register_binary_sensor(var.device_state_active, config[CONF_DEVICE_STATE_ACTIVE])
    yield sensor.register_sensor(var.device_state_last_updated, config[CONF_DEVICE_STATE_LAST_UPDATED])
    yield binary_sensor.register_binary_sensor(var.device_status_operating, config[CONF_DEVICE_STATUS_OPERATING])
    yield sensor.register_sensor(var.device_status_current_temperature, config[CONF_DEVICE_STATUS_CURRENT_TEMPERATURE])
    yield sensor.register_sensor(var.device_status_compressor_frequency, config[CONF_DEVICE_STATUS_COMPRESSOR_FREQUENCY])
    yield sensor.register_sensor(var.device_status_last_updated, config[CONF_DEVICE_STATUS_LAST_UPDATED])
    yield sensor.register_sensor(var.device_set_point, config[CONF_DEVICE_SET_POINT])

    cg.add_library(
        name="HeatPump",
        repository="https://github.com/SwiCago/HeatPump#5d1e146771d2f458907a855bf9d5d4b9bf5ff033",
        version=None, # this appears to be ignored?
    )
