# components/max30102_custom/sensor.py

import esphome.codegen as cg
import esphome.config_validation as cv

from esphome.components import sensor, binary_sensor, i2c

DEPENDENCIES = ["i2c"]
AUTO_LOAD = ["sensor", "binary_sensor"]

max30102_ns = cg.esphome_ns.namespace("max30102_custom")

MAX30102CustomSensor = max30102_ns.class_(
    "MAX30102CustomSensor",
    cg.PollingComponent,
    i2c.I2CDevice,
    sensor.Sensor
)

# ---------------------------------------------------------------
#  CONFIG SCHEMA
# ---------------------------------------------------------------
CONFIG_SCHEMA = (
    sensor.sensor_schema(MAX30102CustomSensor)
    .extend(
        {
            # --- HARDWARE CONFIG ---
            cv.Optional("address", default=0x57): cv.i2c_address,
            cv.Optional("update_interval", default="20ms"): cv.update_interval,

            cv.Optional("sample_rate_hz",  default=100):  cv.int_,    # 50/100/200/400
            cv.Optional("sample_average",  default=4):    cv.int_,    # 1/2/4/8/16
            cv.Optional("pulse_width_us",  default=411):  cv.int_,    # 69/118/215/411
            cv.Optional("adc_range_na",    default=16384): cv.int_,   # 2048/4096/8192/16384

            # --- LED POWER PROFILES ---
            cv.Optional("idle_led_ir",   default=1.0): cv.float_,
            cv.Optional("idle_led_red",  default=1.0): cv.float_,

            cv.Optional("touch_led_ir",  default=6.0): cv.float_,
            cv.Optional("touch_led_red", default=3.0): cv.float_,

            cv.Optional("active_led_ir", default=12.6): cv.float_,
            cv.Optional("active_led_red", default=7.6): cv.float_,

            # --- TOUCH / FINGER DETECTION ---
            cv.Optional("finger_threshold",  default=15000.0): cv.float_,
            cv.Optional("finger_hysteresis", default=0.80):    cv.float_,
            cv.Optional("long_touch_ms",     default=2000):    cv.positive_int,

            # --- FILTERS & PHYSIO ---
            cv.Optional("baseline_alpha", default=0.01): cv.float_,
            cv.Optional("rms_beta",       default=0.02): cv.float_,

            cv.Optional("min_bpm", default=35):  cv.int_,
            cv.Optional("max_bpm", default=220): cv.int_,

            # --- SUB-SENSORS (OPTIONAL) ---
            cv.Optional("ir"):        sensor.sensor_schema(),
            cv.Optional("red"):       sensor.sensor_schema(),
            cv.Optional("heart_rate"):sensor.sensor_schema(),
            cv.Optional("spo2"):      sensor.sensor_schema(),

            cv.Optional("finger_present"): binary_sensor.binary_sensor_schema(),
            cv.Optional("short_touch"):    binary_sensor.binary_sensor_schema(),
            cv.Optional("long_touch"):     binary_sensor.binary_sensor_schema(),
        }
    )
    .extend(i2c.i2c_device_schema(0x57))
    .extend(cv.polling_component_schema("20ms"))
)

# ---------------------------------------------------------------
#  CODE GENERATION
# ---------------------------------------------------------------
async def to_code(config):
    var = cg.new_Pvariable(config["id"])

    await sensor.register_sensor(var, config)
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # BASIC HW SETTINGS
    cg.add(var.set_sample_rate(config["sample_rate_hz"]))
    cg.add(var.set_sample_average(config["sample_average"]))
    cg.add(var.set_pulse_width(config["pulse_width_us"]))
    cg.add(var.set_adc_range(config["adc_range_na"]))

    # LED MODES
    cg.add(var.set_idle_leds(config["idle_led_red"], config["idle_led_ir"]))
    cg.add(var.set_touch_leds(config["touch_led_red"], config["touch_led_ir"]))
    cg.add(var.set_active_leds(config["active_led_red"], config["active_led_ir"]))

    # FINGER DETECTION
    cg.add(var.set_finger_threshold(config["finger_threshold"]))
    cg.add(var.set_finger_hysteresis(config["finger_hysteresis"]))
    cg.add(var.set_touch_long_ms(config["long_touch_ms"]))

    # PHYSIO FILTERS
    cg.add(var.set_baseline_alpha(config["baseline_alpha"]))
    cg.add(var.set_rms_beta(config["rms_beta"]))
    cg.add(var.set_min_bpm(config["min_bpm"]))
    cg.add(var.set_max_bpm(config["max_bpm"]))

    # SUB-SENSORS
    if "ir" in config:
        sens = await sensor.new_sensor(config["ir"])
        cg.add(var.set_ir_sensor(sens))

    if "red" in config:
        sens = await sensor.new_sensor(config["red"])
        cg.add(var.set_red_sensor(sens))

    if "heart_rate" in config:
        sens = await sensor.new_sensor(config["heart_rate"])
        cg.add(var.set_hr_sensor(sens))

    if "spo2" in config:
        sens = await sensor.new_sensor(config["spo2"])
        cg.add(var.set_spo2_sensor(sens))

    # BINARY SENSORS
    if "finger_present" in config:
        bs = await binary_sensor.new_binary_sensor(config["finger_present"])
        cg.add(var.set_finger_sensor(bs))

    if "short_touch" in config:
        bs = await binary_sensor.new_binary_sensor(config["short_touch"])
        cg.add(var.set_short_touch_sensor(bs))

    if "long_touch" in config:
        bs = await binary_sensor.new_binary_sensor(config["long_touch"])
        cg.add(var.set_long_touch_sensor(bs))
