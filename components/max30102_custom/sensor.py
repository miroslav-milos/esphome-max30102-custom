import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, binary_sensor
from esphome import core

DEPENDENCIES = []
AUTO_LOAD = ["sensor", "binary_sensor"]

max30102_ns = cg.esphome_ns.namespace("max30102_custom")
MAX30102CustomSensor = max30102_ns.class_("MAX30102CustomSensor", sensor.Sensor)

CONFIG_SCHEMA = (
    sensor.sensor_schema(MAX30102CustomSensor)
    .extend(
        {
            cv.Optional("address", default=0x57): cv.i2c_address,
            cv.Optional("update_interval", default="20ms"): cv.update_interval,
            cv.Optional("led_red_ma", default=7.6): cv.float_,
            cv.Optional("led_ir_ma", default=7.6): cv.float_,
            cv.Optional("sample_rate_hz", default=100): cv.int_,
            cv.Optional("pulse_width_us", default=411): cv.int_,
            cv.Optional("adc_range_na", default=16384): cv.int_,
            cv.Optional("baseline_alpha", default=0.01): cv.float_,
            cv.Optional("rms_beta", default=0.02): cv.float_,
            cv.Optional("min_bpm", default=35): cv.int_,
            cv.Optional("max_bpm", default=220): cv.int_,

            # dodatni parametri za tvoju logiku
            cv.Optional("hr_median_window", default=5): cv.int_,
            cv.Optional("spo2_median_window", default=7): cv.int_,
            cv.Optional("finger_ir_threshold", default=300.0): cv.float_,
            cv.Optional("finger_hysteresis", default=0.8): cv.float_,
            cv.Optional("perf_min", default=0.002): cv.float_,

            # pod-senzori (IR, RED, HR, SpO2)
            cv.Optional("ir"): sensor.sensor_schema(),
            cv.Optional("red"): sensor.sensor_schema(),
            cv.Optional("heart_rate"): sensor.sensor_schema(),
            cv.Optional("spo2"): sensor.sensor_schema(),

            # binary_sensor – prisutnost prsta
            cv.Optional("finger"): binary_sensor.binary_sensor_schema(),
        }
    )
    .extend(cv.polling_component_schema("20ms"))
)

async def to_code(config):
    var = cg.new_Pvariable(config["id"])
    await sensor.register_sensor(var, config)

    # osnovni parametri
    cg.add(var.set_address(config["address"]))
    cg.add(var.set_led_red_ma(config["led_red_ma"]))
    cg.add(var.set_led_ir_ma(config["led_ir_ma"]))
    cg.add(var.set_sample_rate(config["sample_rate_hz"]))
    cg.add(var.set_pulse_width(config["pulse_width_us"]))
    cg.add(var.set_adc_range(config["adc_range_na"]))
    cg.add(var.set_baseline_alpha(config["baseline_alpha"]))
    cg.add(var.set_rms_beta(config["rms_beta"]))
    cg.add(var.set_min_bpm(config["min_bpm"]))
    cg.add(var.set_max_bpm(config["max_bpm"]))

    # dodatni parametri
    cg.add(var.set_hr_median_window(config["hr_median_window"]))
    cg.add(var.set_spo2_median_window(config["spo2_median_window"]))
    cg.add(var.set_finger_ir_threshold(config["finger_ir_threshold"]))
    cg.add(var.set_finger_hysteresis(config["finger_hysteresis"]))
    cg.add(var.set_perf_min(config["perf_min"]))

    # pod senzori
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

    # binary sensor finger detection
    if "finger" in config:
        bs = await binary_sensor.new_binary_sensor(config["finger"])
        cg.add(var.set_finger_sensor(bs))
