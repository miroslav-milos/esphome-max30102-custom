import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome import core

DEPENDENCIES = []
AUTO_LOAD = ["sensor"]

max30102_ns = cg.esphome_ns.namespace("max30102_custom")
MAX30102CustomSensor = max30102_ns.class_("MAX30102CustomSensor", sensor.Sensor)

CONFIG_SCHEMA = (
    sensor.sensor_schema(MAX30102CustomSensor)
    .extend(
        {
            cv.Optional("address", default=0x57): cv.i2c_address,
            cv.Optional("update_interval", default="1s"): cv.update_interval,
            cv.Optional("led_red_ma", default=7.6): cv.float_,
            cv.Optional("led_ir_ma", default=7.6): cv.float_,
            cv.Optional("sample_rate_hz", default=100): cv.int_,
            cv.Optional("pulse_width_us", default=411): cv.int_,
            cv.Optional("adc_range_na", default=16384): cv.int_,
            cv.Optional("baseline_alpha", default=0.01): cv.float_,
            cv.Optional("rms_beta", default=0.02): cv.float_,
            cv.Optional("min_bpm", default=35): cv.int_,
        }
    )
    .extend(cv.polling_component_schema("20ms"))
)

async def to_code(config):
    var = cg.new_Pvariable(config[core.CONF_ID])
    await sensor.register_sensor(var, config)

    cg.add(var.set_address(config["address"]))
    cg.add(var.set_led_red_ma(config["led_red_ma"]))
    cg.add(var.set_led_ir_ma(config["led_ir_ma"]))
    cg.add(var.set_sample_rate(config["sample_rate_hz"]))
    cg.add(var.set_pulse_width(config["pulse_width_us"]))
    cg.add(var.set_adc_range(config["adc_range_na"]))
    cg.add(var.set_baseline_alpha(config["baseline_alpha"]))
    cg.add(var.set_rms_beta(config["rms_beta"]))
    cg.add(var.set_min_bpm(config["min_bpm"]))
