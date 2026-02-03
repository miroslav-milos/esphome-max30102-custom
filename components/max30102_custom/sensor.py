import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome import core

DEPENDENCIES = []
AUTO_LOAD = ["sensor"]

max30102_ns = cg.esphome_ns.namespace("max30102_custom")
MAX30102CustomSensor = max30102_ns.class_("MAX30102CustomSensor", sensor.Sensor)# all implemented in __init__.py (placeholder)
