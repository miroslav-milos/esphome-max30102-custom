
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID
from esphome.components import sensor, i2c, binary_sensor
max30102_ns = cg.esphome_ns.namespace('max30102_custom')
MAX30102Custom = max30102_ns.class_('MAX30102Custom', cg.PollingComponent, i2c.I2CDevice)
CONFIG_SCHEMA = cv.Schema({cv.GenerateID(): cv.declare_id(MAX30102Custom)}).extend(cv.polling_component_schema('20ms'))
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await cg.register_i2c_device(var)
