import esphome.codegen as cg
import esphome.config_validation as cv

i2c_sniffer_ns = cg.esphome_ns.namespace('i2c_sniffer')
I2CSniffer = i2c_sniffer_ns.class_('I2CSniffer', cg.Component)

CONFIG_SCHEMA = cv.Schema({}).extend({})

async def to_code(config):
    var = cg.new_Pvariable("i2c_sniffer", I2CSniffer())
    await cg.register_component(var, config)
