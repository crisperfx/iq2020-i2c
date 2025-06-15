import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID

i2c_sniffer_ns = cg.esphome_ns.namespace("i2c_sniffer")
I2CSniffer = i2c_sniffer_ns.class_("I2CSniffer", cg.Component)

CONF_I2C_SNIFFER_ID = "i2c_sniffer_id"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_I2C_SNIFFER_ID): cv.declare_id(I2CSniffer),
}).extend({})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_I2C_SNIFFER_ID])
    await cg.register_component(var, config)
