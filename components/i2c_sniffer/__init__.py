import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import logger
from esphome.const import CONF_ID
from esphome.components import gpio

i2c_sniffer_ns = cg.esphome_ns.namespace('i2c_sniffer')
I2CSniffer = i2c_sniffer_ns.class_('I2CSniffer', cg.Component)

CONF_SDA_PIN = "sda_pin"
CONF_SCL_PIN = "scl_pin"
CONF_LOG_LEVEL = "log_level"
CONF_BUFFER_SIZE = "buffer_size"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(I2CSniffer),
    cv.Required(CONF_SDA_PIN): cv.int_,
    cv.Required(CONF_SCL_PIN): cv.int_,
    cv.Optional(CONF_LOG_LEVEL, default=2): cv.int_,  # 0=ERROR,1=WARN,2=INFO,3=DEBUG
    cv.Optional(CONF_BUFFER_SIZE, default=16): cv.positive_int,
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    sniffer = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(sniffer, config)
    cg.add(sniffer.set_pins(config[CONF_SDA_PIN], config[CONF_SCL_PIN]))
    if CONF_LOG_LEVEL in config:
        cg.add(sniffer.set_log_level(config[CONF_LOG_LEVEL]))
    if CONF_BUFFER_SIZE in config:
        cg.add(sniffer.set_buffer_size(config[CONF_BUFFER_SIZE]))
