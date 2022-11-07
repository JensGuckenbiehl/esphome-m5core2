import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome import pins
from esphome.const import CONF_ID,\
    CONF_BATTERY_LEVEL, CONF_BRIGHTNESS, UNIT_PERCENT, ICON_BATTERY, CONF_MODEL

CONF_AXP192_ID = "axp192_id"

DEPENDENCIES = ['i2c']
MULTI_CONF = True

axp192_ns = cg.esphome_ns.namespace('axp192')
AXP192Component = axp192_ns.class_('AXP192Component', cg.Component, i2c.I2CDevice)
AXP192Model = axp192_ns.enum("AXP192Model")

MODELS = {
    "M5CORE2": AXP192Model.AXP192_M5CORE2,
}

AXP192_MODEL = cv.enum(MODELS, upper=True, space="_")

CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_ID): cv.declare_id(AXP192Component),
    cv.Required(CONF_MODEL): AXP192_MODEL,
    cv.Optional("sound", default=False): cv.boolean,
}).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x77))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_model(config[CONF_MODEL]))
    
    if "sound" in config:
        conf = config["sound"]
        cg.add(var.set_sound(conf))