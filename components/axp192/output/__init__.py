import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import output
from esphome.const import CONF_PIN, CONF_ID
from .. import AXP192Component, axp192_ns, CONF_AXP192_ID

DEPENDENCIES = ["axp192"]

AXP192ComponentOutput = axp192_ns.class_(
    "AXP192ComponentOutput", output.FloatOutput, cg.Component
)

CONFIG_SCHEMA = output.FLOAT_OUTPUT_SCHEMA.extend(
    {
        cv.Required(CONF_ID): cv.declare_id(AXP192ComponentOutput),
        cv.GenerateID(CONF_AXP192_ID): cv.use_id(AXP192Component),
        cv.Required(CONF_PIN): cv.int_range(min=1, max=3),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AXP192_ID])
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await output.register_output(var, config)
    cg.add(var.set_pin(config[CONF_PIN]))
    cg.add(var.set_parent(parent))