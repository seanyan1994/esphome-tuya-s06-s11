import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, remote_base
from esphome.const import CONF_ID

hitachi_ac344_ns = cg.esphome_ns.namespace("hitachi_ac344")
HitachiClimate = hitachi_ac344_ns.class_(
    "HitachiClimate", climate.Climate, cg.Component, remote_base.RemoteReceiverListener
)

CONFIG_SCHEMA = (
    climate.climate_schema(HitachiClimate)
    .extend(cv.COMPONENT_SCHEMA)
    .extend(
        {
            cv.Optional(remote_base.CONF_RECEIVER_ID): cv.use_id(
                remote_base.RemoteReceiverBase
            ),
            cv.Optional("repeat", default=1): cv.int_range(min=1, max=5),
        }
    )
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    if "repeat" in config:
        cg.add(var.set_repeat(config["repeat"]))
    if remote_base.CONF_RECEIVER_ID in config:
        await remote_base.register_listener(var, config)
