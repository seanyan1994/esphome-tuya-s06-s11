import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, remote_base
from esphome.const import CONF_ID

maxe_coolix_ns = cg.esphome_ns.namespace("maxe_coolix")
MaxeCoolixClimate = maxe_coolix_ns.class_(
    "MaxeCoolixClimate", climate.Climate, cg.Component, remote_base.RemoteReceiverListener
)

CONFIG_SCHEMA = (
    climate.climate_schema(MaxeCoolixClimate)
    .extend(cv.COMPONENT_SCHEMA)
    .extend(
        {
            cv.Optional(remote_base.CONF_RECEIVER_ID): cv.use_id(
                remote_base.RemoteReceiverBase
            ),
        }
    )
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
    if remote_base.CONF_RECEIVER_ID in config:
        await remote_base.register_listener(var, config)
