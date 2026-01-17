# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

from esphome import codegen as cg
import esphome.config_validation as cv
from esphome.components.light.effects import register_addressable_effect
from esphome.components.light.types import AddressableLightEffect
from esphome.const import CONF_ID, CONF_NAME, CONF_PORT

CODEOWNERS = ["@stuartparmenter"]
DEPENDENCIES = ["network"]
AUTO_LOAD = ["light"]

ddp_ns = cg.esphome_ns.namespace("ddp")
DdpComponent = ddp_ns.class_("DdpComponent", cg.Component)
DdpLightEffect = ddp_ns.class_("DdpLightEffect", AddressableLightEffect)
BrightnessMethod = ddp_ns.enum("BrightnessMethod")

# Configuration keys
CONF_DDP_ID = "ddp_id"
CONF_STREAM = "stream"
CONF_METRICS = "metrics"
CONF_BRIGHTNESS_METHOD = "brightness_method"

# Brightness method enum values
BRIGHTNESS_METHODS = {
    "max": BrightnessMethod.MAX,
    "average": BrightnessMethod.AVERAGE,
    "luminance": BrightnessMethod.LUMINANCE,
}

# Shared stream ID registry (used by ddp_canvas and ddp_light_effect)
_used_stream_ids = set()
_next_auto_stream_id = 2


def register_stream_id(stream_id):
    """Register an explicitly specified stream ID."""
    if stream_id in _used_stream_ids:
        raise cv.Invalid(
            f"Duplicate DDP stream ID {stream_id}! Each stream ID must be unique across all ddp_canvas and ddp_light_effect components."
        )
    _used_stream_ids.add(stream_id)


def allocate_stream_id():
    """Allocate the next available auto-generated stream ID."""
    global _next_auto_stream_id

    while _next_auto_stream_id in _used_stream_ids and _next_auto_stream_id < 250:
        _next_auto_stream_id += 1

    if _next_auto_stream_id > 249:
        raise cv.Invalid("Too many DDP streams: exceeded limit (2-249)")

    stream_id = _next_auto_stream_id
    _used_stream_ids.add(stream_id)
    _next_auto_stream_id += 1

    return stream_id


def _consume_ddp_sockets(config):
    """Register socket needs for DDP component (ESPHome 2025.11+)."""
    try:
        from esphome.components import socket

        # DDP needs 1 listening UDP socket
        socket.consume_sockets(1, "ddp")(config)
    except AttributeError:
        # Backwards compatibility: socket.consume_sockets not available in older ESPHome
        pass
    return config


# DDP component schema
CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DdpComponent),
            cv.Optional(CONF_PORT, default=4048): cv.port,
            cv.Optional(CONF_METRICS, default=False): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _consume_ddp_sockets,
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))

    # Define DDP_METRICS build flag if metrics are enabled
    if config[CONF_METRICS]:
        cg.add_build_flag("-DDDP_METRICS")

    # Enable wake_loop_threadsafe for low-latency wakeup (ESPHome 2025.11+)
    try:
        from esphome.components import socket
        socket.require_wake_loop_threadsafe()
    except AttributeError:
        # Backward compatibility: not available in older ESPHome
        pass


# Register ddp_light_effect as an addressable light effect
@register_addressable_effect(
    "ddp_light_effect",
    DdpLightEffect,
    "DDP Stream",
    {
        cv.GenerateID(CONF_DDP_ID): cv.use_id(DdpComponent),
        cv.Optional(CONF_STREAM): cv.int_range(
            min=1, max=249
        ),  # Optional - auto-generates if not specified
        cv.Optional(CONF_BRIGHTNESS_METHOD, default="max"): cv.enum(
            BRIGHTNESS_METHODS, lower=True
        ),  # RGB to brightness conversion method for monochromatic lights
    },
)
async def ddp_light_effect_to_code(config, effect_id):
    """Register a light effect renderer"""
    parent = await cg.get_variable(config[CONF_DDP_ID])

    # Auto-generate stream ID if not specified (using shared registry)
    if CONF_STREAM in config:
        stream_id = config[CONF_STREAM]
        register_stream_id(stream_id)
    else:
        stream_id = allocate_stream_id()

    effect = cg.new_Pvariable(effect_id, config[CONF_NAME])
    cg.add(effect.set_parent(parent))
    cg.add(effect.set_stream_id(stream_id))
    cg.add(effect.set_brightness_method(config[CONF_BRIGHTNESS_METHOD]))

    # Register with parent DDP component
    cg.add(parent.register_renderer(effect))

    return effect
