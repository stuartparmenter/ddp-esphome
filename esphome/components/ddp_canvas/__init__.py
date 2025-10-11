# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

from esphome import codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.components.ddp import register_stream_id, allocate_stream_id
from esphome.const import CONF_ID

CODEOWNERS = ["@stuartparmenter"]
DEPENDENCIES = ["ddp", "lvgl"]
AUTO_LOAD = ["binary_sensor"]
MULTI_CONF = True

ddp_ns = cg.esphome_ns.namespace("ddp")
DdpComponent = ddp_ns.class_("DdpComponent", cg.Component)
DdpCanvas = ddp_ns.class_("DdpCanvas", cg.Component)

# Configuration keys
CONF_DDP_ID = "ddp_id"
CONF_STREAM = "stream"
CONF_CANVAS = "canvas"
CONF_BACK_BUFFERS = "back_buffers"
CONF_RECEIVING = "receiving"

# Canvas renderer schema
CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DdpCanvas),
    cv.GenerateID(CONF_DDP_ID): cv.use_id(DdpComponent),
    cv.Optional(CONF_STREAM): cv.int_range(min=1, max=249),  # Optional now!
    cv.Required(CONF_CANVAS): cv.string,
    cv.Optional("width", default=-1): cv.int_,
    cv.Optional("height", default=-1): cv.int_,
    cv.Optional(CONF_BACK_BUFFERS, default=2): cv.one_of(0, 1, 2, int=True),
    cv.Optional(CONF_RECEIVING): binary_sensor.binary_sensor_schema(
        binary_sensor.BinarySensor
    ),
}).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    """Register a canvas renderer"""
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Set parent DDP component
    parent = await cg.get_variable(config[CONF_DDP_ID])
    cg.add(var.set_parent(parent))

    # Auto-generate stream ID if not specified (using shared registry)
    if CONF_STREAM in config:
        stream_id = config[CONF_STREAM]
        register_stream_id(stream_id)
    else:
        stream_id = allocate_stream_id()

    # Set stream ID
    cg.add(var.set_stream_id(stream_id))

    # Set canvas getter (lambda to resolve LVGL object at runtime)
    canvas = config[CONF_CANVAS]
    getter = cg.RawExpression(f'[]() -> lv_obj_t* {{ return &id({canvas}); }}')
    cg.add(var.set_canvas_getter(getter))

    # Set dimensions
    cg.add(var.set_size(config.get("width", -1), config.get("height", -1)))

    # Set back buffers
    cg.add(var.set_back_buffers(config[CONF_BACK_BUFFERS]))

    # Configure optional receiving sensor
    if CONF_RECEIVING in config:
        sens = cg.new_Pvariable(config[CONF_RECEIVING][CONF_ID])
        await binary_sensor.register_binary_sensor(sens, config[CONF_RECEIVING])
        cg.add(var.set_receiving_sensor(sens))

    # Register with parent DDP component (at codegen time so mDNS has complete info)
    cg.add(parent.register_renderer(var))

    return var
