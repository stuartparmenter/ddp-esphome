# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

from esphome import codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT

AUTO_LOAD = ["lvgl"]

ddp_ns = cg.esphome_ns.namespace("ddp_stream")
DdpStream = ddp_ns.class_("DdpStream", cg.Component)

CONF_STREAMS = "streams"

STREAM_SCHEMA = cv.Schema({
    cv.Required("id"): cv.int_range(min=0, max=255),
    cv.Required("canvas_id"): cv.string,   # YAML id of the LVGL canvas
    cv.Required("width"): cv.int_range(min=1, max=255),
    cv.Required("height"): cv.int_range(min=1, max=255),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DdpStream),
    cv.Optional(CONF_PORT, default=4048): cv.port,
    cv.Optional(CONF_STREAMS, default=[]): cv.ensure_list(STREAM_SCHEMA),
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))

    # Defer binding: hand a pointer-getter lambda that will be called after LVGL creates widgets
    for s in config.get(CONF_STREAMS, []):
        canvas = s["canvas_id"]
        # NOTE: avoid f-strings so the { } braces aren't interpreted by Python.
        getter = cg.RawExpression('[]() -> lv_obj_t* { return &id(%s); }' % canvas)
        cg.add(var.add_stream_binding(s["id"], getter, s["width"], s["height"]))
