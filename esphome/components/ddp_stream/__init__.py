# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

from esphome import codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_PORT

DEPENDENCIES = ["network", "lvgl"]

ddp_ns = cg.esphome_ns.namespace("ddp_stream")
DdpStream = ddp_ns.class_("DdpStream", cg.Component)

CONF_STREAMS = "streams"

STREAM_SCHEMA = cv.Schema({
    cv.Required("id"): cv.int_range(min=0, max=255),
    cv.Required("canvas_id"): cv.string,   # YAML id of the LVGL canvas
    cv.Optional("width", default=-1): cv.int_,
    cv.Optional("height", default=-1): cv.int_,
    cv.Optional("back_buffers", default=None): cv.one_of(0, 1, 2, int=True),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DdpStream),
    cv.Optional(CONF_PORT, default=4048): cv.port,
    cv.Optional(CONF_STREAMS, default=[]): cv.ensure_list(STREAM_SCHEMA),
    cv.Optional("back_buffers", default=0): cv.one_of(0, 1, 2, int=True),
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_default_back_buffers(config["back_buffers"]))

    # Defer binding: hand a pointer-getter lambda that will be called after LVGL creates widgets
    for s in config.get(CONF_STREAMS, []):
        canvas = s["canvas_id"]
        # NOTE: avoid f-strings so the { } braces aren't interpreted by Python.
        getter = cg.RawExpression('[]() -> lv_obj_t* { return &id(%s); }' % canvas)
        cg.add(var.add_stream_binding(s["id"], getter, s.get("width", -1), s.get("height", -1)))
        if s.get("back_buffers") is not None:
            cg.add(var.set_stream_back_buffers(s["id"], s["back_buffers"]))