# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

from esphome import codegen as cg
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID, CONF_URL, CONF_PORT
)
from esphome.components.esp32 import add_idf_component

ws_ns = cg.esphome_ns.namespace("ws_ddp_control")
WsDdpControl = ws_ns.class_("WsDdpControl", cg.Component)

# New optional fields; if url is omitted we build one from these
CONF_WS_HOST   = "ws_host"
CONF_WS_PORT   = "ws_port"
CONF_WIDTH     = "width"
CONF_HEIGHT    = "height"
CONF_OUT       = "out"
CONF_SRC       = "src"
CONF_PACE      = "pace"
CONF_EMA       = "ema"
CONF_EXPAND    = "expand"   # 0|1|2 or "never|auto|force"
CONF_LOOP      = "loop"
CONF_DDP_PORT  = "ddp_port"
CONF_HW        = "hw"       # "auto|cuda|qsv|vaapi|videotoolbox|none"

ITEM_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ID): cv.declare_id(WsDdpControl),
    cv.Optional(CONF_URL, default=""): cv.templatable(cv.string),

    cv.Optional(CONF_WS_HOST): cv.templatable(cv.string),
    cv.Optional(CONF_WS_PORT, default=8788): cv.int_range(min=1, max=65535),
    cv.Optional(CONF_WIDTH, default=64): cv.int_range(min=1, max=255),
    cv.Optional(CONF_HEIGHT, default=64): cv.int_range(min=1, max=255),
    cv.Optional(CONF_OUT, default=1): cv.int_range(min=0, max=255),
    cv.Optional(CONF_SRC, default=""): cv.templatable(cv.string),

    cv.Optional(CONF_PACE, default=0): cv.int_range(min=0, max=240),
    cv.Optional(CONF_EMA, default=0.0): cv.float_range(min=0.0, max=1.0),
    cv.Optional(CONF_EXPAND, default="auto"): cv.one_of(0,1,2,"never","auto","force", lower=True),
    cv.Optional(CONF_LOOP, default=True): cv.boolean,
    cv.Optional(CONF_DDP_PORT, default=4048): cv.port,
    cv.Optional(CONF_HW, default="auto"): cv.one_of("auto","none","cuda","qsv","vaapi","videotoolbox","d3d11va", lower=True),
}).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.All(cv.ensure_list(ITEM_SCHEMA))

def _templ(expr):
    return cg.templatable(expr, [], cg.std_string)

async def to_code(config):
    for conf in config:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)

        # keep existing url path working
        url_expr = await _templ(conf.get(CONF_URL, ""))
        cg.add(var.set_url(url_expr))

        # Pass structured fields down; the C++ will build URL if url is empty
        if CONF_WS_HOST in conf:
            cg.add(var.set_ws_host(await _templ(conf[CONF_WS_HOST])))
        cg.add(var.set_ws_port(conf.get(CONF_WS_PORT, 8788)))
        cg.add(var.set_width(conf.get(CONF_WIDTH, 64)))
        cg.add(var.set_height(conf.get(CONF_HEIGHT, 64)))
        cg.add(var.set_out_id(conf.get(CONF_OUT, 1)))
        if CONF_SRC in conf:
            cg.add(var.set_src(await _templ(conf[CONF_SRC])))

        cg.add(var.set_pace(conf.get(CONF_PACE, 0)))
        cg.add(var.set_ema(conf.get(CONF_EMA, 0.0)))
        # normalize expand into 0/1/2 for C++
        exp = conf.get(CONF_EXPAND, "auto")
        exp_i = 1 if isinstance(exp, str) and exp.lower()=="auto" else (2 if exp in (2,"force") else 0)
        cg.add(var.set_expand(exp_i))

        cg.add(var.set_loop(conf.get(CONF_LOOP, True)))
        cg.add(var.set_ddp_port(conf.get(CONF_DDP_PORT, 4048)))
        cg.add(var.set_hw(await _templ(conf.get(CONF_HW, "auto"))))

        # ensure esp_websocket_client (ESP‑IDF)
        add_idf_component(name="espressif/esp_websocket_client", ref="1.5.0")
