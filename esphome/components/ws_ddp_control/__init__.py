# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

from esphome import codegen as cg, config_validation as cv
from esphome.const import CONF_ID, CONF_URL
from esphome.components.esp32 import add_idf_component

# We require the base sink + LVGL so we can query canvas size
DEPENDENCIES = ["network", "lvgl", "ddp_stream"]

ws_ns = cg.esphome_ns.namespace("ws_ddp_control")
WsDdpControl = ws_ns.class_("WsDdpControl", cg.Component)

ddp_ns = cg.esphome_ns.namespace("ddp_stream")
DdpStream = ddp_ns.class_("DdpStream", cg.Component)

CONF_WS_HOST   = "ws_host"
CONF_WS_PORT   = "ws_port"
CONF_DEVICE_ID = "device_id"
CONF_DDP       = "ddp"
CONF_OUTPUTS   = "outputs"

OUTPUT_SCHEMA = cv.Schema({
    cv.Required("id"): cv.int_range(min=0, max=255),
    # Width/height optional; if omitted we auto-detect from the canvas via ddp_stream
    cv.Optional("width"): cv.int_range(min=1, max=4096),
    cv.Optional("height"): cv.int_range(min=1, max=4096),

    # Orchestration defaults (all can be changed at runtime via setters)
    cv.Optional("src"): cv.string,
    cv.Optional("pace"): cv.int_range(min=0, max=240),
    cv.Optional("ema"): cv.float_range(min=0.0, max=1.0),
    # expand: 0=never, 1=auto, 2=force
    # Accept ints and aliases, plus numeric strings ("0","1","2")
    cv.Optional("expand"): cv.Any(
        cv.int_range(min=0, max=2),
        cv.one_of("never", "auto", "force", "0", "1", "2", lower=True),
    ),
    cv.Optional("loop"): cv.boolean,
    cv.Optional("hw"): cv.one_of(
        "auto", "none", "cuda", "qsv", "vaapi", "videotoolbox", "d3d11va", lower=True
    ),
    # Per-output transport format. 'rgb565' will auto-pick endian from the sink.
    cv.Optional("format"): cv.one_of(
        "rgb888", "rgb565", "rgb565le", "rgb565be", lower=True
    ),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(CONF_ID): cv.declare_id(WsDdpControl),
    # URL override (optional)
    cv.Optional(CONF_URL, default=""): cv.templatable(cv.string),
    cv.Optional(CONF_WS_HOST): cv.templatable(cv.string),
    cv.Optional(CONF_WS_PORT, default=8788): cv.int_range(min=1, max=65535),
    cv.Optional(CONF_DEVICE_ID, default="unknown"): cv.templatable(cv.string),

    # Orchestration
    cv.Required(CONF_DDP): cv.use_id(DdpStream),
    cv.Optional(CONF_OUTPUTS, default=[]): cv.ensure_list(OUTPUT_SCHEMA),
}).extend(cv.COMPONENT_SCHEMA)

def _templ(expr):
    return cg.templatable(expr, [], cg.std_string)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    # Networking / socket config
    cg.add(var.set_url(await _templ(config.get(CONF_URL, ""))))
    if CONF_WS_HOST in config:
        cg.add(var.set_ws_host(await _templ(config[CONF_WS_HOST])))
    cg.add(var.set_ws_port(config.get(CONF_WS_PORT, 8788)))
    cg.add(var.set_device_id(await _templ(config.get(CONF_DEVICE_ID, "unknown"))))

    # Bind ddp sink
    ddp = await cg.get_variable(config[CONF_DDP])
    cg.add(var.set_ddp(ddp))

    # Outputs with optional explicit w/h (pass -1 to auto-detect)
    for s in config[CONF_OUTPUTS]:
        exp = s.get("expand", "auto")
        # normalize expand → 0/1/2
        if isinstance(exp, (int, float)):
            exp_i = max(0, min(2, int(exp)))
        else:
            e = str(exp).strip().lower()
            if e in ("never", "0"):   exp_i = 0
            elif e in ("auto",  "1"): exp_i = 1
            elif e in ("force", "2"): exp_i = 2
            else:                     exp_i = 1

        cg.add(var.add_output_basic(
            s["id"],
            s.get("width", -1),
            s.get("height", -1)
        ))

        if "src" in s: cg.add(var.set_src(s["id"], s["src"]))
        if "pace" in s: cg.add(var.set_pace(s["id"], s["pace"]))
        if "ema" in s: cg.add(var.set_ema(s["id"], s["ema"]))
        if exp_i is not None: cg.add(var.set_expand(s["id"], exp_i))
        if "loop" in s: cg.add(var.set_loop(s["id"], s["loop"]))
        if "hw" in s: cg.add(var.set_hw(s["id"], s["hw"]))
        if "format" in s: cg.add(var.set_format(s["id"], s["format"]))

    # Ensure esp_websocket_client is linked
    add_idf_component(name="espressif/esp_websocket_client", ref="1.5.0")
