# © Copyright 2025 Stuart Parmenter
# SPDX-License-Identifier: MIT

from esphome import codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, mdns
from esphome.components.mdns import MDNSComponent
from esphome.const import CONF_ID, CONF_PORT

DEPENDENCIES = ["network", "lvgl"]
AUTO_LOAD = ["binary_sensor", "mdns"]

ddp_ns = cg.esphome_ns.namespace("ddp_stream")
DdpStream = ddp_ns.class_("DdpStream", cg.Component)
DdpStreamOutput = ddp_ns.class_("DdpStreamOutput", cg.Component)

CONF_STREAMS = "streams"
CONF_CANVAS = "canvas"
CONF_STREAM = "stream"
CONF_RECEIVING = "receiving"
CONF_MDNS_ID = "mdns_id"

STREAM_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DdpStreamOutput),
    cv.Required(CONF_CANVAS): cv.string,
    cv.Optional(CONF_STREAM): cv.int_range(min=1, max=249),
    cv.Optional("width", default=-1): cv.int_,
    cv.Optional("height", default=-1): cv.int_,
    cv.Optional("back_buffers", default=None): cv.one_of(0, 1, 2, int=True),
    cv.Optional(CONF_RECEIVING): binary_sensor.binary_sensor_schema(
        binary_sensor.BinarySensor
    ),
})

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DdpStream),
    cv.GenerateID(CONF_MDNS_ID): cv.use_id(MDNSComponent),
    cv.Optional(CONF_PORT, default=4048): cv.port,
    cv.Optional(CONF_STREAMS, default=[]): cv.ensure_list(STREAM_SCHEMA),
    cv.Optional("back_buffers", default=0): cv.one_of(0, 1, 2, int=True),
})

async def _register_mdns_service(config, stream_configs):
    """Register mDNS service for DDP stream discovery (RFC 6763 compliant).

    Args:
        config: Component configuration dict
        stream_configs: List of dicts with 'ddp_id' (int) and 'width'/'height' (int, optional)
    """
    # Enable extra services support in mDNS component
    cg.add_define("USE_MDNS_EXTRA_SERVICES")

    mdns_comp = await cg.get_variable(config[CONF_MDNS_ID])

    # Build TXT records following RFC 6763 conventions:
    # - Keys are lowercase ASCII, ≤9 characters recommended
    # - txtvers: TXT record format version (increment only if format breaks compatibility)
    # - protovers: DDP protocol version (1-3, matches 2-bit version field in DDP header)
    txt_records = []
    txt_records.append(mdns.mdns_txt_record("txtvers", "1"))
    txt_records.append(mdns.mdns_txt_record("protovers", "1"))
    txt_records.append(mdns.mdns_txt_record("fmts", "rgb888,rgb565"))

    # Add DDP Destination ID information (using DDP protocol terminology)
    if stream_configs:
        ddp_id_list = []

        for stream_cfg in stream_configs:
            ddp_id = stream_cfg['ddp_id']
            ddp_id_list.append(ddp_id)

            # If explicit dimensions, add to TXT records (e.g., "id1=64x64")
            width = stream_cfg.get('width', -1)
            height = stream_cfg.get('height', -1)
            if width > 0 and height > 0:
                txt_records.append(
                    mdns.mdns_txt_record(f"id{ddp_id}", f"{width}x{height}")
                )

        # Sort DDP IDs numerically and advertise (e.g., "ids=1,2,3")
        ddp_id_list.sort()
        txt_records.append(mdns.mdns_txt_record("ids", ",".join(str(s) for s in ddp_id_list)))

    # Create and register mDNS service
    service = mdns.mdns_service(
        "_ddp",              # service name
        "_udp",              # protocol
        config[CONF_PORT],   # port
        txt_records          # TXT records
    )

    cg.add(mdns_comp.add_extra_service(service))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    cg.add(var.set_port(config[CONF_PORT]))
    cg.add(var.set_default_back_buffers(config["back_buffers"]))

    # Auto-generate DDP stream numbers if not provided (DDP spec: use 2-249, avoid reserved values)
    streams = config.get(CONF_STREAMS, [])
    used_stream_numbers = set()
    next_auto_stream = 2

    # DDP spec reserved values to skip during auto-generation
    reserved_values = {0, 1, 246, 250, 251, 254, 255}

    # First pass: collect explicitly specified DDP stream numbers
    for s in streams:
        if CONF_STREAM in s:
            used_stream_numbers.add(s[CONF_STREAM])

    # Second pass: assign auto-generated DDP stream numbers and create components
    # Build stream info list for mDNS registration
    stream_configs = []

    for s in streams:
        if CONF_STREAM in s:
            ddp_stream_num = s[CONF_STREAM]
        else:
            # Auto-generate DDP stream number in valid range (2-249), skipping reserved values
            while (next_auto_stream in used_stream_numbers or
                   next_auto_stream in reserved_values or
                   next_auto_stream > 249):
                next_auto_stream += 1
                if next_auto_stream > 249:
                    raise cv.Invalid("Too many streams: exceeded DDP spec limit (2-249)")
            ddp_stream_num = next_auto_stream
            used_stream_numbers.add(ddp_stream_num)
            next_auto_stream += 1

        # Create DdpStreamOutput component
        stream_component = cg.new_Pvariable(s[CONF_ID])
        await cg.register_component(stream_component, s)

        # Configure the stream component
        cg.add(stream_component.set_ddp_stream_id(ddp_stream_num))
        cg.add(stream_component.set_parent(var))

        # Configure the stream component
        canvas = s[CONF_CANVAS]
        # NOTE: avoid f-strings so the { } braces aren't interpreted by Python.
        getter = cg.RawExpression('[]() -> lv_obj_t* { return &id(%s); }' % canvas)
        cg.add(stream_component.set_canvas_getter(getter))
        cg.add(stream_component.set_size(s.get("width", -1), s.get("height", -1)))
        if s.get("back_buffers") is not None:
            cg.add(stream_component.set_back_buffers(s["back_buffers"]))

        # Configure optional receiving sensor
        if CONF_RECEIVING in s:
            sens = cg.new_Pvariable(s[CONF_RECEIVING][CONF_ID])
            await binary_sensor.register_binary_sensor(sens, s[CONF_RECEIVING])
            cg.add(stream_component.set_receiving_sensor(sens))

        # Collect stream info for mDNS registration
        stream_configs.append({
            'ddp_id': ddp_stream_num,
            'width': s.get("width", -1),
            'height': s.get("height", -1),
        })

    # Register mDNS service for network discovery
    await _register_mdns_service(config, stream_configs)