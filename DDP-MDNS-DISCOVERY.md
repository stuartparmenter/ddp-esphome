# DDP mDNS/Zeroconf Discovery Specification

**Version:** 1.0
**Last Updated:** 2025-10-08
**Status:** Draft

## Overview

This document specifies how DDP (Distributed Display Protocol) devices advertise themselves on local networks using mDNS/Zeroconf (RFC 6763) for automatic discovery.

This is an **extension** to the DDP protocol specification and provides a standardized discovery mechanism.

## Motivation

The DDP protocol specification mentions discovery via JSON packets to special destination IDs (246, 250, 251) but doesn't standardize the JSON format—it only provides vendor-specific examples. In practice, this discovery method is rarely implemented.

mDNS/Zeroconf offers a simpler, more standardized alternative:
- Zero-configuration automatic discovery
- Operating system integration
- Standard tooling across platforms
- No application-level protocol knowledge required
- Browser-based discovery support

## Service Type

DDP devices advertise as:

```
_ddp._udp.local.
```

### Components:
- **Service Name:** `_ddp` (DDP protocol identifier)
- **Protocol:** `_udp` (UDP transport)
- **Domain:** `local.` (link-local multicast DNS)

### Instance Name

The instance name is the user-visible device name, typically the device hostname:

```
<hostname>._ddp._udp.local.
```

Example: `apollo-display._ddp._udp.local.`

## Port

DDP devices listen on **UDP port 4048** (as specified in DDP protocol).

## TXT Records

TXT records follow RFC 6763 conventions and provide device capabilities and available endpoints.

### Required TXT Records

| Key | Value | Description |
|-----|-------|-------------|
| `txtvers` | `1` | TXT record format version (RFC 6763 standard) |
| `protovers` | `1` | DDP protocol version (1-3, matches DDP header 2-bit version field) |
| `fmts` | `rgb888,rgb565` | Supported pixel formats (comma-separated) |

### Optional TXT Records

| Key | Value | Description | Example |
|-----|-------|-------------|---------|
| `ids` | `1,2,3` | Available DDP Destination IDs (comma-separated) | `ids=1,2,3,4` |
| `id{N}` | `{width}x{height}` | Dimensions for DDP ID {N} | `id1=64x64` |

### TXT Record Rules

Following RFC 6763 best practices:

1. **Keys:** Lowercase ASCII, ≤9 characters recommended
2. **Format:** `key=value` (no spaces around `=`)
3. **Case:** Keys are case-insensitive but lowercase preferred
4. **Uniqueness:** Each key appears only once

### Version Semantics

#### txtvers (TXT Record Format Version)
- Current version: `1`
- Increment **only** if TXT record format becomes incompatible
- Example: Changing key names, removing required keys

#### protovers (DDP Protocol Version)
- Matches DDP protocol header version field (2 bits: 1-3)
- Current DDP protocol version: `1`
- Aligns with DDP spec: "this document specifies version 1 (01)"

## Pixel Format Codes

The `fmts` TXT record lists supported DDP data types using the following identifiers:

| Format | Description | DDP Type Byte | Notes |
|--------|-------------|---------------|-------|
| `rgb888` | 24-bit RGB (8 bits per channel) | `0x0B` | Per DDP spec: TTT=001 (RGB), SSS=011 (8-bit). Historical note: Many implementations use `0x01` instead. |
| `rgb565` | 16-bit RGB (5-6-5 bit distribution) | `0x61` or `0x62` | Extension to DDP spec (not standard). Little-endian (`0x61`) or big-endian (`0x62`) byte order. |
| `rgbw` | 32-bit RGBW (8 bits per channel) | `0x1B` | Per DDP spec: TTT=011 (RGBW), SSS=011 (8-bit). Used by WLED and compatible devices. |

Multiple formats are comma-separated: `fmts=rgb888,rgb565`

**Note on RGB565:** The RGB565 format is an extension not in the original DDP specification. It's useful for embedded devices (ESP32) where displays typically run in 16-bit color mode, reducing bandwidth by ~33% compared to RGB888.

## DDP Destination IDs

The `ids` TXT record advertises which DDP Destination IDs (byte 3 of DDP header) this device accepts.

### ID Ranges (per DDP spec):
- `0` - Reserved
- `1` - Default output device
- `2-249` - Custom IDs
- `246` - JSON control (read/write)
- `250` - JSON config (read/write)
- `251` - JSON status (read only)
- `254` - DMX transit
- `255` - All devices (broadcast)

### Format:
Comma-separated list of integers: `ids=1,2,3`

### Dimension Records

If the device knows the dimensions for a DDP ID at configuration time, it can advertise them:

```
id1=64x64
id2=128x32
```

**Format:** `id{N}={width}x{height}`

**Note:** Dimensions are optional. Many devices determine dimensions dynamically from bound display objects at runtime.

## Discovery Examples

### Python (using zeroconf)

```python
from zeroconf import ServiceBrowser, ServiceListener, Zeroconf
import socket

class DdpDiscoveryListener(ServiceListener):
    def add_service(self, zeroconf, service_type, name):
        info = zeroconf.get_service_info(service_type, name)
        if info:
            ip = socket.inet_ntoa(info.addresses[0])
            port = info.port

            # Parse TXT records
            props = {}
            for key, value in info.properties.items():
                props[key.decode()] = value.decode()

            print(f"Found DDP device: {name}")
            print(f"  Address: {ip}:{port}")
            print(f"  Formats: {props.get('fmts', 'unknown')}")

            ids = props.get('ids', '')
            if ids:
                print(f"  IDs: {ids}")
                for ddp_id in ids.split(','):
                    if f'id{ddp_id}' in props:
                        print(f"    {ddp_id}: {props[f'id{ddp_id}']}")

# Start discovery
zc = Zeroconf()
listener = DdpDiscoveryListener()
browser = ServiceBrowser(zc, "_ddp._udp.local.", listener)

try:
    input("Press enter to exit...\n\n")
finally:
    zc.close()
```

### Node.js (using bonjour-service)

```javascript
const Bonjour = require('bonjour-service');

const instance = new Bonjour.Bonjour();

instance.find({ type: 'ddp', protocol: 'udp' }, (service) => {
  console.log('Found DDP device:', service.name);
  console.log('  Address:', service.addresses[0], ':', service.port);
  console.log('  Protocol Version:', service.txt.protovers || 'unknown');
  console.log('  Formats:', service.txt.fmts || 'unknown');
  console.log('  DDP IDs:', service.txt.ids || 'none');

  // Check for dimension records
  const ddpIds = service.txt.ids?.split(',') || [];
  ddpIds.forEach(id => {
    const dimKey = `id${id}`;
    if (service.txt[dimKey]) {
      console.log(`  ID ${id}:`, service.txt[dimKey]);
    }
  });
});
```

### Command Line Tools

**Linux (avahi-utils):**
```bash
avahi-browse -r _ddp._udp
```

**macOS:**
```bash
dns-sd -B _ddp._udp
```

**Windows (dns-sd.exe from Bonjour SDK):**
```cmd
dns-sd -B _ddp._udp
```

## Example Advertisement

A complete mDNS advertisement for a DDP device with multiple outputs:

```
apollo-display._ddp._udp.local.
  Port: 4048
  TXT Records:
    txtvers=1
    protovers=1
    fmts=rgb888,rgb565
    ids=1,2,3
    id1=64x64
    id2=128x32
    id3=320x240
```

## Why mDNS Instead of DDP JSON Discovery?

The DDP protocol specification mentions JSON-based discovery using special destination IDs (246=control, 250=config, 251=status), but doesn't standardize the JSON format. The spec only provides vendor-specific examples, and in practice this discovery method is rarely implemented.

mDNS/Zeroconf provides several advantages:
- **Standardized**: Well-defined RFC 6763 specification
- **Simple**: No application-level protocol knowledge required
- **Universal**: Works with standard OS tools and libraries
- **Efficient**: Leverages existing multicast DNS infrastructure

## Network Considerations

### Multicast Requirements
- mDNS requires multicast support on the network
- Most home/office LANs support multicast
- Some enterprise networks filter multicast traffic

### WiFi Limitations
- Multicast can be unreliable on WiFi networks
- For critical applications, use unicast with known IP addresses
- Consider NTP synchronization for multi-device displays (per DDP spec)

## Security

mDNS discovery provides **no security**:
- Devices are advertised to entire local network
- No authentication or encryption
- Suitable for trusted LANs only

This aligns with DDP protocol's security stance:
> "None provided. Many lighting systems will run on private networks not connected to the Internet."

## Future Extensions

Possible future TXT record additions (backward-compatible, no version increment required):

- `maxfps` - Maximum supported frame rate
- `buffers` - Number of frame buffers
- `timecode` - Timecode support (boolean)
- `storage` - Storage support for local playback
- `ntp` - NTP synchronization support

**Note:** Adding optional TXT records is backward-compatible. Only increment `txtvers` if the format becomes incompatible (e.g., removing required fields or changing field semantics).

## References

- [RFC 6763 - DNS-Based Service Discovery](https://www.rfc-editor.org/rfc/rfc6763.html)
- [DDP Protocol Specification](http://www.3waylabs.com/ddp/)
- [Multicast DNS (mDNS) - RFC 6762](https://www.rfc-editor.org/rfc/rfc6762.html)

## Implementations

Known implementations of this specification:

- **lvgl-ddp-stream** - ESPHome component for LVGL displays ([GitHub](https://github.com/stuartparmenter/lvgl-ddp-stream))

### Related DDP Implementations

While these don't (yet) implement mDNS discovery, they're compatible DDP implementations:

- **[WLED](https://github.com/Aircoookie/WLED)** - Popular LED controller firmware with DDP receiver support (RGB888 and RGBW)
- **[LedFx](https://github.com/LedFx/LedFx)** - Real-time music visualization with DDP sender support

## Contributing

This specification is open for community input. Suggested changes should:
1. Maintain backward compatibility when possible
2. Follow RFC 6763 conventions
3. Align with DDP protocol terminology
4. Consider network efficiency

## License

This specification is released into the public domain for free use and implementation.

---

**Document History:**
- 2025-10-08: Version 1.0 - Initial specification
