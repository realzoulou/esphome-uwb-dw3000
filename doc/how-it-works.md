# How it works

The `tag` device is performing UWB ranging with the `anchor` devices, calculates its position using the known positions of the `anchor` devices and communicates its position to Home Assistant over WiFi.

```mermaid
---
title: System view
---
flowchart LR
  node_1[["`**Tag**`"]]
  node_2(["Home Assistant"])
  node_3[["Anchor 1"]]
  node_4[["Anchor 2"]]
  node_5[["`Anchor _n_`"]]
  node_3 -- "UWB" --- node_1
  node_4 -- "UWB" --- node_1
  node_5 -- "UWB" --- node_1
  node_1 -- "WiFi" --- node_2
```

## UWB ranging

Every `ranging_interval_ms` (configurable, default 5 s) the `tag` iterates over all configured `anchor` devices and performs UWB ranging.

UWB ranging is implemened as Dual-Sided Two-Way-Ranging (DS-TWR) with 4 messages. It does not use encryption (non-STS). Following 3 types of messages are used:

* Initial : Triggers `anchor` to return a Response.
* Response : Response back to `tag`.
* Final : Carries timestamps of reception/transmission of Initial, Response and Final message.

With the three timestamps included in Final message, both anchor and tag ('Dual-Sided') can calculate the Time-Of-Flight (TOF) and therefore the distance between each other, without any need to synchronize their internal clocks.

All messages follow IEEE 802.15.4 UWB standard, but are proprietary in their payloads.

Configuration of the DW3000 IC:

* Channel 5 (6.5 GHz)
* Data rate 6.81 Mb/s

### Message sequence

```mermaid
sequenceDiagram
  participant tag as tag
  participant anchor1 as anchor 1
  participant anchor2 as anchor 2
  participant anchorN as anchor n
  participant HA as Home Assistant
  loop every ranging_interval_ms (5 sec)
    rect rgba(0, 0, 255, 0.1)
      loop max 3 attempts
        tag ->> anchor1: Initial
        anchor1 ->> tag: Response
        tag ->> anchor1: Final
        anchor1 ->> tag: Final
      end
    tag --> tag: detect implausible<br/>distance or outliers
    end
    rect rgba(0, 255, 0, 0.1)
      loop max 3 attempts
        tag ->> anchor2: Initial
        anchor2 ->> tag: Response
        tag ->> anchor2: Final
        anchor2 ->> tag: Final
        tag --> tag: detect implausible<br/>distance or outliers
      end
    end
    rect rgba(255, 0, 0, 0.1)
    loop max 3 attempts
      tag ->> anchorN: Initial
      anchorN ->> tag: Response
      tag ->> anchorN: Final
      anchorN ->> tag: Final
    end
    tag --> tag: detect implausible<br/>distance or outliers
    end
    tag --> tag: calculate position<br/>using last known good<br/>distances to anchors
    rect rgba(255, 3, 159, 0.1)
      tag ->>HA: report latitude/longitude sensors
    end
  end
```

## UWB frames/messages

**Legend**</br>
MHR: MAC Header</br>
MFR: MAC Footer</br>
LSB: Least Significant Byte</br>
PAN: Personal Area Network

### Initial message

Frame size is 14 bytes.

```mermaid
---
title: "Initial message"
config:
    packet:
        bitsPerRow: 32
        showBits: true
---
packet-beta
0-15: "u16:MHR Frame Control (LSB first)"
16-23: "u8:MHR Sequence Number"
24-39: "u16:MHR PAN identifier (0xDECA, LSB first)"
40-47: "u8:Target Device ID"
48-55: "u8:Source Device ID"
56-63: "u8:reserved"
64-71: "u8:reserved"
72-79: "u8:FunctionCode"
80-95: "u16:FunctionData"
96-111: "u16: MFR Frame Checking Sequence"
```

### Response message

Frame size is 15 bytes.

```mermaid
---
title: "Initial message"
config:
    packet:
        bitsPerRow: 32
        showBits: true
---
packet-beta
0-15: "u16:MHR Frame Control (LSB first)"
16-23: "u8:MHR Sequence Number"
24-39: "u16:MHR PAN identifier (0xDECA, LSB first)"
40-47: "u8:Target Device ID"
48-55: "u8:Source Device ID"
56-63: "u8:reserved"
64-71: "u8:reserved"
72-79: "u8:FunctionCode"
80-87: "u8:FunctionDataLen"
88-103: "FunctionData"
104-119: "u16: MFR Frame Checking Sequence"
```

### Final message

Frame size is 26 bytes.

```mermaid
---
title: "Initial message"
config:
    packet:
        bitsPerRow: 32
        showBits: true
---
packet-beta
0-15: "u16:MHR Frame Control (LSB first)"
16-23: "u8:MHR Sequence Number"
24-39: "u16:MHR PAN identifier (0xDECA, LSB first)"
40-47: "u8:Target Device ID"
48-55: "u8:Source Device ID"
56-63: "u8:reserved"
64-71: "u8:reserved"
72-79: "u8:FunctionCode"
80-95: "FunctionData"
96-127: "Initial timestamp"
128-159: "Response timestamp"
160-191: "Final timestamp"
192-207: "u16: MFR Frame Checking Sequence"
```
