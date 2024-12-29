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
* Final :
  - Timestamps of reception/transmission of Initial, Response and Final message,
  - If Final message sent from `anchor` to `tag`: Distance calculated by `anchor`.

With the 3 timestamps included in Final message, both anchor and tag ('Dual-Sided') can calculate the Time-Of-Flight (TOF) and therefore the distance between each other, without the need to synchronize their internal clocks.

With the distance measured by the `anchor` transmitted with the Final message, and the `tag` own measured distance, the `tag` can estimate the error in meters for its calculated position.

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
MHR: Medium Access (MAC) Header</br>
MFR: Medium Access (MAC) Footer</br>
LSB: Least Significant Byte</br>
MSB: Most Significant Byte</br>
PAN: Personal Area Network</br>

### Initial message

Frame size is 14 bytes.

FunctionCode definition:
* RANGING: 0x21 (=default), FunctionData is not used
* ANT_DELAY_CALIBRATION: 0x22, FunctionData contains antenna delay, which shall be used by `anchor` from now on

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
80-95: "u16:FunctionData (MSB first)"
96-111: "u16: MFR Frame Checking Sequence"
```

### Response message

Frame size is 15 bytes.

FunctionCode definition:
* RANGING: 0x10 (=default), FunctionData is not used
* ANT_DELAY_CALIBRATION: 0x11, FunctionData is not used

```mermaid
---
title: "Response message"
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
88-103: "2 u8:FunctionData"
104-119: "u16: MFR Frame Checking Sequence"
```

### Final message

Frame size is 26 bytes.

FunctionCode definition:
* NO_DATA: 0x00 (=default), FunctionData is not used
* RANGING_DIST: 0x23, FunctionData contains distance calculated by `anchor`.<p>Note: During antenna delay calibration it may contain an implausible distance value (e.g. negative distance).</p>

```mermaid
---
title: "Final message"
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
80-95: "u16:FunctionData (MSB first)"
96-127: "u32:Initial timestamp (MSB first)"
128-159: "u32:Response timestamp (MSB first)"
160-191: "u32:Final timestamp (MSB first)"
192-207: "u16: MFR Frame Checking Sequence"
```

## Calculation of `tag` location

The `tag` calculates its WGS 84 location using the measured distances to all `anchor` devices with known WGS 84 locations.

**Example:**

Given are three anchors A<sub>1</sub>, A<sub>2</sub>, A<sub>3</sub> with known locations and therefore also known distances to each other: distA<sub>1</sub>A<sub>2</sub>, distA<sub>1</sub>A<sub>3</sub>, distA<sub>2</sub>A<sub>3</sub>.

The tag T measures distances using UWB ranging to the anchors: distA<sub>1</sub>T, distA<sub>2</sub>T, distA<sub>3</sub>T.

<p style="text-align:center;">
<img src="distances.svg" alt="Anchors and Tag distances" width="50%">
</p>

A distance between 2 locations on earth is calculated with the **Haversine Formula** https://en.wikipedia.org/wiki/Haversine_formula implemented in class [Location](../components/uwb/Location.cpp) method [`getHaversineDistance`](https://github.com/realzoulou/esphome-uwb-dw3000/blob/bef4ecd089f4dad634a4c27da354f3d116f4baf3/components/uwb/Location.cpp#L227).


### Find all distinct combinations (=pairs) of anchors A<sub>1</sub>...A<sub>n</sub>

In above example, the list of **distinct** anchor pairs is:

* (A<sub>1</sub>,A<sub>2</sub>)
* (A<sub>1</sub>, A<sub>3</sub>)
* (A<sub>2</sub>, A<sub>3</sub>)

'Distinct' means that (A<sub>2</sub>, A<sub>1</sub>), (A<sub>3</sub>, A<sub>1</sub>) and (A<sub>3</sub>, A<sub>2</sub>) are **not** added to the list.

### Find tag position candidates

* **For each** pair of anchors:<p>Find the (usually) 2 positions `S` and `S'` where 2 circles **intersect**, the circle center being the anchor position and the circle radius being the distance to the tag.</p>

The list of all `S` and `S'` forms the list of position **candidates**.

| Anchor pair | Example |
| :-: | :-: |
| (A<sub>1</sub>, A<sub>2</sub>) | <img src="circle-intersects-A1-A2.svg" alt="Circles intersections (A1, A2)" width="100%"> |
| (A<sub>1</sub>, A<sub>3</sub>) | <img src="circle-intersects-A1-A3.svg" alt="Circles intersections (A1, A3)" width="100%"> |
| (A<sub>2</sub>, A<sub>3</sub>) | <img src="circle-intersects-A2-A3.svg" alt="Circles intersections (A2, A3)" width="100%"> |

### Filter position candidates that are not within the anchors area

From the list of position candidates, remove those that are outside of the area within the anchors.

The area is a rectangle with bounds:
* top-left
  - x = `min` of longitude of all anchors
  - y = `max` of latitude of all anchors
* bottom-right
  - x = `max` of longitude of all anchors
  - y = `min` of latitude of all anchors

<p style="text-align:center;">
<img src="filter-area.svg" alt="Circles intersections (A1, A2)" width="60%">
</p>

### Select best-matching `tag` position

From the now reduced list of position candidates, select the 'best-matching' one, where the **sum of distances** from anchors to the tag is at **minimum**.

For above example this is trivial: `S` is the only position.

It resulted as being always one of the 2 circle intersection points for all anchor pairs.<br>
This stems from unrealisticly (!) accurate distance measurements from tag to anchors in above example.

In **reality**, the distance measurements of the tag are either too short, or too large, resulting in a list of position candidates like in below example.

<p style="text-align:center;">
<img src="realistic.svg" alt="Realistic position candidates" width="60%">
</p>

In order to select the **best-matching** from the list of candidates:

* **For each** position candidate:<p>Sum up the distances from the candidate to all anchors.</p>
* The candidate with the **minimal** summed distance is considered the `tag` position.

<p style="text-align:center;">
<img src="select-best.svg" alt="Best match" width="60%">
</p>

### Position error estimate

After the best-matching position candidate is selected, the position error estimate in meters is calculated for this position using the distance error estimates collected during exchange of the Final messages between anchor and tag.

During UWB ranging between tag and anchor, the anchor measured `D` as distance to tag, while the tag measured `D'`.<br>
The **distance** error estimate between an anchor and the tag is:
* `half` of `|D - D'|`

The **position** error estimate of the `tag` position is:
* **square root** of the **sum** of distance error estimates for all anchors

The position error estimate is an optional `sensor` in ESPHome YAML configuration. See `error_estimate` in [Full Documentation](full-documentation.adoc).
