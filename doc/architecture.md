# Source code documentation

## Class overview

### UwbComponent

Class `UwbComponent` implements `esphome::Component` and receives `setup()` and `loop()` calls from ESPHome framework.

In `setup()` it processes the ESPHome YAML configuration calls from Python code, instantiates either `UwbAnchorDevice` or `UwbTagDevice` (`Dw3000Device` sub classes) according to `role`, sets any configured sensors on the instance and calls `setup()` on it.

`loop()` calls the `loop()` of the `Dw3000Device` subclass instance.

```mermaid
classDiagram
  direction TB
  class `esphome::Component` {
    setup()
    loop()
  }
  class UwbComponent {
    setup()
    loop()
  }
  `esphome::Component` <|-- UwbComponent : implements
  UwbComponent *-- UwbAnchorDevice : instantiates
  UwbComponent *-- UwbTagDevice : instantiates
  note for UwbComponent "depending on role:<br>UwbAnchorDevice<br>-or- UwbTagDevice"
  note for UwbComponent "calls from:<br>__init__.py<br>button.py<br>number.py<br>select.py<br>sensor.py<br>text_sensor.py"
```

### Dw3000Device

Class `Dw3000Device` is the base class of `UwbAnchorDevice` and `UwbTagDevice`.

It contains common `esphome::text::TextSensor`s, if configured in ESPHome YAML, and allows the sub classes to use those.

`Dw3000Device` performs common initialization of the DW3000 IC and implements common code used by `UwbAnchorDevice` and `UwbTagDevice`.

```mermaid
classDiagram
  direction TB
  class Dw3000Device {
    setup()
    loop()
  }
  Dw3000Device <|-- UwbAnchorDevice : implements
  Dw3000Device <|-- UwbTagDevice : implements
  class UwbAnchorDevice {
    setup()
    loop()
  }
    class UwbTagDevice {
    setup()
    loop()
  }
  Dw3000Device *-- `esphome::text::TextSensor`: contains
```

### UwbMessage

`UwbMessage` is the base class for the 3 types of UWB frames/messages used: `InitialMsg`, `ResponseMsg`, `FinalMsg`.

Refer to [How it works](how-it-works.md) for detailed description of messages.

```mermaid
classDiagram
  direction TB
  UwbMessage <|-- InitialMsg : realizes
  UwbMessage <|-- ResponseMsg : realizes
  UwbMessage <|-- FinalMsg : realizes

```

### UwbAnchorDevice

Class `UwbAnchorDevice` is the main implementation of role `anchor`.

It contains a complex state machine for handling incoming UWB ranging or antenna delay calibration requests from `UwbTagDevice`.

```mermaid
classDiagram
  direction TB
  UwbAnchorDevice "1" *-- InitialMsg : contains
  UwbAnchorDevice "1" *-- ResponseMsg : contains
  UwbAnchorDevice "1" *-- FinalMsg : contains
  UwbAnchorDevice "0..n" *-- `esphome::sensor::Sensor`: contains many
```

### UwbTagDevice

Class `UwbTagDevice` is the main implementation of role `tag`.

It contains a complex state machine for initiating UWB ranging or antenna delay calibration with all of configured `UwbAnchorDevice`s.

```mermaid
classDiagram
  direction TB
  UwbTagDevice "0..n" *-- `esphome::sensor::Sensor`: contains many
  UwbTagDevice *-- "1...n" UwbAnchorData : contains

  UwbTagDevice "1" *-- InitialMsg : contains
  UwbTagDevice "1" *-- ResponseMsg : contains
  UwbTagDevice "1" *-- FinalMsg : contains

  UwbTagDevice *-- Location : contains
  UwbTagDevice *-- AntDelayCalibration : contains
  UwbTagDevice *-- AntDelayCalibDeviceSelect: contains
  UwbTagDevice *-- AntDelayCalibDistanceNumber: contains
  UwbTagDevice *-- AntDelayCalibStartButton: contains

  `esphome::select::Select` <|-- AntDelayCalibDeviceSelect : implements
  `esphome::number::Number` <|-- AntDelayCalibDistanceNumber : implements
  `esphome::button::Button` <|-- AntDelayCalibStartButton : implements
```

### UwbAnchorData

An instance of `UwbTagDevice` contains an **array** of `UwbAnchorData`, which each hold data about one of the configured anchors like its statically configured latitude/longitude and device ID, as well as runtime measured distance and distance error estimate.

`UwbAnchorData` holds the distance sensor for reporting to Home Assistant.

### Location

An instance of `UwbTagDevice` contains a `Location` instance as a helper class to calculate the tag's WGS 84 position and the Haversine distance between two WGS 84 positions on earth.

`Location` implements all of the concept described in [How it works](how-it-works.md) regarding calculation of the `tag` location.

### AntDelayCalibration

An instance of `UwbTagDevice` contains an `AntDelayCalibration` instance as a helper class for performing the calculations needed during an antenna delay calibration. Refer to [Antenna delay calibration](ant-delay-calibration.md) for more details.

## Sequence of UWB messages between `tag` and `anchor`s

> **Note**
> Method names in below sequence do appear in code.
> Error handling is not shown.

```mermaid
zenuml
  @Control Tag as UwbTagDevice
  @Boundary Anchor as UwbAnchorDevice
  @Database HA as HomeAssistant

  par {
    // **time configurable as `ranging_interval_ms`**
    Tag->Tag.waitNextRangingInterval()
    Anchor->Anchor.prepareWaitRecvInitial()
    while (forever) {
      Anchor->Anchor.waitRecvInitial()
    }
  }

  // **For all configured `anchor`s of `tag`**
  forEach(Anchors) {
    Tag->Tag.waitNextAnchorRanging()
    Tag->Tag.prepareSendInitial() {
      Tag->Anchor:InitialMsg
      Tag->Tag.sentInitial()
    }
    Anchor->Anchor.recvdFrameInitial() {
        Anchor->Tag:ResponseMsg
        Anchor->Anchor.sentResponse()
    }
    // WAIT_RX_TIMEOUT_MS : **100 microseconds**
    while (WAIT_RX_TIMEOUT_MS) {
      Anchor->Anchor.waitRecvFinal()
    }
    Tag->Tag.recvdFrameResponse() {
      Tag->Anchor:FinalMsg
      Tag->Tag.sentFinal()
    }
    // WAIT_RX_TIMEOUT_MS : **100 microseconds**
    while (WAIT_RX_TIMEOUT_MS) {
      Tag->Tag.waitRecvFinal()
    }
    Anchor->Anchor.recvdFrameFinal() {
        Anchor->Tag:FinalMsg
        Anchor->Anchor.sentFinal()
    }
    Tag->Tag.recvdFrameFinal() {
      rangingDone()
    }
  }

  Tag->Tag.calculateLocationPrepare()
  // enum CalculationPhase:
  // ANCHOR_COMBINATIONS
  // COLLECT_CANDIDATES
  // FILTER_CANDIDATES
  // SELECT_BEST_CANDIDATE
  forEach(CalculationPhase) {
    Tag->Tag.calculateLocationInPhases(phase)
  }
  Tag->Tag.locationPostProcessing() {
    Tag->HA:publish_state(latitude)
    Tag->HA:publish_state(longitude)
    opt {
      Tag->HA:publish_state(error_estimate)
      Tag->HA:publish_state(anchors_in_use)
      forEach(anchor) {
        Tag->HA:publish_state(distance_tag_to_anchor)
      }
    }
  }
```
