= Makerfabs ESP32 UWB DW3000 ESPHome component
:reproducible:
:toc:
:toclevels: 1
// Enable for testing 'env-github'
//:env-github:
// Usage:
// ifdef::env-github[]
// ...
// endif::[]

== Abstract

This repository contains an https://esphome.io/[ESPHome] component `uwb` that allows using https://www.makerfabs.com/esp32-uwb-dw3000.html[Makerfabs ESP32 UWB DW3000] devices and setup for real-time precise localization of a device.

For **practical use**, a minimum of four DW3000 devices are needed:

* 1x DW3000 as `tag`: **Moving** device to be located,
* 3x DW3000 as `anchor`: **Fixed** device with known location.

Using UWB distance/range measurements between the tag with (at least) 3 anchors, the location of the tag can be calculated.
The tag reports its calculated location to a https://www.home-assistant.io/[Home Assistant] instance.

NOTE: Apple AirTags are **not** supported!

== Quick start
For a quick introduction to the essence of ESPHome and Home Assistant configuration, refer to xref:doc/quick_start.adoc[Quick start]

== How it works
For detailed discussion on how `tag` and `anchor` communicate to each other and also to Home Assistant, refer to xref:doc/how-it-works.md[How it works]

== Full documentation
For detailed documentation of all configuration possibilities in `uwb` component, for both `tag` and `anchor`, refer to xref:doc/full-documentation.adoc[Full documentation]

== Antenna delay calibration
For a description on how to perform antenna delay calibration using a `tag` and an `anchor`, and how this works in detail, see xref:doc/ant-delay-calibration.md[Antenna delay calibration]

== Source code documentation
An overview about the C++ classes and their purpose is given in xref:doc/architecture.md[Source code documentation]

== License

The author's sources are provided under `Apache-2.0` license.

This repository contains a copy of https://github.com/Makerfabs/Makerfabs-ESP32-UWB-DW3000[Makerfabs-ESP32-UWB-DW3000] folder `Dw3000` licensed under `Apache-2.0`.

== Disclaimer

The original author of this repository has no business relationship with Makerfabs and does not intend to promote the use of Makerfabs ESP32 UWB DW3000 devices.
