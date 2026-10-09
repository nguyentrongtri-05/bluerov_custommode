# ArduPilot Project

[![Discord](https://img.shields.io/discord/674039678562861068.svg)](https://ardupilot.org/discord)

[![Test Copter](https://github.com/ArduPilot/ardupilot/workflows/test%20copter/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_copter.yml) [![Test Plane](https://github.com/ArduPilot/ardupilot/workflows/test%20plane/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_plane.yml) [![Test Rover](https://github.com/ArduPilot/ardupilot/workflows/test%20rover/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_rover.yml) [![Test Sub](https://github.com/ArduPilot/ardupilot/workflows/test%20sub/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_sub.yml) [![Test Tracker](https://github.com/ArduPilot/ardupilot/workflows/test%20tracker/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_tracker.yml)

[![Test AP_Periph](https://github.com/ArduPilot/ardupilot/workflows/test%20ap_periph/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_sitl_periph.yml) [![Test Chibios](https://github.com/ArduPilot/ardupilot/workflows/test%20chibios/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_chibios.yml) [![Test Linux SBC](https://github.com/ArduPilot/ardupilot/workflows/test%20Linux%20SBC/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_linux_sbc.yml) [![Test Replay](https://github.com/ArduPilot/ardupilot/workflows/test%20replay/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_replay.yml)

[![Test Unit Tests](https://github.com/ArduPilot/ardupilot/workflows/test%20unit%20tests%20and%20sitl%20building/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_unit_tests.yml)[![test size](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_size.yml)

[![Test Environment Setup](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_environment.yml)

[![Cygwin Build](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/cygwin_build.yml) [![Macos Build](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot/actions/workflows/macos_build.yml)

[![Coverity Scan Build Status](https://scan.coverity.com/projects/5331/badge.svg)](https://scan.coverity.com/projects/ardupilot-ardupilot)

[![Test Coverage](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml/badge.svg?branch=master)](https://github.com/ArduPilot/ardupilot/actions/workflows/test_coverage.yml)

[![Autotest Status](https://autotest.ardupilot.org/autotest-badge.svg)](https://autotest.ardupilot.org/)

[![OpenSSF Best Practices](https://www.bestpractices.dev/projects/10598/badge)](https://www.bestpractices.dev/projects/10598)

## Custom Branch: BlueROV PosHold Distance / Contact

This repository contains a custom ArduSub flight mode for BlueROV-type vehicles working close to structures (e.g. bridge piers).

### Mode

- **Name:** `PosHoldDistance` (short name `PHDS`), implemented as `ModePosholdDist` (inherits `ModePoshold`).
- **Mode number:** reported as `7` (`CIRCLE`) so that QGroundControl can select it from its existing mode list (shown as "Circle"). `Mode::Number::POSHOLD_DIST` (`22`) also maps to it. The joystick button function `mode_circle` enters this mode. The stock `ModeCircle` is no longer reachable.
- **Requires** a position estimate (DVL through EKF3), same as `PosHold`. Depth and heading hold are unchanged from `PosHold`.

### Sensors

- **Forward Ping** on rangefinder instance 1: `RNGFND1_TYPE=10` (MAVLink), `RNGFND1_ORIENT=0` (forward). Samples with signal quality below `RNGFND_SQ_MIN` are ignored (`-1` = unknown is accepted).
- **DVL** velocity, used through the AHRS/EKF3 velocity estimate.

### Behaviour (`PHDS_ACTION`)

- **0 - Keep distance** (uses the Ping distance selected by `PHDS_USE_EKF`):
  - Between `PHDS_DIST_MIN` and `PHDS_DIST_MIN + 0.3 m`, forward speed is limited linearly to `PILOT_SPEED * (d - PHDS_DIST_MIN) / 0.3`.
  - Below `PHDS_DIST_MIN`, the vehicle reverses automatically at `100 cm/s per metre` of intrusion (max 50% of `PILOT_SPEED`), even if the pilot pushes forward.
  - If the distance is invalid (no Ping for 500 ms, or `<= 0.1 m`), forward motion is blocked; reverse and lateral are still allowed.
- **1 - Contact** (does not use Ping):
  - Normal `PosHold` while approaching.
  - If the pilot pushes forward, the position controller is pushing forward and the forward velocity stays below `PHDS_STALL_VEL` for `PHDS_STALL_T` seconds, contact is declared.
  - In contact, the position controller is stopped (its accumulated thrust is discarded) and a fixed forward thrust `PHDS_PUSH` is applied. Lateral is manual (no position hold on that axis while in contact).
  - Pulling the stick back (or setting `PHDS_ACTION=0`, or changing mode) releases contact; `PosHold` restarts from the current position.

### Distance filters (`PHDS_USE_EKF`)

| Value | Source | Notes |
|---|---|---|
| 0 | Raw | Ping distance as received |
| 1 | KF | Ping only, states distance/velocity, predicted with horizontal acceleration from AHRS |
| 2 | EKF | Same as KF plus forward velocity from AHRS (DVL). Linear Kalman filter despite the name |

All three are computed continuously; only the selected one is used for control. Filters are updated only when a new Ping sample arrives, re-initialised after 500 ms without Ping, and use a 5-sigma innovation gate: a sample much closer than predicted resets the filter immediately, a sample much farther is rejected (reset after 5 consecutive rejections). Noise constants are hard-coded in `libraries/AP_CustomEKF` and should be tuned from real Ping/DVL logs.

### Parameters

| Parameter | Default | Description |
|---|---|---|
| `PHDS_ACTION` | 0 | 0: keep distance, 1: contact |
| `PHDS_USE_EKF` | 0 | Distance source: 0 Raw, 1 KF, 2 EKF |
| `PHDS_DIST_MIN` | 0.5 m | Keep-distance threshold |
| `PHDS_STALL_VEL` | 0.05 m/s | Contact: forward speed below this while pushing counts as blocked |
| `PHDS_STALL_T` | 1.0 s | Contact: blocked time before contact is declared |
| `PHDS_PUSH` | 0.08 | Contact: fixed forward thrust while in contact (0..0.3) |

### Telemetry and logging

- `NAMED_VALUE_INT` at 5 Hz: `PHDS_Raw`, `PHDS_KF`, `PHDS_EKF` (cm, `-1` = invalid).
- GCS text messages: `PHDS: contact, push N%` and `PHDS: contact released`.
- DataFlash message `PHDS` at 10 Hz: `Sel, Raw, KF, EKF, KFv, EKFv, RawOk, FiltOk, Cont`.

### SITL testing

```bash
./waf configure --board sitl && ./waf sub
./Tools/autotest/sim_vehicle.py -v ArduSub --console --map
# in MAVProxy: param set RNGFND1_TYPE 10 ; param set RNGFND1_ORIENT 0 ; reboot
python3 fake_wall_ping.py --wall 5 --noise 0.1   # virtual wall in front of the vehicle
```

`fake_wall_ping.py` sends `DISTANCE_SENSOR` computed from the vehicle position and heading to a virtual wall. Enter the mode before driving towards the wall (SITL has no physical obstacle; behind the wall the script reports the minimum range). Contact mode can be exercised with a strong water current against the heading (`SIM_WIND_SPD`, `SIM_WIND_DIR`).

### Status and known limitations

- Keep-distance mode and the three filters were tested in SITL. With `PHDS_DIST_MIN=0.5` and about 0.75 m/s approach speed, the vehicle stopped at 0.12-0.2 m (overshoot of 0.3-0.4 m) because the 0.3 m slow-down zone is shorter than the stopping distance.
- Contact mode compiles but has not yet been tested in SITL or in water. A strong current against the heading can trigger contact without a surface.

### Implementation

- `ArduSub/mode_poshold_dist.cpp`, `ArduSub/mode.h`, `ArduSub/mode.cpp`
- `ArduSub/Parameters.cpp`, `ArduSub/Parameters.h`
- `libraries/AP_CustomEKF` (KF and EKF)
- `fake_wall_ping.py` (SITL virtual wall)

---

ArduPilot is the most advanced, full-featured, and reliable open source autopilot software available.
It has been under development since 2010 by a diverse team of professional engineers, computer scientists, and community contributors.
Our autopilot software is capable of controlling almost any vehicle system imaginable, from conventional airplanes, quad planes, multi-rotors, and helicopters to rovers, boats, balance bots, and even submarines.
It is continually being expanded to provide support for new emerging vehicle types.

## The ArduPilot project is made up of

- ArduCopter: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduCopter), [wiki](https://ardupilot.org/copter/index.html)

- ArduPlane: [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduPlane), [wiki](https://ardupilot.org/plane/index.html)

- Rover: [code](https://github.com/ArduPilot/ardupilot/tree/master/Rover), [wiki](https://ardupilot.org/rover/index.html)

- ArduSub : [code](https://github.com/ArduPilot/ardupilot/tree/master/ArduSub), [wiki](http://ardusub.com/)

- Antenna Tracker : [code](https://github.com/ArduPilot/ardupilot/tree/master/AntennaTracker), [wiki](https://ardupilot.org/antennatracker/index.html)

## User Support & Discussion Forums

- Support Forum: <https://discuss.ardupilot.org/>

- Community Site: <https://ardupilot.org>

## Developer Information

- Github repository: <https://github.com/ArduPilot/ardupilot>

- Main developer wiki: <https://ardupilot.org/dev/>

- Developer discussion: <https://discuss.ardupilot.org>

- Developer chat: <https://discord.com/channels/ardupilot>

## Top Contributors

- [Flight code contributors](https://github.com/ArduPilot/ardupilot/graphs/contributors)
- [Wiki contributors](https://github.com/ArduPilot/ardupilot_wiki/graphs/contributors)
- [Most active support forum users](https://discuss.ardupilot.org/u?order=post_count&period=quarterly)
- [Partners who contribute financially](https://ardupilot.org/about/Partners)

## How To Get Involved

- The ArduPilot project is open source and we encourage participation and code contributions: [guidelines for contributors to the ardupilot codebase](https://ardupilot.org/dev/docs/contributing.html)

- We have an active group of Beta Testers to help us improve our code: [release procedures](https://ardupilot.org/dev/docs/release-procedures.html)

- Desired Enhancements and Bugs can be posted to the [issues list](https://github.com/ArduPilot/ardupilot/issues).

- Help other users with log analysis in the [support forums](https://discuss.ardupilot.org/)

- Improve the wiki and chat with other [wiki editors on Discord #documentation](https://discord.com/channels/ardupilot)

- Contact the developers on one of the [communication channels](https://ardupilot.org/copter/docs/common-contact-us.html)

## License

The ArduPilot project is licensed under the GNU General Public
License, version 3.

- [Overview of license](https://ardupilot.org/dev/docs/license-gplv3.html)

- [Full Text](https://github.com/ArduPilot/ardupilot/blob/master/COPYING.txt)

## Maintainers

ArduPilot is comprised of several parts, vehicles and boards. The list below
contains the people that regularly contribute to the project and are responsible
for reviewing patches on their specific area.

- [Andrew Tridgell](https://github.com/tridge):
  - ***Vehicle***: Plane, AntennaTracker
  - ***Board***: Pixhawk, Pixhawk2, PixRacer
- [Francisco Ferreira](https://github.com/oxinarf):
  - ***Bug Master***
- [Grant Morphett](https://github.com/gmorph):
  - ***Vehicle***: Rover
- [Willian Galvani](https://github.com/williangalvani):
  - ***Vehicle***: Sub
  - ***Board***: Navigator
- [Michael du Breuil](https://github.com/WickedShell):
  - ***Subsystem***: Batteries
  - ***Subsystem***: GPS
  - ***Subsystem***: Scripting
- [Peter Barker](https://github.com/peterbarker):
  - ***Subsystem***: DataFlash, Tools
- [Randy Mackay](https://github.com/rmackay9):
  - ***Vehicle***: Copter, Rover, AntennaTracker
- [Siddharth Purohit](https://github.com/bugobliterator):
  - ***Subsystem***: CAN, Compass
  - ***Board***: Cube*
- [Tom Pittenger](https://github.com/magicrub):
  - ***Vehicle***: Plane
- [Bill Geyer](https://github.com/bnsgeyer):
  - ***Vehicle***: TradHeli
- [Emile Castelnuovo](https://github.com/emilecastelnuovo):
  - ***Board***: VRBrain
- [Georgii Staroselskii](https://github.com/staroselskii):
  - ***Board***: NavIO
- [Gustavo José de Sousa](https://github.com/guludo):
  - ***Subsystem***: Build system
- [Julien Beraud](https://github.com/jberaud):
  - ***Board***: Bebop & Bebop 2
- [Leonard Hall](https://github.com/lthall):
  - ***Subsystem***: Copter attitude control and navigation
- [Matt Lawrence](https://github.com/Pedals2Paddles):
  - ***Vehicle***: 3DR Solo & Solo based vehicles
- [Matthias Badaire](https://github.com/badzz):
  - ***Subsystem***: FRSky
- [Mirko Denecke](https://github.com/mirkix):
  - ***Board***: BBBmini, BeagleBone Blue, PocketPilot
- [Paul Riseborough](https://github.com/priseborough):
  - ***Subsystem***: AP_NavEKF2
  - ***Subsystem***: AP_NavEKF3
- [Víctor Mayoral Vilches](https://github.com/vmayoral):
  - ***Board***: PXF, Erle-Brain 2, PXFmini
- [Amilcar Lucas](https://github.com/amilcarlucas):
  - ***Subsystem***: Marvelmind
- [Samuel Tabor](https://github.com/samuelctabor):
  - ***Subsystem***: Soaring/Gliding
- [Henry Wurzburg](https://github.com/Hwurzburg):
  - ***Subsystem***: OSD
  - ***Site***: Wiki
- [Peter Hall](https://github.com/IamPete1):
  - ***Vehicle***: Tailsitters
  - ***Vehicle***: Sailboat
  - ***Subsystem***: Scripting
- [Andy Piper](https://github.com/andyp1per):
  - ***Subsystem***: Crossfire
  - ***Subsystem***: ESC
  - ***Subsystem***: OSD
  - ***Subsystem***: SmartAudio
- [Alessandro Apostoli](https://github.com/yaapu):
  - ***Subsystem***: Telemetry
  - ***Subsystem***: OSD
- [Rishabh Singh](https://github.com/rishabsingh3003):
  - ***Subsystem***: Avoidance/Proximity
- [David Bussenschutt](https://github.com/davidbuzz):
  - ***Subsystem***: ESP32,AP_HAL_ESP32
- [Charles Villard](https://github.com/Silvanosky):
  - ***Subsystem***: ESP32,AP_HAL_ESP32
