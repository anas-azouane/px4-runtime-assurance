# Architecture

## Data flow

Telemetry is parsed and written to one `vehicle_state`. A 50 Hz task copies that state
under a lock, runs the four rules on the copy, and hands the worst verdict to the
arbiter, which decides what to send.

```
transport -> mavlink_rx -> state          RX context, only updates state
                             |
                          snapshot
                             v
             geofence, envelope, battery, liveness
                             v
                          arbiter -> override_tx -> transport      50 Hz task
```

Rules are pure functions of `(state, now, params)`, which is why they can be tested on
a PC with no scheduler. Debounce counters live in the state and advance per sample
rather than per tick, so "5 consecutive samples" means samples. There is no dynamic
allocation in the monitor. libmetal takes its small heap at init on the M4.

## Arbiter

```
NONE -> RTL -> LAND -> TERMINATE
```

An action latches until PX4 reports disarmed, and only moves up. It moves up when a
rule wants something higher, when PX4 rejects the command, or after three unanswered
transmissions 200 ms apart. Termination keeps repeating until disarm is seen, since an
acknowledgement does not show the motors have stopped.

## Why the onboard link

| Link | `ATTITUDE` | `GLOBAL_POSITION_INT` | `SYS_STATUS` |
|---|---|---|---|
| GCS, 14550 | 50 Hz | 50 Hz | 1 Hz |
| Onboard, 14540 | 100 Hz | 50 Hz | 5 Hz |

## On the M4

| Thread | Priority | Job |
|---|---|---|
| main | 0 | rule evaluation every 20 ms, on absolute deadlines |
| rpmsg | preempt 1 | virtio doorbell, MAVLink parsing, state updates |

Rule evaluation preempts parsing. Sending uses `rpmsg_trysend`, so a Linux side that
stops reading cannot stall the rules; the arbiter's retransmit covers a dropped frame.
`CONFIG_FPU_SHARING` is required because both threads use floats. Parsing runs in the
rpmsg thread and takes the state mutex for each received buffer.

Code and the resource table live in RETRAM at `0x00000000`, data in MCUSRAM at
`0x10000000`, and vrings and buffers at `0x10040000`, which matches the OpenSTLinux
reserved-memory nodes.

## Measuring latency

SITL, the A7 and the M4 each have their own clock, so all timing comes from the bench
proxy. It sees telemetry going into the monitor and commands coming out, on one clock,
whichever transport is underneath. Fault onset depends on the fault:

- proxy faults: the first frame the proxy altered or dropped
- PX4 hang: the last `ATTITUDE` before the first command
- fence exit: the first position sample past the line, checked by the runner with its
  own copy of the fence so the check does not trust the code it is judging
- Gazebo faults: when the injecting command returned

A run also fails if the real-time factor drifts more than 15 %, if the vehicle never
climbs above 5 m, or if PX4 keeps publishing after a hang is injected. The last one is
there because a SIGSTOP sent to PID 1 in a container is silently ignored, and that run
looked like a monitor failure until the bench was fixed.

## Injecting faults under Gazebo

`failure gps off` and `failure motor off` do not reach the physics in PX4 v1.16 under
Gazebo: the first is only handled by `SimulatorMavlink`, the second only fakes ESC
status. The runner sets `SIM_GPS_USED` to 0 to stop GPS, and clamps `SIM_GZ_EC_MIN2`
and `SIM_GZ_EC_MAX2` to 0 to stop a motor. Setting `SIM_GZ_EC_FUNC2` to 0 instead had no visible
effect on the flight.
