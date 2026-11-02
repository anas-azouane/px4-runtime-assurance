# Requirements

Numbers are justified from rates measured on the PX4 v1.16.2 onboard link
(UDP 14540) with `tools/telemetry_probe.py`, 15 s on the ground:

| Message | Rate | Largest gap |
|---|---|---|
| `ATTITUDE` | 99.9 Hz | 21.7 ms |
| `GLOBAL_POSITION_INT` | 50.0 Hz | 30.2 ms |
| `SYS_STATUS` | 5.0 Hz | 211 ms |
| `EXTENDED_SYS_STATE` | 5.0 Hz | 211 ms |
| `HEARTBEAT` | 1.0 Hz | 1010 ms |

The GCS link (14550) only carries `ATTITUDE` at 50 Hz, and `SYS_STATUS` at 1 Hz,
so the monitor belongs on the onboard link. On a real UART the rates depend on
the baud rate and have to be measured again.

All latencies run from when the condition became observable at the bench proxy
to when the command reached the proxy.

| ID | Requirement |
|---|---|
| SR-01 | The monitor shall command RTL within 200 ms when the vehicle leaves the geofence polygon or passes its ceiling. |
| SR-02 | The monitor shall command LAND within 200 ms when roll or pitch exceeds 45° for 5 consecutive samples, or a non-finite attitude is reported for 5 consecutive samples. Below 5 m it shall command flight termination instead. |
| SR-03 | The monitor shall command RTL within 500 ms when remaining battery is below the return-home estimate plus 20 % margin. |
| SR-04 | The monitor shall command flight termination within 500 ms when no `ATTITUDE` message is received for 300 ms while airborne. |
| SR-05 | The monitor shall not issue any override during a nominal mission, or while PX4 lands itself after losing GPS. |
| SR-06 | The monitor shall command flight termination within 200 ms when the vehicle is more than 30 m outside the geofence. |
| SR-07 | The monitor shall command LAND within 1200 ms when no `GLOBAL_POSITION_INT` is received for 1000 ms while airborne. |
| SR-08 | The monitor shall escalate one level (RTL, LAND, termination) when PX4 rejects a command or leaves 3 transmissions, 200 ms apart, unanswered. |

## Rationale

**SR-01.** At 50 Hz the two-sample debounce takes up to 40 ms, plus one 20 ms
tick, leaving margin for transport. One sample would be faster, but a single
position outlier would then trigger a return.

**SR-02.** 45° is PX4's own `MPC_TILTMAX_AIR` default, so a healthy controller
should not hold beyond it. Five samples at 100 Hz is 50 ms, long enough to reject
a glitch and short compared with how fast a quad diverges. A NaN compares false
against any limit, so the check is written to count it as a violation. Below 5 m
there is no height left to recover or land on purpose.

**SR-03.** `SYS_STATUS` at 5 Hz can be 211 ms late, and a 20 ms tick follows.
Battery falls slowly, so nothing is gained by a tighter number. The estimate uses
compiled-in cruise speed, descent speed and drain rate: distance home / 5 m/s plus
altitude / 1.5 m/s, at 0.2 %/s, plus the 20 % margin. Those are placeholder values
for a simulated x500, not measurements of an airframe.

**SR-04.** 300 ms is about 14 times the largest measured `ATTITUDE` gap, so a
scheduling hiccup will not terminate a flight. `HEARTBEAT` cannot do this job at
1 Hz. 200 ms is then left for the tick, the send and transport.

**SR-05.** Takeoff and landing are the usual sources of false alarms, so all rules
are gated on an airborne state: armed, and either PX4 reports in-air or landing,
or relative altitude is above 1 m.

GPS loss is in here because of what SC-10 showed. PX4 keeps publishing its estimate
at 50 Hz and starts a blind landing on its own, so there is nothing for the monitor
to add. It also means the fence is checked against PX4's estimate, not raw GPS.

**SR-06.** The guide's "breach that keeps growing". 30 m is well past anything PX4
overshoots when told to return, so reaching it means the return is not working.

**SR-07.** Added during design: a frozen position would otherwise pass the fence
check forever. It lands instead of returning, because RTL needs the position that
was just lost. 1000 ms is about 30 times the largest measured gap.

**SR-08.** Three tries at 200 ms ride out a lost frame. A command that is
acknowledged but then not carried out is not detected; see the limitations in the
README.
