# Linux build and pendant setup

NGC builds on Linux with Clang, Ninja, CMake, GLM, OpenGL/GLU, X11 or
Wayland development files, and HIDAPI's hidraw backend. GLFW, ImGui, toml++,
PathTempo, and Ruckig come from the repository submodules.

Configure and build a Release tree from the repository root:

```bash
git submodule update --init --recursive
cmake -S . -B build -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER=clang \
    -DCMAKE_CXX_COMPILER=clang++
cmake --build build --parallel
```

Run the core suite from the repository root because it loads `machine.toml`
and `tool_table.txt` relative to its working directory. Run the other tests
through CTest:

```bash
./build/ngc_tests
ctest --test-dir build -E '^ngc_tests$' --output-on-failure
```

## Real-time executor host

The configured Mesa Real backend uses the existing
`ProductionExecutorRuntime` servo thread as its RT thread. The Real backend
selection names the executable and its backend-owned configuration, while
`machine.toml` remains authoritative for the shared servo period:

```toml
[real_backend]
executable = "build/ngc_mesa_backend"
configuration = "physical_backend.toml"
servo_period = 0.001
```

The selected backend configuration owns the host policy:

```toml
[runtime]
realtime_cpu = 15
realtime_priority = 98
lock_memory = true
```

The selected user must have sufficient `rtprio` and `memlock` limits. Verify
the login session before starting NGC:

```bash
ulimit -r
ulimit -l
```

The executor rejects startup if Linux cannot lock memory, remove its NRT host
from the selected CPU, pin the servo thread, or enter the configured
`SCHED_FIFO` priority. It does not silently use ordinary scheduling. Keep the
selected CPU's SMT sibling free of ordinary work and route routine IRQs to
housekeeping CPUs.

### Proven Arch Linux RT host configuration

The physical Mesa development host is an AMD Ryzen 7 7700X with eight cores
and two hardware threads per core. The configuration below was validated on
`6.18.40-rt6-arch2-3-rt-lts` with a one-millisecond servo period. CPU numbering
and cpuidle state numbering are host-specific; inspect another machine rather
than copying them blindly.

The relevant sibling pairs and assigned roles are:

| Physical core | Logical CPUs | Role |
| --- | --- | --- |
| Core 6 | 6, 14 | CPU 6 handles the dedicated Mesa NIC IRQ; CPU 14 is kept idle |
| Core 7 | 7, 15 | CPU 15 hosts the executor servo thread; CPU 7 is kept idle |
| Remaining cores | 0-5, 8-13 | Housekeeping and ordinary NRT work |

Isolating both hardware threads of each RT-facing core prevents ordinary work
on an SMT sibling from competing for shared core resources. CPU 6 is isolated
from scheduler domains even though it deliberately handles the Mesa NIC IRQ.
CPU 15 should normally see only its local timer interrupt, unavoidable kernel
IPIs, and the wakeups required by the executor.

The RT-LTS GRUB entry appends:

```text
isolcpus=domain,managed_irq,6,7,14,15
nohz_full=6,7,14,15
irqaffinity=0-5,8-13
rcu_nocbs=6,7,14,15
```

The options have distinct purposes:

- `isolcpus=domain,managed_irq,...` removes the CPUs from ordinary scheduler
  load balancing and directs compatible managed interrupts away from them.
- `nohz_full=...` suppresses the periodic scheduler tick while an isolated CPU
  runs one eligible task.
- `irqaffinity=...` gives ordinary IRQs a housekeeping default.
- `rcu_nocbs=...` moves RCU callback processing away from the isolated CPUs.

On this Arch installation the entry is stored in `/etc/grub.d/40_custom` and
the generated `/boot/grub/grub.cfg` is refreshed with:

```bash
sudo grub-mkconfig -o /boot/grub/grub.cfg
```

After reboot, verify the selected kernel and effective command line rather
than assuming the menu entry was used:

```bash
uname -r
cat /proc/cmdline
```

Boot-time isolation is complemented by
`/usr/local/sbin/ngc-realtime-cpus`. The current host script is:

```sh
#!/bin/sh
set -eu

isolated_cpus="6 7 14 15"
housekeeping_mask="3f3f"

set_cpu_policy() {
    cpu="$1"
    governor="$2"
    preference="$3"
    idle_state_3_disabled="$4"
    cpu_root="/sys/devices/system/cpu/cpu${cpu}"

    printf '%s\n' "$governor" \
        > "${cpu_root}/cpufreq/scaling_governor"
    if [ -w "${cpu_root}/cpufreq/energy_performance_preference" ]; then
        printf '%s\n' "$preference" \
            > "${cpu_root}/cpufreq/energy_performance_preference"
    fi
    if [ -w "${cpu_root}/cpuidle/state3/disable" ]; then
        printf '%s\n' "$idle_state_3_disabled" \
            > "${cpu_root}/cpuidle/state3/disable"
    fi
}

mesa_irq() {
    awk '$NF == "enp17s0" {
        sub(/:$/, "", $1)
        print $1
        exit
    }' /proc/interrupts
}

start() {
    printf '0\n' > /proc/sys/kernel/timer_migration

    for cpu in $isolated_cpus; do
        set_cpu_policy "$cpu" performance performance 1
    done

    printf '%s\n' "$housekeeping_mask" \
        > /sys/devices/virtual/workqueue/cpumask
    printf '%s\n' "$housekeeping_mask" \
        > /sys/bus/workqueue/devices/writeback/cpumask

    attempts=0
    irq=""
    while [ -z "$irq" ] && [ "$attempts" -lt 50 ]; do
        irq="$(mesa_irq)"
        if [ -z "$irq" ]; then
            sleep 0.1
        fi
        attempts=$((attempts + 1))
    done
    if [ -z "$irq" ]; then
        echo "could not find the enp17s0 interrupt" >&2
        exit 1
    fi
    printf '6\n' > "/proc/irq/${irq}/smp_affinity_list"
}

stop() {
    printf '1\n' > /proc/sys/kernel/timer_migration

    for cpu in $isolated_cpus; do
        set_cpu_policy "$cpu" powersave balance_performance 0
    done

    printf 'ffff\n' > /sys/devices/virtual/workqueue/cpumask
    printf 'ffff\n' > /sys/bus/workqueue/devices/writeback/cpumask
}

case "${1:-}" in
    start)
        start
        ;;
    stop)
        stop
        ;;
    *)
        echo "usage: $0 start|stop" >&2
        exit 2
        ;;
esac
```

On this processor `cpuidle/state3` is C3 with a reported 350-microsecond exit
latency. The service disables C3 on CPUs 6, 7, 14, and 15, while leaving POLL,
C1, and C2 available. It also selects the `performance` frequency governor and
`performance` energy-performance preference on those CPUs. Disabling C2 did
not provide a useful improvement during commissioning, so the shallower state
remains enabled.

The hexadecimal workqueue mask `3f3f` selects CPUs 0-5 and 8-13. Both the
general and writeback workqueues are restricted to those housekeeping CPUs.
The Mesa NIC's MSI-X IRQ number is discovered from `/proc/interrupts` on every
start because IRQ numbers are not stable across boots, then its affinity is
set to CPU 6.

The systemd unit is:

```ini
[Unit]
Description=Tune NGC isolated real-time CPUs and Mesa IRQ
After=systemd-modules-load.service
ConditionPathExists=/sys/devices/system/cpu/cpu15/cpufreq/scaling_governor

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/ngc-realtime-cpus start
ExecStop=/usr/local/sbin/ngc-realtime-cpus stop
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
```

Install the script as `/usr/local/sbin/ngc-realtime-cpus` with mode `0755` and
the unit as `/etc/systemd/system/ngc-realtime-cpus.service`, then enable and
apply the unit with:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now ngc-realtime-cpus.service
```

#### Why timer migration is disabled

NGC sleeps to absolute `CLOCK_MONOTONIC` deadlines with
`clock_nanosleep()`. Because the thread is `SCHED_FIFO`, PREEMPT_RT delivers
its hrtimer wakeup from hard-IRQ context. Unlike the timerlat tracer, however,
the userspace API does not request `HRTIMER_MODE_PINNED`; the kernel can move
the timer to a housekeeping CPU and wake CPU 15 with an IPI.

A trace of 916 periodic NGC sleeps with the default
`kernel.timer_migration=1` found 662 expirations on CPU 15 and 254 expirations
on other CPUs. Disabling timer migration kept all 916 expirations on CPU 15.
In the corresponding short tests, maximum wake lateness fell from 12.518
microseconds to 3.106 microseconds and maximum period jitter fell from 9.510
microseconds to 1.100 microseconds. This closes the most important behavioral
gap between NGC's userspace sleeper and timerlat's pinned kernel hrtimer.

`kernel.timer_migration` is a runtime sysctl, not a kernel command-line
option. The host service writes zero when RT tuning starts and restores one
when it stops. This setting is system-wide and can reduce timer coalescing,
although CPU 15 must wake every millisecond for this workload in either case.

Verify the complete active policy with:

```bash
systemctl is-active ngc-realtime-cpus.service
cat /proc/sys/kernel/timer_migration
cat /sys/devices/virtual/workqueue/cpumask
cat /sys/bus/workqueue/devices/writeback/cpumask
cat /sys/devices/system/cpu/cpu15/cpufreq/scaling_governor
cat /sys/devices/system/cpu/cpu15/cpufreq/energy_performance_preference
awk '$NF == "enp17s0" { print }' /proc/interrupts
```

The expected values are `active`, timer migration `0`, both workqueue masks
`3f3f`, governor and energy preference `performance`, and all `enp17s0`
interrupts counted on CPU 6. Also inspect storage IRQ counts after a test; no
NVMe queue assigned to an isolated CPU should be active there.

#### StepGen timing validation

Build and run the active diagnostic only on an isolated bare board with the
commissioning safety conditions described below:

```bash
cmake --build build --target ngc_mesa_stepgen_diagnostic
./build/ngc_mesa_stepgen_diagnostic
```

The diagnostic primes each requested rate, captures the first DPLL-latched
accumulator sample after that rate is active, and measures an exact number of
latch-to-latch intervals. Its output keeps three quantities distinct:

- `ideal_error` compares measured travel with the requested floating-point
  rate, so duration-dependent FPGA rate quantization remains visible.
- `quantization_error` reports the expected difference introduced by the
  signed 32-bit DDS word and its effective representable rate.
- `residual` is measured travel minus the DDS-encoded expectation. Only this
  unexplained component is used for the diagnostic fault. Its tolerance is a
  small fixed margin plus the travel represented by the measured start-to-end
  DPLL phase displacement.

Per-interval accumulator delta range and worst residual are also reported. Do
not replace the residual check with the production
`maximum_generated_step_error`: that setting bounds instantaneous closed-loop
executor following error, while this diagnostic deliberately commands an
uncorrected constant raw StepGen rate.

The diagnostic also reports the accumulator travel across rate activation and
stop. These command-transition samples retain evidence of a stale or
unexpected boundary latch that would otherwise disappear when only the aligned
steady-rate interval is examined.

With timer migration disabled, the ordinary 8,016-exchange run measured:

| Measurement | Average | Maximum absolute |
| --- | ---: | ---: |
| Wake lateness | 1.896 us | 4.802 us |
| Period jitter | approximately 0 us | 3.053 us |
| UDP exchange duration | 79.698 us | 91.263 us |
| DPLL phase error | 2.071 us absolute | 43.199 us |

There were no missed deadlines, and all StepGen channel checks passed.

The longer validation saturates only the housekeeping CPUs while executing
10,000 cycles in each direction on every configured StepGen:

```bash
taskset -c 0-5,8-13 \
    stress-ng --cpu 12 --cpu-method all \
    --timeout 90s --verify --metrics-brief &
stress_pid=$!
trap 'kill "$stress_pid" 2>/dev/null || true' EXIT INT TERM
sleep 1
./build/ngc_mesa_stepgen_diagnostic --cycles 10000
wait "$stress_pid"
trap - EXIT INT TERM
```

The 80,016-exchange stressed run measured:

| Measurement | Average | Maximum absolute |
| --- | ---: | ---: |
| Wake lateness | 1.993 us | 11.981 us |
| Period jitter | approximately 0 us | 10.129 us |
| UDP exchange duration | 80.382 us | 88.598 us |
| DPLL phase error | 0.761 us absolute | 48.157 us |

There were no missed deadlines, the watchdog remained serviced, and all 12
CPU stress workers and all StepGen channel checks passed. Before timer
migration was disabled, a comparable 80,016-exchange CPU-stressed run reached
119.049 microseconds of wake lateness.

After aligning the accumulator measurement with the DPLL latch and separating
DDS quantization from unexplained error, another 80,016-exchange stressed run
at 800 steps/s measured 0.182-0.190 step of visible ideal-rate error. The
known ten-second DDS contribution was 0.171915 step; unexplained residuals
were 0.010-0.018 step against phase-aware thresholds of 0.050-0.054 step.
Every individual one-millisecond interval was within 0.000132 step of its
DDS-encoded expectation. This check would still reject the earlier
approximately 3.42-step observation: after removing its known approximately
0.572-step accounting and quantization contribution, approximately 2.85 steps
would remain unexplained.

The earlier diagnostic retained only the accumulator values at the two ends of
an unaligned host-cycle window, so it did not preserve enough evidence to
identify which boundary produced that remaining discrepancy. Repeating the
exact CPU, VM, cache, fork, and context-switch workload with the aligned
diagnostic did not reproduce it. With timer migration disabled, unexplained
residual stayed below 0.044 step, activation and stop travel stayed between
0.392 and 0.426 step in the commanded direction, and every steady interval
stayed within 0.000331 step of the encoded expectation. Re-enabling timer
migration for the same workload increased maximum wake lateness from 11.845
microseconds to 232.948 microseconds but produced the same correct accumulator
intervals and transitions. Timer migration is therefore a demonstrated source
of wake-latency tail growth, but it is not sufficient to produce the historical
accumulator discrepancy.

The commissioning progression makes the effect of each host-level change
visible. These were separate long stressed runs, so treat the numbers as
observed tails rather than deterministic bounds:

| Host policy | Maximum wake lateness | Maximum UDP exchange |
| --- | ---: | ---: |
| Before CPU 14, the CPU 6 NIC sibling, was also isolated | 267.725 us | 339.316 us |
| CPUs 6, 7, 14, and 15 isolated; NIC IRQ on CPU 6; timer migration enabled | 119.049 us | 91.954 us |
| Same CPU and IRQ policy; timer migration disabled | 11.981 us | 88.598 us |

The native timerlat tracer and oslat were useful independent checks. Under
housekeeping-CPU load, timerlat completed 120,000 one-millisecond periods
without crossing its 100-microsecond stop threshold, while oslat reported a
five-microsecond maximum continuous interruption. Those results indicated
that hardware and kernel latency were already small and motivated comparing
timerlat's pinned hrtimer with NGC's migratable userspace sleeper.

Do not interpret CPU saturation as proof that arbitrary memory pressure is
safe. An intentionally extreme run added four cache workers and four VM
workers manipulating approximately 10.8 GB while the 12 CPU workers ran. It
produced one 11.983-millisecond wake delay, a 2.054-millisecond UDP maximum,
85 missed deadlines, and the expected Mesa watchdog trip after 38,550
exchanges. There were no NIC errors, packet drops, OOM reports, or kernel
warnings, so the test did not identify a specific cause. Possible contributors
include memory-controller contention, page allocation and faults, TLB
shootdown IPIs, kernel memory-management contention, or firmware latency.
Avoid unbounded memory churn on the production host and retain the watchdog as
the final safety response.

`ngc_ipc_test_peer` is a hardware-free test fixture, not a selectable Real
backend. Run the ordinary portable IPC suite with:

```bash
./build/ngc_ipc_tests ./build/ngc_ipc_test_peer.exe
```

On a configured RT development host, exercise the same Real-session path with
RT hosting enabled:

```bash
./build/ngc_ipc_tests ./build/ngc_ipc_test_peer.exe --realtime
```

## Mesa 7I96 development network

The Arch Linux development host reaches the dedicated Mesa 7I96 directly over
the onboard Ethernet controller. Its confirmed network identities are:

| Role | Interface or device | Address |
| --- | --- | --- |
| Dedicated host NIC | `enp17s0` | `10.10.10.1/24` |
| Mesa 7I96 | `00:60:1b:16:01:31` | `10.10.10.10` |
| USB internet NIC | `enp24s0u2u4` | DHCP |

NetworkManager owns both host interfaces. The `Mesa 7i96` connection uses
manual IPv4, has no gateway or DNS server, sets `ipv4.never-default`, ignores
automatic routes and DNS, and disables IPv6. The `USB Internet` connection
uses DHCP and owns the ordinary default route and DNS. Do not run the global
`dhcpcd` service alongside NetworkManager; competing managers can install an
unwanted route when the dedicated interface gains carrier.

The equivalent persistent Mesa profile can be established with:

```bash
sudo nmcli connection add type ethernet \
    ifname enp17s0 \
    con-name "Mesa 7i96" \
    ipv4.method manual \
    ipv4.addresses 10.10.10.1/24 \
    ipv4.never-default yes \
    ipv4.ignore-auto-routes yes \
    ipv4.ignore-auto-dns yes \
    ipv6.method disabled
sudo systemctl disable --now dhcpcd.service
```

NetworkManager keyfiles live under
`/etc/NetworkManager/system-connections/`. Verify that the board route remains
local and internet remains on the USB interface with:

```bash
ip route get 10.10.10.10
ip route get 1.1.1.1
ping -I enp17s0 10.10.10.10
```

The first route must select `enp17s0` with source `10.10.10.1`; the second must
select `enp24s0u2u4`. The board should answer with sub-millisecond latency on a
direct cable.

Build and run NGC's read-only HostMot2 discovery utility with:

```bash
cmake --build build --target ngc_mesa_discover
./build/ngc_mesa_discover \
    --address 10.10.10.10 \
    --validate-7i96
```

The utility and the initial physical backend share the `ngc_mesa` library's
UDP/LBP16 transport, IDROM parser, module and pin descriptor parser, and typed
capability model. `--validate-7i96` additionally requires the supported 7I96
IDROM topology, DPLL, watchdog, I/O-port, StepGen, encoder, and SSR module
versions and layouts together with the five step/direction pairs, eleven
isolated inputs, six isolated outputs, and encoder pins. The validator also
accepts backend-selected channels and pins through its typed API so duplicate
or out-of-range physical mappings fail during NRT startup. The utility exposes
no register-write operation. Compare its inventory with MesaFlash while
bringing up a board:

```bash
mesaflash --device 7i96 --addr 10.10.10.10 --readhmid
```

Build and run the read-only cyclic-latency utility with:

```bash
cmake --build build --target ngc_mesa_latency
./build/ngc_mesa_latency \
    --address 10.10.10.10 \
    --samples 10000 \
    --period-us 1000 \
    --timeout-ms 10
```

The utility first validates HostMot2 discovery, then reads the cookie register
at absolute periodic deadlines. It reports round-trip and host wake-lateness
minimum, mean, p50, p95, p99, and maximum values, together with read failures,
unexpected cookie contents, and periods skipped after an overrun. Exit status
2 means the run completed but observed a read failure or content mismatch.
This is an NRT link and scheduler diagnostic. It does not enable outputs,
service the watchdog, exercise packet sequencing, prove RT cyclic I/O, or
commission physical motion.

Build the initial physical backend and validate its two configuration inputs
without opening the board or starting an RT thread with:

```bash
cmake --build build --target ngc_mesa_backend
./build/ngc_mesa_backend \
    --machine-configuration machine.toml \
    --backend-configuration physical_backend.toml \
    --validate-config-only
```

The executable parses both files only during NRT bootstrap. The physical
backend configuration owns the executor host policy and composes independent
Mesa motion and optional spindle roles. It passes typed runtime configuration to
`ProductionExecutorRuntime` and typed motion configuration to the Mesa
adapter. The Huanyang spindle role remains configured but disabled until its
physical commissioning is complete. Its Linux serial implementation uses
bounded transaction timeouts and the Huanyang proprietary RTU packet shape,
establishes stop before reading the VFD's stored setup, and validates all
responses. It has not yet been exercised against physical VFD hardware. For
bare-board commissioning only, the
current configuration treats energized INPUT2 as the active-high
`external_enable` through the named physical operand
`external_enable_field`. For isolated bare-board testing, INPUT COMMON is
grounded and INPUT2 is energized from the board's PTC-protected +5VP supply.
Removing that voltage must latch the external-enable fault and safe outputs.
This temporary wire is not verified E-stop or enable-permission feedback.
Replace it before powering any drive, motor, spindle, or other output. A normal
invocation is launched by the application's external-runtime IPC boundary; do
not select it as the Real target beyond this isolated commissioning setup until
the staged checks are complete.

Measure the exact configured digital-I/O assembly program without opening the
board or adding clock reads to the servo cycle:

```bash
cmake --build build --target ngc_mesa_io_program_benchmark
./build/ngc_mesa_io_program_benchmark \
    --machine-config machine.toml \
    --backend-config physical_backend.toml
```

The benchmark uses the same compiler entry point as `ngc_mesa_backend`, warms
the stateful debounce instructions, and reports batched input-pass,
output-pass, and combined-pass nanoseconds together with their fraction of the
configured Real servo period. Batch timing amortizes clock overhead and does
not subtract a synthetic baseline. `--batches` and
`--iterations-per-batch` control the sample count.

Exercise the same physical process through the production IPC boundary without
commanding motion with:

```bash
cmake --build build --target ngc_mesa_backend ngc_mesa_backend_smoke
./build/ngc_mesa_backend_smoke \
    --peer build/ngc_mesa_backend \
    --machine-config machine.toml \
    --backend-config physical_backend.toml
```

This write-capable bare-board test configures Mesa, services the watchdog,
enables and disables the executor, and requires zero commanded joint motion.
Run it only with no drives, motors, spindle, or other outputs connected.

To prove active-high external-enable loss, energize INPUT2 before startup and
run the same test with:

```bash
./build/ngc_mesa_backend_smoke \
    --peer build/ngc_mesa_backend \
    --machine-config machine.toml \
    --backend-config physical_backend.toml \
    --expect-external-enable-loss
```

Wait for its `READY` message, then remove only the +5VP connection from INPUT2.
Success requires the exact `MESA_EXTERNAL_ENABLE_FAULT`, a faulted snapshot,
and zero commanded joint velocity and acceleration. The mode times out after
two minutes and treats every other backend fault as failure.

Build the standalone Huanyang commissioning diagnostic with:

```bash
cmake --build build --target ngc_huanyang_spindle_diagnostic
```

Its default path opens the configured serial device, immediately establishes
Stop, reads and reports PD004, PD005, PD011, and PD141 through PD144, waits for
reported zero speed, and polls output speed and current without commanding Run:

```bash
./build/ngc_huanyang_spindle_diagnostic \
    --backend-configuration physical_backend.toml \
    --samples 20
```

This intentionally opens the configured spindle role even though production
configuration keeps it disabled. Use the default path first with the motor
disconnected. A software Stop is not a substitute for verified hardwired
E-stop, STO, contactor, and drive-enable behavior.

Only after those physical protections are verified, opt into a bounded
direction test by supplying a speed:

```bash
./build/ngc_huanyang_spindle_diagnostic \
    --backend-configuration physical_backend.toml \
    --command-test-speed 6000 \
    --command-duration-ms 1000 \
    --stop-timeout-ms 5000
```

That path commands CW, requires reported zero speed after Stop, commands CCW,
and again requires reported zero speed after Stop. Interruption and all normal
destruction paths attempt Stop. The diagnostic remains uncommissioned until its
behavior is captured against the physical USB-to-RS-485 adapter and VFD.

## VistaCNC P2-S access

The Linux transport uses HIDAPI's hidraw backend. Install the supplied udev
rule to grant the active desktop user and members of `plugdev` access to the
P2-S without running NGC as root:

```bash
sudo install -m 0644 udev/70-ngc-vistacnc-p2s.rules \
    /etc/udev/rules.d/70-ngc-vistacnc-p2s.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --action=change --subsystem-match=hidraw
```

If the current distribution does not provide a `plugdev` group, remove the
`GROUP="plugdev"` clause and retain `TAG+="uaccess"`. Reconnect the pendant if
the existing hidraw node does not receive the updated permissions.

Verify input and display transport with:

```bash
./build/vistacnc_p2s_probe
./build/vistacnc_p2s_probe --display "NGC     READY"
```
