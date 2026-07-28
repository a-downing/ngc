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

The configured Linux IPC executor uses the existing
`ProductionExecutorRuntime` servo thread as its RT thread. The Real backend
settings select its period, CPU, FIFO priority, and memory-locking policy:

```toml
[real_backend]
type = "ipc_executor"
executable = "build/ngc_ipc_backend.exe"
servo_period = 0.001
realtime_cpu = 15
realtime_priority = 95
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

Run the ordinary portable IPC suite with:

```bash
./build/ngc_ipc_tests ./build/ngc_ipc_backend.exe
```

On a configured RT development host, exercise the same Real-session path with
RT hosting enabled:

```bash
./build/ngc_ipc_tests ./build/ngc_ipc_backend.exe --realtime
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
backend configuration composes independent Mesa motion and optional spindle
roles. It passes typed runtime configuration to
`ProductionExecutorRuntime` and typed motion configuration to the Mesa
adapter. The Huanyang spindle role remains configured but disabled until its
physical commissioning is complete. Its Linux serial implementation uses
bounded transaction timeouts and the Huanyang proprietary RTU packet shape,
establishes stop before reading the VFD's stored setup, and validates all
responses. It has not yet been exercised against physical VFD hardware. For
bare-board commissioning only, the
current configuration treats disconnected `fieldin2` as an active-low
`external_enable` through the named physical operand
`external_enable_field`. This permits a normal physical-peer start but is not
fail-safe. Replace it with verified E-stop/enable-permission feedback before
connecting any drive, motor, spindle, or other output. A normal invocation is
launched by the application's external-runtime IPC boundary; do not select it
as the Real target beyond this isolated commissioning setup until the staged
checks are complete.

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
