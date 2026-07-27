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
