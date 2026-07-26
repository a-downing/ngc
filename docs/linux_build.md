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
