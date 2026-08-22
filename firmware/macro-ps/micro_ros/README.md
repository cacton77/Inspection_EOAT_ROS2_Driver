# micro-ROS library build config for the RP2040

`micro_ros_arduino` ships `libmicroros.a` **precompiled**. Two things are fixed
at the time that archive is built, and both of them block this firmware:

1. **The type set.** `ps_interfaces` is not in the upstream archive, so
   `ROSIDL_GET_MSG_TYPE_SUPPORT(ps_interfaces, msg, LensCommand)` will not link
   until the library is rebuilt with the package in `mcu_ws`.

2. **The entity budget.** Upstream builds the `cortex_m0` target — which is what
   the QT Py RP2040 links against — with `colcon_verylowmem.meta`:

   | | upstream `cortex_m0` | this firmware needs |
   |---|---|---|
   | `RMW_UXRCE_MAX_PUBLISHERS` | 2 | 8 |
   | `RMW_UXRCE_MAX_SUBSCRIPTIONS` | 1 | 2 |
   | `RMW_UXRCE_MAX_SERVICES` | **0** | 6 |
   | `RMW_UXRCE_MAX_HISTORY` | 1 | — |

   `MAX_SERVICES=0` is the hard one: the stock archive cannot create a service
   server *at all*, let alone an action server. `rcl_action_server_init` costs
   3 services + 2 publishers each, so `/lens/home` and `/ps_capture` together
   need 6 services and 4 publishers on top of the 4 plain topic publishers.

   These are not header constants the sketch can override — they size static
   arrays *inside* the archive. You can read the values back out of any
   `libmicroros.a` without rebuilding it:

   ```bash
   ar x src/cortex-m0plus/libmicroros.a
   nm --print-size *.obj | grep -E 'C custom_(publishers|subscriptions|services)'
   # slot sizes: publisher 216 B, subscription 216 B, service 200 B, topic 28 B
   ```

## What's here

`colcon_rp2040.meta` — upstream's standard `colcon.meta` with two changes:

**`RMW_UXRCE_MAX_SERVICES` 1 → 8.** Enough for two action servers
(`/lens/home`, `/ps_capture`) at 3 services each, plus room. The publisher and
subscription counts come from `colcon.meta` and are already generous.

Cost over the stock `cortex_m0` build is about 4.5 KB of static RAM for the
entity pools plus roughly 6 KB for the deeper input history — call it 11 KB of
the RP2040's 264 KB.

**`RCUTILS_NO_64_ATOMIC` is left ON** (upstream's value). This one is worth
reading before you change it, because the obvious "fix" is a trap.

Despite the name the flag *adds* `src/atomic_64bits.c` to micro-ROS's rcutils
fork — it means "this platform has no native 64-bit atomics, supply software
ones":

```cmake
option(RCUTILS_NO_64_ATOMIC "Enable alternative support for 64 bits atomic
       operations in platforms with no native support." OFF)
...
$<$<BOOL:${RCUTILS_NO_64_ATOMIC}>:src/atomic_64bits.c>
```

pico-sdk *also* supplies `__atomic_{load,store,exchange,fetch_add}_8` in
`libpico.a(atomic.c.o)`, so with the flag ON both exist and any sketch that
drags in that object fails to link:

```
multiple definition of `__atomic_fetch_add_8';
  libmicroros.a(librcutils-atomic_64bits.c.obj): first defined here
```

This is latent in the **upstream** archive too (the objects are byte-identical),
but it only bites once a sketch pulls in enough of rcutils to need that object —
which an action server does and a lone publisher does not. The IMU-only firmware
links against the stock archive quite happily.

### Do not "fix" this by turning the flag off

Turning it OFF makes the link error disappear, and **breaks the IMU on real
hardware**: `imu_init()` fails at boot, core 1 traps, and the QT Py sits there
blinking magenta. Bisected on hardware — same firmware source, three archives:

| archive | `/camera_head/imu` |
|---|---|
| upstream stock | 200 Hz |
| ours, `NO_64_ATOMIC=OFF` | **dead — `begin_I2C()` returns false** |
| ours, `NO_64_ATOMIC=ON` | 200 Hz |

The two implementations are not equivalent. Disassembled:

- **pico-sdk's** is `bl <lock>` / two `ldr` / `bl <unlock>` — a real **hardware
  spinlock**, which on RP2040 disables interrupts while held.
- **rcutils'** hashes the address and spins on a 23-entry byte array with plain
  `ldrb`/`strb` — no hardware spinlock, and strictly speaking not even atomic on
  a dual-core M0+.

pico-sdk's is the more *correct* primitive in isolation. The problem is that it
serialises both cores through one hardware spinlock with interrupts off, while
core 0 is hammering 64-bit atomics during micro-ROS transport bring-up and core 1
is concurrently running its first I2C transaction in `setup1()`. The I2C
transaction loses its timing budget and `Adafruit_I2CDevice::begin()` reports the
device absent. (The bisect and the disassembly are verified; that last step —
exactly which contention path blows the timeout — is inference.)

So: keep the flag ON and deal with the duplicate symbol at link time instead.

### Linking a sketch that needs the action server

Add `-Wl,--allow-multiple-definition`:

```bash
arduino-cli compile --fqbn rp2040:rp2040:adafruit_qtpy \
  --build-property compiler.c.elf.extra_flags=-Wl,--allow-multiple-definition ...
```

Verified that this keeps the *working* implementation: arduino-cli links
`libmicroros.a` ahead of `libpico.a`, so rcutils' definitions win. Confirm it in
the ELF by size rather than trusting link order — rcutils' are 0x4c-0x54 bytes,
pico-sdk's 0x16-0x28:

```bash
nm --print-size --defined-only <sketch>.ino.elf | grep __atomic_.*_8
```

`install.sh` does **not** pass this flag today, because the current firmware does
not pull in that object and enabling it globally would mask genuine duplicate
symbols. Add it when the action server lands.

## Only an x86_64 host can build this

The builder image is published for arm64, but the ARM cross toolchains bundled
inside it are **x86_64 ELF binaries** — ARM never shipped AArch64-host builds of
`gcc-arm-none-eabi-7-2017-q4-major`, which is what `library_generation.sh` uses
for `cortex_m0`. On the Pi the build dies almost immediately with the shell
trying to interpret an ELF as a script:

```
/uros_ws/gcc-arm-none-eabi-7-2017-q4-major/bin/arm-none-eabi-gcc: 1: ELF: not found
/uros_ws/gcc-arm-none-eabi-7-2017-q4-major/bin/arm-none-eabi-gcc: 1: Syntax error: Unterminated quoted string
```

```bash
$ docker run --rm --entrypoint readelf microros/micro_ros_static_library_builder:jazzy \
      -h /uros_ws/gcc-arm-none-eabi-7-2017-q4-major/bin/arm-none-eabi-gcc | grep Machine
  Machine:  Advanced Micro Devices X86-64     # ...on an arm64 host
```

That is fine, because **`libmicroros.a` is a Cortex-M0+ cross-compiled artifact
and does not care which machine produced it** — exactly why upstream ships one
precompiled in the first place. So the workflow is: build on x86_64, commit the
result, and every host installs it.

`prebuilt/libmicroros-<distro>-<target>.tar.gz` (about 3 MB) holds the built
`src/` tree plus `available_ros2_types` and `built_packages`, and
`prebuilt/*.stamp` records the digest it was built from. `install.sh` prefers a
matching bundle over rebuilding even on x86_64 — it takes seconds instead of 20
minutes — and re-exports it automatically whenever it does build, so the
committed copy stays in step.

**After changing anything in `ps_interfaces` or `colcon_rp2040.meta`, run
`install.sh` on an x86_64 machine and commit the regenerated bundle**, or the Pi
will refuse to install with a stamp mismatch. The tarball is written with
normalised metadata and `gzip -n`, so identical inputs produce an identical file
and the git history does not churn.

Emulating amd64 on the Pi (`--platform linux/amd64` with qemu binfmt) would work
in principle but takes hours; it is not worth it against a 3 MB file.

## How it gets used

`install.sh` drives this; there is nothing to run by hand. For each sketch
whose `firmware.yaml` has a `micro_ros_library:` block it will, before
compiling:

1. Copy the listed interface packages into the cloned
   `micro_ros_arduino/extras/library_generation/extra_packages/`.
2. Overwrite the three upstream `.meta` files in that clone with
   `colcon_rp2040.meta`, so it applies whichever one the requested target
   selects.
3. Run `microros/micro_ros_static_library_builder:$ROS_DISTRO` for that target
   only, which regenerates `src/` and `src/cortex-m0plus/libmicroros.a` in
   place.

...unless a committed `prebuilt/` bundle already matches, in which case it just
unpacks that and skips all of the above.

The rebuild takes 10-20 minutes and needs Docker, so it is **stamped**: a
digest of the interface definitions, the `.meta`, `$ROS_DISTRO`, the target and
the upstream commit is written to `.microros_build_stamp` in the clone, and the
rebuild is skipped when nothing has changed. `arduino/extra-libraries/` is
gitignored, so without the bundle every fresh machine would pay the build once —
and an arm64 machine could not pay it at all.

If the builder container cannot reach the network, `MICROROS_DOCKER_ARGS` is
passed through to `docker run` — useful for a proxy, or for `--network host`
on a machine where the default bridge is unavailable:

```bash
# "failed to add the host (vethX) <=> sandbox (vethY) pair interfaces:
#  operation not supported" means the veth module is not loaded. Prefer
#  fixing that:
sudo modprobe veth && echo veth | sudo tee /etc/modules-load.d/veth.conf
# ...but if you cannot, this works around it:
MICROROS_DOCKER_ARGS='--network host' ./install.sh
```

To force a rebuild:

```bash
rm arduino/extra-libraries/micro_ros_arduino/.microros_build_stamp
```

## Gotchas

- The builder wipes `src/` and only repopulates the target you asked for, so
  the clone ends up carrying `cortex-m0plus` alone. That is fine — nothing else
  in this repo builds for another MCU — but it means the clone is *not*
  interchangeable with a fresh upstream checkout.
- Everything the builder writes is owned by root. `install.sh` chowns the clone
  back afterwards; without that, a later `rm -rf` of the directory fails.
- `src/rmw_microxrcedds_c/config.h` in the clone reflects whichever target was
  built last, not necessarily the one linked. Trust the archive symbols, not
  that header.
- The `cortex_m0` target is cross-compiled inside the builder with
  `gcc-arm-none-eabi-7-2017-q4-major`, while arduino-pico 4.4.4 links with GCC
  14.2. That newlib gap is why `firmware/macro-ps/compat.cpp` has to re-export
  `__locale_ctype_ptr` — it was dropped from newer newlib but the archive's
  `rmw_validate_*_name()` still calls it. Keep that shim; it is not optional
  once anything validates a topic or node name.
- Adding *any* interface to `ps_interfaces` invalidates the stamp and buys
  another full rebuild. Batch changes.
- There is one shared clone of `micro_ros_arduino` and one archive in it, so
  the config here is effectively global. A second sketch declaring a
  `micro_ros_library:` block with a different `.meta` or a different package
  set would invalidate this one's stamp on every run and the two would
  rebuild each other in a loop. If that ever comes up, give each sketch its
  own clone rather than trying to reconcile the configs.
