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

**`RCUTILS_NO_64_ATOMIC` ON → OFF.** Despite the name this flag *adds*
`src/atomic_64bits.c` to micro-ROS's rcutils fork:

```cmake
option(RCUTILS_NO_64_ATOMIC "Enable alternative support for 64 bits atomic
       operations in platforms with no native support." OFF)
...
$<$<BOOL:${RCUTILS_NO_64_ATOMIC}>:src/atomic_64bits.c>
```

It is meant for platforms without native 64-bit atomics. The RP2040 has them —
pico-sdk's `pico_atomic` provides `__atomic_{load,store,exchange,fetch_add}_8`
in `libpico.a(atomic.c.o)` — so enabling it produces four duplicate symbols and
the link fails:

```
multiple definition of `__atomic_fetch_add_8';
  libmicroros.a(librcutils-atomic_64bits.c.obj): first defined here
```

This is latent in the **upstream** archive too (the objects are byte-identical),
but it only bites once a sketch pulls in enough of rcutils to drag that object
in — which an action server does and a lone publisher does not. That is why the
current IMU-only firmware links against the stock archive quite happily and the
first thing to use `/lens/home` would not have.

Turning it off is the correct fix rather than a workaround: pico-sdk's
implementations are the dual-core-safe ones for this chip, which rcutils'
generic emulation is not.

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

The rebuild takes 10-20 minutes and needs Docker, so it is **stamped**: a
digest of the interface definitions, the `.meta`, `$ROS_DISTRO`, the target and
the upstream commit is written to `.microros_build_stamp` in the clone, and the
rebuild is skipped when nothing has changed. `arduino/extra-libraries/` is
gitignored, so the first `install.sh` on a fresh machine always pays it once.

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
