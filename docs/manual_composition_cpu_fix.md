# Fix: `manual_composition` burning ~110% CPU

Date: 2026-08-17  
Robot: Raspberry Pi 4B, VMX-pi HAL (`libvmxpi_hal_cpp.so`), `studica_control`  
Symptom: `manual_composition` sat at **104–116% CPU** (`top` PR −50 / `SCHED_FIFO`) for the whole nav run. Lowering Titan motor / encoder publish rates from 50 Hz to 10 Hz did not change it.

This document records **what was actually wrong**, **what we changed**, and **where in the tree**. Line numbers refer to the files as they exist in this workspace after the fix.

---

## 1. What was *not* the cause

`manual_composition` is a C++ ROS2 process that hosts VMX hardware components (Titan motors, IMU, power, …) via the closed-source Studica VMX HAL. We first assumed the ROS timers were the load:

| Hypothesis | Result |
|---|---|
| Titan motor update 50 Hz → 10 Hz | CPU unchanged (~110%) |
| Encoder / IMU ROS publish rate | CPU unchanged |
| Fewer ROS components enabled | CPU still ~100%+ as long as the HAL opened SPI |

The ROS rates only decide *how often we ask* the HAL. The HAL itself was spinning on SPI regardless.

---

## 2. Root cause (two bugs, both in closed-source HAL)

`libvmxpi_hal_cpp.so` cannot be patched. Both bugs were confirmed with `gdb` on the live process and `readelf` on the HAL (it is **not** built `-Bsymbolic`, so PLT calls can be interposed).

### Bug A — `gpioDelay` busy-spins for delays under 100 µs

The HAL's `wait_for_comm_ready()` polls SPI with `gpioDelay(1)` / `gpioDelay(10)` quanta. pigpio's `gpioDelay` **busy-spins** for any request `< ~100 µs`. Every SPI transfer (AHRS, CAN / Titan, power) therefore burns CPU for the whole wait, independent of ROS timer rates.

### Bug B — AHRS thread pacing uses seconds as milliseconds

`RegisterIORpi::Run()` computes the AHRS loop delay as:

```
update_rate_ms = 1.0 / update_rate_hz     // 0.05 at 20 Hz
```

instead of `1000.0 / update_rate_hz`. gdb on the live process showed:

```
update_rate_ms = 0.050000000000000003   (update_rate_hz = 20)
```

That value is truncated to `int` and passed to `std::condition_variable::wait_for(lock, 0ms)`, so the AHRS thread **never sleeps**. It re-reads the full 109-byte AHRS register bank over SPI thousands of times a second. Combined with Bug A, that is ~90% of one core by itself.

Together these two bugs produced the constant ~110% on `manual_composition`.

---

## 3. The fix: symbol interposition (two new source files)

We cannot edit the HAL, so we ship a small shared library that is **linked into `manual_composition` before `libvmxpi_hal_cpp.so`**. The dynamic linker then resolves the HAL's PLT calls to our implementations.

Both sources are compiled into one library named `gpio_delay_shim`.

### 3.1 `gpioDelay` → real kernel sleep

**File:** `src/studica_ws/src/studica_control/src/gpio_delay_shim.c`  
**New file.** 54 lines.

| Lines | What |
|---|---|
| 1–19 | Comment: why pigpio spins, why a kernel sleep is a legal replacement (`gpioDelay` is documented as “delay for **at least** micros”). |
| 20–25 | `_GNU_SOURCE`, `dlfcn.h`, `clock_gettime` / `clock_nanosleep`. |
| 27–32 | `now_us()` — monotonic clock for the return value pigpio would have given. |
| 34–53 | **`gpioDelay(uint32_t micros)`** — the interposed symbol. |

Behaviour:

- `micros >= 100` (lines 38–45): forward to the real pigpio `gpioDelay` via `dlsym(RTLD_NEXT, …)`. pigpio already sleeps in this range.
- `micros < 100` (lines 47–52): `clock_nanosleep(CLOCK_MONOTONIC, …)` instead of a busy spin. Typical Pi 4 wakeup is ~60–120 µs, so SPI waits get a little extra latency and **stop burning a core**.

```34:53:src/studica_ws/src/studica_control/src/gpio_delay_shim.c
uint32_t gpioDelay(uint32_t micros)
{
    uint64_t t0 = now_us();

    if (micros >= 100) {
        static gpio_delay_fn real;
        if (!real)
            real = (gpio_delay_fn)dlsym(RTLD_NEXT, "gpioDelay");
        if (real)
            return real(micros);
    }

    struct timespec ts = {
        (time_t)(micros / 1000000u),
        (long)(micros % 1000000u) * 1000L,
    };
    clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, NULL);
    return (uint32_t)(now_us() - t0);
}
```

### 3.2 AHRS `wait_for(0ms)` → clamp to 45 ms for HAL callers only

**File:** `src/studica_ws/src/studica_control/src/hal_pacing_fix.cpp`  
**New file.** 71 lines.

| Lines | What |
|---|---|
| 1–23 | Comment: units bug, gdb evidence, why we interpose `wait_for<long, milli>`. |
| 31–35 | `ms_t` alias; `kHalClampMs = 45` (intended 20 Hz period minus ~4 ms of IO). |
| 37–43 | `caller_is_vmx_hal()` — `dladdr` on the return address; only clamp if the caller lives in `libvmxpi_hal_cpp.so`. Other `wait_for` users are untouched. |
| 47–57 | `asm("…")` alias of the **exact mangled** libstdc++ symbol `std::condition_variable::wait_for<long, std::milli>`. |
| 59–70 | Implementation: if duration `<= 0` **and** the caller is the HAL, clamp to 45 ms; then call `wait_until` (a different symbol) so we do not recurse. |

```59:70:src/studica_ws/src/studica_control/src/hal_pacing_fix.cpp
std::cv_status vmx_hal_fixed_wait_for(
    std::condition_variable *cv,
    std::unique_lock<std::mutex> &lock,
    const ms_t &rel_time)
{
    ms_t d = rel_time;
    if (d.count() <= 0 && caller_is_vmx_hal(__builtin_return_address(0)))
        d = ms_t(kHalClampMs);
    return cv->wait_until(lock, std::chrono::steady_clock::now() + d);
}
```

45 ms is `1000/20 − 4`, i.e. the sleep the HAL *meant* to take at 20 Hz after SPI IO. The AHRS thread now actually sleeps between register-bank reads.

---

## 4. CMake: build the shim and force it in front of the HAL

**File:** `src/studica_ws/src/studica_control/CMakeLists.txt`

| Lines | What |
|---|---|
| 152–161 | Comment block describing both interpositions and why `--no-as-needed` is required. |
| 162–166 | `add_library(gpio_delay_shim SHARED src/gpio_delay_shim.c src/hal_pacing_fix.cpp)` and link `libdl` (`${CMAKE_DL_LIBS}`) for `dlsym` / `dladdr`. |
| 225–226 | `install(TARGETS gpio_delay_shim …)` so the `.so` lands in `install/studica_control/lib/`. |
| 250–269 | `add_executable(manual_composition …)` — **link order is the whole fix.** |

Critical link order (`CMakeLists.txt` 250–269):

```250:269:src/studica_ws/src/studica_control/CMakeLists.txt
add_executable(manual_composition src/manual_composition.cpp)
target_link_libraries(manual_composition
  -Wl,--no-as-needed
  gpio_delay_shim
  -Wl,--as-needed
  cobra_component
  ...
  /usr/local/lib/vmxpi/libvmxpi_hal_cpp.so
  /home/vmx/skill23_ws/local_sysroot/lib/libstudica_drivers.so
)
```

Why this order:

1. `--no-as-needed` (line 252) keeps a `DT_NEEDED` entry for `gpio_delay_shim` even though `manual_composition.cpp` never calls those symbols itself. Without it, the linker would drop the library and the HAL would keep using pigpio / libstdc++.
2. `gpio_delay_shim` is the **first** `NEEDED` library (line 253), so it is searched before the HAL (line 269).
3. `--as-needed` is restored (line 254) for the rest of the link so unused component libs are still eligible for dropping.

`INSTALL_RPATH` (lines 277–279) already includes `$ORIGIN/..`, which is `install/studica_control/lib/` at runtime — that is where `libgpio_delay_shim.so` is installed, so no extra `LD_LIBRARY_PATH` is required.

---

## 5. Supporting changes (not the root-cause fix)

These reduced contention / isolated the remaining HAL load. They did **not** drop CPU from 110% to 40% on their own.

### 5.1 HAL constructed non-realtime at 20 Hz

**File:** `src/studica_ws/src/studica_control/src/manual_composition.cpp`

| Lines | What |
|---|---|
| 26–28 | `VMXPi(false, 20)` — `realtime=false` keeps the HAL off `SCHED_FIFO`; AHRS update 20 Hz is enough for the EKF. |
| 35–37 | `SingleThreadedExecutor` instead of multi-threaded, so Titan / IMU / power do not pile concurrent `wait_for_comm_ready` waiters onto SPI. |

```26:37:src/studica_ws/src/studica_control/src/manual_composition.cpp
        // realtime=false keeps the HAL off SCHED_FIFO; AHRS at 20 Hz is enough
        // for the EKF and cuts the SPI busy-poll rate that was burning ~1 core.
        vmx_ = std::make_shared<VMXPi>(false, 20);
        ...
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
```

### 5.2 Pin the process to cores 2,3

Even after the shims, the HAL still does real SPI work. Unpinned, that work can preempt AMCL and the lidar merger on any core.

**File:** `src/skill23_studio/launch/studio_nav.launch.py` lines 40–60  
**File:** `src/skill23_studio/launch/studio_localize.launch.py` (same `_control_server_cmd`)

```40:54:src/skill23_studio/launch/studio_nav.launch.py
def _control_server_cmd(exec_path, hw_params, cpus='2,3'):
    ...
    cmd = ['sudo', '-E']
    if cpus:
        cmd += ['taskset', '-c', cpus]
```

The process is confined to the last two cores so AMCL / lidar stay on 0–1.

### 5.3 Measurement helper

**File:** `tools/measure_mc_cpu.sh` (new)

Idle-run CPU sampler. Starts `manual_composition` via the existing passwordless sudoers entry, waits 10 s for HAL init, then reads `/proc/<pid>/stat` over a window. Optional `PARAMS_OVERRIDE` points at a different `hw_params_*.yaml` so Titan/IMU can be toggled for A/B tests.

Usage:

```bash
bash tools/measure_mc_cpu.sh after_shim 30
```

---

## 6. Measured result

| Condition | `manual_composition` CPU |
|---|---|
| Before (busy-spin `gpioDelay` + `wait_for(0ms)`) | **~102.7–116%** |
| After both shims | **~42.5%** |

That is the drop that also freed the rest of the Pi (AMCL could keep up once motion started). Pinning to cores 2,3 is still in the launch files so the remaining ~40% cannot steal AMCL's cores.

---

## 7. Rebuild / verify

The shim is part of `studica_control`. After pulling these sources on the Pi:

```bash
cd ~/skill23_ws
colcon build --packages-select studica_control --symlink-install
# confirm the shim is the first NEEDED library
readelf -d install/studica_control/lib/studica_control/manual_composition | grep NEEDED
# should list libgpio_delay_shim.so before libvmxpi_hal_cpp.so
```

Then restart the Studio nav stack. `top` should show `manual_composit` around 40%, not 110%.

---

## 8. File list (everything touched for this CPU issue)

| Path | Role |
|---|---|
| `src/studica_ws/src/studica_control/src/gpio_delay_shim.c` | **New.** Interpose `gpioDelay` → `clock_nanosleep`. |
| `src/studica_ws/src/studica_control/src/hal_pacing_fix.cpp` | **New.** Interpose HAL `wait_for(0ms)` → 45 ms. |
| `src/studica_ws/src/studica_control/CMakeLists.txt` | Build + link the shim **before** the HAL (`152–166`, `225–226`, `250–269`). |
| `src/studica_ws/src/studica_control/src/manual_composition.cpp` | `VMXPi(false, 20)` + `SingleThreadedExecutor` (`26–37`). |
| `src/skill23_studio/launch/studio_nav.launch.py` | `taskset -c 2,3` (`40–60`). |
| `src/skill23_studio/launch/studio_localize.launch.py` | Same pin. |
| `tools/measure_mc_cpu.sh` | **New.** Repeatable CPU measurement. |

The HAL binary itself (`/usr/local/lib/vmxpi/libvmxpi_hal_cpp.so`) is **not** modified. Both bugs are still in that file; the shims sit in front of them at runtime.
