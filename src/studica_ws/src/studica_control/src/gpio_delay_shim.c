/* gpioDelay interposer — stops the VMX HAL burning a full CPU core.
 *
 * The closed-source libvmxpi_hal_cpp.so busy-polls SPI in
 * wait_for_comm_ready() using gpioDelay(1) / gpioDelay(10) quanta.
 * pigpio's gpioDelay busy-SPINS for requests under ~100 us, so every SPI
 * transfer (AHRS, CAN, power) spins the CPU — independent of any ROS
 * timer rates. That is the constant ~110% CPU on manual_composition.
 *
 * pigpio documents gpioDelay as "delay for AT LEAST micros", so replacing
 * the spin with a real kernel sleep is semantically safe: callers may
 * simply observe a slightly longer delay (~60-120 us wakeup granularity
 * on the Pi 4), which only adds a little latency per SPI transaction.
 *
 * This library is linked into manual_composition BEFORE the HAL, so the
 * dynamic linker resolves the HAL's own PLT calls to gpioDelay here
 * (the HAL is not built with -Bsymbolic; verified via readelf).
 * Requests >= 100 us are forwarded to the real pigpio implementation,
 * which already sleeps for that range.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdint.h>
#include <time.h>

typedef uint32_t (*gpio_delay_fn)(uint32_t);

static uint64_t now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)(ts.tv_nsec / 1000);
}

uint32_t gpioDelay(uint32_t micros)
{
    uint64_t t0 = now_us();

    if (micros >= 100) {
        /* pigpio already sleeps (does not spin) for >= 100 us. */
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
