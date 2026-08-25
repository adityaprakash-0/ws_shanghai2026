/* Fix for the VMX HAL AHRS pacing bug (~90% of one core).
 *
 * RegisterIORpi::Run() in the closed-source libvmxpi_hal_cpp.so computes
 * its loop delay as   update_rate_ms = 1.0 / update_rate_hz   (= 0.05 for
 * 20 Hz) instead of 1000.0 / update_rate_hz — a seconds-vs-milliseconds
 * units bug, confirmed with gdb on the live process:
 *
 *     update_rate_ms = 0.050000000000000003   (update_rate_hz = 20)
 *
 * The value is truncated to an int and passed to
 * std::condition_variable::wait_for(lock, 0ms), so the AHRS thread never
 * sleeps: it re-reads the full 109-byte AHRS register bank over SPI
 * back-to-back, thousands of times a second (pigpio aux-SPI transfers
 * busy-poll the FIFO for their whole duration).
 *
 * wait_for<long, std::milli> is a weak template instantiation that the HAL
 * calls through the PLT, so this library (linked before the HAL in
 * manual_composition) provides the definition the dynamic linker picks.
 * Behaviour is identical to libstdc++ except: when the requested duration
 * is <= 0 AND the caller is inside libvmxpi_hal_cpp.so, the duration is
 * clamped to 45 ms (the delay the HAL intended for its fixed 20 Hz rate:
 * 1000/20 - 4 ms of IO time). Non-HAL callers are never affected.
 */
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>

#include <dlfcn.h>

namespace {

using ms_t = std::chrono::duration<long, std::milli>;

constexpr long kHalClampMs = 45;

bool caller_is_vmx_hal(void *ret_addr)
{
    Dl_info info;
    if (dladdr(ret_addr, &info) == 0 || info.dli_fname == nullptr)
        return false;
    return std::strstr(info.dli_fname, "libvmxpi_hal_cpp") != nullptr;
}

}  // namespace

// Exact mangled name of std::cv_status std::condition_variable::wait_for<
//   long, std::ratio<1,1000>>(std::unique_lock<std::mutex>&,
//   const std::chrono::duration<long, std::ratio<1,1000>>&).
// A free function with the object pointer as first argument matches the
// member-function calling convention on AArch64.
std::cv_status vmx_hal_fixed_wait_for(
    std::condition_variable *cv,
    std::unique_lock<std::mutex> &lock,
    const ms_t &rel_time)
    asm("_ZNSt18condition_variable8wait_forIlSt5ratioILl1ELl1000EEEESt9cv_"
        "statusRSt11unique_lockISt5mutexERKNSt6chrono8durationIT_T0_EE");

std::cv_status vmx_hal_fixed_wait_for(
    std::condition_variable *cv,
    std::unique_lock<std::mutex> &lock,
    const ms_t &rel_time)
{
    ms_t d = rel_time;
    if (d.count() <= 0 && caller_is_vmx_hal(__builtin_return_address(0)))
        d = ms_t(kHalClampMs);
    // wait_until (a different symbol) implements the standard semantics;
    // calling wait_for here would recurse into this function.
    return cv->wait_until(lock, std::chrono::steady_clock::now() + d);
}
