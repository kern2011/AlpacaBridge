// AlpacaCore
// Copyright (c) 2025-2026 Joey Troy and contributors
//
// This file is part of AlpacaCore.
//
// AlpacaCore is licensed under the GNU Affero General Public License,
// version 3 or (at your option) any later version (AGPL-3.0-or-later),
// with an additional permission allowing combination with proprietary
// device-vendor SDKs. See the LICENSE file in this repository for the full
// license text and the vendor-SDK linking exception, or the license online at:
// https://www.gnu.org/licenses/agpl-3.0.html

// open-astro#666: on the EQ-AL55i Pro (mount code 0x09) ONLY, a Declination
// pulse guide is a classic-command position move (a one-axis GOTO: ":K" + wait,
// ":G" low-speed goto mode, ":S" target, ":J" start) instead of the timed
// speed-mode rate nudge every other board uses. RA guiding is unchanged on
// every board, and Dec guiding on every non-0x09 board is unchanged. These
// cases run the REAL protocol wrapper and UDP transport against
// FakeSkyWatcherMount (a loopback motor-controller simulator with a continuous
// axis model), gated on FakeMountProfile::eq_al55i() (mount code 0x09) so the
// new path is exercised against the real captured geometry.
//
// Discriminator: a Dec GOTO sends ":S" (set_goto_target); the speed-mode rate
// pulse never does (it sends ":I" set_step_period instead). Connect and
// tracking never send ":S", so frames_seen('S') cleanly separates the two.

#ifndef _WIN32

#include <alpacacore/telescope_driver.h>
#include <alpacacore/util/task_clock.h>
#include <alpacacore/vendor/skywatcher/skywatcher_telescope_driver.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include "catch2_compat.h"
#include "fake_skywatcher_mount.h"
#include "fake_task_clock.h"

namespace sw = alpacacore::vendor::skywatcher;
using alpacacore::test::FakeMountProfile;
using alpacacore::test::FakeSkyWatcherMount;
using alpacacore::test::FakeTaskClock;

namespace {

sw::ConnectionInfo endpoint(const FakeSkyWatcherMount& mount) {
    sw::ConnectionInfo info;
    info.type = sw::ConnectionType::Network;
    info.host = "127.0.0.1";
    info.udp_port = mount.port();
    info.response_timeout_ms = 250;
    return info;
}

std::unique_ptr<alpacacore::TelescopeDriver> connected_driver(const FakeSkyWatcherMount& mount,
                                                              double site_latitude_deg = 39.7392,
                                                              double site_longitude_deg = -104.9903) {
    auto driver = sw::create_skywatcher_telescope(0, endpoint(mount), site_latitude_deg, site_longitude_deg, 1609.0);
    driver->set_connected(true);
    return driver;
}

std::unique_ptr<alpacacore::TelescopeDriver> connected_driver(const FakeSkyWatcherMount& mount, FakeTaskClock& clock,
                                                              double site_latitude_deg = 39.7392,
                                                              double site_longitude_deg = -104.9903) {
    auto driver = sw::create_skywatcher_telescope(0, endpoint(mount), site_latitude_deg, site_longitude_deg, 1609.0, {},
                                                  {}, clock);
    driver->set_connected(true);
    return driver;
}

bool wait_until(const std::function<bool()>& pred, int timeout_ms) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return pred();
}

// ── FakeTaskClock stepping helpers (copied from test_skywatcher_async.cpp) ──
constexpr auto kRendezvous = std::chrono::milliseconds(2000);
constexpr auto kClockStep = std::chrono::milliseconds(50);

bool step_clock(FakeTaskClock& clock, std::chrono::milliseconds step) {
    if (!clock.wait_for_waiters(1, kRendezvous)) {
        return false;
    }
    clock.advance(step);
    return clock.wait_for_woken_settled(kRendezvous);
}

bool advance_through(FakeTaskClock& clock, std::chrono::milliseconds total) {
    for (std::chrono::milliseconds done{0}; done < total; done += kClockStep) {
        if (!step_clock(clock, std::min(kClockStep, total - done))) {
            return false;
        }
    }
    return true;
}

bool run_clock_until(FakeTaskClock& clock, const std::function<bool()>& pred, std::chrono::milliseconds budget) {
    for (std::chrono::milliseconds done{0}; done < budget; done += kClockStep) {
        const auto give_up = std::chrono::steady_clock::now() + kRendezvous;
        while (!clock.wait_for_waiters(1, std::chrono::milliseconds(20))) {
            if (pred()) {
                return true;
            }
            if (std::chrono::steady_clock::now() >= give_up) {
                return pred();
            }
        }
        if (pred()) {
            return true;
        }
        if (!step_clock(clock, kClockStep)) {
            return pred();
        }
    }
    return wait_until(pred, 200);
}

// Counts a Dec guide move of `duration_ms` must cover at the current Dec guide
// rate: distance = rate * duration, converted with the Dec counts-per-rev and
// rounded to the nearest count.
long expected_counts(double guide_dec_deg_per_sec, int duration_ms, uint32_t cpr_dec) {
    return std::lround(guide_dec_deg_per_sec * (static_cast<double>(duration_ms) / 1000.0) *
                       static_cast<double>(cpr_dec) / 360.0);
}

long axis_counts(FakeSkyWatcherMount& mount, int axis, uint32_t cpr) {
    return std::lround(mount.axis_degrees(axis) * static_cast<double>(cpr) / 360.0);
}

}  // namespace

// ── The GOTO sequence, right distance, both directions / hemispheres / piers ─
// A 0x09 Dec pulse sends a one-axis GOTO (":S" present) of the commanded count
// distance, and the sky-level direction follows the ASCOM contract (North
// raises reported Declination, South lowers it) in every hemisphere and on
// both pier-side branches -- which is exactly the sign the speed-mode rate
// pulse carries, so hemisphere / pier / measured_dec_axis_sense handling is
// unchanged.
TEST_CASE("SkyWatcher Dec guide GOTO - right distance and direction across hemispheres and pier sides (#666)",
          "[skywatcher][async][al55i][dec-guide]") {
    struct Case {
        const char* name;
        double latitude;
        double dec_axis_deg;  // pier-side branch: a2 > 0 vs a2 < 0
    };
    const Case cases[] = {
        {"north, a2>0 (pierEast)", 39.7392, 45.0},
        {"north, a2<0 (pierWest)", 39.7392, -45.0},
        {"south, a2>0 (pierEast)", -33.9, 45.0},
        {"south, a2<0 (pierWest)", -33.9, -45.0},
    };
    const int duration_ms = 500;  // ~10 Dec counts at the 0.5x default guide rate

    for (const auto& c : cases) {
        for (int direction : {0, 1}) {  // 0 = North, 1 = South
            INFO(c.name << " direction=" << (direction == 0 ? "North" : "South"));
            FakeTaskClock clock;
            FakeSkyWatcherMount mount(FakeMountProfile::eq_al55i(), clock);
            REQUIRE(mount.ok());
            auto driver = connected_driver(mount, clock, c.latitude);
            driver->set_tracking(true);
            mount.jump_axis_degrees(2, c.dec_axis_deg);

            const auto guide = driver->get_guide_rate();
            const long want = expected_counts(guide.dec, duration_ms, mount.kCprDec);
            REQUIRE(want > 0);

            const int s_before = mount.frames_seen('S');
            const long dec_before = axis_counts(mount, 2, mount.kCprDec);
            const double sky_dec_before = driver->get_declination();

            driver->pulse_guide(direction, duration_ms);
            REQUIRE(clock.wait_for_waiters(1, kRendezvous));  // dispatched, holding
            // Run the whole pulse so the end-of-move cache invalidation lets the
            // next get_declination() re-read the landed position (the driver's
            // position cache has a real-clock TTL that the fake's virtual motion
            // does not advance, so a mid-hold read would be stale).
            REQUIRE(run_clock_until(clock, [&] { return !driver->get_is_pulse_guiding(); },
                                    std::chrono::milliseconds(duration_ms + 3000)));

            const long dec_after = axis_counts(mount, 2, mount.kCprDec);
            const double sky_dec_after = driver->get_declination();

            // Exactly one ":S" goto-target command reached the board.
            CHECK(mount.frames_seen('S') == s_before + 1);
            // The axis moved the commanded distance (sign checked at the sky level).
            CHECK(std::abs(std::abs(dec_after - dec_before) - want) <= 1);
            // ASCOM contract, hemisphere/pier independent: North raises Dec.
            if (direction == 0) {
                CHECK(sky_dec_after > sky_dec_before);
            } else {
                CHECK(sky_dec_after < sky_dec_before);
            }
            driver->set_connected(false);
        }
    }
}

// Distance scales with duration (the 10 / 42 / 104-count bench table shape).
TEST_CASE("SkyWatcher Dec guide GOTO - distance scales with duration (#666)",
          "[skywatcher][async][al55i][dec-guide]") {
    for (int duration_ms : {500, 2000, 5000}) {
        INFO("duration_ms=" << duration_ms);
        FakeTaskClock clock;
        FakeSkyWatcherMount mount(FakeMountProfile::eq_al55i(), clock);
        REQUIRE(mount.ok());
        auto driver = connected_driver(mount, clock);
        driver->set_tracking(true);
        mount.jump_axis_degrees(2, 45.0);

        const auto guide = driver->get_guide_rate();
        const long want = expected_counts(guide.dec, duration_ms, mount.kCprDec);
        const long dec_before = axis_counts(mount, 2, mount.kCprDec);

        driver->pulse_guide(0, duration_ms);
        REQUIRE(clock.wait_for_waiters(1, kRendezvous));
        REQUIRE(advance_through(clock, std::chrono::milliseconds(300)));  // the goto lands (800x sidereal)
        const long moved = std::abs(axis_counts(mount, 2, mount.kCprDec) - dec_before);
        CHECK(std::abs(moved - want) <= 1);
        driver->set_connected(false);
    }
}

// A move that rounds to zero counts sends nothing to the board and still
// completes the pulse normally (no ":S", no motion, IsPulseGuiding ends false).
TEST_CASE("SkyWatcher Dec guide GOTO - zero-count move sends nothing (#666)",
          "[skywatcher][async][al55i][dec-guide]") {
    FakeSkyWatcherMount mount(FakeMountProfile::eq_al55i());
    REQUIRE(mount.ok());
    auto driver = connected_driver(mount);
    driver->set_tracking(true);
    mount.jump_axis_degrees(2, 45.0);

    const auto guide = driver->get_guide_rate();
    const int tiny_ms = 10;  // well under one Dec count at the default guide rate
    REQUIRE(expected_counts(guide.dec, tiny_ms, mount.kCprDec) == 0);

    const int s_before = mount.frames_seen('S');
    const long dec_before = axis_counts(mount, 2, mount.kCprDec);

    driver->pulse_guide(0, tiny_ms);
    REQUIRE(wait_until([&] { return !driver->get_is_pulse_guiding(); }, 3000));

    CHECK(mount.frames_seen('S') == s_before);           // no goto target command
    CHECK(axis_counts(mount, 2, mount.kCprDec) == dec_before);  // axis did not move
    driver->set_connected(false);
}

// A non-zero DeclinationRate means the Dec axis is already moving, so the pulse
// falls back to the speed-mode rate nudge (no ":S") for that pulse.
TEST_CASE("SkyWatcher Dec guide GOTO - non-zero DeclinationRate falls back to the rate pulse (#666)",
          "[skywatcher][async][al55i][dec-guide]") {
    FakeSkyWatcherMount mount(FakeMountProfile::eq_al55i());
    REQUIRE(mount.ok());
    auto driver = connected_driver(mount);
    driver->set_tracking(true);
    mount.jump_axis_degrees(2, 45.0);
    driver->set_declination_rate(10.0);  // arcsec/s, continuous (above the floor)
    REQUIRE(wait_until([&] { return mount.axis_running(2); }, 3000));

    const int s_before = mount.frames_seen('S');
    driver->pulse_guide(0, 500);  // North, while the offset runs
    REQUIRE(wait_until([&] { return !driver->get_is_pulse_guiding(); }, 5000));
    CHECK(mount.frames_seen('S') == s_before);  // rate pulse, not a goto

    driver->set_declination_rate(0.0);
    driver->set_connected(false);
}

// A superseding Dec pulse reaps the running one; the CANCELLED pulse must not
// touch the hardware (the superseding pulse re-commands). Each GOTO dispatch
// sends exactly one ":K" (stop-if-moving) and one ":J" (start); if the reaped
// pulse's cancel path wrongly issued its own stop, the second pulse's dispatch
// would see TWO extra ":K"s instead of one.
TEST_CASE("SkyWatcher Dec guide GOTO - a superseded pulse sends nothing on cancel (#666)",
          "[skywatcher][async][al55i][dec-guide]") {
    FakeTaskClock clock;
    FakeSkyWatcherMount mount(FakeMountProfile::eq_al55i(), clock);
    REQUIRE(mount.ok());
    auto driver = connected_driver(mount, clock);
    driver->set_tracking(true);
    mount.jump_axis_degrees(2, 45.0);

    driver->pulse_guide(0, 4000);                     // North, long: parks on the clock mid-hold
    REQUIRE(clock.wait_for_waiters(1, kRendezvous));  // first pulse dispatched + holding
    const int stop_before = mount.stop_count(2);
    const int start_before = mount.start_count(2);

    driver->pulse_guide(0, 500);  // supersedes: reaps the first, dispatches its own goto
    REQUIRE(clock.wait_for_waiters(1, kRendezvous));

    // Only the SECOND pulse's dispatch touched the Dec axis: one stop, one start.
    CHECK(mount.stop_count(2) == stop_before + 1);
    CHECK(mount.start_count(2) == start_before + 1);

    REQUIRE(run_clock_until(clock, [&] { return !driver->get_is_pulse_guiding(); }, std::chrono::milliseconds(8000)));
    driver->set_connected(false);
}

// IsPulseGuiding stays true until the requested duration has elapsed AND ":f"
// reports the axis stopped, bounded by a hard cap (kAxisStopTimeout, 5 s). It
// does NOT wait for the post-stop creep. Forcing ":f" to keep reporting running
// drives the hard cap: the flag must still be true well past the duration and
// must clear once the cap is reached, even though the axis never reports stopped.
TEST_CASE("SkyWatcher Dec guide GOTO - IsPulseGuiding holds for duration-and-stopped, bounded by the hard cap (#666)",
          "[skywatcher][async][al55i][dec-guide]") {
    FakeTaskClock clock;
    FakeSkyWatcherMount mount(FakeMountProfile::eq_al55i(), clock);
    REQUIRE(mount.ok());
    auto driver = connected_driver(mount, clock);
    driver->set_tracking(true);
    mount.jump_axis_degrees(2, 45.0);
    mount.force_running_reads(2, 1000000);  // ":f" never reports the Dec axis stopped

    driver->pulse_guide(0, 2000);  // North, 2 s
    REQUIRE(clock.wait_for_waiters(1, kRendezvous));
    CHECK(driver->get_is_pulse_guiding());

    // Past the requested duration but inside the cap: still guiding, because
    // ":f" has not reported stopped.
    REQUIRE(advance_through(clock, std::chrono::milliseconds(4000)));
    CHECK(driver->get_is_pulse_guiding());

    // Past duration + hard cap (2 s + 5 s): the cap releases it even though the
    // axis never reported stopped.
    REQUIRE(run_clock_until(clock, [&] { return !driver->get_is_pulse_guiding(); }, std::chrono::milliseconds(6000)));
    CHECK_FALSE(driver->get_is_pulse_guiding());
    driver->set_connected(false);
}

// A GOTO-mode guide move must NOT report Slewing: the driver defines Slewing as
// running AND NOT speed-mode, which a goto-mode axis would otherwise trip.
TEST_CASE("SkyWatcher Dec guide GOTO - Slewing stays false during the guide move (#666)",
          "[skywatcher][async][al55i][dec-guide]") {
    FakeTaskClock clock;
    FakeSkyWatcherMount mount(FakeMountProfile::eq_al55i(), clock);
    REQUIRE(mount.ok());
    auto driver = connected_driver(mount, clock);
    driver->set_tracking(true);
    mount.jump_axis_degrees(2, 45.0);
    mount.force_running_reads(2, 1000000);  // keep the Dec axis "running" so the move is clearly in flight

    driver->pulse_guide(0, 2000);  // North
    REQUIRE(clock.wait_for_waiters(1, kRendezvous));
    CHECK(driver->get_is_pulse_guiding());
    CHECK_FALSE(driver->get_slewing());  // goto-mode, but a guide move is not a slew
    driver->set_connected(false);
}

// After a Dec guide GOTO the axis is left in goto mode; the next motion
// command (a slew here) must re-establish its mode and run normally.
TEST_CASE("SkyWatcher Dec guide GOTO - a following slew works (#666)", "[skywatcher][async][al55i][dec-guide]") {
    FakeSkyWatcherMount mount(FakeMountProfile::eq_al55i());
    REQUIRE(mount.ok());
    auto driver = connected_driver(mount);
    driver->set_tracking(true);
    mount.jump_axis_degrees(2, 45.0);

    driver->pulse_guide(0, 500);  // North guide move via GOTO
    REQUIRE(wait_until([&] { return !driver->get_is_pulse_guiding(); }, 5000));

    const double lst = driver->get_sidereal_time();
    const double target_ra = std::fmod(lst - 2.0 + 24.0, 24.0);
    driver->slew_to_coordinates_async(target_ra, 40.0);
    REQUIRE(driver->get_slewing());
    REQUIRE(wait_until([&] { return !driver->get_slewing(); }, 30000));
    CHECK(std::abs(driver->get_declination() - 40.0) < 0.1);
    CHECK(driver->get_tracking());
    driver->set_connected(false);
}

// ── Every non-0x09 board keeps the speed-mode rate pulse, unchanged ──────────
// A Dec pulse on the Wave 100i and the EQM-35 Pro must still be the speed-mode
// nudge: no ":S" goto-target command, and the Dec axis still physically moves.
TEST_CASE("SkyWatcher Dec guide - non-0x09 boards still use the rate pulse (#666)",
          "[skywatcher][async][dec-guide]") {
    const struct {
        const char* name;
        FakeMountProfile profile;
    } boards[] = {
        {"Wave 100i (0x44)", FakeMountProfile::wave_100i()},
        {"EQM-35 Pro (0x32)", FakeMountProfile::eqm35_pro()},
    };
    for (const auto& b : boards) {
        INFO(b.name);
        FakeSkyWatcherMount mount(b.profile);
        REQUIRE(mount.ok());
        auto driver = connected_driver(mount);
        driver->set_tracking(true);
        mount.jump_axis_degrees(2, 45.0);

        const int s_before = mount.frames_seen('S');
        const double dec_before = mount.axis_degrees(2);
        driver->pulse_guide(0, 1500);  // North
        REQUIRE(wait_until([&] { return !driver->get_is_pulse_guiding(); }, 6000));

        CHECK(mount.frames_seen('S') == s_before);  // rate pulse: never a goto
        // The rate pulse still moved the Dec axis (east branch: +Dec is negative
        // axis motion in the northern hemisphere).
        const double moved_arcsec = (mount.axis_degrees(2) - dec_before) * 3600.0;
        CHECK(moved_arcsec < -3.0);
        driver->set_connected(false);
    }
}

#endif  // !_WIN32
