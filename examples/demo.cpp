// demo.cpp — a runnable end-to-end dispatch simulation.
//
// Spins up a synthetic fleet, ticks it forward one second at a time with every
// driver moving and pinging, sends rider requests through the index, assigns
// drivers, and lets some drivers go silent so the expiry sweep has something to
// do. Prints what happens at each tick.
//
//   ./build/demo                                  # 45 ticks, 5000 drivers
//   ./build/demo --ticks 90 --drivers 20000 --seed 7
//
// Per-rider detail is printed for the first few ticks only; after that each
// tick prints one summary line, so a long run stays readable.

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geoindex/driver_index.hpp"

using namespace geoindex;

namespace {

// Chennai-ish bounding box, matching bench/benchmark.cpp.
constexpr double kLatMin = 12.90, kLatMax = 13.20;
constexpr double kLngMin = 80.10, kLngMax = 80.35;

constexpr uint64_t kTickMs = 1000;   // one simulated second per tick
constexpr uint64_t kPingTtlMs = 30'000;  // a driver silent for 30s is dropped
constexpr double kSearchRadiusM = 3000.0;
constexpr size_t kCandidates = 5;

struct SimDriver {
    uint64_t id;
    double lat, lng;
    bool available;
    bool pinging;        // false = phone died, stops reporting
    uint64_t busy_until; // simulated ms; while busy the driver is unavailable
};

struct Trip {
    uint64_t rider_id;
    uint64_t driver_id;
    double pickup_distance_m;
    uint64_t assigned_at_ms;
};

struct Options {
    size_t drivers = 5000;
    size_t ticks = 45;  // long enough for the 30s TTL sweep to fire
    size_t riders_per_tick = 4;
    unsigned seed = 20260905;
    double cell_size_m = 250.0;
};

Options parse(int argc, char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> double { return (i + 1 < argc) ? std::atof(argv[++i]) : 0.0; };
        if (a == "--drivers") o.drivers = static_cast<size_t>(next());
        else if (a == "--ticks") o.ticks = static_cast<size_t>(next());
        else if (a == "--riders") o.riders_per_tick = static_cast<size_t>(next());
        else if (a == "--seed") o.seed = static_cast<unsigned>(next());
        else if (a == "--cell") o.cell_size_m = next();
        else if (a == "--help") {
            std::printf("usage: demo [--drivers N] [--ticks N] [--riders N] [--seed N] [--cell M]\n");
            std::exit(0);
        }
    }
    return o;
}

}  // namespace

int main(int argc, char** argv) {
    const Options opt = parse(argc, argv);

    std::mt19937_64 rng(opt.seed);
    std::uniform_real_distribution<double> lat_d(kLatMin, kLatMax);
    std::uniform_real_distribution<double> lng_d(kLngMin, kLngMax);
    std::uniform_int_distribution<int> pct(0, 99);
    // ~22 m of movement per second is roughly 80 km/h; plenty for a demo.
    std::normal_distribution<double> step(0.0, 0.0002);

    DriverIndex index(opt.cell_size_m, (kLatMin + kLatMax) / 2.0);

    std::vector<SimDriver> fleet;
    fleet.reserve(opt.drivers);
    for (uint64_t id = 0; id < opt.drivers; ++id) {
        fleet.push_back(SimDriver{id, lat_d(rng), lng_d(rng), pct(rng) < 70, true, 0});
    }

    std::printf("=== dispatch simulation ===\n");
    std::printf("fleet=%zu  ticks=%zu  riders/tick=%zu  cell=%.0fm  radius=%.0fm  k=%zu  seed=%u\n\n",
                opt.drivers, opt.ticks, opt.riders_per_tick, opt.cell_size_m, kSearchRadiusM,
                kCandidates, opt.seed);

    uint64_t now_ms = 1'000'000;
    uint64_t next_rider_id = 1;

    std::vector<Trip> trips;
    size_t unmatched = 0;
    size_t total_requests = 0;
    double total_pickup_m = 0.0;
    size_t total_expired = 0;

    for (size_t tick = 0; tick < opt.ticks; ++tick) {
        now_ms += kTickMs;

        // --- 1. every driver moves and pings -------------------------------
        size_t pings = 0;
        for (SimDriver& d : fleet) {
            if (!d.pinging) continue;

            d.lat = std::clamp(d.lat + step(rng), kLatMin, kLatMax);
            d.lng = std::clamp(d.lng + step(rng), kLngMin, kLngMax);

            // A driver on a trip becomes available again when it finishes.
            if (d.busy_until != 0 && now_ms >= d.busy_until) {
                d.busy_until = 0;
                d.available = true;
            }

            index.upsert(d.id, d.lat, d.lng, d.available && d.busy_until == 0, now_ms);
            ++pings;
        }

        // --- 2. a few phones die each tick ---------------------------------
        // These drivers stop pinging. They stay in the index until their last
        // ping ages past the TTL, at which point expire() removes them. Until
        // then nearest() filters them out inline, so no rider is ever matched
        // to a stale driver.
        size_t died = 0;
        if (tick > 0 && tick % 3 == 0) {
            for (SimDriver& d : fleet) {
                if (d.pinging && pct(rng) < 2) {
                    d.pinging = false;
                    ++died;
                }
            }
        }

        // --- 3. riders request trips ---------------------------------------
        size_t matched_this_tick = 0;
        std::unordered_set<uint64_t> claimed_this_tick;
        std::vector<std::string> lines;

        for (size_t r = 0; r < opt.riders_per_tick; ++r) {
            const double rlat = lat_d(rng), rlng = lng_d(rng);
            const uint64_t rider = next_rider_id++;
            ++total_requests;

            auto candidates =
                index.nearest(rlat, rlng, kSearchRadiusM, kCandidates, now_ms, kPingTtlMs);

            // Take the nearest candidate not already offered to another rider
            // in this same tick. This is the one place the index alone is not
            // enough: two riders can be returned the same driver, so dispatch
            // has to arbitrate.
            const Match* chosen = nullptr;
            for (const Match& m : candidates) {
                if (claimed_this_tick.insert(m.driver_id).second) {
                    chosen = &m;
                    break;
                }
            }

            if (chosen == nullptr) {
                ++unmatched;
                char buf[160];
                std::snprintf(buf, sizeof(buf),
                              "    rider %-5llu  no driver available within %.0fm",
                              static_cast<unsigned long long>(rider), kSearchRadiusM);
                lines.emplace_back(buf);
                continue;
            }

            SimDriver& d = fleet[chosen->driver_id];
            d.available = false;
            // Trip length 30-120 simulated seconds.
            d.busy_until = now_ms + 30'000 + static_cast<uint64_t>(pct(rng)) * 900;
            index.upsert(d.id, d.lat, d.lng, false, now_ms);

            trips.push_back(Trip{rider, d.id, chosen->distance_m, now_ms});
            total_pickup_m += chosen->distance_m;
            ++matched_this_tick;

            char buf[160];
            std::snprintf(buf, sizeof(buf),
                          "    rider %-5llu -> driver %-6llu  pickup %6.0f m  (%zu candidates)",
                          static_cast<unsigned long long>(rider),
                          static_cast<unsigned long long>(d.id), chosen->distance_m,
                          candidates.size());
            lines.emplace_back(buf);
        }

        // --- 4. sweep out drivers that stopped reporting --------------------
        const size_t expired = index.expire(now_ms, kPingTtlMs);
        total_expired += expired;

        std::printf("tick %2zu  t=%llus  live=%-6zu pings=%-6zu matched=%zu/%zu  "
                    "phones_lost=%zu  expired=%zu\n",
                    tick, static_cast<unsigned long long>((now_ms - 1'000'000) / 1000),
                    index.size(), pings, matched_this_tick, opt.riders_per_tick, died, expired);
        // Detail for the opening ticks and for anything that failed to match;
        // otherwise the tick summary line above is enough.
        if (tick < 4) {
            for (const std::string& l : lines) std::printf("%s\n", l.c_str());
        } else {
            for (const std::string& l : lines) {
                if (l.find("no driver") != std::string::npos) std::printf("%s\n", l.c_str());
            }
        }
    }

    // --- summary -----------------------------------------------------------
    std::printf("\n=== summary ===\n");
    std::printf("  requests            %zu\n", total_requests);
    std::printf("  matched             %zu (%.1f%%)\n", trips.size(),
                total_requests ? 100.0 * static_cast<double>(trips.size()) /
                                     static_cast<double>(total_requests)
                               : 0.0);
    std::printf("  unmatched           %zu\n", unmatched);
    if (!trips.empty()) {
        std::vector<double> d;
        d.reserve(trips.size());
        for (const Trip& t : trips) d.push_back(t.pickup_distance_m);
        std::sort(d.begin(), d.end());
        std::printf("  pickup distance     mean %.0f m   p50 %.0f m   p99 %.0f m   max %.0f m\n",
                    total_pickup_m / static_cast<double>(trips.size()), d[d.size() / 2],
                    d[static_cast<size_t>(0.99 * static_cast<double>(d.size() - 1))], d.back());
    }
    std::printf("  drivers expired     %zu (stopped pinging, swept by TTL)\n", total_expired);
    std::printf("  drivers still live  %zu across %zu occupied cells\n", index.size(),
                index.cell_count());
    return 0;
}
