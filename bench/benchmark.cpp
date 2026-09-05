// benchmark.cpp — the measurement. This is the project's actual output.
//
// Reports p50/p90/p99/p99.9, never just a mean. Tail latency is what a
// dispatch system is judged on; an average hides exactly the requests that
// matter.

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>
#include <vector>

#include "geoindex/driver_index.hpp"

using namespace geoindex;
using Clock = std::chrono::steady_clock;

namespace {

struct Config {
    size_t drivers = 100'000;
    size_t queries = 200'000;
    size_t updates = 200'000;
    double radius_m = 3000.0;
    size_t k = 5;
    double cell_size_m = 250.0;  // measured optimum, see --sweep
    bool sweep = false;
    bool compare_linear = false;
    unsigned seed = 20260905;
};

// City bounding box (Chennai). Uniform density is not realistic — real driver
// distribution is heavily clustered — so treat these numbers as a floor and
// note the caveat in the README rather than quietly ignoring it.
constexpr double kLatMin = 12.90, kLatMax = 13.20;
constexpr double kLngMin = 80.10, kLngMax = 80.35;

double percentile(std::vector<double>& sorted_us, double p) {
    if (sorted_us.empty()) return 0.0;
    const size_t idx = static_cast<size_t>(p * (static_cast<double>(sorted_us.size()) - 1));
    return sorted_us[idx];
}

struct Stats {
    double p50, p90, p99, p999, mean, max;
    double qps;
};

Stats summarise(std::vector<double>& us, double wall_seconds) {
    std::sort(us.begin(), us.end());
    double sum = 0.0;
    for (double v : us) sum += v;
    return Stats{percentile(us, 0.50), percentile(us, 0.90),  percentile(us, 0.99),
                 percentile(us, 0.999), sum / static_cast<double>(us.size()),
                 us.back(),             static_cast<double>(us.size()) / wall_seconds};
}

void print_stats(const char* label, const Stats& s) {
    std::printf("  %-22s p50=%8.3f  p90=%8.3f  p99=%8.3f  p99.9=%8.3f  max=%9.3f  mean=%8.3f  %10.0f op/s\n",
                label, s.p50, s.p90, s.p99, s.p999, s.max, s.mean, s.qps);
}

// Adapter so the same timing harness can drive either query strategy.
struct RadiusScanAdapter {
    const DriverIndex& inner;
    mutable uint64_t last_query_candidates = 0;
    std::vector<Match> nearest(double lat, double lng, double r, size_t k, uint64_t now,
                               uint64_t ttl) const {
        auto out = inner.nearest_radius_scan(lat, lng, r, k, now, ttl);
        last_query_candidates = inner.last_query_candidates;
        return out;
    }
};

template <typename Index>
void populate(Index& idx, size_t n, std::mt19937_64& rng, uint64_t now_ms) {
    std::uniform_real_distribution<double> lat_d(kLatMin, kLatMax);
    std::uniform_real_distribution<double> lng_d(kLngMin, kLngMax);
    std::uniform_int_distribution<int> avail_d(0, 9);
    for (uint64_t id = 0; id < n; ++id) {
        // ~70% of drivers available at any moment.
        idx.upsert(id, lat_d(rng), lng_d(rng), avail_d(rng) < 7, now_ms);
    }
}

template <typename Index>
Stats bench_queries(const Index& idx, const Config& cfg, std::mt19937_64& rng, uint64_t now_ms,
                    uint64_t* avg_candidates = nullptr) {
    std::uniform_real_distribution<double> lat_d(kLatMin, kLatMax);
    std::uniform_real_distribution<double> lng_d(kLngMin, kLngMax);

    // Pre-generate the query points so RNG cost stays out of the timed region.
    std::vector<std::pair<double, double>> points(cfg.queries);
    for (auto& p : points) p = {lat_d(rng), lng_d(rng)};

    // Warm-up: touch the structure so we are not measuring first-touch page
    // faults and a cold cache.
    for (size_t i = 0; i < std::min<size_t>(5000, cfg.queries); ++i) {
        auto r = idx.nearest(points[i].first, points[i].second, cfg.radius_m, cfg.k, now_ms, 30'000);
        (void)r;
    }

    std::vector<double> us;
    us.reserve(cfg.queries);
    uint64_t candidate_total = 0;
    size_t sink = 0;

    const auto wall_start = Clock::now();
    for (const auto& p : points) {
        const auto t0 = Clock::now();
        auto r = idx.nearest(p.first, p.second, cfg.radius_m, cfg.k, now_ms, 30'000);
        const auto t1 = Clock::now();
        us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
        sink += r.size();  // keep the optimiser honest
        if constexpr (!std::is_same_v<Index, LinearScanIndex>) {
            candidate_total += idx.last_query_candidates;
        }
    }
    const double wall = std::chrono::duration<double>(Clock::now() - wall_start).count();

    if (sink == SIZE_MAX) std::printf("unreachable %zu\n", sink);
    if (avg_candidates) *avg_candidates = candidate_total / cfg.queries;
    return summarise(us, wall);
}

Stats bench_updates(DriverIndex& idx, const Config& cfg, std::mt19937_64& rng, uint64_t now_ms) {
    // Drivers move ~10-30 m between pings, which is what a 1 Hz ping at urban
    // speed looks like. Most such moves stay inside the same cell; that is the
    // realistic case and it is why upserts are cheap.
    std::uniform_int_distribution<uint64_t> id_d(0, cfg.drivers - 1);
    std::normal_distribution<double> jitter(0.0, 0.0002);  // ~22 m

    std::vector<std::tuple<uint64_t, double, double>> ops(cfg.updates);
    std::uniform_real_distribution<double> lat_d(kLatMin, kLatMax);
    std::uniform_real_distribution<double> lng_d(kLngMin, kLngMax);
    for (auto& op : ops) op = {id_d(rng), lat_d(rng) + jitter(rng), lng_d(rng) + jitter(rng)};

    std::vector<double> us;
    us.reserve(cfg.updates);
    const auto wall_start = Clock::now();
    for (const auto& op : ops) {
        const auto t0 = Clock::now();
        idx.upsert(std::get<0>(op), std::get<1>(op), std::get<2>(op), true, now_ms);
        const auto t1 = Clock::now();
        us.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    const double wall = std::chrono::duration<double>(Clock::now() - wall_start).count();
    return summarise(us, wall);
}

void run_sweep(const Config& base) {
    std::printf("\nCELL SIZE SWEEP  (%zu drivers, %.0f m radius, k=%zu)\n", base.drivers,
                base.radius_m, base.k);
    std::printf("  Trade-off: small cells mean more hash lookups per query, large cells\n"
                "  mean more drivers scanned and discarded. The minimum is measured, not guessed.\n\n");

    for (double cell : {100.0, 250.0, 500.0, 1000.0, 2000.0}) {
        Config cfg = base;
        cfg.cell_size_m = cell;
        cfg.queries = 50'000;

        std::mt19937_64 rng(cfg.seed);
        const uint64_t now = 1'000'000;
        DriverIndex idx(cell, 13.0);
        populate(idx, cfg.drivers, rng, now);

        uint64_t candidates = 0;
        Stats s = bench_queries(idx, cfg, rng, now, &candidates);

        char label[64];
        std::snprintf(label, sizeof(label), "cell=%6.0fm", cell);
        std::printf("  %-12s cells=%7zu  candidates/query=%6llu  p50=%7.2fus  p99=%7.2fus  %9.0f q/s\n",
                    label, idx.cell_count(), static_cast<unsigned long long>(candidates), s.p50,
                    s.p99, s.qps);
    }
}

Config parse_args(int argc, char** argv) {
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> double { return (i + 1 < argc) ? std::atof(argv[++i]) : 0.0; };
        if (a == "--drivers") cfg.drivers = static_cast<size_t>(next());
        else if (a == "--queries") cfg.queries = static_cast<size_t>(next());
        else if (a == "--updates") cfg.updates = static_cast<size_t>(next());
        else if (a == "--radius") cfg.radius_m = next();
        else if (a == "--k") cfg.k = static_cast<size_t>(next());
        else if (a == "--cell") cfg.cell_size_m = next();
        else if (a == "--seed") cfg.seed = static_cast<unsigned>(next());
        else if (a == "--sweep") cfg.sweep = true;
        else if (a == "--compare-linear") cfg.compare_linear = true;
        else if (a == "--help") {
            std::printf("usage: bench [--drivers N] [--queries N] [--updates N] [--radius M]\n"
                        "             [--k N] [--cell M] [--seed N] [--sweep] [--compare-linear]\n");
            std::exit(0);
        }
    }
    return cfg;
}

}  // namespace

int main(int argc, char** argv) {
    const Config cfg = parse_args(argc, argv);
    const uint64_t now = 1'000'000;

    std::printf("geo-driver-index benchmark\n");
    std::printf("  drivers=%zu  queries=%zu  updates=%zu  radius=%.0fm  k=%zu  cell=%.0fm  seed=%u\n\n",
                cfg.drivers, cfg.queries, cfg.updates, cfg.radius_m, cfg.k, cfg.cell_size_m,
                cfg.seed);

    std::mt19937_64 rng(cfg.seed);
    DriverIndex idx(cfg.cell_size_m, 13.0);
    populate(idx, cfg.drivers, rng, now);
    std::printf("  loaded %zu drivers into %zu occupied cells\n\n", idx.size(), idx.cell_count());

    std::printf("QUERY LATENCY (microseconds)\n");

    // Strategy 1: the first version — scan every cell the radius touches.
    RadiusScanAdapter scan_view{idx};
    uint64_t scan_candidates = 0;
    Stats s_scan = bench_queries(scan_view, cfg, rng, now, &scan_candidates);
    print_stats("radius scan", s_scan);
    std::printf("  %-22s %llu drivers examined per query\n", "",
                static_cast<unsigned long long>(scan_candidates));

    // Strategy 2: expanding ring with early exit.
    uint64_t candidates = 0;
    Stats q = bench_queries(idx, cfg, rng, now, &candidates);
    print_stats("expanding ring", q);
    std::printf("  %-22s %llu drivers examined per query\n", "",
                static_cast<unsigned long long>(candidates));
    std::printf("  %-22s ring vs radius scan: %.1fx at p50, %.1fx at p99\n\n", "",
                s_scan.p50 / q.p50, s_scan.p99 / q.p99);

    if (cfg.compare_linear) {
        // Sanity reference only. The headline comparison is against Redis
        // GEOSEARCH (see bench/redis_baseline.sh) — beating your own naive
        // version proves nothing an interviewer will credit.
        Config small = cfg;
        small.queries = std::min<size_t>(cfg.queries, 2000);
        std::mt19937_64 rng2(cfg.seed);
        LinearScanIndex linear;
        populate(linear, cfg.drivers, rng2, now);
        Stats l = bench_queries(linear, small, rng2, now);
        print_stats("linear scan (ref)", l);
        std::printf("  %-22s speedup at p50: %.1fx\n\n", "", l.p50 / q.p50);
    }

    std::printf("UPDATE LATENCY (microseconds)\n");
    Stats u = bench_updates(idx, cfg, rng, now);
    print_stats("upsert", u);

    if (cfg.sweep) run_sweep(cfg);

    std::printf("\nNote: timing uses steady_clock per operation (~20-30ns overhead). For\n"
                "operations this short, report the median of at least three runs.\n");
    return 0;
}
