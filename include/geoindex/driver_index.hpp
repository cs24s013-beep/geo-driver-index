// driver_index.hpp — in-memory index of live drivers, answering
// "k nearest available drivers within r metres of this point".

#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include "geoindex/geo.hpp"

namespace geoindex {

struct Driver {
    uint64_t id;
    double lat;
    double lng;
    bool available;
    uint64_t last_seen_ms;  // wall clock of the driver's most recent ping
};

struct Match {
    uint64_t driver_id;
    double distance_m;
};

// Single-threaded by design. See README for the sharding plan; adding a lock
// here before measuring the single-threaded cost would be premature.
class DriverIndex {
  public:
    // 250 m is not a guess: it is the minimum of the cell-size sweep in
    // bench/, at 100k drivers, a 3 km radius and k=5. The curve is not
    // monotonic — smaller cells cut the candidate count but add hash lookups,
    // and below ~250 m the lookups win. Re-run the sweep for a different
    // driver density; do not inherit this number blindly.
    explicit DriverIndex(double cell_size_m = 250.0, double reference_lat = 13.0);

    // Insert a driver or update one that already exists. If the driver crossed
    // a cell boundary this moves it between buckets. O(1) amortised, plus a
    // scan of the old bucket, which is small.
    void upsert(uint64_t id, double lat, double lng, bool available, uint64_t now_ms);

    // Returns false if the driver was not present.
    bool remove(uint64_t id);

    // Drop every driver whose last ping is older than ttl_ms. Returns how many
    // were removed. This is the sweep; nearest() also filters stale drivers
    // inline so a query is never served from stale data between sweeps.
    size_t expire(uint64_t now_ms, uint64_t ttl_ms);

    // The query. Results are sorted by ascending distance and capped at k.
    // Drivers that are unavailable or stale are excluded.
    //
    // NOT THREAD-SAFE despite being const: it reuses an internal scratch
    // buffer to keep allocation out of the hot path. That trade is deliberate
    // and is the first thing to undo when adding concurrency.
    std::vector<Match> nearest(double lat, double lng, double radius_m, size_t k,
                               uint64_t now_ms, uint64_t ttl_ms) const;

    // The first implementation: scan every cell the radius touches, then sort.
    // Correct, but does work proportional to the number of drivers inside the
    // radius rather than to k. Kept because the gap between this and nearest()
    // is the most interesting result this repo produces.
    std::vector<Match> nearest_radius_scan(double lat, double lng, double radius_m, size_t k,
                                           uint64_t now_ms, uint64_t ttl_ms) const;

    size_t size() const noexcept { return driver_cell_.size(); }
    size_t cell_count() const noexcept { return cells_.size(); }
    const CellGrid& grid() const noexcept { return grid_; }

    // Diagnostics for the benchmark: how many drivers a query had to look at
    // before distance filtering. This is the number that moves when you change
    // cell size, and the reason the sweep is worth running.
    mutable uint64_t last_query_candidates = 0;

  private:
    CellGrid grid_;

    // Driver records live inside the cell buckets, not behind a pointer or a
    // second hash lookup. A query walks contiguous memory; that locality is
    // most of the performance.
    std::unordered_map<CellKey, std::vector<Driver>> cells_;

    // driver id -> the cell it currently sits in, so upsert and remove can
    // find it without scanning every bucket.
    std::unordered_map<uint64_t, CellKey> driver_cell_;

    mutable std::vector<Match> scratch_;

    // swap-and-pop removal from a bucket; order within a bucket is meaningless
    static bool erase_from_bucket(std::vector<Driver>& bucket, uint64_t id);
};

// Brute-force reference implementation.
//
// This exists to be the ORACLE for the tests: DriverIndex::nearest must return
// exactly what this returns, for every random input. It is also timed in the
// benchmark to show the algorithmic win, but that comparison is not the
// headline — see bench/redis_baseline.sh for the one that is.
class LinearScanIndex {
  public:
    void upsert(uint64_t id, double lat, double lng, bool available, uint64_t now_ms);
    bool remove(uint64_t id);
    std::vector<Match> nearest(double lat, double lng, double radius_m, size_t k,
                               uint64_t now_ms, uint64_t ttl_ms) const;
    size_t size() const noexcept { return drivers_.size(); }

  private:
    std::vector<Driver> drivers_;
    std::unordered_map<uint64_t, size_t> pos_;
};

}  // namespace geoindex
