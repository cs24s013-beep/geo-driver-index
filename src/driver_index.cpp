#include "geoindex/driver_index.hpp"

#include <algorithm>
#include <cmath>

namespace geoindex {

DriverIndex::DriverIndex(double cell_size_m, double reference_lat)
    : grid_(cell_size_m, reference_lat) {
    scratch_.reserve(1024);
}

bool DriverIndex::erase_from_bucket(std::vector<Driver>& bucket, uint64_t id) {
    for (size_t i = 0; i < bucket.size(); ++i) {
        if (bucket[i].id == id) {
            bucket[i] = bucket.back();  // swap with last, then shrink
            bucket.pop_back();
            return true;
        }
    }
    return false;
}

void DriverIndex::upsert(uint64_t id, double lat, double lng, bool available, uint64_t now_ms) {
    const CellKey new_cell = grid_.key(lat, lng);
    const Driver record{id, lat, lng, available, now_ms};

    auto it = driver_cell_.find(id);
    if (it == driver_cell_.end()) {
        cells_[new_cell].push_back(record);
        driver_cell_.emplace(id, new_cell);
        return;
    }

    const CellKey old_cell = it->second;
    if (old_cell == new_cell) {
        // Common case by a wide margin: a driver moving a few metres stays in
        // the same cell, so this is an in-place field update with no bucket
        // churn at all. Worth knowing when someone asks about update cost.
        auto& bucket = cells_[old_cell];
        for (auto& d : bucket) {
            if (d.id == id) {
                d = record;
                return;
            }
        }
        bucket.push_back(record);  // defensive: maps disagreed, repair
        return;
    }

    auto old_it = cells_.find(old_cell);
    if (old_it != cells_.end()) {
        erase_from_bucket(old_it->second, id);
        // Erasing empty buckets keeps cells_ from growing without bound as
        // drivers sweep across the city over a long run.
        if (old_it->second.empty()) cells_.erase(old_it);
    }
    cells_[new_cell].push_back(record);
    it->second = new_cell;
}

bool DriverIndex::remove(uint64_t id) {
    auto it = driver_cell_.find(id);
    if (it == driver_cell_.end()) return false;

    auto cell_it = cells_.find(it->second);
    if (cell_it != cells_.end()) {
        erase_from_bucket(cell_it->second, id);
        if (cell_it->second.empty()) cells_.erase(cell_it);
    }
    driver_cell_.erase(it);
    return true;
}

size_t DriverIndex::expire(uint64_t now_ms, uint64_t ttl_ms) {
    size_t removed = 0;
    for (auto cell_it = cells_.begin(); cell_it != cells_.end();) {
        auto& bucket = cell_it->second;
        for (size_t i = 0; i < bucket.size();) {
            if (now_ms - bucket[i].last_seen_ms > ttl_ms) {
                driver_cell_.erase(bucket[i].id);
                bucket[i] = bucket.back();
                bucket.pop_back();
                ++removed;
                // no ++i: the swapped-in element still needs checking
            } else {
                ++i;
            }
        }
        cell_it = bucket.empty() ? cells_.erase(cell_it) : std::next(cell_it);
    }
    return removed;
}

std::vector<Match> DriverIndex::nearest_radius_scan(double lat, double lng, double radius_m,
                                                    size_t k, uint64_t now_ms,
                                                    uint64_t ttl_ms) const {
    last_query_candidates = 0;
    if (k == 0 || radius_m <= 0.0) return {};

    // A bounding box that is a hair too small silently loses drivers; one that
    // is a hair too large only costs a few extra distance computations that
    // then get rejected. The asymmetry is total, so widen by 1 ppm (about
    // 3 mm over a 3 km radius) and stop worrying about floating-point rounding
    // between the box arithmetic and haversine.
    constexpr double kBoxMargin = 1.000001;

    // Degrees of latitude covered by the search radius.
    const double dlat_deg = radius_m * kBoxMargin / kMetersPerDegLat;

    // Degrees of longitude — and this one is subtle enough to have been a real
    // bug here, caught by the randomised oracle test.
    //
    // Degrees of longitude get shorter as you move away from the equator. If
    // you size the box using the QUERY's latitude, then a driver near the
    // box's northern corner (in the northern hemisphere) needs a slightly
    // WIDER longitude delta than the one you computed, and falls outside the
    // box even though it is genuinely inside the circle. The error is under a
    // metre at city scale, which is exactly why it survives casual testing and
    // shows up once every few thousand random queries.
    //
    // Fix: size the box at whichever latitude in the search band is furthest
    // from the equator, which is the worst case. The box is then guaranteed to
    // be a superset of the circle, which is the only property the pre-filter
    // needs.
    const double worst_lat = std::max(std::fabs(lat - dlat_deg), std::fabs(lat + dlat_deg));
    const double cos_worst = std::max(std::cos(deg2rad(worst_lat)), 1e-9);  // pole guard
    const double dlng_deg = radius_m * kBoxMargin / (kMetersPerDegLat * cos_worst);

    // THE BOUNDARY CASE. A driver 10 m away can sit in a different cell to the
    // rider. Scanning only the containing cell silently loses them. So we walk
    // every cell the bounding box of the circle touches, which by construction
    // includes the neighbours. Get this wrong and the index returns plausible
    // but incorrect answers, which is the worst kind of bug — it looks fine.
    const int32_t row_lo = grid_.row(lat - dlat_deg);
    const int32_t row_hi = grid_.row(lat + dlat_deg);
    const int32_t col_lo = grid_.col(lng - dlng_deg);
    const int32_t col_hi = grid_.col(lng + dlng_deg);

    scratch_.clear();
    for (int32_t r = row_lo; r <= row_hi; ++r) {
        for (int32_t c = col_lo; c <= col_hi; ++c) {
            auto it = cells_.find(CellGrid::pack(r, c));
            if (it == cells_.end()) continue;

            for (const Driver& d : it->second) {
                ++last_query_candidates;
                if (!d.available) continue;
                if (now_ms - d.last_seen_ms > ttl_ms) continue;

                // Cheap rectangular reject before the trigonometry. The
                // bounding box is a superset of the circle, so this can only
                // discard drivers that haversine would have discarded anyway.
                if (std::fabs(d.lat - lat) > dlat_deg) continue;
                if (std::fabs(d.lng - lng) > dlng_deg) continue;

                const double dist = haversine_m(lat, lng, d.lat, d.lng);
                if (dist <= radius_m) scratch_.push_back(Match{d.id, dist});
            }
        }
    }

    // Partial sort: we only need the k smallest, not a full ordering of a few
    // thousand candidates.
    const size_t n = std::min(k, scratch_.size());
    std::partial_sort(scratch_.begin(), scratch_.begin() + static_cast<long>(n), scratch_.end(),
                      [](const Match& a, const Match& b) {
                          // Tie-break on id so results are deterministic and
                          // can be compared against the oracle exactly.
                          if (a.distance_m != b.distance_m) return a.distance_m < b.distance_m;
                          return a.driver_id < b.driver_id;
                      });

    return std::vector<Match>(scratch_.begin(), scratch_.begin() + static_cast<long>(n));
}


// The query that actually ships: expanding ring search.
//
// WHY THIS EXISTS. The radius scan above is correct and was the first version.
// Profiling it exposed the problem: with 100k drivers spread over a city, a
// 3 km radius genuinely contains a few thousand of them, so returning the 5
// nearest meant computing several thousand distances and throwing almost all
// of them away. The work scaled with the search area, not with k.
//
// The insight is that cells are ordered by distance. Walk outward from the
// rider's own cell one ring at a time, keeping the best k found so far in a
// max-heap. Every cell in ring r+1 is at least r*cell_size away from the
// rider, so the moment the heap holds k drivers and its worst entry is closer
// than that bound, no unexplored cell can improve the answer and the search
// stops. For k=5 that is usually one or two rings — tens of candidates
// instead of thousands.
//
// This is the same idea Redis GEOSEARCH uses when asked for COUNT nearest.
std::vector<Match> DriverIndex::nearest(double lat, double lng, double radius_m, size_t k,
                                        uint64_t now_ms, uint64_t ttl_ms) const {
    last_query_candidates = 0;
    if (k == 0 || radius_m <= 0.0) return {};

    // Conservative metric size of one cell at THIS latitude. The grid was laid
    // out at a reference latitude, so a cell's east-west extent shrinks as you
    // move away from it. Taking the smaller of the two dimensions keeps the
    // ring lower bound valid, which is what makes early exit safe.
    const double cell_h_m = grid_.lat_step_deg() * kMetersPerDegLat;
    const double cell_w_m = grid_.lng_step_deg() * kMetersPerDegLat *
                            std::max(std::cos(deg2rad(lat)), 1e-9);
    const double min_cell_m = std::min(cell_h_m, cell_w_m);

    const int32_t centre_row = grid_.row(lat);
    const int32_t centre_col = grid_.col(lng);
    const int32_t max_ring = static_cast<int32_t>(std::ceil(radius_m / min_cell_m)) + 2;

    scratch_.clear();

    // std::less on distance makes push_heap/pop_heap a MAX-heap, so the worst
    // of the current best-k sits at front() and is the one to evict.
    const auto worse = [](const Match& a, const Match& b) {
        if (a.distance_m != b.distance_m) return a.distance_m < b.distance_m;
        return a.driver_id < b.driver_id;
    };

    const auto consider = [&](const Driver& d, double cutoff) {
        ++last_query_candidates;
        if (!d.available) return;
        if (now_ms - d.last_seen_ms > ttl_ms) return;
        const double dist = haversine_m(lat, lng, d.lat, d.lng);
        if (dist > cutoff) return;
        scratch_.push_back(Match{d.id, dist});
        std::push_heap(scratch_.begin(), scratch_.end(), worse);
        if (scratch_.size() > k) {
            std::pop_heap(scratch_.begin(), scratch_.end(), worse);
            scratch_.pop_back();
        }
    };

    for (int32_t ring = 0; ring <= max_ring; ++ring) {
        // Closest any point in this ring can possibly be. The rider may sit
        // anywhere inside its own cell, so ring r is only guaranteed to be
        // (r-1) cells away, not r. Getting this off by one is how an early
        // exit starts returning wrong answers.
        const double ring_lower_bound = (ring == 0) ? 0.0 : (ring - 1) * min_cell_m;
        if (ring_lower_bound > radius_m) break;
        if (scratch_.size() == k && ring_lower_bound > scratch_.front().distance_m) break;

        const double cutoff =
            (scratch_.size() == k) ? std::min(radius_m, scratch_.front().distance_m) : radius_m;

        const int32_t r_lo = centre_row - ring, r_hi = centre_row + ring;
        const int32_t c_lo = centre_col - ring, c_hi = centre_col + ring;

        for (int32_t r = r_lo; r <= r_hi; ++r) {
            const bool edge_row = (r == r_lo || r == r_hi);
            // Full row on the ring's top and bottom edges; only the two end
            // cells otherwise. This visits each cell of the perimeter exactly
            // once and never revisits an inner ring.
            const int32_t step = edge_row ? 1 : (c_hi - c_lo == 0 ? 1 : c_hi - c_lo);
            for (int32_t c = c_lo; c <= c_hi; c += step) {
                auto it = cells_.find(CellGrid::pack(r, c));
                if (it == cells_.end()) continue;
                for (const Driver& d : it->second) consider(d, cutoff);
            }
        }
    }

    std::sort_heap(scratch_.begin(), scratch_.end(), worse);  // ascending distance
    return std::vector<Match>(scratch_.begin(), scratch_.end());
}

// ---------------------------------------------------------------------------
// LinearScanIndex — the oracle.
// ---------------------------------------------------------------------------

void LinearScanIndex::upsert(uint64_t id, double lat, double lng, bool available,
                             uint64_t now_ms) {
    const Driver record{id, lat, lng, available, now_ms};
    auto it = pos_.find(id);
    if (it == pos_.end()) {
        pos_.emplace(id, drivers_.size());
        drivers_.push_back(record);
    } else {
        drivers_[it->second] = record;
    }
}

bool LinearScanIndex::remove(uint64_t id) {
    auto it = pos_.find(id);
    if (it == pos_.end()) return false;
    const size_t idx = it->second;
    const uint64_t moved_id = drivers_.back().id;
    drivers_[idx] = drivers_.back();
    drivers_.pop_back();
    pos_.erase(it);
    if (moved_id != id) pos_[moved_id] = idx;
    return true;
}

std::vector<Match> LinearScanIndex::nearest(double lat, double lng, double radius_m, size_t k,
                                            uint64_t now_ms, uint64_t ttl_ms) const {
    std::vector<Match> out;
    if (k == 0 || radius_m <= 0.0) return out;
    for (const Driver& d : drivers_) {
        if (!d.available) continue;
        if (now_ms - d.last_seen_ms > ttl_ms) continue;
        const double dist = haversine_m(lat, lng, d.lat, d.lng);
        if (dist <= radius_m) out.push_back(Match{d.id, dist});
    }
    std::sort(out.begin(), out.end(), [](const Match& a, const Match& b) {
        if (a.distance_m != b.distance_m) return a.distance_m < b.distance_m;
        return a.driver_id < b.driver_id;
    });
    if (out.size() > k) out.resize(k);
    return out;
}

}  // namespace geoindex
