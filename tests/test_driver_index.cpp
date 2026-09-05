// test_driver_index.cpp — dependency-free assertion tests.
//
// No gtest on purpose: one binary, no package manager, `make test` works on a
// fresh clone. For a repo this size that is worth more than the framework.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>

#include "geoindex/driver_index.hpp"

using namespace geoindex;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        ++g_checks;                                                              \
        if (!(cond)) {                                                           \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);        \
            ++g_failures;                                                        \
        }                                                                        \
    } while (0)

namespace {

constexpr uint64_t kNow = 1'000'000;
constexpr uint64_t kTtl = 30'000;

void test_insert_and_find() {
    std::printf("insert_and_find\n");
    DriverIndex idx(500.0, 13.0);
    idx.upsert(1, 13.0000, 80.2000, true, kNow);
    idx.upsert(2, 13.0100, 80.2000, true, kNow);  // ~1.1 km north

    auto r = idx.nearest(13.0000, 80.2000, 3000.0, 5, kNow, kTtl);
    CHECK(r.size() == 2);
    CHECK(r[0].driver_id == 1);
    CHECK(r[1].driver_id == 2);
    CHECK(r[0].distance_m < 1.0);
    CHECK(std::fabs(r[1].distance_m - 1113.0) < 50.0);
}

void test_radius_excludes() {
    std::printf("radius_excludes\n");
    DriverIndex idx(500.0, 13.0);
    idx.upsert(1, 13.0000, 80.2000, true, kNow);
    idx.upsert(2, 13.0500, 80.2000, true, kNow);  // ~5.5 km away

    auto r = idx.nearest(13.0000, 80.2000, 3000.0, 5, kNow, kTtl);
    CHECK(r.size() == 1);
    CHECK(r[0].driver_id == 1);
}

// The bug the grid exists to avoid. Two drivers metres apart either side of a
// cell edge must both be found.
void test_cell_boundary() {
    std::printf("cell_boundary\n");
    const double cell_m = 500.0;
    DriverIndex idx(cell_m, 13.0);
    const double lat_step = idx.grid().lat_step_deg();

    // Put the rider just below a cell boundary and a driver just above it.
    const double boundary_lat = std::floor(13.0 / lat_step) * lat_step + lat_step;
    const double rider_lat = boundary_lat - 1e-6;
    const double driver_lat = boundary_lat + 1e-6;

    CHECK(idx.grid().row(rider_lat) != idx.grid().row(driver_lat));  // truly split

    idx.upsert(7, driver_lat, 80.2000, true, kNow);
    auto r = idx.nearest(rider_lat, 80.2000, 500.0, 5, kNow, kTtl);
    CHECK(r.size() == 1);
    CHECK(r[0].driver_id == 7);
}

void test_movement_no_duplicates() {
    std::printf("movement_no_duplicates\n");
    DriverIndex idx(500.0, 13.0);
    idx.upsert(1, 13.0000, 80.2000, true, kNow);
    for (int i = 1; i <= 40; ++i) {
        idx.upsert(1, 13.0000 + i * 0.001, 80.2000, true, kNow);  // walks across cells
    }
    CHECK(idx.size() == 1);

    auto r = idx.nearest(13.0400, 80.2000, 3000.0, 10, kNow, kTtl);
    CHECK(r.size() == 1);  // one record, not 41
    CHECK(r[0].driver_id == 1);
}

void test_availability_and_staleness() {
    std::printf("availability_and_staleness\n");
    DriverIndex idx(500.0, 13.0);
    idx.upsert(1, 13.0, 80.2, false, kNow);           // busy
    idx.upsert(2, 13.0, 80.2, true, kNow - 60'000);   // stale ping
    idx.upsert(3, 13.0, 80.2, true, kNow);            // good

    auto r = idx.nearest(13.0, 80.2, 3000.0, 10, kNow, kTtl);
    CHECK(r.size() == 1);
    CHECK(r[0].driver_id == 3);
}

void test_expire_sweep() {
    std::printf("expire_sweep\n");
    DriverIndex idx(500.0, 13.0);
    for (uint64_t i = 0; i < 100; ++i) {
        idx.upsert(i, 13.0 + i * 0.0005, 80.2, true, (i % 2 == 0) ? kNow : kNow - 60'000);
    }
    CHECK(idx.size() == 100);
    const size_t removed = idx.expire(kNow, kTtl);
    CHECK(removed == 50);
    CHECK(idx.size() == 50);
}

void test_remove() {
    std::printf("remove\n");
    DriverIndex idx(500.0, 13.0);
    idx.upsert(1, 13.0, 80.2, true, kNow);
    CHECK(idx.remove(1) == true);
    CHECK(idx.remove(1) == false);
    CHECK(idx.size() == 0);
    CHECK(idx.nearest(13.0, 80.2, 3000.0, 5, kNow, kTtl).empty());
}

void test_k_limit_and_ordering() {
    std::printf("k_limit_and_ordering\n");
    DriverIndex idx(500.0, 13.0);
    for (uint64_t i = 0; i < 50; ++i) idx.upsert(i, 13.0 + i * 0.0002, 80.2, true, kNow);

    auto r = idx.nearest(13.0, 80.2, 5000.0, 5, kNow, kTtl);
    CHECK(r.size() == 5);
    for (size_t i = 1; i < r.size(); ++i) CHECK(r[i - 1].distance_m <= r[i].distance_m);
}

// The test that matters most: against a brute-force oracle, on random data,
// many times. If the grid ever loses a driver this catches it.
void test_against_oracle() {
    std::printf("against_oracle (randomised)\n");
    std::mt19937_64 rng(20260905);
    std::uniform_real_distribution<double> lat_d(12.90, 13.20);
    std::uniform_real_distribution<double> lng_d(80.10, 80.35);
    std::uniform_int_distribution<int> avail_d(0, 3);
    std::uniform_real_distribution<double> radius_d(200.0, 4000.0);

    for (int trial = 0; trial < 20; ++trial) {
        DriverIndex fast(300.0 + trial * 50.0, 13.0);
        LinearScanIndex oracle;

        for (uint64_t id = 0; id < 2000; ++id) {
            const double lat = lat_d(rng), lng = lng_d(rng);
            const bool avail = avail_d(rng) != 0;
            const uint64_t seen = kNow - (id % 7 == 0 ? 60'000 : 1'000);
            fast.upsert(id, lat, lng, avail, seen);
            oracle.upsert(id, lat, lng, avail, seen);
        }
        // Move a slice of them, to exercise cross-cell updates.
        for (uint64_t id = 0; id < 500; ++id) {
            const double lat = lat_d(rng), lng = lng_d(rng);
            fast.upsert(id, lat, lng, true, kNow);
            oracle.upsert(id, lat, lng, true, kNow);
        }

        for (int q = 0; q < 50; ++q) {
            const double qlat = lat_d(rng), qlng = lng_d(rng), r = radius_d(rng);
            auto a = fast.nearest(qlat, qlng, r, 5, kNow, kTtl);
            auto b = oracle.nearest(qlat, qlng, r, 5, kNow, kTtl);

            CHECK(a.size() == b.size());
            if (a.size() != b.size()) continue;
            for (size_t i = 0; i < a.size(); ++i) {
                CHECK(a[i].driver_id == b[i].driver_id);
                CHECK(std::fabs(a[i].distance_m - b[i].distance_m) < 1e-6);
            }
        }
    }
}

}  // namespace

int main() {
    test_insert_and_find();
    test_radius_excludes();
    test_cell_boundary();
    test_movement_no_duplicates();
    test_availability_and_staleness();
    test_expire_sweep();
    test_remove();
    test_k_limit_and_ordering();
    test_against_oracle();

    std::printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
