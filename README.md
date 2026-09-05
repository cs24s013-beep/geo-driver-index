# geo-driver-index

An in-memory geospatial index for ride-hailing dispatch. Given a rider's
position, it returns the *k* nearest available drivers within a radius, over a
driver set where every driver reports a new position roughly once a second.

Single-file-per-concern C++17, no dependencies, builds with `make`.

```
make run      # build, test, run the dispatch simulation, then benchmark
```

Or individually:

```
make          # build all three binaries into build/
make test     # correctness, including a randomised brute-force comparison
make demo     # end-to-end dispatch simulation
make bench    # latency measurements
make sweep    # cell-size tuning curve
make asan     # tests under AddressSanitizer + UBSan
```

No dependencies beyond a C++17 compiler and make. A CMake build is also
provided (`cmake -B build && cmake --build build`).

## The demo

`make demo` runs a simulated fleet forward one second at a time: every driver
moves and pings, riders request trips, the index returns the nearest available
candidates, dispatch assigns one and marks the driver busy, and some drivers'
phones go silent so the TTL sweep has something to reclaim.

```
tick 40  t=41s  live=4697  pings=3875  matched=4/4  phones_lost=0   expired=102
tick 43  t=44s  live=4604  pings=3804  matched=4/4  phones_lost=71  expired=93

=== summary ===
  requests            180
  matched             180 (100.0%)
  pickup distance     mean 261 m   p50 242 m   p99 593 m   max 730 m
  drivers expired     397 (stopped pinging, swept by TTL)
```

One thing the demo makes visible that the index alone does not handle: two
riders in the same tick can be returned the same nearest driver. The index
answers a spatial question; deciding who gets the driver is dispatch's job, and
the demo arbitrates it explicitly.

## The problem

A city-scale fleet is ~100,000 drivers, each pinging once per second. That is
100k writes/second against a structure that also has to answer "who is near
this rider?" in single-digit milliseconds, thousands of times a second.

Two properties drive every decision below:

1. **Writes dominate reads.** Every driver moves constantly; only some riders
   are searching at any moment.
2. **k is tiny.** Dispatch wants the 5 nearest drivers, not the 3,000 drivers
   within 3 km.

## Design

### Why a uniform grid, not a tree

Drivers are bucketed into fixed-size cells of a flat lat/lng grid, held in an
`unordered_map<CellKey, vector<Driver>>`.

A k-d tree or R-tree gives tighter query locality, but has to be rebalanced or
rebuilt under this write rate, and that cost dominates. A grid update is a hash
lookup and a vector push — O(1), no restructuring, and a driver that moves a
few metres usually stays in the same cell, making the common update an in-place
field write with no bucket churn at all.

The access pattern picks the structure. That is the whole argument.

### Driver records live inside the buckets

Buckets hold `Driver` values, not ids or pointers. A query walks contiguous
memory instead of chasing a second hash lookup per candidate. A parallel
`unordered_map<driver_id, CellKey>` exists only so updates and deletes can find
a driver without scanning every bucket.

### Expanding ring search

The first working version scanned every cell the search radius touched, then
sorted. It was correct, and it was the wrong algorithm: with 100k drivers, a
3 km radius genuinely contains a few thousand of them, so returning 5 meant
computing ~3,900 distances and discarding almost all of them. Work scaled with
search *area*, not with k.

The shipped version walks outward from the rider's own cell one ring at a time,
keeping the best k in a max-heap. Every cell in ring *r+1* is at least
*(r−1) × cell_size* from the rider, so once the heap holds k drivers and its
worst entry is closer than that bound, no unexplored cell can improve the
answer and the search stops. For k=5 that is usually one or two rings.

Both implementations are kept — `nearest()` and `nearest_radius_scan()` — and
the benchmark reports both, because the gap between them is the most
interesting thing here.

### Cell size is measured, not chosen

The default of 250 m is the minimum of the sweep below. The curve is not
monotonic: smaller cells cut the candidate count but add hash lookups, and
below ~250 m the lookups win. Re-run `make sweep` for a different driver
density rather than inheriting this number.

## Results

Intel Xeon @ 2.10 GHz, single core, GCC 13.3, `-O2`. 100,000 drivers uniformly
distributed over a 33 × 27 km bounding box, 3 km radius, k=5, 100,000 queries.
Median of three runs; JVM-style warm-up discarded before timing.

**Regenerate these on your own machine before quoting them anywhere.**

| Query strategy | candidates/query | p50 | p99 | p99.9 | throughput |
|---|---|---|---|---|---|
| Radius scan | 3,878 | 162 µs | 244 µs | 312 µs | 6,290 q/s |
| Expanding ring | 61 | **3.4 µs** | **6.1 µs** | 33 µs | 275,000 q/s |

Updates (`upsert`, driver moving ~22 m between pings): p50 0.42 µs, p99
1.16 µs, ~1.9M ops/s. A 100k-driver fleet pinging at 1 Hz uses roughly 5% of
one core.

### Cell-size sweep

| Cell size | occupied cells | candidates/query | p50 | p99 |
|---|---|---|---|---|
| 100 m | 60,615 | 26 | 4.52 µs | 10.99 µs |
| **250 m** | **14,684** | **61** | **3.52 µs** | **7.29 µs** |
| 500 m | 3,790 | 242 | 6.39 µs | 16.96 µs |
| 1000 m | 952 | 941 | 18.16 µs | 49.35 µs |
| 2000 m | 238 | 3,578 | 54.88 µs | 93.83 µs |

### Against Redis

`bench/redis_baseline.sh` runs `GEOSEARCH ... COUNT k` over the same data
shape. Redis figures include a TCP round trip and RESP parsing that the
in-process C++ index does not pay, so the defensible claim is *comparable
per-query cost with the network removed* — not "faster than Redis."

## Correctness

`LinearScanIndex` is a brute-force implementation used as an oracle. The test
suite generates random driver populations and random queries and asserts the
grid returns *exactly* the same drivers in the same order — across both
hemispheres, latitudes from −55° to +55°, cell sizes from 120 m to 2 km, radii
from 30 m to 8 km, and k from 1 to 9.

That harness earned its place immediately by finding two real bugs, both
sub-metre, both invisible to hand-written tests:

1. **Mismatched Earth constants.** The bounding box converted metres to degrees
   using the WGS-84 figure of 111,320 m/degree, while `haversine_m` measured on
   a sphere of radius 6,371,008.8 m, where a degree is 111,194.9 m. The 0.11%
   disagreement made the box marginally *smaller* than the circle it was meant
   to enclose, so drivers in the outermost ~1 m of the radius were rejected
   before the distance check ran. Fixed by deriving both from one constant, and
   widening the box by 1 ppm so floating-point rounding cannot reintroduce it.

2. **Longitude box sized at the wrong latitude.** Degrees of longitude shrink
   away from the equator, so a box sized at the query's latitude is too narrow
   at its poleward corners. Fixed by sizing at whichever latitude in the search
   band is furthest from the equator.

Both produced plausible, almost-correct answers — the failure mode where an
index quietly returns the second-nearest driver and nobody notices.

The suite also covers the case the grid exists to get wrong: two points metres
apart on opposite sides of a cell boundary.

## Deliberate scope cuts

- **Single-threaded.** Adding a lock before measuring the single-threaded cost
  would be premature. The plan is to shard `cells_` by cell key so writers in
  different parts of the city never contend; a single global mutex would
  serialise the entire fleet's pings, which is the wrong first move.
  `nearest()` is `const` but *not* thread-safe today — it reuses a scratch
  buffer to keep allocation off the hot path. That buffer is the first thing to
  remove when concurrency arrives.
- **No networking.** This is a library plus a benchmark, not a server.
- **Uniform driver density in the benchmark.** Real fleets cluster heavily
  around transit hubs and nightlife. Clustering makes the worst-case cell much
  denser, so the tail numbers here are optimistic. Adding a clustered generator
  is the most useful next experiment.
- **Flat lat/lng grid, city scale only.** The grid is built around a single
  reference latitude, which is accurate over a few hundred kilometres and wrong
  globally. A global index would want S2 or H3 instead.

## Layout

```
include/geoindex/geo.hpp            haversine, cell grid, Earth constants
include/geoindex/driver_index.hpp   public interface
src/driver_index.cpp                index + brute-force oracle
tests/test_driver_index.cpp         assertions + randomised oracle comparison
bench/benchmark.cpp                 latency harness, percentiles, sweep
bench/redis_baseline.sh             GEOSEARCH comparison
examples/demo.cpp                   end-to-end dispatch simulation
```
