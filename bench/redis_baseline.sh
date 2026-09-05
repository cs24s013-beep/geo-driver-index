#!/usr/bin/env bash
#
# redis_baseline.sh — time Redis GEOSEARCH on the same data shape.
#
# This is the comparison that belongs in the README. Beating your own naive
# implementation proves only that the naive one was naive; Redis is what a
# real system would use for this, so it is the number worth reporting.
#
# Requires a local redis-server and redis-cli. Nothing else in this repo
# depends on Redis — the C++ build has no dependencies at all.
#
# Usage:  ./bench/redis_baseline.sh [num_drivers] [radius_m] [k] [num_queries]

set -euo pipefail

DRIVERS=${1:-100000}
RADIUS=${2:-3000}
K=${3:-5}
QUERIES=${4:-20000}
KEY="drivers:bench"

# Same bounding box as bench/benchmark.cpp. Keep these in sync or the
# comparison is meaningless.
LAT_MIN=12.90; LAT_MAX=13.20
LNG_MIN=80.10; LNG_MAX=80.35

command -v redis-cli >/dev/null || { echo "redis-cli not found"; exit 1; }
redis-cli PING >/dev/null || { echo "redis-server not running"; exit 1; }

echo "Loading $DRIVERS drivers into Redis..."
redis-cli DEL "$KEY" >/dev/null

# Pipe mode: one GEOADD per line, bulk loaded. Building this with a loop of
# redis-cli calls instead would measure process startup, not Redis.
awk -v n="$DRIVERS" -v key="$KEY" \
    -v latmin="$LAT_MIN" -v latmax="$LAT_MAX" \
    -v lngmin="$LNG_MIN" -v lngmax="$LNG_MAX" '
BEGIN {
  srand(20260905);
  for (i = 0; i < n; i++) {
    lat = latmin + rand() * (latmax - latmin);
    lng = lngmin + rand() * (lngmax - lngmin);
    printf "GEOADD %s %.6f %.6f d%d\r\n", key, lng, lat, i;
  }
}' | redis-cli --pipe

echo "Loaded: $(redis-cli ZCARD "$KEY") members"
echo

# GEOSEARCH with COUNT is the closest equivalent to DriverIndex::nearest:
# k nearest within a radius. Redis also applies the COUNT-based early exit,
# which is exactly the optimisation the expanding ring search implements.
CENTRE_LAT=13.05
CENTRE_LNG=80.22

echo "Timing GEOSEARCH (radius=${RADIUS}m, COUNT=${K}, $QUERIES queries)..."
redis-cli --latency-history -i 5 >/dev/null 2>&1 || true

redis-benchmark -n "$QUERIES" -c 1 -q \
  "GEOSEARCH" "$KEY" "FROMLONLAT" "$CENTRE_LNG" "$CENTRE_LAT" \
  "BYRADIUS" "$RADIUS" "m" "ASC" "COUNT" "$K"

echo
echo "NOTE ON FAIRNESS — put this in the README, do not let a reader assume otherwise:"
echo "  * Redis figures include TCP round-trip and RESP parsing; the C++ index is an"
echo "    in-process library call. Redis is doing strictly more work per operation."
echo "  * redis-benchmark reuses one query point; the C++ harness randomises them."
echo "  * The honest claim is 'comparable per-query cost with the network removed',"
echo "    NOT 'faster than Redis'. Say the weaker thing; it survives questioning."
