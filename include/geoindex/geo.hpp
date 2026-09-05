// geo.hpp — coordinate math and the cell grid used to bucket drivers.
//
// Everything here is header-only and free of state, which keeps it easy to
// test in isolation.

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace geoindex {

inline constexpr double kPi = 3.14159265358979323846;

// IUGG mean Earth radius. Any value in the 6371 km range is fine here; the
// haversine model itself is a bigger source of error than the constant.
inline constexpr double kEarthRadiusM = 6371008.8;

// Metres per degree of latitude, DERIVED FROM kEarthRadiusM rather than
// hard-coded.
//
// This matters more than it looks. The obvious constant to write here is
// 111320, the WGS-84 figure quoted everywhere. But haversine_m() below
// measures on a sphere of radius kEarthRadiusM, where a degree of latitude is
// 111194.9 m — a 0.11% disagreement. The query path converts a radius in
// metres into a bounding box in degrees, and that box must be a superset of
// the circle haversine will test against. With 111320 the box comes out
// fractionally SMALLER than the circle, so drivers in the outermost ~1 m of
// the search radius are discarded before the distance check ever runs.
//
// It is a sub-metre error, invisible in hand-written tests, and it showed up
// here roughly once every few thousand randomised queries against the
// brute-force oracle. Deriving both from one constant makes the two agree by
// construction.
inline constexpr double kMetersPerDegLat = kPi * kEarthRadiusM / 180.0;

inline constexpr double deg2rad(double d) noexcept { return d * kPi / 180.0; }

// Great-circle distance in metres.
//
// Haversine assumes a sphere, so it carries up to ~0.5% error against the
// WGS-84 ellipsoid. Over a 3 km dispatch radius that is ~15 m, which is well
// inside GPS noise, so a more expensive geodesic (Vincenty, Karney) would buy
// nothing here. Say this out loud if anyone asks why not Vincenty.
inline double haversine_m(double lat1, double lng1, double lat2, double lng2) noexcept {
    const double dlat = deg2rad(lat2 - lat1);
    const double dlng = deg2rad(lng2 - lng1);
    const double s_lat = std::sin(dlat * 0.5);
    const double s_lng = std::sin(dlng * 0.5);
    const double a = s_lat * s_lat +
                     std::cos(deg2rad(lat1)) * std::cos(deg2rad(lat2)) * s_lng * s_lng;
    // clamp guards against a > 1 from floating-point rounding at tiny distances
    return 2.0 * kEarthRadiusM * std::asin(std::min(1.0, std::sqrt(a)));
}

using CellKey = uint64_t;

// A flat equirectangular grid over the service area.
//
// WHY A GRID AND NOT A TREE:
// In a dispatch system every driver moves every second, so writes vastly
// outnumber reads. A grid update is a hash lookup plus a vector push — O(1),
// no rebalancing. A k-d tree or R-tree gives better query locality but has to
// be rebuilt or rebalanced under this write rate, which dominates. The access
// pattern picks the structure, not the other way round.
//
// LIMITATION, STATED DELIBERATELY:
// Longitude degrees shrink as you move away from the equator, so the grid is
// built around a single reference latitude. That is exact enough for a
// city-sized service area (a few hundred km) and wrong for a global index.
// Zipo operates city by city, so this is the right trade. If it ever needed to
// be global, the fix is a proper space-filling curve (S2, H3) rather than a
// lat/lng grid.
class CellGrid {
  public:
    CellGrid(double cell_size_m, double reference_lat)
        : cell_size_m_(cell_size_m),
          lat_step_deg_(cell_size_m / kMetersPerDegLat),
          lng_step_deg_(cell_size_m / (kMetersPerDegLat * std::cos(deg2rad(reference_lat)))) {}

    int32_t row(double lat) const noexcept {
        return static_cast<int32_t>(std::floor(lat / lat_step_deg_));
    }
    int32_t col(double lng) const noexcept {
        return static_cast<int32_t>(std::floor(lng / lng_step_deg_));
    }

    // Pack two signed 32-bit cell coordinates into one 64-bit key.
    // The casts through uint32_t are deliberate: they make the conversion of
    // negative rows/cols well defined instead of implementation-dependent.
    static CellKey pack(int32_t r, int32_t c) noexcept {
        return (static_cast<uint64_t>(static_cast<uint32_t>(r)) << 32) |
               static_cast<uint64_t>(static_cast<uint32_t>(c));
    }

    CellKey key(double lat, double lng) const noexcept { return pack(row(lat), col(lng)); }

    double cell_size_m() const noexcept { return cell_size_m_; }
    double lat_step_deg() const noexcept { return lat_step_deg_; }
    double lng_step_deg() const noexcept { return lng_step_deg_; }

  private:
    double cell_size_m_;
    double lat_step_deg_;
    double lng_step_deg_;
};

}  // namespace geoindex
