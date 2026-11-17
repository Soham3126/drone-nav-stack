"""Close-range LiDAR guard (not in the guidebook; added in I2).

The map that paths are planned and checked on is published once a second, and a wall needs a few scans to
appear in it. Flying at 0.5 m/s, a wall corner can first show up in the map when the drone is already too close
to stop (I2 x10c run 10: shown 0.35 m before contact; ArduPilot needs ~0.2 m to stop from 0.55 m/s). The raw
scan comes 20 times a second, so the bridge checks it directly: if the drone, carrying straight on, would touch
something within its stopping distance, the target is frozen where the drone is until the way is clear.

Only what lies in the drone's swept band counts (the strip its body covers going straight on): walls beside a
corridor, or the end of a wall the drone is turning round, are not in it.
"""
import math


def time_to_contact_m(ranges, angle_min, angle_inc, yaw, direction, radius, range_min=0.12, range_max=12.0):
    """Distance the drone (a disc of `radius`) can travel along unit `direction` (map frame) before its edge
    touches a scan point. `ranges` is a LaserScan in the body frame (LiDAR at the body centre), `yaw` the body
    heading in the map frame. inf if nothing lies in the band."""
    dx, dy = direction
    best = math.inf
    a = angle_min + yaw
    r2 = radius * radius
    for r in ranges:
        if range_min <= r <= range_max:
            px, py = r * math.cos(a), r * math.sin(a)
            along = px * dx + py * dy
            if along > 0.0:
                lat = abs(px * dy - py * dx)
                if lat < radius:
                    hit = along - math.sqrt(r2 - lat * lat)
                    if hit < best:
                        best = hit
        a += angle_inc
    return best


def stop_distance_m(speed, decel, margin):
    """How far the drone travels while braking from `speed` at `decel`, plus `margin`."""
    return speed * speed / (2.0 * decel) + margin
