#!/usr/bin/env python3
"""Lab kitchen -> MoveIt collision boxes, published in base_link.

Frame model (agreed with the measurements):
  world := base_link at HOME (carriage=0.85, lift=0.35), axes PARALLEL to base_link.
    +X = arm-left, along the rail   (carriage UP moves the arm -X, i.e. to its right)
    +Y = toward the back wall
    +Z = DOWN toward the floor       (ceiling-mounted arm; "up" = -z)
  base_link never rotates on the rail (pure prismatic X + Z), so a static object at
  world point P maps into base_link by SUBTRACTION only:
      P_base_link = P - origin(carriage, lift)
  MoveIt caches the scene in the planning frame and does NOT re-transform it as
  base_link rides the rail, so re-apply this after EVERY Elmo move (and at startup).
  See the rail-carriage-workflow / elmo-axis-mapping notes.

Usage:
  python3 scripts/kitchen_scene.py [carriage] [lift]   # apply scene (default = home)
  python3 scripts/kitchen_scene.py check               # math self-check, no ROS needed
"""

import sys

# ----- calibration knobs: measure-once. Tweak here; everything downstream follows. -----
HOME_CARRIAGE, HOME_LIFT = 0.85, 0.35
WALL_Y  = 0.49                # base centre -> back wall
FLOOR_Z = 2.006               # base_link -> floor at home (interp of min/max heights)

CEIL_Z    = FLOOR_Z - 3.0              # 3 m ceiling assumed (296 was room WIDTH, not height)
COUNTER_H = 0.91
TOP_Z     = FLOOR_Z - COUNTER_H        # z of the worktop surface

# ----- layout anchors in world (metres). "Nearest counter edge = 0.50 to arm-left." -----
X_NEAR   = 0.50                        # cooktop peninsula right (nearest) edge, +X side
COOK_W, COOK_D = 0.62, 1.70            # cooktop: along-wall X, out-from-wall Y
SINK_W, SINK_D = 1.24, 0.635          # sink counter: along-wall X, out-from-wall Y

X_COOK  = (X_NEAR, X_NEAR + COOK_W)                 # cooktop spans X
X_SINK  = (X_COOK[1], X_COOK[1] + SINK_W)          # sink counter, no gap, further +X
Y_COOK  = (WALL_Y - COOK_D, WALL_Y)
Y_SINK  = (WALL_Y - SINK_D, WALL_Y)
Z_CAB   = (TOP_Z, FLOOR_Z)                          # cabinet: worktop down to floor

# sink bowl void: 50x40x18. Gaps along the counter: 36 corner side / 50 bowl / 38 far end = 124.
BOWL_W, BOWL_D, BOWL_H = 0.50, 0.40, 0.18
BOWL_GAP_CORNER = 0.36                              # corner (cooktop) edge -> bowl near edge
bx0 = X_SINK[0] + BOWL_GAP_CORNER
X_BOWL = (bx0, bx0 + BOWL_W)
_sy = (Y_SINK[0] + Y_SINK[1]) / 2                   # bowl centred in counter depth
Y_BOWL = (_sy - BOWL_D / 2, _sy + BOWL_D / 2)
Z_RIM  = (TOP_Z, TOP_Z + BOWL_H)                    # top 18cm slab holding the bowl hole
Z_BASE = (Z_RIM[1], FLOOR_Z)                        # solid cabinet below the bowl

# faucet: 29 tall post at the bowl's back rim; 26-long spout, 5 thick, 24 above worktop.
FX = ((X_BOWL[0] + X_BOWL[1]) / 2 - 0.025, (X_BOWL[0] + X_BOWL[1]) / 2 + 0.025)
Z_POST  = (TOP_Z - 0.29, TOP_Z)
Y_POST  = (Y_BOWL[1] - 0.05, Y_BOWL[1])
Z_SPOUT = (TOP_Z - 0.29, TOP_Z - 0.24)              # 5cm band, 24..29 above worktop
Y_SPOUT = (Y_BOWL[1] - 0.26, Y_BOWL[1])

# coffee machine (Siemens EQ700) on the sink_ext nook: three boxes leaving the cup bay OPEN --
# bulk behind it, drip tray below, spout/display head above.
# Bay only. Angled flanks would need a rotated primitive_pose quaternion
COFFEE_W, COFFEE_D, COFFEE_H = 0.27, 0.44, 0.365  # KNOBS: front face, depth, height
COFFEE_BAY_D  = 0.12       # front face -> concave back wall apex
COFFEE_TRAY_H = 0.055      # drip tray top, above the worktop
COFFEE_BAY_H  = 0.14       # clear height above the tray with the spout PARKED HIGH.
COFFEE_TRAY_W = 0.21

COFFEE_ROT90  = True       # True: the 44 depth runs along X, machine faces -X (as it sits today)
COFFEE_X0, COFFEE_Y0 = X_SINK[1] - 0.27, Y_SINK[0]   # KNOBS: -X/-Y corner of the footprint


def _coffee(rot):
    """(bulk, tray, head, bay) as world (xrange, yrange, zrange) at this orientation.

    Defined once in machine-local coords -- lx along the 27 face, ly from the FRONT face,
    lz up from the worktop -- then mapped out. `rot` is a true 90 deg turn, not an x/y
    dimension swap: a swap mirrors the machine and puts the cup bay on the wrong face.
    """
    def box(lx, ly, lz):
        pts = []
        for i in (0, 1):                                   # rotation swaps which corner is min
            px, py = (ly[i], COFFEE_W - lx[i]) if rot else (lx[i], ly[i])
            pts.append((COFFEE_X0 + px, COFFEE_Y0 + py, TOP_Z - lz[i]))
        return tuple((min(p, q), max(p, q)) for p, q in zip(*pts))

    full, front = (0.0, COFFEE_W), (0.0, COFFEE_BAY_D)
    bay_top = COFFEE_TRAY_H + COFFEE_BAY_H
    return (box(full, (COFFEE_BAY_D, COFFEE_D), (0.0, COFFEE_H)),   # bulk, behind the bay
            box(full, front, (0.0, COFFEE_TRAY_H)),                 # drip tray, below
            box(full, front, (bay_top, COFFEE_H)),                  # spout + display, above
            box(full, front, (COFFEE_TRAY_H, bay_top)))             # the bay itself: OPEN

# desk: nearest (left) edge 98 to arm-right (-X); 120 wide, 70 deep, top at 105.
DESK_TOP_H = 1.05
X_DESK = (-0.98 - 1.20, -0.98)
Y_DESK = (WALL_Y - 0.70, WALL_Y)
Z_DESK = (FLOOR_Z - DESK_TOP_H, FLOOR_Z)
Y_MON  = (WALL_Y - 0.35, WALL_Y)                    # monitors on the back half
Z_MON  = (FLOOR_Z - DESK_TOP_H - 0.45, FLOOR_Z - DESK_TOP_H)   # ASSUMED 45 tall -- knob

# wall lamp: 15^3 cube, on the wall, 53 above the worktop, edge at the counter junction.
LAMP = 0.15
Z_LAMP = (TOP_Z - 0.53 - LAMP, TOP_Z - 0.53)
X_LAMP = (X_COOK[1] - LAMP, X_COOK[1])             # cooktop side; lamp's far edge at the junction
Y_LAMP = (WALL_Y - LAMP, WALL_Y)


def scene():
    """(name, xrange, yrange, zrange) in world metres. Ranges are (min, max)."""
    b = []
    # cooktop peninsula (solid)
    b.append(("cooktop", X_COOK, Y_COOK, Z_CAB))
    # sink counter = solid base + a rimmed top slab that leaves the 50x40x18 bowl open
    b.append(("sink_base",  X_SINK,               Y_SINK,                 Z_BASE))
    b.append(("sink_rimR",  (X_SINK[0], X_BOWL[0]), Y_SINK,               Z_RIM))  # -X of bowl
    b.append(("sink_rimL",  (X_BOWL[1], X_SINK[1]), Y_SINK,               Z_RIM))  # +X of bowl
    b.append(("sink_rimBk", X_BOWL,               (Y_BOWL[1], Y_SINK[1]), Z_RIM))  # wall side
    b.append(("sink_rimFr", X_BOWL,               (Y_SINK[0], Y_BOWL[0]), Z_RIM))  # room side
    b.append(("sink_ext",   (X_SINK[1], X_SINK[1] + 0.5), Y_SINK,         Z_RIM)) # cavity at the end
    # coffee machine: three primitives sharing ONE id, so the gap between them stays open
    for part in _coffee(COFFEE_ROT90)[:3]:
        b.append(("coffee_machine",) + part)
    # faucet
    b.append(("faucet_post",  FX, Y_POST,  Z_POST))
    b.append(("faucet_spout", FX, Y_SPOUT, Z_SPOUT))
    # desk + monitors (monitor height assumed)
    b.append(("desk",     X_DESK, Y_DESK, Z_DESK))
    b.append(("monitors", X_DESK, Y_MON,  Z_MON))
    # wall-mounted lamp (position a bit uncertain -- easy to nudge or drop)
    b.append(("wall_lamp", X_LAMP, Y_LAMP, Z_LAMP))
    # room shell
    b.append(("floor", (-3.0, 3.0), (-2.0, 1.0), (FLOOR_Z, FLOOR_Z + 0.10)))
    b.append(("wall",  (-3.0, 3.0), (WALL_Y, WALL_Y + 0.10), (CEIL_Z, FLOOR_Z + 0.10)))
    b.append(("left_cupboard", (X_SINK[1], X_SINK[1] + 0.10), (WALL_Y - SINK_D - 1.10, WALL_Y - SINK_D), (CEIL_Z, FLOOR_Z)))
    return b


def origin(carriage, lift):
    """base_link origin expressed in world = how far base_link has moved off home."""
    dx = -(carriage - HOME_CARRIAGE)   # carriage UP -> arm -X
    dz = (lift - HOME_LIFT)            # lift UP(value) -> arm down -> +Z
    return (dx, 0.0, dz)


def _center_size(xr, yr, zr, o):
    """World box bounds -> (centre in base_link, full size). base_link = world - origin."""
    assert xr[1] > xr[0] and yr[1] > yr[0] and zr[1] > zr[0], "box has non-positive size"
    c = ((xr[0] + xr[1]) / 2 - o[0], (yr[0] + yr[1]) / 2 - o[1], (zr[0] + zr[1]) / 2 - o[2])
    s = (xr[1] - xr[0], yr[1] - yr[0], zr[1] - zr[0])
    return c, s


# ---------------------------------------------------------------------------- ROS ----

def build_scene(carriage, lift):
    """CollisionObjects for the kitchen at this carriage/lift, in base_link frame.

    Pure message-building (no rclpy) so a running node — e.g. arm_controller — can
    reuse it and publish on its own /apply_planning_scene client after each rail move.
    """
    from moveit_msgs.msg import CollisionObject
    from shape_msgs.msg import SolidPrimitive
    from geometry_msgs.msg import Pose

    o = origin(carriage, lift)
    objs = {}
    for name, xr, yr, zr in scene():                # boxes sharing a name -> one multi-part object
        c, s = _center_size(xr, yr, zr, o)
        co = objs.get(name)
        if co is None:
            co = objs[name] = CollisionObject()
            co.header.frame_id = "base_link"
            co.id = name
            co.pose.orientation.w = 1.0
            co.operation = CollisionObject.ADD
        prim_pose = Pose()
        prim_pose.position.x, prim_pose.position.y, prim_pose.position.z = c
        prim_pose.orientation.w = 1.0
        co.primitives.append(SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(s)))
        co.primitive_poses.append(prim_pose)
    return list(objs.values())


def apply_scene(carriage, lift):
    import rclpy
    from moveit_msgs.srv import ApplyPlanningScene
    from moveit_msgs.msg import PlanningScene

    rclpy.init()
    node = rclpy.create_node("kitchen_scene")
    client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    node.get_logger().info("waiting for /apply_planning_scene ...")
    client.wait_for_service()

    ps = PlanningScene(is_diff=True)
    ps.world.collision_objects = build_scene(carriage, lift)

    fut = client.call_async(ApplyPlanningScene.Request(scene=ps))
    rclpy.spin_until_future_complete(node, fut)
    ok = fut.result() is not None and fut.result().success
    node.get_logger().info(f"applied {len(ps.world.collision_objects)} boxes "
                           f"@ carriage={carriage} lift={lift} -> success={ok}")
    rclpy.shutdown()


# -------------------------------------------------------------------------- check ----

def demo():
    # shift math -- the only non-trivial logic; a sign flip here wrecks the whole scene.
    assert origin(0.85, 0.35) == (0.0, 0.0, 0.0)
    assert abs(origin(0.95, 0.35)[0] - (-0.10)) < 1e-9        # carriage +0.1 -> arm -X
    assert abs(origin(0.75, 0.35)[0] - (+0.10)) < 1e-9        # carriage -0.1 -> arm +X
    assert abs(origin(0.85, 0.45)[2] - 0.10) < 1e-9           # lift +0.1 -> arm down +Z (1:1)

    # at home the scene sits in world coords unchanged (origin is zero)
    c, s = _center_size(X_COOK, Y_COOK, Z_CAB, origin(HOME_CARRIAGE, HOME_LIFT))
    assert abs(c[0] - (X_NEAR + COOK_W / 2)) < 1e-9
    assert abs(s[2] - COUNTER_H) < 1e-9

    # bowl sits 36 corner / 50 wide / 38 far end within the 124 counter (trips on an INSET slip)
    assert abs((X_BOWL[0] - X_SINK[0]) - 0.36) < 1e-9
    assert abs((X_SINK[1] - X_BOWL[1]) - 0.38) < 1e-9

    # the bowl void really is clear: no rim/base box covers the opening at rim height
    bcx, bcy, bcz = (sum(X_BOWL) / 2, sum(Y_BOWL) / 2, sum(Z_RIM) / 2)
    for name, xr, yr, zr in scene():
        inside = (xr[0] < bcx < xr[1] and yr[0] < bcy < yr[1] and zr[0] < bcz < zr[1])
        assert not inside, f"{name} intrudes into the sink bowl void"

    # the cup bay stays open in BOTH orientations -- the only reason to trust _coffee()'s rotation
    for rot in (False, True):
        *parts, bay = _coffee(rot)
        cup = [sum(r) / 2 for r in bay]
        for i, p in enumerate(parts):
            assert not all(p[k][0] < cup[k] < p[k][1] for k in range(3)), \
                f"coffee part {i} fills the cup bay (rot={rot})"

    # machine stands ON the nook slab and its footprint stays on it
    bulk, tray, head, _ = _coffee(COFFEE_ROT90)
    assert abs(tray[2][1] - TOP_Z) < 1e-9, "machine floats above / sinks into the worktop"
    for p in (bulk, tray, head):
        assert X_SINK[1] <= p[0][0] and p[0][1] <= X_SINK[1] + 0.5, "machine hangs off the nook slab"
        assert Y_SINK[0] <= p[1][0] and p[1][1] <= Y_SINK[1], "machine overhangs the counter depth"

    # every box has positive size (also asserted per-box in _center_size)
    for name, xr, yr, zr in scene():
        _center_size(xr, yr, zr, (0, 0, 0))

    print(f"check OK -- {len(scene())} boxes, shift math, sink void and cup bay verified")


def main():
    if len(sys.argv) > 1 and sys.argv[1] == "check":
        demo()
        return
    carriage = float(sys.argv[1]) if len(sys.argv) > 1 else HOME_CARRIAGE
    lift = float(sys.argv[2]) if len(sys.argv) > 2 else HOME_LIFT
    apply_scene(carriage, lift)


if __name__ == "__main__":
    main()
