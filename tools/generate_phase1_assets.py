"""Write the neutral PROC BODY and the synthetic Phase 1 motions.

This is an authoring tool, not a runtime dependency. Re-run it from the
repository root to regenerate assets/bodies and assets/motions.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SKELETON = "proc_humanoid_v0"

# Joint centers in meters. +X right, +Y forward, +Z up.
# Left limbs are authored here and mirrored. Neck and head are not on the
# root vertical line. Toes sit on z = 0. These are joint centers, not the
# external anthropometry block.
JOINTS = {
    "legrootj": (0.0, 0.0, 0.992),
    # Mild sagittal curve. Keep the neutral torso centered on X and avoid
    # alternating large spine angles that make a three-bone spine look kinked.
    "spine1j": (0.0, -0.010, 1.116),
    "spine2j": (0.0, -0.004, 1.260),
    "armrootj": (0.0, 0.006, 1.421),
    "neckj": (0.0, 0.018, 1.535),
    "headj": (0.0, 0.036, 1.754),
    "lleguppj": (-0.096, -0.018, 0.928),
    "lleglowj": (-0.072, 0.014, 0.500),
    "lfootj": (-0.088, -0.006, 0.082),
    "ltoesj": (-0.086, 0.196, 0.000),
    # Translate the arm chain with armrootj so the existing shoulder/arm
    # rest vectors and lengths remain unchanged.
    "larmuppj": (-0.188, 0.026, 1.399),
    "larmlowj": (-0.496, 0.042, 1.371),
    "lhandj": (-0.748, 0.056, 1.361),
    "lfingj": (-0.852, 0.076, 1.355),
}

MIRROR = {
    "lleguppj": "rleguppj",
    "lleglowj": "rleglowj",
    "lfootj": "rfootj",
    "ltoesj": "rtoesj",
    "larmuppj": "rarmuppj",
    "larmlowj": "rarmlowj",
    "lhandj": "rhandj",
    "lfingj": "rfingj",
}

BONES = [
    ("spine1b", "legrootj", "spine1j"),
    ("spine2b", "spine1j", "spine2j"),
    ("spine3b", "spine2j", "armrootj"),
    ("neckb", "armrootj", "neckj"),
    ("headb", "neckj", "headj"),
    ("llegrootb", "legrootj", "lleguppj"),
    ("lleguppb", "lleguppj", "lleglowj"),
    ("lleglowb", "lleglowj", "lfootj"),
    ("lfootb", "lfootj", "ltoesj"),
    ("rlegrootb", "legrootj", "rleguppj"),
    ("rleguppb", "rleguppj", "rleglowj"),
    ("rleglowb", "rleglowj", "rfootj"),
    ("rfootb", "rfootj", "rtoesj"),
    ("larmrootb", "armrootj", "larmuppj"),
    ("larmuppb", "larmuppj", "larmlowj"),
    ("larmlowb", "larmlowj", "lhandj"),
    ("lhandb", "lhandj", "lfingj"),
    ("rarmrootb", "armrootj", "rarmuppj"),
    ("rarmuppb", "rarmuppj", "rarmlowj"),
    ("rarmlowb", "rarmlowj", "rhandj"),
    ("rhandb", "rhandj", "rfingj"),
]

NOTE = "Preliminary placeholder. Not a clinical or measured joint limit."


def vsub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def vadd(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def vscale(a, s):
    return (a[0] * s, a[1] * s, a[2] * s)


def vdot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def vlen(a):
    return math.sqrt(vdot(a, a))


def vnorm(a):
    length = vlen(a)
    return vscale(a, 1.0 / length)


def num(value):
    value = float(value)
    if abs(value) < 1e-12:
        return 0.0
    return round(value, 10)


def vec(value):
    return [num(value[0]), num(value[1]), num(value[2])]


def complete_joints():
    joints = dict(JOINTS)
    for left, right in MIRROR.items():
        x, y, z = joints[left]
        joints[right] = (-x, y, z)
    return joints


def build_body():
    joints = complete_joints()
    bone_docs = []
    bone_by_child = {}
    for name, parent, child in BONES:
        raw = vsub(joints[child], joints[parent])
        stored = vec(raw)
        length = math.sqrt(stored[0] ** 2 + stored[1] ** 2 + stored[2] ** 2)
        direction = [stored[0] / length, stored[1] / length, stored[2] / length]
        doc = {
            "name": name,
            "parent_joint": parent,
            "child_joint": child,
            "length_m": length,
            "rest_direction": [num(c) for c in direction],
            "rest_vector_m": stored,
            "locked": True,
        }
        # Recompute length from the rounded vector so the file is consistent.
        vector = doc["rest_vector_m"]
        doc["length_m"] = math.sqrt(vector[0] ** 2 + vector[1] ** 2 + vector[2] ** 2)
        doc["rest_direction"] = [c / doc["length_m"] for c in vector]
        bone_docs.append(doc)
        bone_by_child[child] = doc

    positions = {"legrootj": vec(joints["legrootj"])}
    for bone in bone_docs:
        parent_pos = positions[bone["parent_joint"]]
        vector = bone["rest_vector_m"]
        positions[bone["child_joint"]] = [
            parent_pos[0] + vector[0],
            parent_pos[1] + vector[1],
            parent_pos[2] + vector[2],
        ]

    def outgoing_direction(joint_name):
        for bone in bone_docs:
            if bone["parent_joint"] == joint_name:
                return bone["rest_direction"]
        return [0.0, 0.0, 1.0]

    constraints = []

    def add(joint, kind, **fields):
        entry = {
            "joint": joint,
            "type": kind,
            "authority": "placeholder",
            "note": NOTE,
        }
        entry.update(fields)
        constraints.append(entry)

    add("legrootj", "root")
    for name in ("spine1j", "spine2j", "armrootj"):
        add(
            name,
            "ball",
            twist_axis=outgoing_direction(name),
            max_swing_deg=40.0,
            max_twist_deg=25.0,
        )
    add(
        "neckj",
        "ball",
        twist_axis=outgoing_direction("neckj"),
        max_swing_deg=50.0,
        max_twist_deg=40.0,
    )
    add("headj", "endpoint")
    for name in ("lleguppj", "rleguppj"):
        add(
            name,
            "ball",
            twist_axis=outgoing_direction(name),
            max_swing_deg=90.0,
            max_twist_deg=40.0,
        )
    for name in ("lleglowj", "rleglowj"):
        add(name, "hinge", axis=[1.0, 0.0, 0.0], min_deg=-150.0, max_deg=10.0)
    for name in ("lfootj", "rfootj"):
        add(name, "hinge", axis=[1.0, 0.0, 0.0], min_deg=-40.0, max_deg=30.0)
    add("ltoesj", "endpoint")
    add("rtoesj", "endpoint")
    for name in ("larmuppj", "rarmuppj"):
        add(
            name,
            "ball",
            twist_axis=outgoing_direction(name),
            max_swing_deg=120.0,
            max_twist_deg=60.0,
        )
    # Positive rotation about these axes swings the forearm forward (+Y).
    add("larmlowj", "hinge", axis=[0.0, 0.0, -1.0], min_deg=0.0, max_deg=150.0)
    add("rarmlowj", "hinge", axis=[0.0, 0.0, 1.0], min_deg=0.0, max_deg=150.0)
    for name in ("lhandj", "rhandj"):
        add(
            name,
            "ball",
            twist_axis=outgoing_direction(name),
            max_swing_deg=60.0,
            max_twist_deg=40.0,
        )
    add("lfingj", "endpoint")
    add("rfingj", "endpoint")

    joint_docs = [{"name": name, "rest_position_m": positions[name]} for name in joints_in_order()]
    return {
        "format": "PROC_BODY",
        "version": 1,
        "skeleton": SKELETON,
        "name": "neutral",
        "anthropometry": {
            "description": (
                "External reference measurements only. They are not joint-center "
                "distances and they do not drive bone lengths."
            ),
            "stature_m": 1.75,
            "shoulder_breadth_m": 0.41,
            "hip_breadth_m": 0.34,
            "chest_breadth_m": 0.31,
            "hand_length_m": 0.19,
            "foot_length_m": 0.265,
            "head_height_m": 0.23,
            "arm_span_m": 1.78,
        },
        "joints": joint_docs,
        "bones": bone_docs,
        "constraints": constraints,
    }


def joints_in_order():
    return [
        "legrootj",
        "spine1j",
        "spine2j",
        "armrootj",
        "neckj",
        "headj",
        "lleguppj",
        "lleglowj",
        "lfootj",
        "ltoesj",
        "rleguppj",
        "rleglowj",
        "rfootj",
        "rtoesj",
        "larmuppj",
        "larmlowj",
        "lhandj",
        "lfingj",
        "rarmuppj",
        "rarmlowj",
        "rhandj",
        "rfingj",
    ]


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]


def qrot(q, v):
    x, y, z, w = q
    ix = w * v[0] + y * v[2] - z * v[1]
    iy = w * v[1] + z * v[0] - x * v[2]
    iz = w * v[2] + x * v[1] - y * v[0]
    iw = -x * v[0] - y * v[1] - z * v[2]
    return [
        ix * w + iw * -x + iy * -z - iz * -y,
        iy * w + iw * -y + iz * -x - ix * -z,
        iz * w + iw * -z + ix * -y - iy * -x,
    ]


def axis_quat(axis, degrees):
    half = math.radians(degrees) * 0.5
    length = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    s = math.sin(half) / length
    return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(half)]


def qclean(q):
    length = math.sqrt(sum(c * c for c in q))
    q = [c / length for c in q]
    if q[3] < 0.0:
        q = [-c for c in q]
    return [num(c) for c in q]


def q_from(pairs):
    q = [0.0, 0.0, 0.0, 1.0]
    for axis, degrees in pairs:
        if abs(degrees) < 1e-10:
            continue
        q = qmul(axis_quat(axis, degrees), q)
    return qclean(q)


IDENT = [0.0, 0.0, 0.0, 1.0]


def smooth(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


def lerp(a, b, u):
    return a + (b - a) * u


def lobe(phase, center, half_width):
    delta = (phase - center + 0.5) % 1.0 - 0.5
    if abs(delta) >= half_width:
        return 0.0
    x = delta / half_width
    return 0.5 * (1.0 + math.cos(math.pi * x))


def write_json(path: Path, doc):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {path.relative_to(ROOT)}")


def make_clip(name, seconds, fps, channels, root_fn, contact_fn=None):
    frame_count = int(round(seconds * fps))
    frames = []
    for index in range(frame_count):
        time_s = index / fps
        translation, rotation = root_fn(time_s, seconds)
        joints = {}
        for joint_name, sampler in channels.items():
            joints[joint_name] = qclean(sampler(time_s, seconds))
        frame = {
            "time": num(time_s),
            "root": {"translation": [num(c) for c in translation], "rotation": qclean(rotation)},
            "joints": joints,
        }
        if contact_fn is not None:
            frame["contacts"] = contact_fn(time_s, seconds)
        frames.append(frame)
    return {
        "format": "PROC_MOTION",
        "version": 1,
        "skeleton": SKELETON,
        "source": {"type": "synthetic", "dataset": None, "clip": name},
        "timing": {
            "fps": fps,
            "frame_count": frame_count,
            "duration_s": frame_count / fps,
        },
        "frames": frames,
    }


def still_root(_time_s, _seconds):
    return (0.0, 0.0, 0.0), IDENT


def arm_channels():
    def shoulder(side, time_s, seconds):
        swing = 40.0 * math.sin(2.0 * math.pi * time_s / seconds)
        lift = 18.0 * (0.5 - 0.5 * math.cos(2.0 * math.pi * time_s / seconds))
        # Positive rotation about side * -Z swings that arm forward.
        # A small rotation about Y raises or lowers it.
        sign = -1.0 if side == "l" else 1.0
        return q_from((((0.0, 0.0, sign), swing), ((0.0, 1.0, 0.0), -lift if side == "r" else lift)))

    def elbow(time_s, seconds):
        bend = 15.0 + 80.0 * (0.5 - 0.5 * math.cos(2.0 * math.pi * time_s / seconds))
        return bend

    def wrist(time_s, seconds):
        twist = 30.0 * math.sin(2.0 * math.pi * time_s / seconds)
        wag = 15.0 * math.sin(4.0 * math.pi * time_s / seconds)
        return q_from((((1.0, 0.0, 0.0), twist), ((0.0, 1.0, 0.0), wag)))

    return {
        "larmuppj": lambda t, s: shoulder("l", t, s),
        "rarmuppj": lambda t, s: shoulder("r", t, s),
        "larmlowj": lambda t, s: q_from((((0.0, 0.0, -1.0), elbow(t, s)),)),
        "rarmlowj": lambda t, s: q_from((((0.0, 0.0, 1.0), elbow(t, s)),)),
        "lhandj": wrist,
        "rhandj": wrist,
    }


def leg_piece(time_s):
    """Featured leg: hip, knee, ankle degrees. Knee passes the placeholder limit."""
    if time_s < 1.0:
        u = smooth(time_s / 1.0)
        return 32.0 * math.sin(u * math.pi), -12.0 * u, 0.0
    if time_s < 2.0:
        u = smooth((time_s - 1.0) / 1.0)
        return lerp(0.0, 5.0, u), lerp(0.0, -90.0, u), lerp(0.0, 8.0, u)
    if time_s < 2.7:
        u = smooth((time_s - 2.0) / 0.7)
        return 5.0, lerp(-90.0, -165.0, u), 8.0
    if time_s < 3.3:
        u = math.sin(math.pi * (time_s - 2.7) / 0.6)
        return 5.0, -165.0, 8.0 + 18.0 * u
    u = smooth((time_s - 3.3) / 0.7)
    return lerp(5.0, 0.0, u), lerp(-165.0, 0.0, u), lerp(8.0, 0.0, u)


def leg_channels():
    def pack(scale):
        def sampler(time_s, _seconds, which):
            hip, knee, ankle = leg_piece(time_s)
            value = {"hip": hip * scale, "knee": knee * scale, "ankle": ankle * scale}[which]
            return value

        return sampler

    right = pack(1.0)
    left = pack(0.35)
    return {
        "rleguppj": lambda t, s: q_from((((1.0, 0.0, 0.0), right(t, s, "hip")),)),
        "rleglowj": lambda t, s: q_from((((1.0, 0.0, 0.0), right(t, s, "knee")),)),
        "rfootj": lambda t, s: q_from((((1.0, 0.0, 0.0), right(t, s, "ankle")),)),
        "lleguppj": lambda t, s: q_from((((1.0, 0.0, 0.0), left(t, s, "hip")),)),
        "lleglowj": lambda t, s: q_from((((1.0, 0.0, 0.0), left(t, s, "knee")),)),
        "lfootj": lambda t, s: q_from((((1.0, 0.0, 0.0), left(t, s, "ankle")),)),
    }


def leg_contacts(time_s, _seconds):
    _hip, knee, _ankle = leg_piece(time_s)
    planted = knee > -30.0
    return {
        "left_heel": True,
        "left_toe": True,
        "right_heel": planted,
        "right_toe": planted,
    }


def spine_channels():
    def wave(time_s, seconds, amplitude, phase):
        return amplitude * math.sin(2.0 * math.pi * time_s / seconds + phase)

    return {
        "spine1j": lambda t, s: q_from(
            (((1.0, 0.0, 0.0), wave(t, s, -18.0, 0.0)), ((0.0, 1.0, 0.0), wave(t, s, 10.0, 0.4)))
        ),
        "spine2j": lambda t, s: q_from(
            (((1.0, 0.0, 0.0), wave(t, s, 16.0, 0.7)), ((0.0, 1.0, 0.0), wave(t, s, -8.0, 1.0)))
        ),
        "armrootj": lambda t, s: q_from((((1.0, 0.0, 0.0), wave(t, s, -12.0, 1.2)),)),
        "neckj": lambda t, s: q_from(
            (((1.0, 0.0, 0.0), wave(t, s, -22.0, 0.5)), ((0.0, 1.0, 0.0), wave(t, s, 12.0, 1.4)))
        ),
    }


def root_motion(time_s, seconds):
    phase = time_s / seconds
    translation = (
        0.75 * math.sin(2.0 * math.pi * phase),
        0.75 * (1.0 - math.cos(2.0 * math.pi * phase)),
        0.03 * math.sin(4.0 * math.pi * phase),
    )
    rotation = q_from((((0.0, 0.0, 1.0), 360.0 * phase),))
    return translation, rotation


def root_channels():
    knees = q_from((((1.0, 0.0, 0.0), -18.0),))
    return {
        "lleglowj": lambda _t, _s: knees,
        "rleglowj": lambda _t, _s: knees,
    }


def walk_phase(time_s, seconds):
    return time_s / seconds


def walk_channels():
    def angles(phase, swing_center):
        swing = lobe(phase, swing_center, 0.25)
        hip = 34.0 * swing + (-14.0) * (1.0 - swing)
        knee = -10.0 + (-58.0) * swing
        ankle = 14.0 * swing + (-10.0) * (1.0 - swing)
        return hip, knee, ankle

    def arm(phase, swing_center):
        swing = lobe(phase, swing_center, 0.25)
        return 30.0 * swing + (-20.0) * (1.0 - swing)

    def elbow(phase, swing_center):
        swing = lobe(phase, swing_center, 0.25)
        return 16.0 + 12.0 * (1.0 - swing)

    return {
        "lleguppj": lambda t, s: q_from((((1.0, 0.0, 0.0), angles(walk_phase(t, s), 0.25)[0]),)),
        "lleglowj": lambda t, s: q_from((((1.0, 0.0, 0.0), angles(walk_phase(t, s), 0.25)[1]),)),
        "lfootj": lambda t, s: q_from((((1.0, 0.0, 0.0), angles(walk_phase(t, s), 0.25)[2]),)),
        "rleguppj": lambda t, s: q_from((((1.0, 0.0, 0.0), angles(walk_phase(t, s), 0.75)[0]),)),
        "rleglowj": lambda t, s: q_from((((1.0, 0.0, 0.0), angles(walk_phase(t, s), 0.75)[1]),)),
        "rfootj": lambda t, s: q_from((((1.0, 0.0, 0.0), angles(walk_phase(t, s), 0.75)[2]),)),
        "larmuppj": lambda t, s: q_from((((0.0, 0.0, -1.0), arm(walk_phase(t, s), 0.75)),)),
        "larmlowj": lambda t, s: q_from((((0.0, 0.0, -1.0), elbow(walk_phase(t, s), 0.75)),)),
        "rarmuppj": lambda t, s: q_from((((0.0, 0.0, 1.0), arm(walk_phase(t, s), 0.25)),)),
        "rarmlowj": lambda t, s: q_from((((0.0, 0.0, 1.0), elbow(walk_phase(t, s), 0.25)),)),
    }


def walk_root(time_s, seconds):
    phase = time_s / seconds
    return (0.0, 1.15 * time_s, 0.012 * math.sin(4.0 * math.pi * phase)), IDENT


def leg_contact_flags(phase, center):
    delta = (phase - center + 0.5) % 1.0 - 0.5
    if abs(delta) < 0.18:
        return False, False
    if -0.30 <= delta < -0.18:
        return False, True
    if 0.18 < delta <= 0.30:
        return True, False
    return True, True


def walk_contacts(time_s, seconds):
    phase = time_s / seconds
    left_heel, left_toe = leg_contact_flags(phase, 0.25)
    right_heel, right_toe = leg_contact_flags(phase, 0.75)
    return {
        "left_heel": left_heel,
        "left_toe": left_toe,
        "right_heel": right_heel,
        "right_toe": right_toe,
    }


def verify(body, motions):
    bones = body["bones"]
    parents = {bone["child_joint"]: bone for bone in bones}
    max_error = 0.0
    for motion in motions:
        for frame in motion["frames"]:
            pos = {"legrootj": tuple(frame["root"]["translation"])}
            # Rest offset is applied by the runtime. This check only proves that
            # parent rotation cannot change a bone's length, so the root rest
            # position can be ignored.
            rot = {"legrootj": frame["root"]["rotation"]}
            pending = list(bones)
            guard = 0
            while pending:
                guard += 1
                if guard > 100:
                    raise RuntimeError("hierarchy did not resolve")
                bone = pending.pop(0)
                parent = bone["parent_joint"]
                if parent not in pos:
                    pending.append(bone)
                    continue
                child = bone["child_joint"]
                vector = bone["rest_vector_m"]
                pos[child] = vadd(pos[parent], qrot(rot[parent], vector))
                local = frame["joints"].get(child, IDENT)
                rot[child] = qmul(rot[parent], local)
                error = abs(vlen(vsub(pos[child], pos[parent])) - bone["length_m"])
                max_error = max(max_error, error)
    print(f"generator bone-length check max error {max_error:.3e} m")
    if max_error > 1e-8:
        raise SystemExit("synthetic motion broke a bone length")


def main():
    body = build_body()
    write_json(ROOT / "assets" / "bodies" / "neutral.procbody.json", body)
    print("neutral bone lengths:")
    for bone in body["bones"]:
        print(f"  {bone['name']:10} {bone['length_m']:.4f} m")

    motions = [
        ("arm_test", make_clip("arm_test", 2.0, 30.0, arm_channels(), still_root)),
        ("leg_test", make_clip("leg_test", 4.0, 30.0, leg_channels(), still_root, leg_contacts)),
        ("spine_test", make_clip("spine_test", 3.0, 30.0, spine_channels(), still_root)),
        ("root_motion_test", make_clip("root_motion_test", 4.0, 30.0, root_channels(), root_motion)),
        ("walkish_test", make_clip("walkish_test", 2.0, 30.0, walk_channels(), walk_root, walk_contacts)),
    ]
    for name, motion in motions:
        write_json(ROOT / "assets" / "motions" / f"{name}.procmo", motion)
    verify(body, [motion for _name, motion in motions])


if __name__ == "__main__":
    main()
