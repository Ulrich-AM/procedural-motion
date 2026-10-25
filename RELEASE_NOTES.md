# v0.1.0 — Motion Import & Bone-Preserving Retargeting

First public release of PROC Motion Lab.

## PROC BODY

A PROC BODY owns the skeleton, the rest pose, and the bone lengths. The sample body is `assets/bodies/neutral.procbody.json`: skeleton `proc_humanoid_v0`, 22 joints, 21 locked bones, meters, T-pose. Motion can rotate joints and can move or rotate the root. It cannot resize a bone or translate any joint except the root.

## .procmo

`PROC_MOTION` version 1 stores timing, root translation, root rotation, and local joint rotations. It does not store bone lengths, scale, or non-root positions. One clip plays on any PROC body. Joint-limit clamping happens at preview time and is not written into the file.

## Import

- Native BVH parser. Channel order is taken from the file and converted to quaternions at import.
- FBX through Assimp: hierarchy, rest transforms, and translation and rotation keys. No mesh import.
- Both paths produce a source-neutral clip in the PROC coordinate system: +X right, +Y forward, +Z up, meters.

## Retarget

Bind-relative rotational retarget. The rotational change of a source segment, measured from that source's own rest pose, is applied to the matching PROC segment. Source bone lengths are not copied. Mixamo and LAFAN1 profiles are included. Mappings can be detected, edited, and saved. The viewer can show the PROC skeleton, the source skeleton, or both with roots aligned.

## Validation

The lab reports bone-length error, rest-retarget angular error, and segment-direction error. Automated tests cover the Phase 1 body and player, BVH parsing, bind-relative retarget math, playback of one clip on two different proportions, and `.procmo` export and reload.

## Known limitations

- No foot IK, no foot locking, and no automatic contact detection. Foot sliding is expected.
- A target with different leg length will not reproduce the source stride. Root travel is not scaled to the target.
- No procedural locomotion.
- No AMASS importer.
- No mesh rendering.
- Joint limits are still preliminary placeholders.
- Mixamo and LAFAN1 files are not bundled. Load them from a local path.
