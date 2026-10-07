`kinematics_fixed/` contains the imported CAD meshes for display and FK/IK;
their hashes are recorded in `config/kinematics_fixed.provenance.json`.
This import does not establish physical calibration or dynamics accuracy.

Place any future calibrated robot meshes here and reference them with
`package://wheel_human_description/meshes/...` URIs. The dummy model uses
primitive geometry and needs no mesh assets.
