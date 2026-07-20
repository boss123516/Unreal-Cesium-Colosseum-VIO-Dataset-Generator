# RC Cessna visual source

`rc_cessna_body.fbx` is a mechanical FBX conversion of PX4's
`Tools/simulation/gz/models/rc_cessna/meshes/body.dae`, authored by Benjamin
Perseghetti for the PX4 `rc_cessna` simulation model. It is used only as the
UCC rendering proxy; Gazebo remains the physics source.

Conversion command:

```bash
assimp export body.dae rc_cessna_body.fbx -f fbx
```
