# Hiwonder assets (not committed)

The JetArm URDF/xacro and STL meshes are **not vendored** in this repository
(licensing + size). Fetch them once:

```bash
bash sim/scripts/fetch_assets.sh
```

Source: <https://github.com/Hiwonder/JetArm>, branch `Jetson_Orin_NX_ros1`,
package `jetarm/src/jetarm_6dof_simulate/jetarm_6dof_description`.

© Hiwonder. These files are used here for interoperability with a purchased
JetArm. Do not redistribute them in a public repository; keep this directory
git-ignored and let users fetch from Hiwonder's repository.

The generated `sim/urdf/jetarm_6dof.urdf` **is** committed, so you can inspect
the model before fetching; fetching is required for the meshes (visual/collision)
and for regenerating the URDF from xacro.
