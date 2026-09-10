# Vendored JS

No bundler, no CDN at runtime — the motor probe is served as static files from a
ROS package, so its dependencies live here. Pinned versions, fetched once.

| Library     | Version | License |
| ----------- | ------- | ------- |
| three       | 0.180.0 | MIT     |
| urdf-loader | 0.13.1  | MIT     |

## Refetch

Run from this directory.

```sh
THREE=0.180.0
URDF=0.13.1
mkdir -p three/examples/jsm/loaders three/examples/jsm/controls urdf-loader
curl -sSL -o three/three.module.js "https://cdn.jsdelivr.net/npm/three@$THREE/build/three.module.js"
curl -sSL -o three/three.core.js "https://cdn.jsdelivr.net/npm/three@$THREE/build/three.core.js"
curl -sSL -o three/examples/jsm/loaders/STLLoader.js "https://cdn.jsdelivr.net/npm/three@$THREE/examples/jsm/loaders/STLLoader.js"
curl -sSL -o three/examples/jsm/loaders/ColladaLoader.js "https://cdn.jsdelivr.net/npm/three@$THREE/examples/jsm/loaders/ColladaLoader.js"
curl -sSL -o three/examples/jsm/loaders/TGALoader.js "https://cdn.jsdelivr.net/npm/three@$THREE/examples/jsm/loaders/TGALoader.js"
curl -sSL -o three/examples/jsm/controls/OrbitControls.js "https://cdn.jsdelivr.net/npm/three@$THREE/examples/jsm/controls/OrbitControls.js"
curl -sSL -o urdf-loader/URDFLoader.js "https://cdn.jsdelivr.net/npm/urdf-loader@$URDF/src/URDFLoader.js"
curl -sSL -o urdf-loader/URDFClasses.js "https://cdn.jsdelivr.net/npm/urdf-loader@$URDF/src/URDFClasses.js"
```

## Why each file

- `three.module.js` imports `./three.core.js` — both are required.
- `URDFLoader.js` statically imports `STLLoader` and `ColladaLoader`; `ColladaLoader`
  imports `TGALoader`. All three ship even though the G1 meshes are STL only.
- `OrbitControls.js` drives the camera.

## What breaks it

- Deleting a file here: `app.js` catches the failed dynamic import and falls back to
  the list-only view. The page stays usable, the 3D pane does not.
- Bumping `three` without refetching the addons: addon and core versions must match.
