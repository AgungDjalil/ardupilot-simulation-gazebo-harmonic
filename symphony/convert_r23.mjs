// R23's web models (Symphony public/models/r23) as OBJ meshes Gazebo can load.
//
//   node symphony/convert_r23.mjs [SYMPHONY_DIR] [OUT_MODELS_DIR]
//   (defaults ~/Documents/symphony and ardupilot_sim/models)
//
// The GLBs are made for three.js: meshopt-compressed, quantized and GPU-instanced (EXT_meshopt_compression,
// KHR_mesh_quantization, EXT_mesh_gpu_instancing), none of which Gazebo's Assimp loader reads. This decodes them
// with the three.js in Symphony's node_modules (read only) and writes plain OBJ, in the frame Symphony's
// PanggungR23.tsx draws them in: metres, Z up, the Hilux in its CAD frame (Y towards the rear), and the drones
// fused the way its fusi() does (every mesh and instance in one, centred in x/y, base at z = 0; the quad scaled
// 1.45 to the R23's 13" / 600 mm motor diagonal).
import fs from "fs";
import os from "os";
import path from "path";
import { pathToFileURL } from "url";

const SYM = process.argv[2] ?? path.join(os.homedir(), "Documents/symphony");
const OUT = process.argv[3] ?? path.join(path.dirname(path.dirname(new URL(import.meta.url).pathname)), "models");
const mod = (p) => import(pathToFileURL(path.join(SYM, "node_modules/three", p)).href);
const THREE = await mod("build/three.module.js");
const { GLTFLoader } = await mod("examples/jsm/loaders/GLTFLoader.js");
const { MeshoptDecoder } = await mod("examples/jsm/libs/meshopt_decoder.module.js");
const { mergeVertices } = await mod("examples/jsm/utils/BufferGeometryUtils.js");

const loader = new GLTFLoader();
loader.setMeshoptDecoder(MeshoptDecoder);
const load = (f) => {
  const buf = fs.readFileSync(path.join(SYM, "public/models/r23", f));
  return new Promise((res, rej) => loader.parse(buf.buffer.slice(buf.byteOffset, buf.byteOffset + buf.byteLength), "", res, rej));
};

/** Every mesh and instance of the scene in one position-only geometry, in world coordinates. */
function fuse(root) {
  root.updateMatrixWorld(true);
  const pos = [];
  const v = new THREE.Vector3(), m = new THREE.Matrix4(), im = new THREE.Matrix4();
  root.traverse((o) => {
    if (!o.isMesh) return;
    const p = o.geometry.attributes.position, idx = o.geometry.index;
    const n = idx ? idx.count : p.count;
    for (let k = 0; k < (o.isInstancedMesh ? o.count : 1); k++) {
      m.copy(o.matrixWorld);
      if (o.isInstancedMesh) { o.getMatrixAt(k, im); m.multiply(im); }
      for (let i = 0; i < n; i++) {
        v.fromBufferAttribute(p, idx ? idx.getX(i) : i).applyMatrix4(m);
        pos.push(v.x, v.y, v.z);
      }
    }
  });
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  return g;
}

/** PanggungR23's fusi(): centred in x/y, base at z = 0, then scaled. */
function centre(g, scale) {
  g.computeBoundingBox();
  const b = g.boundingBox;
  g.translate(-(b.min.x + b.max.x) / 2, -(b.min.y + b.max.y) / 2, -b.min.z);
  g.scale(scale, scale, scale);
  return g;
}

function writeObj(g, file, note) {
  g = mergeVertices(g, 1e-5);
  const p = g.attributes.position, idx = g.index;
  const lines = [`# ${note}`];
  for (let i = 0; i < p.count; i++) lines.push(`v ${p.getX(i).toFixed(5)} ${p.getY(i).toFixed(5)} ${p.getZ(i).toFixed(5)}`);
  for (let i = 0; i < idx.count; i += 3) lines.push(`f ${idx.getX(i) + 1} ${idx.getX(i + 1) + 1} ${idx.getX(i + 2) + 1}`);
  fs.mkdirSync(path.dirname(file), { recursive: true });
  fs.writeFileSync(file, lines.join("\n") + "\n");
  g.computeBoundingBox();
  const b = g.boundingBox, s = (k) => (b.max[k] - b.min[k]).toFixed(3);
  console.log(`${path.relative(OUT, file)}: ${idx.count / 3} triangles, ${s("x")} x ${s("y")} x ${s("z")} m, ` +
    `min (${b.min.x.toFixed(3)}, ${b.min.y.toFixed(3)}, ${b.min.z.toFixed(3)})`);
}

const credit = 'from Symphony public/models/r23; "2026 Toyota Hilux" by missyubibi attage, CC-BY-4.0';
writeObj(fuse((await load("hilux-body-lite.glb")).scene), path.join(OUT, "r23_hilux/meshes/hilux_body.obj"), credit);
writeObj(centre(fuse((await load("quad.glb")).scene), 1.45), path.join(OUT, "r23_quad/meshes/quad.obj"),
  "R23 quad (ego_uav, own CAD) from Symphony public/models/r23, scaled 1.45");
writeObj(centre(fuse((await load("hexa.glb")).scene), 1), path.join(OUT, "r23_hilux/meshes/hexa.obj"),
  "R23 hexa (hexa_norden, own CAD) from Symphony public/models/r23");
