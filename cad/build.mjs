// Packs the CAD export (cad/quadruped.py) into the single binary the hero scene loads.
//
//   npm install --no-save meshoptimizer
//   node build.mjs <cad-output-dir> ../assets/quadruped.bin
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
const [,, SRC, OUT] = process.argv;
const { MeshoptSimplifier } = createRequire(import.meta.url)('meshoptimizer');
await MeshoptSimplifier.ready;

const manifest = JSON.parse(fs.readFileSync(path.join(SRC, 'manifest.json'), 'utf8'));

function loadObj(file) {
    const v = [], tris = [];
    for (const line of fs.readFileSync(path.join(SRC, file), 'utf8').split('\n')) {
        const p = line.split(' ');
        if (p[0] === 'v') v.push(+p[1], +p[2], +p[3]);
        else if (p[0] === 'f') tris.push(p[1] - 1, p[2] - 1, p[3] - 1);
    }
    return { v: new Float32Array(v), tris: new Uint32Array(tris) };
}
// weld by position, drop degenerate triangles
function weld(mesh, mirror) {
    const map = new Map(), verts = [], remap = new Uint32Array(mesh.v.length / 3);
    for (let i = 0; i < mesh.v.length / 3; i++) {
        const p = [mesh.v[i * 3], mirror ? -mesh.v[i * 3 + 1] : mesh.v[i * 3 + 1], mesh.v[i * 3 + 2]];
        const key = p.map((c) => Math.round(c * 2e5)).join(',');
        let id = map.get(key);
        if (id === undefined) { id = verts.length / 3; map.set(key, id); verts.push(...p); }
        remap[i] = id;
    }
    const tris = [];
    for (let t = 0; t < mesh.tris.length; t += 3) {
        const a = remap[mesh.tris[t]], b = remap[mesh.tris[t + 1]], c = remap[mesh.tris[t + 2]];
        if (a !== b && b !== c && a !== c) tris.push(a, mirror ? c : b, mirror ? b : c);    // mirroring flips the winding
    }
    return { v: new Float32Array(verts), tris: new Uint32Array(tris) };
}
function simplify(mesh, targetTris) {
    if (mesh.tris.length / 3 <= targetTris) return mesh;
    let [idx] = MeshoptSimplifier.simplify(mesh.tris, mesh.v, 3, targetTris * 3, 0.05);
    if (idx.length / 3 > targetTris * 1.4 && MeshoptSimplifier.simplifySloppy) {
        [idx] = MeshoptSimplifier.simplifySloppy(mesh.tris, mesh.v, 3, null, targetTris * 3, 0.08);
    }
    return { v: mesh.v, tris: idx };
}
// crease-aware vertex normals, then (position, normal) dedupe
function shade(mesh, color, out) {
    const { v, tris } = mesh, nt = tris.length / 3;
    const fn = new Float32Array(nt * 3), adj = new Map();
    for (let t = 0; t < nt; t++) {
        const [a, b, c] = [tris[t * 3], tris[t * 3 + 1], tris[t * 3 + 2]];
        const ux = v[b * 3] - v[a * 3], uy = v[b * 3 + 1] - v[a * 3 + 1], uz = v[b * 3 + 2] - v[a * 3 + 2];
        const wx = v[c * 3] - v[a * 3], wy = v[c * 3 + 1] - v[a * 3 + 1], wz = v[c * 3 + 2] - v[a * 3 + 2];
        fn[t * 3] = uy * wz - uz * wy; fn[t * 3 + 1] = uz * wx - ux * wz; fn[t * 3 + 2] = ux * wy - uy * wx;   // area-weighted
        for (const i of [a, b, c]) { if (!adj.has(i)) adj.set(i, []); adj.get(i).push(t); }
    }
    const unit = (x, y, z) => { const l = Math.hypot(x, y, z) || 1; return [x / l, y / l, z / l]; };
    const COS = Math.cos(42 * Math.PI / 180), seen = new Map();
    for (let t = 0; t < nt; t++) {
        const f = unit(fn[t * 3], fn[t * 3 + 1], fn[t * 3 + 2]);
        for (let k = 0; k < 3; k++) {
            const i = tris[t * 3 + k]; let nx = 0, ny = 0, nz = 0;
            for (const o of adj.get(i)) {
                const g = unit(fn[o * 3], fn[o * 3 + 1], fn[o * 3 + 2]);
                if (g[0] * f[0] + g[1] * f[1] + g[2] * f[2] >= COS) { nx += fn[o * 3]; ny += fn[o * 3 + 1]; nz += fn[o * 3 + 2]; }
            }
            const n = unit(nx, ny, nz).map((c) => Math.round(c * 127));
            const key = i + ':' + n.join(',') + ':' + color.join(',');
            let id = seen.get(key);
            if (id === undefined) {
                id = out.pos.length / 3; seen.set(key, id);
                out.pos.push(v[i * 3], v[i * 3 + 1], v[i * 3 + 2]); out.nrm.push(...n); out.col.push(...color.map((c) => Math.round(c * 255)));
            }
            out.idx.push(id);
        }
    }
}

// web parts: left links as modelled, right links mirrored across the X-Z plane
const PARTS = {
    trunk: { link: 'trunk', tris: 4200 }, lidar: { link: 'lidar', tris: 600 },
    hipL: { link: 'hip', tris: 700 }, hipR: { link: 'hip', tris: 700, mirror: true },
    thighL: { link: 'thigh', tris: 800 }, thighR: { link: 'thigh', tris: 800, mirror: true },
    calfL: { link: 'calf', tris: 620 }, calfR: { link: 'calf', tris: 620, mirror: true },
};

const header = { version: 1, scale: 0, parts: {} };
const all = { pos: [], nrm: [], col: [], idx: [] };
for (const [name, part] of Object.entries(PARTS)) {
    const meshes = manifest.links[part.link].map(({ file, color }) => ({ color: manifest.colors[color], mesh: weld(loadObj(file), part.mirror) }));
    const total = meshes.reduce((s, m) => s + m.mesh.tris.length / 3, 0);
    const out = { pos: [], nrm: [], col: [], idx: [] };
    for (const { color, mesh } of meshes) {
        const share = Math.max(40, Math.round(part.tris * (mesh.tris.length / 3) / total));
        shade(simplify(mesh, share), color, out);
    }
    if (out.pos.length / 3 > 65535) throw new Error(name + ' too large');
    header.parts[name] = { vertexStart: all.pos.length / 3, vertexCount: out.pos.length / 3, indexStart: all.idx.length, indexCount: out.idx.length };
    all.pos.push(...out.pos); all.nrm.push(...out.nrm); all.col.push(...out.col); all.idx.push(...out.idx);
    console.log(name.padEnd(8), 'tris', String(total).padStart(6), '->', out.idx.length / 3, 'verts', out.pos.length / 3);
}
// CAD frame (x forward, y left, z up) -> scene frame (x forward, y up, z right); quantise
const n = all.pos.length / 3;
let extent = 0;
for (const c of all.pos) extent = Math.max(extent, Math.abs(c));
header.scale = extent / 32767;
const pos = new Int16Array(n * 3), nrm = new Int8Array(n * 4), col = new Uint8Array(n * 4), idx = new Uint16Array(all.idx);
for (let i = 0; i < n; i++) {
    const q = (c) => Math.round(c / header.scale);
    pos[i * 3] = q(all.pos[i * 3]); pos[i * 3 + 1] = q(all.pos[i * 3 + 2]); pos[i * 3 + 2] = q(-all.pos[i * 3 + 1]);
    nrm[i * 4] = all.nrm[i * 3]; nrm[i * 4 + 1] = all.nrm[i * 3 + 2]; nrm[i * 4 + 2] = -all.nrm[i * 3 + 1];
    col[i * 4] = all.col[i * 3]; col[i * 4 + 1] = all.col[i * 3 + 1]; col[i * 4 + 2] = all.col[i * 3 + 2]; col[i * 4 + 3] = 255;
}
header.vertexCount = n; header.indexCount = idx.length;
let json = Buffer.from(JSON.stringify(header));
json = Buffer.concat([json, Buffer.alloc((4 - (json.length % 4)) % 4, 0x20)]);
const pad2 = (b) => (b.length % 4 ? Buffer.concat([b, Buffer.alloc(4 - (b.length % 4))]) : b);
const head = Buffer.alloc(4); head.writeUInt32LE(json.length);
const file = Buffer.concat([head, json, pad2(Buffer.from(pos.buffer)), Buffer.from(nrm.buffer), Buffer.from(col.buffer), pad2(Buffer.from(idx.buffer))]);
fs.writeFileSync(OUT, file);
console.log('vertices', n, 'triangles', idx.length / 3, 'bytes', file.length);
