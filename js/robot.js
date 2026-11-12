// The quadruped: CAD-built meshes (cad/quadruped.py), a trot gait that picks footholds from the trunk's
// actual motion, and three-joint inverse kinematics per leg. Works in metres; the trunk pose comes from physics.
import { Transform, Program, Geometry, Mesh } from '../vendor/ogl.min.js';

// keep in sync with the parameters in cad/quadruped.py (scene frame: x forward, y up, z right)
export const KIN = { hipX: 0.205, hipY: 0.058, thighY: 0.094, link: 0.22, footRadius: 0.024, stand: 0.31, lidar: [0.118, 0.088, 0] };

const LEGS = [
    { name: 'FL', sx: 1, sz: -1, side: 'L', group: 0 },
    { name: 'FR', sx: 1, sz: 1, side: 'R', group: 1 },
    { name: 'RL', sx: -1, sz: -1, side: 'L', group: 1 },
    { name: 'RR', sx: -1, sz: 1, side: 'R', group: 0 },
];
const PERIOD = 0.46;        // one trot cycle, seconds; each diagonal pair swings for half of it

export function createRobot(gl, parent, cell) {
    const program = new Program(gl, {
        vertex: `
            attribute vec3 position;
            attribute vec3 normal;
            attribute vec3 color;
            uniform mat4 modelViewMatrix;
            uniform mat4 projectionMatrix;
            uniform mat3 normalMatrix;
            varying vec3 vNormal;
            varying vec3 vColor;
            void main() {
                vNormal = normalMatrix * normal;
                vColor = color;
                gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
            }`,
        fragment: `
            precision highp float;
            uniform vec3 uRim;
            uniform float uAmbient, uTint;
            varying vec3 vNormal;
            varying vec3 vColor;
            void main() {
                vec3 n = normalize(vNormal);
                float diffuse = max(dot(n, normalize(vec3(0.45, 0.75, 0.5))), 0.0);
                float sky = 0.5 + 0.5 * n.y;
                vec3 c = vColor * uTint * (uAmbient + (1.0 - uAmbient) * diffuse) * mix(0.78, 1.06, sky);
                c += uRim * pow(1.0 - max(n.z, 0.0), 2.5);
                gl_FragColor = vec4(c, 1.0);
            }`,
        uniforms: { uRim: { value: [0, 0, 0] }, uAmbient: { value: 0.4 }, uTint: { value: 1 } },
        cullFace: false,
    });

    const root = new Transform();
    root.setParent(parent);
    root.scale.set(1 / cell);
    root.visible = false;
    const lidar = new Transform();
    lidar.position.set(KIN.lidar[0], KIN.lidar[1], KIN.lidar[2]);
    lidar.setParent(root);

    const legs = LEGS.map((def) => {
        const hip = new Transform(), thigh = new Transform(), calf = new Transform();
        hip.position.set(def.sx * KIN.hipX, 0, def.sz * KIN.hipY); hip.setParent(root);
        thigh.position.set(0, 0, def.sz * KIN.thighY); thigh.setParent(hip);
        calf.position.set(0, -KIN.link, 0); calf.setParent(thigh);
        return { ...def, hip, thigh, calf, fx: 0, fz: 0, lift: 0, swinging: false, fromX: 0, fromZ: 0, nx: 0, nz: 0, vx: 0, vz: 0 };
    });

    function mesh(geometry, parentNode) {
        new Mesh(gl, { program, geometry, frustumCulled: false }).setParent(parentNode);
    }

    async function load(url) {
        const res = await fetch(url);
        if (!res.ok) throw new Error('robot model unavailable');
        const buf = await res.arrayBuffer();
        const jsonLength = new DataView(buf).getUint32(0, true);
        const header = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 4, jsonLength)));
        const n = header.vertexCount;
        let offset = 4 + jsonLength;
        const pos = new Int16Array(buf, offset, n * 3); offset += Math.ceil(n * 6 / 4) * 4;
        const nrm = new Int8Array(buf, offset, n * 4); offset += n * 4;
        const col = new Uint8Array(buf, offset, n * 4); offset += n * 4;
        const idx = new Uint16Array(buf, offset, header.indexCount);

        const part = (name) => {
            const p = header.parts[name], count = p.vertexCount;
            const position = new Float32Array(count * 3), normal = new Float32Array(count * 3), color = new Float32Array(count * 3);
            for (let i = 0; i < count; i++) {
                const v = p.vertexStart + i;
                for (let k = 0; k < 3; k++) {
                    position[i * 3 + k] = pos[v * 3 + k] * header.scale;
                    normal[i * 3 + k] = nrm[v * 4 + k] / 127;
                    color[i * 3 + k] = col[v * 4 + k] / 255;
                }
            }
            return new Geometry(gl, {
                position: { size: 3, data: position }, normal: { size: 3, data: normal }, color: { size: 3, data: color },
                index: { data: idx.slice(p.indexStart, p.indexStart + p.indexCount) },
            });
        };
        mesh(part('trunk'), root);
        mesh(part('lidar'), lidar);
        const shared = { hipL: part('hipL'), hipR: part('hipR'), thighL: part('thighL'), thighR: part('thighR'), calfL: part('calfL'), calfR: part('calfR') };
        for (const leg of legs) {
            mesh(shared['hip' + leg.side], leg.hip);
            mesh(shared['thigh' + leg.side], leg.thigh);
            mesh(shared['calf' + leg.side], leg.calf);
        }
        root.visible = true;
    }

    // If the model cannot be fetched, the trunk is still drawn as a plain block so the scene keeps working.
    function useFallback() {
        const p = [], nn = [], c = [], ix = [];
        const [hx, hy, hz] = [0.3, 0.07, 0.085];
        const faces = [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]];
        for (const f of faces) {
            const u = f[0] ? [0, 1, 0] : [1, 0, 0], v = [f[1] * u[2] - f[2] * u[1], f[2] * u[0] - f[0] * u[2], f[0] * u[1] - f[1] * u[0]];
            const b = p.length / 3;
            for (const [a, d] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
                p.push((f[0] + a * u[0] + d * v[0]) * hx, (f[1] + a * u[1] + d * v[1]) * hy, (f[2] + a * u[2] + d * v[2]) * hz);
                nn.push(...f); c.push(1, 0.698, 0.141);
            }
            ix.push(b, b + 1, b + 2, b, b + 2, b + 3);
        }
        mesh(new Geometry(gl, {
            position: { size: 3, data: new Float32Array(p) }, normal: { size: 3, data: new Float32Array(nn) },
            color: { size: 3, data: new Float32Array(c) }, index: { data: new Uint16Array(ix) },
        }), root);
        root.visible = true;
    }

    // ---------- gait ----------
    let phase = 0, running = false, placed = false;

    function footholds(s) {
        const cy = Math.cos(s.yaw), sy = Math.sin(s.yaw);
        let worst = 0;
        for (const leg of legs) {
            const lx = leg.sx * KIN.hipX, lz = leg.sz * (KIN.hipY + KIN.thighY);
            const rx = lx * cy - lz * sy, rz = lx * sy + lz * cy;
            leg.nx = s.x + rx; leg.nz = s.z + rz;
            leg.vx = s.vx + s.wy * rz; leg.vz = s.vz - s.wy * rx;       // velocity of the hip over the ground
            worst = Math.max(worst, Math.hypot(leg.fx - leg.nx, leg.fz - leg.nz));
        }
        return worst;
    }
    function plant() {
        for (const leg of legs) { leg.fx = leg.nx; leg.fz = leg.nz; leg.lift = 0; leg.swinging = false; }
        phase = 0; running = false; placed = true;
    }
    function liftOff(group) {
        for (const leg of legs) if (leg.group === group) { leg.swinging = true; leg.fromX = leg.fx; leg.fromZ = leg.fz; }
    }

    // s: { x, y, z, quat: [x, y, z, w], yaw, vx, vz, wy, sweep } in metres and radians
    function update(dt, s) {
        const worst = footholds(s);
        if (!placed || worst > 0.6) plant();
        const speed = Math.hypot(s.vx, s.vz);
        const active = dt > 0 && (speed > 0.06 || Math.abs(s.wy) > 0.2 || worst > 0.07);

        if (active && !running) { running = true; liftOff(Math.floor(phase * 2) % 2); }
        if (running) {
            const before = Math.floor(phase * 2);
            phase += dt / PERIOD;
            if (Math.floor(phase * 2) !== before) {
                // the swinging pair lands on its footholds; the other pair lifts off, or the robot settles
                for (const leg of legs) if (leg.swinging) { leg.swinging = false; leg.fx = leg.tx; leg.fz = leg.tz; leg.lift = 0; }
                phase %= 1;
                if (active) liftOff(Math.floor(phase * 2) % 2);
                else { running = false; phase = Math.floor(phase * 2) / 2; }
            }
        }
        const u = (phase * 2) % 1, ease = u * u * (3 - 2 * u);
        const height = 0.045 + 0.04 * Math.min(1, speed);
        for (const leg of legs) {
            // Raibert heuristic: land where the hip will be halfway through the coming stance
            leg.tx = leg.nx + leg.vx * PERIOD / 4; leg.tz = leg.nz + leg.vz * PERIOD / 4;
            if (!leg.swinging) continue;
            leg.fx = leg.fromX + (leg.tx - leg.fromX) * ease;
            leg.fz = leg.fromZ + (leg.tz - leg.fromZ) * ease;
            leg.lift = height * Math.sin(Math.PI * u);
        }

        // ---------- pose ----------
        const [qx, qy, qz, qw] = s.quat;
        root.position.set(s.x / cell, s.y / cell, s.z / cell);
        root.quaternion.set(qx, qy, qz, qw);
        lidar.rotation.y = -(s.sweep - s.yaw);

        const L = KIN.link;
        for (const leg of legs) {
            // foot target in the trunk frame (rotate by the inverse of the trunk orientation)
            const wx = leg.fx - s.x, wy = KIN.footRadius + leg.lift - s.y, wz = leg.fz - s.z;
            const tx = 2 * (-qy * wz + qz * wy), ty = 2 * (-qz * wx + qx * wz), tz = 2 * (-qx * wy + qy * wx);
            const lx = wx + qw * tx + (-qy * tz + qz * ty), ly = wy + qw * ty + (-qz * tx + qx * tz), lz = wz + qw * tz + (-qx * ty + qy * tx);
            // relative to the abduction joint
            const px = lx - leg.sx * KIN.hipX, py = ly, pz = lz - leg.sz * KIN.hipY;
            const off = leg.sz * KIN.thighY;
            const h = Math.sqrt(Math.max(py * py + pz * pz - off * off, 1e-6));     // leg length in its own plane
            const abduction = Math.atan2(pz, py) - Math.atan2(off, -h);
            const d = Math.min(Math.max(Math.hypot(px, h), 0.08), 2 * L - 0.004);
            const a = Math.atan2(-h, px), A = Math.acos(d / (2 * L));
            const thx = Math.cos(a - A), thy = Math.sin(a - A);                    // thigh direction, knee behind
            const cx = d * Math.cos(a) - L * thx, cy = d * Math.sin(a) - L * thy;  // calf direction
            const q2 = Math.atan2(-thx, -thy), q3 = Math.atan2(-cx, -cy) - q2;
            leg.hip.rotation.x = abduction;
            leg.thigh.rotation.z = -q2;
            leg.calf.rotation.z = -q3;
        }
    }

    function setTheme(dark) {
        program.uniforms.uRim.value = dark ? [0.1, 0.17, 0.24] : [0.05, 0.05, 0.05];
        program.uniforms.uAmbient.value = dark ? 0.42 : 0.5;
        program.uniforms.uTint.value = dark ? 1 : 0.9;
    }

    return { root, load, useFallback, update, setTheme, get stepping() { return running; } };
}
