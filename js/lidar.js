// Hero scene: a quadruped maps a factory floor with a spinning LiDAR.
// The occupancy grid, ray casting, costmap inflation and A* planner are real, and the robot is a
// physical body: click the floor to send a navigation goal, or drive it yourself and knock things over.
import { Renderer, Camera, Transform, Program, Geometry, Mesh, Texture } from '../vendor/ogl.min.js';
import { createPhysics } from './physics.js';
import { createRobot, KIN } from './robot.js';

const canvas = document.getElementById('lidar');
const hero = canvas && canvas.closest('.hero');

if (canvas && hero) {
    try {
        start();
    } catch (err) {
        hero.classList.add('no-webgl');
    }
}

function start() {
    const root = document.documentElement;
    const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    const coarse = window.matchMedia('(pointer: coarse)').matches;

    // ---------- World ----------
    const W = 76, H = 48;
    const CELL_M = 0.2;             // metres per cell: sets the robot's scale and the pose readout
    const RANGE = 17;               // LiDAR range in cells
    const MAX_H = 5;

    const height = new Float32Array(W * H);     // obstacle height per cell, 0 = free
    const idx = (i, j) => j * W + i;
    const inside = (i, j) => i >= 0 && j >= 0 && i < W && j < H;

    const rects = [];
    function rect(x, y, w, h, hgt) {
        rects.push([x, y, w, h, hgt]);
        for (let j = y; j < y + h; j++) for (let i = x; i < x + w; i++) if (inside(i, j)) height[idx(i, j)] = hgt;
    }

    // outer walls
    rect(0, 0, W, 1, 5); rect(0, H - 1, W, 1, 5); rect(0, 0, 1, H, 5); rect(W - 1, 0, 1, H, 5);
    // storage racks
    rect(8, 6, 2, 14, 4.4); rect(14, 6, 2, 14, 4.4); rect(20, 6, 2, 14, 4.4); rect(26, 6, 2, 14, 4.4);
    rect(6, 28, 18, 2, 4.4); rect(6, 35, 18, 2, 4.4);
    // partition wall with two doorways
    rect(34, 1, 1, 14, 5); rect(34, 21, 1, 12, 5); rect(34, 39, 1, 8, 5);
    // conveyor line and work cells
    rect(41, 9, 26, 3, 1.5);
    rect(42, 20, 5, 4, 3); rect(52, 20, 5, 4, 3); rect(62, 20, 5, 4, 3);
    // pillars
    rect(40, 30, 2, 2, 5); rect(50, 30, 2, 2, 5); rect(60, 30, 2, 2, 5);
    // control room
    rect(56, 38, 14, 1, 5); rect(56, 38, 1, 4, 5); rect(69, 38, 1, 9, 5);
    // pallets and a charging dock
    rect(28, 38, 3, 3, 1.2); rect(44, 40, 3, 3, 1.2); rect(71, 14, 3, 4, 2.2);

    const occ = (i, j) => !inside(i, j) || height[idx(i, j)] > 0;

    // costmap inflation: cells the robot's footprint cannot enter
    const blocked = new Uint8Array(W * H);
    for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
        let b = 0;
        for (let dj = -1; dj <= 1 && !b; dj++) for (let di = -1; di <= 1; di++) if (occ(i + di, j + dj)) { b = 1; break; }
        blocked[idx(i, j)] = b;
    }
    const freeCells = [];
    for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) if (!blocked[idx(i, j)]) freeCells.push([i, j]);

    // cells a LiDAR can ever observe: free space plus obstacle faces
    const observable = new Uint8Array(W * H);
    let observableCount = 0;
    for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
        const o = occ(i, j);
        const seenable = !o || !occ(i - 1, j) || !occ(i + 1, j) || !occ(i, j - 1) || !occ(i, j + 1);
        if (seenable) { observable[idx(i, j)] = 1; observableCount++; }
    }

    const wx = (i) => i - W / 2;                // world x of a cell's left edge
    const wz = (j) => j - H / 2;
    const cellOf = (x, z) => [Math.floor(x + W / 2), Math.floor(z + H / 2)];

    // ---------- Planner (A* on the inflated grid) ----------
    const gScore = new Float32Array(W * H);
    const cameFrom = new Int32Array(W * H);
    const closed = new Uint8Array(W * H);
    const NB = [[1, 0, 1], [-1, 0, 1], [0, 1, 1], [0, -1, 1], [1, 1, Math.SQRT2], [1, -1, Math.SQRT2], [-1, 1, Math.SQRT2], [-1, -1, Math.SQRT2]];

    function astar(si, sj, gi, gj) {
        gScore.fill(Infinity); closed.fill(0); cameFrom.fill(-1);
        const heap = [];
        const push = (f, n) => {
            heap.push([f, n]);
            let c = heap.length - 1;
            while (c > 0) {
                const p = (c - 1) >> 1;
                if (heap[p][0] <= heap[c][0]) break;
                [heap[p], heap[c]] = [heap[c], heap[p]]; c = p;
            }
        };
        const pop = () => {
            const top = heap[0], last = heap.pop();
            if (heap.length) {
                heap[0] = last;
                let p = 0;
                for (;;) {
                    let l = p * 2 + 1, r = l + 1, m = p;
                    if (l < heap.length && heap[l][0] < heap[m][0]) m = l;
                    if (r < heap.length && heap[r][0] < heap[m][0]) m = r;
                    if (m === p) break;
                    [heap[p], heap[m]] = [heap[m], heap[p]]; p = m;
                }
            }
            return top;
        };
        const h = (i, j) => {
            const dx = Math.abs(i - gi), dy = Math.abs(j - gj);
            return Math.max(dx, dy) + (Math.SQRT2 - 1) * Math.min(dx, dy);
        };
        const s = idx(si, sj), g = idx(gi, gj);
        gScore[s] = 0; push(h(si, sj), s);
        while (heap.length) {
            const n = pop()[1];
            if (n === g) break;
            if (closed[n]) continue;
            closed[n] = 1;
            const ci = n % W, cj = (n / W) | 0;
            for (let k = 0; k < 8; k++) {
                const ni = ci + NB[k][0], nj = cj + NB[k][1];
                if (!inside(ni, nj) || shut(idx(ni, nj))) continue;
                if (k > 3 && (shut(idx(ci + NB[k][0], cj)) || shut(idx(ci, cj + NB[k][1])))) continue;
                const m = idx(ni, nj), t = gScore[n] + NB[k][2];
                if (t < gScore[m]) { gScore[m] = t; cameFrom[m] = n; push(t + h(ni, nj), m); }
            }
        }
        if (cameFrom[g] === -1 && g !== s) return null;
        const cells = [];
        for (let n = g; n !== -1; n = cameFrom[n]) cells.push([wx(n % W) + 0.5, wz((n / W) | 0) + 0.5]);
        return cells.reverse();
    }

    function clearLine(a, b) {
        const dx = b[0] - a[0], dz = b[1] - a[1];
        const n = Math.ceil(Math.hypot(dx, dz) / 0.25);
        for (let k = 0; k <= n; k++) {
            const [i, j] = cellOf(a[0] + dx * k / n, a[1] + dz * k / n);
            if (!inside(i, j) || shut(idx(i, j))) return false;
        }
        return true;
    }

    const STEP = 0.25;
    // Dynamic costmap layer: loose props the LiDAR has detected, inflated by the robot's footprint.
    const dyn = new Uint8Array(W * H);
    let avoiding = true;
    const shut = (n) => blocked[n] || (avoiding && dyn[n]);

    // Routes go around detected props. Only `escape` (the robot is boxed in) may plan straight through them.
    function plan(from, goalCell, escape) {
        avoiding = true;
        let p = route(from, goalCell);
        if (!p && escape) {
            avoiding = false;
            p = route(from, goalCell);
            avoiding = true;
            if (p) p.escape = true;
        }
        return p;
    }

    // straight line between two points without crossing a wall
    function inSight(ax, az, bx, bz) {
        const n = Math.ceil(Math.hypot(bx - ax, bz - az) / 0.3);
        for (let k = 1; k < n; k++) {
            const [i, j] = cellOf(ax + (bx - ax) * k / n, az + (bz - az) * k / n);
            if (occ(i, j)) return false;
        }
        return true;
    }

    function route(from, goalCell) {
        let [si, sj] = cellOf(from[0], from[1]);
        if (shut(idx(si, sj))) { const n = nearestFree(si, sj, from); if (!n) return null; [si, sj] = n; }
        const cells = astar(si, sj, goalCell[0], goalCell[1]);
        if (!cells) return null;
        cells[0] = [from[0], from[1]];
        // string pulling
        const pts = [cells[0]];
        let a = 0;
        while (a < cells.length - 1) {
            let b = cells.length - 1;
            while (b > a + 1 && !clearLine(cells[a], cells[b])) b--;
            pts.push(cells[b]); a = b;
        }
        // round the corners
        const smooth = [pts[0]];
        for (let k = 1; k < pts.length - 1; k++) {
            const p0 = pts[k - 1], p1 = pts[k], p2 = pts[k + 1];
            const l0 = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]), l2 = Math.hypot(p2[0] - p1[0], p2[1] - p1[1]);
            const r0 = Math.min(1.4, l0 / 2), r2 = Math.min(1.4, l2 / 2);
            const A = [p1[0] + (p0[0] - p1[0]) * r0 / l0, p1[1] + (p0[1] - p1[1]) * r0 / l0];
            const B = [p1[0] + (p2[0] - p1[0]) * r2 / l2, p1[1] + (p2[1] - p1[1]) * r2 / l2];
            for (let t = 0; t <= 1.0001; t += 0.125) {
                const u = 1 - t;
                smooth.push([u * u * A[0] + 2 * u * t * p1[0] + t * t * B[0], u * u * A[1] + 2 * u * t * p1[1] + t * t * B[1]]);
            }
        }
        smooth.push(pts[pts.length - 1]);
        // resample at a fixed arc-length step
        const out = [smooth[0]];
        let carry = 0;
        for (let k = 1; k < smooth.length; k++) {
            const p = smooth[k - 1], q = smooth[k];
            const len = Math.hypot(q[0] - p[0], q[1] - p[1]);
            let d = STEP - carry;
            while (d <= len) { out.push([p[0] + (q[0] - p[0]) * d / len, p[1] + (q[1] - p[1]) * d / len]); d += STEP; }
            carry = len - (d - STEP);
        }
        out.push(smooth[smooth.length - 1]);
        return out.length > 1 ? out : null;
    }

    // nearest plannable cell; with `from`, only cells the robot can walk straight to (never one behind a wall)
    function nearestFree(i, j, from) {
        for (let r = 0; r <= 8; r++) {
            let best = null, bd = Infinity;
            for (let dj = -r; dj <= r; dj++) for (let di = -r; di <= r; di++) {
                if (Math.max(Math.abs(di), Math.abs(dj)) !== r) continue;
                const ni = i + di, nj = j + dj;
                if (!inside(ni, nj) || shut(idx(ni, nj))) continue;
                if (from && !inSight(from[0], from[1], wx(ni) + 0.5, wz(nj) + 0.5)) continue;
                const d = di * di + dj * dj;
                if (d < bd) { bd = d; best = [ni, nj]; }
            }
            if (best) return best;
        }
        return null;
    }

    // ---------- Renderer ----------
    const dpr = Math.min(window.devicePixelRatio || 1, coarse ? 1.5 : 2);
    const renderer = new Renderer({ canvas, dpr, alpha: false, antialias: true, powerPreference: 'high-performance' });
    const gl = renderer.gl;
    if (!gl) throw new Error('webgl unavailable');

    const camera = new Camera(gl, { fov: 38, near: 1, far: 400 });
    const scene = new Transform();
    const FOV_T = Math.tan((38 * Math.PI / 180) / 2);

    const hex = (s) => {
        const n = parseInt(s.slice(1), 16);
        return [((n >> 16) & 255) / 255, ((n >> 8) & 255) / 255, (n & 255) / 255];
    };
    const THEMES = {
        dark: {
            bg: '#0b0d10', low: '#2c62f2', high: '#74ecdf', hot: '#f3fbff', floor: '#33507f', grid: '#8aa0c0',
            beam: '#9fe8ff', accent: '#ffb224', crate: '#b98d5b', cone: '#ff7a1a', drum: '#4f6f96', additive: true,
        },
        light: {
            bg: '#eceef1', low: '#16233d', high: '#0d6672', hot: '#a85200', floor: '#8793a3', grid: '#5d6672',
            beam: '#0d6672', accent: '#a85200', crate: '#b08453', cone: '#f06a0c', drum: '#48658a', additive: false,
        },
    };

    // ---------- Seen map (occupancy grid the robot has observed) ----------
    const seenTarget = new Uint8Array(W * H);
    const seen = new Float32Array(W * H);
    const seenPixels = new Uint8Array(W * H * 4);
    const seenTex = new Texture(gl, {
        image: seenPixels, width: W, height: H, generateMipmaps: false, flipY: false,
        minFilter: gl.NEAREST, magFilter: gl.NEAREST,
    });
    let seenCount = 0;

    // ---------- Point cloud ----------
    const pos = [], cell = [], seed = [], kind = [];
    let rs = 1234567;
    const rnd = () => { rs = (rs * 1664525 + 1013904223) >>> 0; return rs / 4294967296; };
    const density = coarse ? 0.6 : 1;
    const addPoint = (x, y, z, i, j, k) => {
        pos.push(x, y, z); cell.push((i + 0.5) / W, (j + 0.5) / H); seed.push(rnd()); kind.push(k);
    };
    for (let j = 0; j < H; j++) for (let i = 0; i < W; i++) {
        const h = height[idx(i, j)];
        if (!h) {
            const n = rnd() < 0.55 * density ? 2 : 1;
            for (let k = 0; k < n; k++) addPoint(wx(i) + rnd(), 0, wz(j) + rnd(), i, j, 0);
            continue;
        }
        const faces = [[-1, 0], [1, 0], [0, -1], [0, 1]];
        for (const [di, dj] of faces) {
            if (occ(i + di, j + dj)) continue;
            // scan rings up the face, like stacked LiDAR channels
            for (let y = 0.2; y <= h; y += 0.3) {
                const n = Math.round(4 * density);
                for (let k = 0; k < n; k++) {
                    const u = rnd(), jy = y + (rnd() - 0.5) * 0.06;
                    if (di) addPoint(wx(i) + (di > 0 ? 1 : 0), jy, wz(j) + u, i, j, 1);
                    else addPoint(wx(i) + u, jy, wz(j) + (dj > 0 ? 1 : 0), i, j, 1);
                }
            }
        }
        if (h < 4.6 && observable[idx(i, j)] || h < 3.2) {
            const n = Math.round(2 * density);
            for (let k = 0; k < n; k++) addPoint(wx(i) + rnd(), h, wz(j) + rnd(), i, j, 2);
        }
    }
    const pointCount = seed.length;

    const pointsProgram = new Program(gl, {
        vertex: `
            attribute vec3 position;
            attribute vec2 aCell;
            attribute float aSeed;
            attribute float aKind;
            uniform mat4 modelViewMatrix;
            uniform mat4 projectionMatrix;
            uniform sampler2D tSeen;
            uniform vec3 uRobot;
            uniform float uSweep, uTime, uScale, uRange, uMaxH, uFade, uLive;
            uniform vec3 uLow, uHigh, uHot, uFloor;
            varying vec3 vColor;
            varying float vAlpha;
            void main() {
                float seen = texture2D(tSeen, aCell).r;
                float grow = seen * seen * (3.0 - 2.0 * seen);
                vec3 p = position;
                if (aKind > 0.5) p.y *= mix(0.12, 1.0, grow);
                vec2 d = p.xz - uRobot.xz;
                float dist = length(d);
                float behind = mod(uSweep - atan(d.y, d.x), 6.2831853);
                float trail = uLive * exp(-behind * 4.0) * smoothstep(uRange, uRange * 0.3, dist);
                vec3 col = aKind < 0.5 ? uFloor : mix(uLow, uHigh, clamp(position.y / uMaxH, 0.0, 1.0));
                float shimmer = 0.8 + 0.2 * sin(uTime * 1.6 + aSeed * 43.0);
                float base = aKind < 0.5 ? 0.42 : 0.9;
                vAlpha = uFade * seen * (base * shimmer + trail * 0.45);
                vColor = mix(col, uHot, min(trail * 0.6, 1.0));
                vec4 mv = modelViewMatrix * vec4(p, 1.0);
                gl_Position = projectionMatrix * mv;
                gl_PointSize = uScale * (aKind < 0.5 ? 0.8 : 1.0) * (1.0 + trail * 0.45) / -mv.z;
            }`,
        fragment: `
            precision highp float;
            varying vec3 vColor;
            varying float vAlpha;
            void main() {
                vec2 c = gl_PointCoord - 0.5;
                float r = dot(c, c);
                if (r > 0.25) discard;
                gl_FragColor = vec4(vColor, vAlpha * smoothstep(0.25, 0.03, r));
            }`,
        uniforms: {
            tSeen: { value: seenTex }, uRobot: { value: [0, 0, 0] }, uSweep: { value: 0 }, uTime: { value: 0 },
            uScale: { value: 1 }, uRange: { value: RANGE }, uMaxH: { value: MAX_H }, uFade: { value: 1 }, uLive: { value: 1 },
            uLow: { value: [0, 0, 0] }, uHigh: { value: [0, 0, 0] }, uHot: { value: [0, 0, 0] }, uFloor: { value: [0, 0, 0] },
        },
        transparent: true, depthWrite: false,
    });
    const points = new Mesh(gl, {
        mode: gl.POINTS,
        geometry: new Geometry(gl, {
            position: { size: 3, data: new Float32Array(pos) },
            aCell: { size: 2, data: new Float32Array(cell) },
            aSeed: { size: 1, data: new Float32Array(seed) },
            aKind: { size: 1, data: new Float32Array(kind) },
        }),
        program: pointsProgram, frustumCulled: false,
    });
    points.setParent(scene);

    // ---------- Shared flat / line programs ----------
    const FLAT_VS = `
        attribute vec3 position;
        attribute vec3 normal;
        uniform mat4 modelViewMatrix;
        uniform mat4 projectionMatrix;
        uniform mat4 modelMatrix;
        varying float vLight;
        void main() {
            vec3 n = normalize(mat3(modelMatrix) * normal);
            vLight = 0.5 + 0.5 * max(dot(n, normalize(vec3(0.35, 0.85, 0.4))), 0.0);
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }`;
    const FLAT_FS = `
        precision highp float;
        uniform vec3 uColor, uBackdrop;
        uniform float uOpacity, uReveal;
        varying float vLight;
        void main() { gl_FragColor = vec4(mix(uBackdrop, uColor * vLight, uReveal), uOpacity); }`;
    const LINE_VS = `
        attribute vec3 position;
        attribute float aAlpha;
        uniform mat4 modelViewMatrix;
        uniform mat4 projectionMatrix;
        varying float vAlpha;
        void main() { vAlpha = aAlpha; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`;
    const LINE_FS = `
        precision highp float;
        uniform vec3 uColor;
        uniform float uOpacity;
        varying float vAlpha;
        void main() { gl_FragColor = vec4(uColor, vAlpha * uOpacity); }`;
    const flatProgram = (opts = {}) => new Program(gl, {
        vertex: FLAT_VS, fragment: FLAT_FS, cullFace: false,
        uniforms: { uColor: { value: [1, 1, 1] }, uOpacity: { value: 1 }, uBackdrop: { value: [0, 0, 0] }, uReveal: { value: 1 } }, ...opts,
    });
    const lineProgram = () => new Program(gl, {
        vertex: LINE_VS, fragment: LINE_FS, transparent: true, depthWrite: false,
        uniforms: { uColor: { value: [1, 1, 1] }, uOpacity: { value: 1 } },
    });

    // ---------- Floor grid ----------
    const gridProgram = lineProgram();
    {
        const g = [], a = [];
        for (let i = 0; i <= W; i += 4) { g.push(wx(i), 0, wz(0), wx(i), 0, wz(H)); a.push(1, 1); }
        for (let j = 0; j <= H; j += 4) { g.push(wx(0), 0, wz(j), wx(W), 0, wz(j)); a.push(1, 1); }
        const grid = new Mesh(gl, {
            mode: gl.LINES, program: gridProgram, frustumCulled: false,
            geometry: new Geometry(gl, { position: { size: 3, data: new Float32Array(g) }, aAlpha: { size: 1, data: new Float32Array(a) } }),
        });
        grid.setParent(scene);
    }

    // ---------- LiDAR beams ----------
    const BEAMS = 30, FAN = 0.7;
    const beamPos = new Float32Array(BEAMS * 6), beamAlpha = new Float32Array(BEAMS * 2);
    const beamGeo = new Geometry(gl, {
        position: { size: 3, data: beamPos, usage: gl.DYNAMIC_DRAW },
        aAlpha: { size: 1, data: beamAlpha, usage: gl.DYNAMIC_DRAW },
    });
    const beamProgram = lineProgram();
    const beams = new Mesh(gl, { mode: gl.LINES, geometry: beamGeo, program: beamProgram, frustumCulled: false });
    beams.setParent(scene);

    // ---------- Planned path ----------
    const MAXP = 900;
    const pathPos = new Float32Array(MAXP * 6), pathDist = new Float32Array(MAXP * 2);
    const pathGeo = new Geometry(gl, {
        position: { size: 3, data: pathPos, usage: gl.DYNAMIC_DRAW },
        aDist: { size: 1, data: pathDist, usage: gl.DYNAMIC_DRAW },
    });
    pathGeo.setDrawRange(0, 0);
    const pathProgram = new Program(gl, {
        vertex: `
            attribute vec3 position;
            attribute float aDist;
            uniform mat4 modelViewMatrix;
            uniform mat4 projectionMatrix;
            varying float vDist;
            void main() { vDist = aDist; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
        fragment: `
            precision highp float;
            uniform vec3 uColor;
            uniform float uTravel, uTime, uOpacity, uLength;
            varying float vDist;
            void main() {
                if (vDist < uTravel) discard;
                float dash = step(0.45, fract(vDist * 0.8 - uTime * 1.4));
                float tip = smoothstep(uLength, uLength - 1.5, vDist);
                gl_FragColor = vec4(uColor, uOpacity * tip * (0.22 + 0.78 * dash));
            }`,
        uniforms: {
            uColor: { value: [1, 1, 1] }, uTravel: { value: 0 }, uTime: { value: 0 }, uOpacity: { value: 0 }, uLength: { value: 1 },
        },
        transparent: true, depthWrite: false, depthTest: false, cullFace: false,
    });
    const pathMesh = new Mesh(gl, { mode: gl.TRIANGLE_STRIP, geometry: pathGeo, program: pathProgram, frustumCulled: false });
    pathMesh.setParent(scene);

    // ---------- Rings (goal, ping, pointer reticle) and the robot's light pool ----------
    function ringGeometry(inner, outer) {
        const p = [], n = [];
        for (let k = 0; k <= 64; k++) {
            const a = k / 64 * Math.PI * 2, c = Math.cos(a), s = Math.sin(a);
            p.push(c * inner, 0, s * inner, c * outer, 0, s * outer); n.push(0, 1, 0, 0, 1, 0);
        }
        return new Geometry(gl, { position: { size: 3, data: new Float32Array(p) }, normal: { size: 3, data: new Float32Array(n) } });
    }
    const overlay = { transparent: true, depthWrite: false, depthTest: false };
    const goalProgram = flatProgram(overlay), pingProgram = flatProgram(overlay), reticleProgram = flatProgram(overlay);
    const goal = new Mesh(gl, { mode: gl.TRIANGLE_STRIP, geometry: ringGeometry(0.62, 0.86), program: goalProgram, frustumCulled: false });
    const ping = new Mesh(gl, { mode: gl.TRIANGLE_STRIP, geometry: ringGeometry(0.9, 1.0), program: pingProgram, frustumCulled: false });
    const reticle = new Mesh(gl, { mode: gl.TRIANGLE_STRIP, geometry: ringGeometry(0.42, 0.54), program: reticleProgram, frustumCulled: false });
    for (const m of [goal, ping, reticle]) { m.position.y = 0.05; m.setParent(scene); m.program.uniforms.uOpacity.value = 0; }

    const poolProgram = new Program(gl, {
        vertex: `
            attribute vec3 position;
            uniform mat4 modelViewMatrix;
            uniform mat4 projectionMatrix;
            varying vec2 vP;
            void main() { vP = position.xz; gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
        fragment: `
            precision highp float;
            uniform vec3 uColor;
            uniform float uOpacity;
            varying vec2 vP;
            void main() {
                float a = smoothstep(1.0, 0.0, length(vP) / 3.2);
                gl_FragColor = vec4(uColor, uOpacity * a * a);
            }`,
        uniforms: { uColor: { value: [1, 1, 1] }, uOpacity: { value: 0.3 } },
        transparent: true, depthWrite: false, depthTest: false, cullFace: false,
    });
    const pool = new Mesh(gl, {
        program: poolProgram, frustumCulled: false,
        geometry: new Geometry(gl, { position: { size: 3, data: new Float32Array([-3.2, 0, -3.2, 3.2, 0, -3.2, -3.2, 0, 3.2, 3.2, 0, -3.2, 3.2, 0, 3.2, -3.2, 0, 3.2]) } }),
    });
    pool.position.y = 0.02;
    pool.setParent(scene);

    // ---------- Robot, physics and loose props ----------
    const robot = createRobot(gl, scene, CELL_M);
    const physics = createPhysics({ rects, cols: W, rows: H, cell: CELL_M, robot: KIN });

    function boxGeometry(sx, sy, sz) {
        const p = [], n = [], ix = [];
        for (const f of [[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]]) {
            const u = f[0] ? [0, 1, 0] : [1, 0, 0];
            const v = [f[1] * u[2] - f[2] * u[1], f[2] * u[0] - f[0] * u[2], f[0] * u[1] - f[1] * u[0]];
            const b = p.length / 3;
            for (const [a, d] of [[-1, -1], [1, -1], [1, 1], [-1, 1]]) {
                p.push((f[0] + a * u[0] + d * v[0]) * sx / 2, (f[1] + a * u[1] + d * v[1]) * sy / 2, (f[2] + a * u[2] + d * v[2]) * sz / 2);
                n.push(...f);
            }
            ix.push(b, b + 1, b + 2, b, b + 2, b + 3);
        }
        return new Geometry(gl, { position: { size: 3, data: new Float32Array(p) }, normal: { size: 3, data: new Float32Array(n) }, index: { data: new Uint16Array(ix) } });
    }
    function cylinderGeometry(rTop, rBottom, h, seg = 20) {
        const p = [], n = [], ix = [];
        const slope = (rBottom - rTop) / h, nl = Math.hypot(1, slope);
        for (let k = 0; k <= seg; k++) {
            const a = k / seg * Math.PI * 2, c = Math.cos(a), s = Math.sin(a);
            p.push(c * rTop, h / 2, s * rTop, c * rBottom, -h / 2, s * rBottom);
            n.push(c / nl, slope / nl, s / nl, c / nl, slope / nl, s / nl);
            if (k < seg) ix.push(k * 2, k * 2 + 1, k * 2 + 2, k * 2 + 1, k * 2 + 3, k * 2 + 2);
        }
        for (const [r, y] of [[rTop, h / 2], [rBottom, -h / 2]]) {
            const centre = p.length / 3;
            p.push(0, y, 0); n.push(0, Math.sign(y), 0);
            for (let k = 0; k <= seg; k++) {
                const a = k / seg * Math.PI * 2;
                p.push(Math.cos(a) * r, y, Math.sin(a) * r); n.push(0, Math.sign(y), 0);
                if (k < seg) ix.push(centre, centre + 1 + k, centre + 2 + k);
            }
        }
        return new Geometry(gl, { position: { size: 3, data: new Float32Array(p) }, normal: { size: 3, data: new Float32Array(n) }, index: { data: new Uint16Array(ix) } });
    }
    const propPrograms = { crate: flatProgram(), cone: flatProgram(), drum: flatProgram() };
    const propMeshes = physics.props.map(({ kind, size }) => {
        const mesh = new Mesh(gl, {
            geometry: kind === 'crate' ? boxGeometry(...size) : cylinderGeometry(...size),
            program: propPrograms[kind], frustumCulled: false,
        });
        mesh.scale.set(1 / CELL_M);
        mesh.setParent(scene);
        return mesh;
    });
    // Where the props are right now, as the LiDAR meets them (propAt, propTop), and which ones it has detected.
    const PROP_RADIUS = { crate: 0.22, cone: 0.15, drum: 0.2 };          // metres
    const NOSE = 0.33, HALF_WIDTH = 0.19;                                  // robot outline from its centre, metres
    const propAt = new Int16Array(W * H), propTop = new Float32Array(W * H);
    const detected = new Uint8Array(physics.props.length);
    function stamp(x, z, r, mark) {
        for (let j = Math.floor(z - r + H / 2); j <= Math.floor(z + r + H / 2); j++) {
            for (let i = Math.floor(x - r + W / 2); i <= Math.floor(x + r + W / 2); i++) {
                if (inside(i, j) && Math.hypot(i + 0.5 - W / 2 - x, j + 0.5 - H / 2 - z) <= r + 0.5) mark(idx(i, j));
            }
        }
    }
    function locateProps() {
        propAt.fill(0);
        physics.props.forEach(({ kind, body, size }, i) => {
            const top = (body.position.y + size[size.length - 1] / 2) / CELL_M;
            stamp(body.position.x / CELL_M, body.position.z / CELL_M, PROP_RADIUS[kind] / CELL_M - 0.5, (n) => { propAt[n] = i + 1; propTop[n] = top; });
        });
    }
    // A loose object right in front of the nose (the bumper zone); anything this close also counts as detected.
    function propAhead() {
        let hit = false;
        physics.props.forEach(({ kind, body }, i) => {
            const dx = body.position.x - bot.x * CELL_M, dz = body.position.z - bot.z * CELL_M;
            const gap = Math.hypot(dx, dz) - PROP_RADIUS[kind];
            if (gap > 1.2) return;
            detected[i] = 1;
            // in the robot's frame: just ahead of the nose and within its own width
            const c = Math.cos(bot.yaw), sn = Math.sin(bot.yaw);
            const ahead = dx * c + dz * sn, side = Math.abs(dz * c - dx * sn);
            if (ahead > 0 && ahead < NOSE + PROP_RADIUS[kind] + 0.12 && side < HALF_WIDTH + PROP_RADIUS[kind] + 0.02) hit = true;
        });
        return hit;
    }
    function updateCostmap() {
        dyn.fill(0);
        physics.props.forEach(({ kind, body }, i) => {
            if (detected[i]) stamp(body.position.x / CELL_M, body.position.z / CELL_M, PROP_RADIUS[kind] / CELL_M + 0.8, (n) => { dyn[n] = 1; });
        });
    }

    // props belong to the map too: each one fades in as the LiDAR first sees the floor it stands on
    const propSeen = (body) => {
        const [ci, cj] = cellOf(body.position.x / CELL_M, body.position.z / CELL_M);
        return inside(ci, cj) ? seen[idx(ci, cj)] : 0;
    };
    propMeshes.forEach((mesh, i) => {
        const { kind, body } = physics.props[i];
        mesh.onBeforeRender(() => { propPrograms[kind].uniforms.uReveal.value = propSeen(body); });
    });
    function syncProps() {
        physics.props.forEach(({ body }, i) => {
            const m = propMeshes[i], q = body.quaternion;
            m.position.set(body.position.x / CELL_M, body.position.y / CELL_M, body.position.z / CELL_M);
            m.quaternion.set(q.x, q.y, q.z, q.w);
            m.visible = propSeen(body) > 0.01;
        });
    }

    // ---------- Theme ----------
    function applyTheme() {
        const t = THEMES[root.dataset.theme === 'light' ? 'light' : 'dark'];
        const bg = hex(t.bg);
        gl.clearColor(bg[0], bg[1], bg[2], 1);
        const u = pointsProgram.uniforms;
        u.uLow.value = hex(t.low); u.uHigh.value = hex(t.high); u.uHot.value = hex(t.hot); u.uFloor.value = hex(t.floor);
        const blend = (p) => p.setBlendFunc(gl.SRC_ALPHA, t.additive ? gl.ONE : gl.ONE_MINUS_SRC_ALPHA);
        [pointsProgram, beamProgram, gridProgram, pathProgram, goalProgram, pingProgram, reticleProgram, poolProgram].forEach(blend);
        gridProgram.uniforms.uColor.value = hex(t.grid); gridProgram.uniforms.uOpacity.value = t.additive ? 0.07 : 0.14;
        beamProgram.uniforms.uColor.value = hex(t.beam);
        beamProgram.uniforms.uOpacity.value = t.additive ? 0.5 : 0.55;
        for (const p of [pathProgram, goalProgram, pingProgram, reticleProgram, poolProgram]) p.uniforms.uColor.value = hex(t.accent);
        poolProgram.uniforms.uOpacity.value = t.additive ? 0.28 : 0.2;
        robot.setTheme(t.additive);
        for (const kind of ['crate', 'cone', 'drum']) {
            propPrograms[kind].uniforms.uColor.value = hex(t[kind]);
            propPrograms[kind].uniforms.uBackdrop.value = bg;
        }
        dirty = true;
    }

    // ---------- Simulation state ----------
    const HUD = {};
    hero.querySelectorAll('[data-hud]').forEach((el) => { HUD[el.dataset.hud] = el; });
    const hint = hero.querySelector('.hero-hint');

    // the robot's pose in cells, read back from the physics body every step
    const bot = { x: wx(31) + 0.5, z: wz(24) + 0.5, yaw: 0.4, speed: 0 };
    physics.place(bot.x * CELL_M, bot.z * CELL_M, bot.yaw);
    let path = null, pathIndex = 0, travel = 0, stall = 0, pathLen = 0, pathAlpha = 0, recheck = 0;
    let backing = 0, retries = 0, lost = 0;         // recovery: back away from an obstacle, bounded retries, time without any route
    const drive = { keys: new Set(), stick: null };     // manual control: W A S D, or the on-screen stick
    let mode = 'exploring', modeShown = '', idle = 0, userGoal = false;
    let sweep = 0, time = 0, dirty = true, pingT = 1, goalAlpha = 0;
    const pointer = { x: 0, z: 0, on: false, alpha: 0, nx: 0, ny: 0 };

    function setPath(p, byUser, rerouted) {
        if (!rerouted) retries = 0;
        lost = 0;
        path = p; pathIndex = 0; travel = 0; stall = 0; pathLen = (p.length - 1) * STEP; userGoal = byUser;
        const n = Math.min(p.length, MAXP);
        for (let k = 0; k < n; k++) {
            const a = p[Math.max(k - 1, 0)], b = p[Math.min(k + 1, n - 1)];
            let tx = b[0] - a[0], tz = b[1] - a[1];
            const l = Math.hypot(tx, tz) || 1; tx /= l; tz /= l;
            const hw = 0.13;
            pathPos.set([p[k][0] - tz * hw, 0.04, p[k][1] + tx * hw, p[k][0] + tz * hw, 0.04, p[k][1] - tx * hw], k * 6);
            pathDist[k * 2] = pathDist[k * 2 + 1] = k * STEP;
        }
        pathGeo.attributes.position.needsUpdate = true;
        pathGeo.attributes.aDist.needsUpdate = true;
        pathGeo.setDrawRange(0, n * 2);
        pathProgram.uniforms.uLength.value = pathLen;
        const end = p[p.length - 1];
        goal.position.x = ping.position.x = end[0]; goal.position.z = ping.position.z = end[1];
        if (byUser && !rerouted) pingT = 0;
        mode = byUser ? 'navigating' : (seenCount / observableCount > 0.97 ? 'patrolling' : 'exploring');
    }

    function autoGoal() {
        updateCostmap();
        let best = null, bestScore = -Infinity;
        for (let k = 0; k < 36; k++) {
            const c = freeCells[(Math.random() * freeCells.length) | 0];
            const dx = wx(c[0]) + 0.5 - bot.x, dz = wz(c[1]) + 0.5 - bot.z;
            const dist = Math.hypot(dx, dz);
            if (dist < 8 || dyn[idx(c[0], c[1])]) continue;
            let unknown = 0;
            for (let dj = -3; dj <= 3; dj += 3) for (let di = -3; di <= 3; di += 3) {
                const i = c[0] + di, j = c[1] + dj;
                if (inside(i, j) && observable[idx(i, j)] && !seenTarget[idx(i, j)]) unknown++;
            }
            const score = unknown * 3 - dist * 0.04 + Math.random();
            if (score > bestScore) { bestScore = score; best = c; }
        }
        if (!best) return;
        // boxed in for a while: allow one route straight through the clutter to get out
        const p = plan([bot.x, bot.z], best, lost > 6);
        if (p) setPath(p, false);
    }

    function castRay(angle, mark) {
        const c = Math.cos(angle), s = Math.sin(angle);
        for (let t = 0.3; t <= RANGE; t += 0.35) {
            const x = bot.x + c * t, z = bot.z + s * t;
            const i = Math.floor(x + W / 2), j = Math.floor(z + H / 2);
            if (!inside(i, j)) return [t, 0];
            const n = idx(i, j);
            if (mark && !seenTarget[n]) { seenTarget[n] = 1; if (observable[n]) seenCount++; }
            if (height[n] > 0) return [t, height[n]];
            if (propAt[n]) { if (mark) detected[propAt[n] - 1] = 1; return [t, propTop[n]]; }
        }
        return [RANGE, -1];
    }

    function revealAll() {
        seenTarget.fill(1); seen.fill(1); seenCount = observableCount;
        for (let k = 0; k < W * H; k++) seenPixels[k * 4] = 255;
        seenTex.needsUpdate = true;
    }

    // ---------- Camera ----------
    const cam = { tx: 0, tz: 0, az: -0.36, shiftX: 0, shiftY: 0, dist: 46, px: 0, py: 0, follow: 0.62 };
    const view = { pos: [0, 0, 0], right: [1, 0, 0], up: [0, 1, 0], fwd: [0, 0, -1], aspect: 1 };

    let sizedW = 0, sizedH = 0;
    function layout() {
        const w = hero.clientWidth, h = hero.clientHeight;
        if (w === sizedW && h === sizedH) return false;     // resizing clears the canvas, so only when it really changed
        sizedW = w; sizedH = h;
        renderer.setSize(w, h);
        view.aspect = w / h;
        camera.perspective({ aspect: view.aspect });
        const wide = view.aspect > 1.05;
        cam.shiftX = wide ? 0.2 : 0;
        cam.shiftY = wide ? 0.04 : 0.34;
        cam.dist = wide ? Math.max(42, 62 / Math.max(view.aspect, 1.2) + 14) : 58;
        // a narrow view keeps the robot centred; a wide one lets it roam the frame
        cam.follow = wide ? 0.62 : 1;
        // lens shift keeps the robot clear of the headline
        camera.projectionMatrix[8] = -cam.shiftX;
        camera.projectionMatrix[9] = -cam.shiftY;
        pointsProgram.uniforms.uScale.value = 0.17 * h * dpr / (2 * FOV_T);
        dirty = true;
        return true;
    }

    function updateCamera(dt, scroll) {
        const k = 1 - Math.exp(-dt * 1.6);
        cam.tx += (bot.x * cam.follow - cam.tx) * k;
        cam.tz += (bot.z * cam.follow - cam.tz) * k;
        cam.px += (pointer.nx - cam.px) * (1 - Math.exp(-dt * 3));
        cam.py += (pointer.ny - cam.py) * (1 - Math.exp(-dt * 3));
        const az = cam.az + Math.sin(time * 0.07) * 0.1 + cam.px * 0.07;
        const el = (0.86 + scroll * 0.42) - cam.py * 0.03;
        const dist = cam.dist * (1 + scroll * 0.3);
        const ce = Math.cos(el), se = Math.sin(el);
        const px = cam.tx + Math.sin(az) * ce * dist, py = se * dist, pz = cam.tz + Math.cos(az) * ce * dist;
        camera.position.set(px, py, pz);
        camera.lookAt([cam.tx, 0, cam.tz]);
        const f = [cam.tx - px, -py, cam.tz - pz];
        const fl = Math.hypot(f[0], f[1], f[2]); f[0] /= fl; f[1] /= fl; f[2] /= fl;
        const r = [-f[2], 0, f[0]];
        const rl = Math.hypot(r[0], r[2]); r[0] /= rl; r[2] /= rl;
        view.pos = [px, py, pz]; view.fwd = f; view.right = r;
        view.up = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]];
    }

    function floorPoint(clientX, clientY) {
        const b = canvas.getBoundingClientRect();
        const nx = ((clientX - b.left) / b.width) * 2 - 1, ny = 1 - ((clientY - b.top) / b.height) * 2;
        const dx = (nx - cam.shiftX) * FOV_T * view.aspect, dy = (ny - cam.shiftY) * FOV_T;
        const d = [
            view.right[0] * dx + view.up[0] * dy + view.fwd[0],
            view.right[1] * dx + view.up[1] * dy + view.fwd[1],
            view.right[2] * dx + view.up[2] * dy + view.fwd[2],
        ];
        if (d[1] >= -1e-4) return null;
        const t = -view.pos[1] / d[1];
        return [view.pos[0] + d[0] * t, view.pos[2] + d[2] * t];
    }

    // ---------- Robot state and manual control ----------
    const bodyState = { x: 0, y: 0, z: 0, quat: [0, 0, 0, 1], yaw: 0, vx: 0, vz: 0, wy: 0, sweep: 0 };
    function readBody() {
        const b = physics.chassis, q = b.quaternion;
        bot.x = b.position.x / CELL_M; bot.z = b.position.z / CELL_M; bot.yaw = physics.yaw();
        bot.speed = Math.hypot(b.velocity.x, b.velocity.z);
        bodyState.x = b.position.x; bodyState.y = b.position.y; bodyState.z = b.position.z;
        bodyState.quat[0] = q.x; bodyState.quat[1] = q.y; bodyState.quat[2] = q.z; bodyState.quat[3] = q.w;
        bodyState.yaw = bot.yaw; bodyState.vx = b.velocity.x; bodyState.vz = b.velocity.z; bodyState.wy = b.angularVelocity.y;
        bodyState.sweep = sweep;
    }
    function manualInput() {
        let v = 0, w = 0;
        if (drive.keys.has('KeyW')) v += 1;
        if (drive.keys.has('KeyS')) v -= 0.6;
        if (drive.keys.has('KeyA')) w += 1;
        if (drive.keys.has('KeyD')) w -= 1;
        if (drive.stick) { v += drive.stick.v; w += drive.stick.w; }
        return v || w ? { v, w } : null;
    }

    // ---------- Frame ----------
    function step(dt) {
        time += dt;
        sweep = (sweep + dt * 5.4) % (Math.PI * 2);

        // command the robot: manual control wins, otherwise it follows the planned path
        const cmd = physics.command, manual = manualInput();
        locateProps();
        const bumper = propAhead();
        const reroute = () => {
            const end = path[path.length - 1];
            updateCostmap();
            const next = plan([bot.x, bot.z], cellOf(end[0], end[1]));
            if (next) setPath(next, userGoal, true);
            else { path = null; idle = 0; if (userGoal) mode = 'no path'; }
        };
        if (path && !manual && backing <= 0 && !path.escape) {
            // costmap update: if a prop the LiDAR has picked up sits on the route ahead, plan around it
            recheck += dt;
            if (recheck > 0.25) {
                recheck = 0;
                updateCostmap();
                let obstructed = false;
                for (let k = pathIndex + 4; k < path.length && !obstructed; k += 2) {
                    const [ci, cj] = cellOf(path[k][0], path[k][1]);
                    obstructed = dyn[idx(ci, cj)] === 1;
                }
                if (obstructed) reroute();
            }
        }
        if (manual) {
            path = null; userGoal = true; idle = 0; backing = 0; mode = 'manual';
            cmd.v = manual.v * 1.15; cmd.w = manual.w * 2.4;
        } else if (path && backing > 0) {
            // recovery: back straight out, then plan again from where there is room
            backing -= dt;
            cmd.v = -0.45; cmd.w = 0;
            if (backing <= 0) reroute();
        } else if (path) {
            // pure pursuit: steer at a point a little way along the path
            const n = path.length, far = (k) => Math.hypot(path[k][0] - bot.x, path[k][1] - bot.z);
            const before = pathIndex;
            while (pathIndex < n - 1 && far(pathIndex + 1) <= far(pathIndex)) pathIndex++;
            travel = pathIndex * STEP;
            const target = path[Math.min(pathIndex + 9, n - 1)], remaining = far(n - 1);
            let err = Math.atan2(target[1] - bot.z, target[0] - bot.x) - bot.yaw;
            err = Math.atan2(Math.sin(err), Math.cos(err));
            cmd.w = Math.max(-2.6, Math.min(2.6, -4 * err));
            cmd.v = 1.05 * Math.max(0.12, 1 - Math.abs(err) / 1.1) * Math.min(1, 0.3 + remaining / 3);
            stall = pathIndex > before || bot.speed > 0.25 ? 0 : stall + dt;
            if (remaining < 0.7 || pathIndex >= n - 1) {
                if (userGoal) mode = 'goal reached';
                path = null; idle = 0; cmd.v = 0; cmd.w = 0;
            } else if ((bumper && !path.escape) || stall > 1.5) {
                // never shove an obstacle along: stop, and back off to replan (a few times at most)
                cmd.v = 0; cmd.w = 0; stall = 0;
                if (++retries > 3) { path = null; idle = 0; if (userGoal) mode = 'blocked'; }
                else backing = 0.9;
            }
        } else {
            cmd.v = 0; cmd.w = 0;
            idle += dt; lost += dt;
            if (idle > (userGoal ? 2.4 : 0.5)) autoGoal();
        }
        physics.step(dt);
        readBody();
        robot.update(dt, bodyState);
        syncProps();

        // spin the LiDAR: the fan of beams trails the sweep angle
        const [ox, oy, oz] = physics.worldPoint(KIN.lidar[0], KIN.lidar[1] + 0.015, KIN.lidar[2]);
        for (let k = 0; k < BEAMS; k++) {
            const u = k / (BEAMS - 1);
            const angle = sweep - u * FAN;
            const [t, h] = castRay(angle, true);
            const o = k * 6;
            const hitY = h > 0 ? Math.min(h, 0.4 + ((k * 7) % 5) * 0.55) : 0;
            beamPos[o] = ox / CELL_M; beamPos[o + 1] = oy / CELL_M; beamPos[o + 2] = oz / CELL_M;
            beamPos[o + 3] = bot.x + Math.cos(angle) * t; beamPos[o + 4] = hitY; beamPos[o + 5] = bot.z + Math.sin(angle) * t;
            const a = (1 - u) * (1 - u) * (h < 0 ? 0.35 : 1);
            beamAlpha[k * 2] = a * 0.9; beamAlpha[k * 2 + 1] = a * 0.25;
        }
        beamGeo.attributes.position.needsUpdate = true;
        beamGeo.attributes.aAlpha.needsUpdate = true;

        // newly observed cells fade in
        const rate = 1 - Math.exp(-dt * 4.5);
        for (let k = 0; k < W * H; k++) {
            const d = seenTarget[k] - seen[k];
            if (d > 0.002) { seen[k] += d * rate; seenPixels[k * 4] = (seen[k] * 255) | 0; }
            else if (d > 0) { seen[k] = 1; seenPixels[k * 4] = 255; }
        }
        seenTex.needsUpdate = true;

        // markers
        pathAlpha += ((path ? 1 : 0) - pathAlpha) * (1 - Math.exp(-dt * 6));
        goalAlpha += ((path && userGoal ? 1 : 0) - goalAlpha) * (1 - Math.exp(-dt * 6));
        pingT = Math.min(1, pingT + dt * 1.5);
        pointer.alpha += ((pointer.on ? 1 : 0) - pointer.alpha) * (1 - Math.exp(-dt * 10));
    }

    function draw(scroll, dt) {
        updateCamera(dt, scroll);
        const fade = 1 - scroll * 0.75;

        pool.position.x = bot.x; pool.position.z = bot.z;

        const u = pointsProgram.uniforms;
        u.uRobot.value = [bot.x, 0, bot.z]; u.uSweep.value = sweep; u.uTime.value = time; u.uFade.value = fade;
        beamProgram.uniforms.uOpacity.value = (root.dataset.theme === 'light' ? 0.55 : 0.5) * fade * (reduced ? 0 : 1);

        pathProgram.uniforms.uTravel.value = travel + 0.6;
        pathProgram.uniforms.uTime.value = time;
        pathProgram.uniforms.uOpacity.value = pathAlpha * (userGoal ? 0.95 : 0.5) * fade;

        const pulse = 1 + Math.sin(time * 4) * 0.06;
        goal.scale.set(pulse, 1, pulse);
        goalProgram.uniforms.uOpacity.value = goalAlpha * 0.95 * fade;
        const e = 1 - Math.pow(1 - pingT, 3);
        ping.scale.set(0.6 + e * 3.4, 1, 0.6 + e * 3.4);
        pingProgram.uniforms.uOpacity.value = (1 - pingT) * 0.8 * fade;
        reticle.position.x += (pointer.x - reticle.position.x) * 0.35;
        reticle.position.z += (pointer.z - reticle.position.z) * 0.35;
        reticleProgram.uniforms.uOpacity.value = pointer.alpha * 0.8 * fade;

        renderer.render({ scene, camera });
    }

    let hudClock = 0;
    function updateHud(dt) {
        hudClock += dt;
        if (mode !== modeShown) {
            modeShown = mode;
            const el = HUD.state;
            if (el) {
                if (window.gsap && window.ScrambleTextPlugin && !reduced) {
                    window.gsap.to(el, { duration: 0.5, scrambleText: { text: mode, chars: 'lowerCase', speed: 0.6 }, overwrite: true });
                } else el.textContent = mode;
            }
        }
        if (hudClock < 0.1) return;
        hudClock = 0;
        if (HUD.x) HUD.x.textContent = (bot.x * CELL_M).toFixed(2);
        if (HUD.y) HUD.y.textContent = (-bot.z * CELL_M).toFixed(2);
        if (HUD.yaw) HUD.yaw.textContent = String(Math.round(((-bot.yaw * 180 / Math.PI) % 360 + 540) % 360 - 180));
        if (HUD.map) HUD.map.textContent = String(Math.min(100, Math.round(seenCount / observableCount * 100)));
    }

    // ---------- Loop ----------
    let visible = true, raf = 0, last = 0;
    const scrollProgress = () => Math.min(1, Math.max(0, window.scrollY / Math.max(1, hero.offsetHeight)));

    function frame(now) {
        raf = 0;
        const dt = Math.min(0.05, (now - last) / 1000 || 0.016);
        last = now;
        step(dt);
        draw(scrollProgress(), dt);
        updateHud(dt);
        schedule();
    }
    function schedule() {
        if (!raf && visible && !document.hidden && !reduced) raf = requestAnimationFrame(frame);
    }
    function renderStill() {
        // reduced motion: one finished frame, redrawn only when something changes
        if (!dirty) return;
        dirty = false;
        readBody();
        robot.update(0, bodyState);
        syncProps();
        draw(scrollProgress(), 10);
        updateHud(1);
    }

    // ---------- Input ----------
    canvas.addEventListener('click', (e) => {
        if (reduced) return;
        const p = floorPoint(e.clientX, e.clientY);
        if (!p) return;
        let [i, j] = cellOf(p[0], p[1]);
        if (!inside(i, j)) return;
        updateCostmap();
        if (shut(idx(i, j))) { const n = nearestFree(i, j); if (!n) return; [i, j] = n; }
        const found = plan([bot.x, bot.z], [i, j]);
        if (hint) hint.classList.add('is-used');
        if (found) { setPath(found, true); return; }
        // nothing gets there without pushing things over: say so and stay put
        path = null; userGoal = true; idle = 0; mode = 'no path';
        ping.position.x = wx(i) + 0.5; ping.position.z = wz(j) + 0.5; pingT = 0;
    });
    canvas.addEventListener('pointermove', (e) => {
        const b = canvas.getBoundingClientRect();
        pointer.nx = ((e.clientX - b.left) / b.width) * 2 - 1;
        pointer.ny = 1 - ((e.clientY - b.top) / b.height) * 2;
        if (e.pointerType !== 'mouse' || reduced) return;
        const p = floorPoint(e.clientX, e.clientY);
        if (!p) { pointer.on = false; return; }
        const [i, j] = cellOf(p[0], p[1]);
        pointer.on = inside(i, j);
        pointer.x = p[0]; pointer.z = p[1];
    });
    canvas.addEventListener('pointerleave', () => { pointer.on = false; pointer.nx = 0; pointer.ny = 0; });

    // manual driving: W/S forward and back, A/D turn (by key position, so any keyboard layout works)
    const DRIVE_KEYS = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD']);
    const typing = (el) => el && (el.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(el.tagName));
    if (!reduced) {
        window.addEventListener('keydown', (e) => {
            if (!visible || e.metaKey || e.ctrlKey || e.altKey || typing(e.target) || !DRIVE_KEYS.has(e.code)) return;
            drive.keys.add(e.code);
            if (hint) hint.classList.add('is-used');
        });
        window.addEventListener('keyup', (e) => drive.keys.delete(e.code));
        window.addEventListener('blur', () => drive.keys.clear());

        const stick = hero.querySelector('.stick'), knob = stick && stick.querySelector('.stick-knob');
        if (stick && knob) {
            const move = (e) => {
                const b = stick.getBoundingClientRect(), r = b.width / 2;
                let dx = (e.clientX - b.left - r) / r, dy = (e.clientY - b.top - r) / r;
                const m = Math.hypot(dx, dy);
                if (m > 1) { dx /= m; dy /= m; }
                knob.style.transform = `translate(${dx * (r - 15)}px, ${dy * (r - 15)}px)`;
                const dead = (c) => (Math.abs(c) < 0.18 ? 0 : c);
                drive.stick = { v: dead(-dy) * (dy > 0 ? 0.6 : 1), w: dead(-dx) };
            };
            const release = () => { drive.stick = null; knob.style.transform = ''; stick.classList.remove('is-active'); };
            stick.addEventListener('pointerdown', (e) => {
                stick.setPointerCapture(e.pointerId);
                stick.classList.add('is-active');
                move(e);
                if (hint) hint.classList.add('is-used');
            });
            stick.addEventListener('pointermove', (e) => { if (drive.stick) move(e); });
            // a held thumb must not start a text selection, a callout or a context menu
            stick.addEventListener('touchstart', (e) => e.preventDefault(), { passive: false });
            stick.addEventListener('contextmenu', (e) => e.preventDefault());
            stick.addEventListener('pointerup', release);
            stick.addEventListener('pointercancel', release);
        }
    }

    canvas.addEventListener('webglcontextlost', (e) => {
        e.preventDefault();
        visible = false;
        hero.classList.add('no-webgl');
    });

    new IntersectionObserver((entries) => {
        visible = entries[0].isIntersecting;
        if (visible) { last = performance.now(); schedule(); }
    }, { rootMargin: '80px' }).observe(hero);
    document.addEventListener('visibilitychange', () => { last = performance.now(); schedule(); });
    new ResizeObserver(() => {
        if (!layout()) return;
        // redraw in the same frame, so a resize never shows an empty canvas
        if (reduced) renderStill(); else draw(scrollProgress(), 0);
    }).observe(hero);
    new MutationObserver(() => { applyTheme(); if (reduced) renderStill(); }).observe(root, { attributes: true, attributeFilter: ['data-theme'] });

    // ---------- Go ----------
    layout();
    applyTheme();
    cam.tx = bot.x * cam.follow; cam.tz = bot.z * cam.follow;
    if (reduced) {
        revealAll();
        pointsProgram.uniforms.uLive.value = 0;
        mode = 'idle';
        renderStill();
        window.addEventListener('scroll', () => { dirty = true; renderStill(); }, { passive: true });
    } else {
        last = performance.now();
        schedule();
    }
    // show the scene once the robot is in it, without holding it back on a slow connection
    const reveal = () => hero.classList.add('has-scene');
    robot.load(new URL('../assets/quadruped.bin', import.meta.url))
        .catch(() => robot.useFallback())
        .finally(() => { dirty = true; if (reduced) renderStill(); reveal(); });
    setTimeout(reveal, 2500);
    hero.dataset.points = String(pointCount);
}
