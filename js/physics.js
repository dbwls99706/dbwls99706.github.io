// Rigid-body world for the hero scene (cannon-es), in metres.
// The robot's trunk is a dynamic body: it rides on four leg springs, is pushed by a traction-limited
// velocity controller, and collides with the building and with loose props that it can shove over.
import { World, Body, Box, Cylinder, Plane, Vec3, Material, ContactMaterial } from '../vendor/cannon-es.min.js';

const STEP = 1 / 60;
const GRAVITY = 9.81;

export function createPhysics({ rects, cols, rows, cell, robot }) {
    const world = new World({ gravity: new Vec3(0, -GRAVITY, 0) });
    world.allowSleep = true;
    world.solver.iterations = 12;

    const floorMat = new Material('floor'), bodyMat = new Material('robot'), propMat = new Material('prop');
    world.addContactMaterial(new ContactMaterial(floorMat, propMat, { friction: 0.55, restitution: 0.05 }));
    world.addContactMaterial(new ContactMaterial(propMat, propMat, { friction: 0.5, restitution: 0.05 }));
    // nearly frictionless against the robot, so a prop that ends up under it slides out instead of carrying it
    world.addContactMaterial(new ContactMaterial(bodyMat, propMat, { friction: 0.02, restitution: 0.1 }));
    world.addContactMaterial(new ContactMaterial(bodyMat, floorMat, { friction: 0.05, restitution: 0.15 }));

    // ----- building -----
    const ground = new Body({ mass: 0, material: floorMat, shape: new Plane() });
    ground.quaternion.setFromEuler(-Math.PI / 2, 0, 0);
    world.addBody(ground);
    for (const [x, y, w, h, hgt] of rects) {
        const wall = new Body({ mass: 0, material: floorMat, shape: new Box(new Vec3(w * cell / 2, hgt * cell / 2, h * cell / 2)) });
        wall.position.set((x + w / 2 - cols / 2) * cell, hgt * cell / 2, (y + h / 2 - rows / 2) * cell);
        world.addBody(wall);
    }

    // ----- robot trunk -----
    const { hipX, hipY, thighY, stand, footRadius } = robot;
    const rest = stand + footRadius;                      // trunk height when standing
    const MASS = 15;
    const chassis = new Body({ mass: MASS, material: bodyMat, linearDamping: 0.02, angularDamping: 0.25, allowSleep: false });
    // deeper than the real trunk, so a toppled crate is shoved aside instead of sliding under the belly
    chassis.addShape(new Box(new Vec3(0.3, 0.12, 0.085)), new Vec3(0.03, -0.04, 0));
    // the legs, as far as collisions go: four posts that stop just short of the floor
    const legHalf = (rest - 0.05) / 2;
    for (const sx of [1, -1]) for (const sz of [1, -1]) {
        chassis.addShape(new Box(new Vec3(0.04, legHalf, 0.035)), new Vec3(sx * hipX, -rest + 0.05 + legHalf, sz * (hipY + thighY)));
    }
    world.addBody(chassis);

    const corners = [];
    for (const sx of [1, -1]) for (const sz of [1, -1]) corners.push(new Vec3(sx * hipX, 0, sz * (hipY + thighY)));
    const SPRING = 3200, DAMPER = 175, SAG = MASS * GRAVITY / 4 / SPRING;

    const tmp = new Vec3(), point = new Vec3(), vel = new Vec3(), force = new Vec3(), up = new Vec3(), fwd = new Vec3();
    const X = new Vec3(1, 0, 0), Y = new Vec3(0, 1, 0);
    const command = { v: 0, w: 0 };
    let grounded = 4;

    function control() {
        const q = chassis.quaternion;
        q.vmult(Y, up); q.vmult(X, fwd);

        // leg springs hold the trunk up and level; each acts only while its foot can reach the floor
        grounded = 0;
        for (const c of corners) {
            q.vmult(c, tmp);
            point.copy(chassis.position).vadd(tmp, point);
            const compression = rest + SAG - point.y;
            if (compression < -0.07) continue;
            grounded++;
            chassis.getVelocityAtWorldPoint(point, vel);
            const f = Math.min(Math.max(SPRING * compression - DAMPER * vel.y, 0), MASS * GRAVITY * 2.5);
            force.set(0, f, 0);
            chassis.applyForce(force, tmp);
        }

        // balance: torque that brings the trunk's up axis back to vertical
        const w = chassis.angularVelocity;
        chassis.torque.x += -120 * up.z - 8 * w.x;
        chassis.torque.z += 120 * up.x - 8 * w.z;

        // no traction in free fall; perched on top of something it can still scramble off
        const perched = grounded < 2 && Math.abs(chassis.velocity.y) < 0.25;
        if (grounded < 2 && !perched) return;
        // traction: track the commanded planar velocity, limited by what the feet can push
        const fl = Math.hypot(fwd.x, fwd.z) || 1;
        const ex = fwd.x / fl * command.v - chassis.velocity.x, ez = fwd.z / fl * command.v - chassis.velocity.z;
        let fx = MASS * ex / 0.12, fz = MASS * ez / 0.12;
        const limit = 0.9 * MASS * GRAVITY, mag = Math.hypot(fx, fz);
        if (mag > limit) { fx *= limit / mag; fz *= limit / mag; }
        force.set(fx, 0, fz);
        tmp.set(0, -0.03, 0);                              // pushed from just below the centre of mass, so it pitches a little as it accelerates
        chassis.applyForce(force, tmp);
        chassis.torque.y += Math.max(-14, Math.min(14, 0.45 * (command.w - w.y) / 0.1));
    }

    // ----- loose props -----
    const props = [];
    function addProp(kind, shape, mass, x, y, z, size) {
        const body = new Body({ mass, material: propMat, shape, linearDamping: 0.08, angularDamping: 0.12, sleepSpeedLimit: 0.12, sleepTimeLimit: 0.6 });
        body.position.set((x - cols / 2) * cell, y, (z - rows / 2) * cell);
        world.addBody(body);
        props.push({ kind, body, size });
    }
    const BOX = 0.3;
    const crate = (x, y, z) => addProp('crate', new Box(new Vec3(BOX / 2, BOX / 2, BOX / 2)), 1.1, x, BOX / 2 + y * BOX, z, [BOX, BOX, BOX]);
    const cone = (x, z) => addProp('cone', new Cylinder(0.03, 0.13, 0.36, 10), 0.6, x, 0.18, z, [0.03, 0.13, 0.36]);
    const drum = (x, z) => addProp('drum', new Cylinder(0.15, 0.15, 0.44, 12), 2.6, x, 0.22, z, [0.15, 0.15, 0.44]);

    const d = BOX / cell;                                  // crate size in cells
    for (const [cx, cz] of [[38.5, 17.5], [24, 24.5]]) {  // two small pyramids
        crate(cx, 0, cz - d * 0.52); crate(cx, 0, cz + d * 0.52); crate(cx, 1, cz);
    }
    crate(47, 0, 27); crate(47, 1, 27); crate(47, 2, 27);  // a tower
    for (let k = 0; k < 4; k++) cone(44 + k * 3.2, 15.5);
    drum(56.5, 28); drum(45, 34.5); drum(30.5, 31.5);

    // ----- stepping -----
    let acc = 0;
    function step(dt) {
        acc = Math.min(acc + dt, STEP * 3);
        while (acc >= STEP) {
            control();
            world.step(STEP);
            acc -= STEP;
        }
    }

    function place(x, z, yaw) {
        chassis.position.set(x, rest, z);
        chassis.quaternion.setFromEuler(0, -yaw, 0);
        chassis.velocity.setZero(); chassis.angularVelocity.setZero();
    }

    const local = new Vec3(), out = new Vec3();
    function worldPoint(x, y, z) {
        local.set(x, y, z);
        chassis.pointToWorldFrame(local, out);
        return [out.x, out.y, out.z];
    }

    return {
        chassis, props, command, step, place, worldPoint, rest,
        get grounded() { return grounded; },
        yaw() { chassis.quaternion.vmult(X, fwd); return Math.atan2(fwd.z, fwd.x); },
    };
}
