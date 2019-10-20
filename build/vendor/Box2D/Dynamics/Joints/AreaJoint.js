// DEBUG: import { Assert } from "../../Common/Settings";
import { Sq, Sqrt, Vec2 } from '../../Common/Math';
import { epsilon, linearSlop, MakeNumberArray, maxLinearCorrection, Maybe } from '../../Common/Settings';
import { DistanceJointDef } from './DistanceJoint';
import { Joint, JointDef, JointType } from './Joint';
export class AreaJointDef extends JointDef {
    constructor() {
        super(JointType.e_areaJoint);
        this.bodies = [];
        this.frequencyHz = 0;
        this.dampingRatio = 0;
    }
    AddBody(body) {
        this.bodies.push(body);
        if (this.bodies.length === 1) {
            this.bodyA = body;
        }
        else if (this.bodies.length === 2) {
            this.bodyB = body;
        }
    }
}
export class AreaJoint extends Joint {
    constructor(def) {
        super(def);
        this.m_frequencyHz = 0;
        this.m_dampingRatio = 0;
        // Solver shared
        this.m_impulse = 0;
        this.m_targetArea = 0;
        // DEBUG: Assert(def.bodies.length >= 3, "You cannot create an area joint with less than three bodies.");
        this.m_bodies = def.bodies;
        this.m_frequencyHz = Maybe(def.frequencyHz, 0);
        this.m_dampingRatio = Maybe(def.dampingRatio, 0);
        this.m_targetLengths = MakeNumberArray(def.bodies.length);
        this.m_normals = Vec2.MakeArray(def.bodies.length);
        this.m_joints = []; // MakeNullArray(def.bodies.length);
        this.m_deltas = Vec2.MakeArray(def.bodies.length);
        this.m_delta = new Vec2();
        const djd = new DistanceJointDef();
        djd.frequencyHz = this.m_frequencyHz;
        djd.dampingRatio = this.m_dampingRatio;
        this.m_targetArea = 0;
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const next = this.m_bodies[(i + 1) % this.m_bodies.length];
            const body_c = body.GetWorldCenter();
            const next_c = next.GetWorldCenter();
            this.m_targetLengths[i] = Vec2.DistanceVV(body_c, next_c);
            this.m_targetArea += Vec2.CrossVV(body_c, next_c);
            djd.Initialize(body, next, body_c, next_c);
            this.m_joints[i] = body.GetWorld().CreateJoint(djd);
        }
        this.m_targetArea *= 0.5;
    }
    GetAnchorA(out) {
        return out;
    }
    GetAnchorB(out) {
        return out;
    }
    GetReactionForce(inv_dt, out) {
        return out;
    }
    GetReactionTorque(inv_dt) {
        return 0;
    }
    SetFrequency(hz) {
        this.m_frequencyHz = hz;
        for (const joint of this.m_joints) {
            joint.SetFrequency(hz);
        }
    }
    GetFrequency() {
        return this.m_frequencyHz;
    }
    SetDampingRatio(ratio) {
        this.m_dampingRatio = ratio;
        for (const joint of this.m_joints) {
            joint.SetDampingRatio(ratio);
        }
    }
    GetDampingRatio() {
        return this.m_dampingRatio;
    }
    Dump(log) {
        log('Area joint dumping is not supported.\n');
    }
    InitVelocityConstraints(data) {
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const prev = this.m_bodies[(i + this.m_bodies.length - 1) % this.m_bodies.length];
            const next = this.m_bodies[(i + 1) % this.m_bodies.length];
            const prev_c = data.positions[prev.m_islandIndex].c;
            const next_c = data.positions[next.m_islandIndex].c;
            const delta = this.m_deltas[i];
            Vec2.SubVV(next_c, prev_c, delta);
        }
        if (data.step.warmStarting) {
            this.m_impulse *= data.step.dtRatio;
            for (let i = 0; i < this.m_bodies.length; ++i) {
                const body = this.m_bodies[i];
                const body_v = data.velocities[body.m_islandIndex].v;
                const delta = this.m_deltas[i];
                body_v.x += body.m_invMass * delta.y * 0.5 * this.m_impulse;
                body_v.y += body.m_invMass * -delta.x * 0.5 * this.m_impulse;
            }
        }
        else {
            this.m_impulse = 0;
        }
    }
    SolveVelocityConstraints(data) {
        let dotMassSum = 0;
        let crossMassSum = 0;
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const body_v = data.velocities[body.m_islandIndex].v;
            const delta = this.m_deltas[i];
            dotMassSum += delta.LengthSquared() / body.GetMass();
            crossMassSum += Vec2.CrossVV(body_v, delta);
        }
        const lambda = (-2 * crossMassSum) / dotMassSum;
        // lambda = Clamp(lambda, -maxLinearCorrection, maxLinearCorrection);
        this.m_impulse += lambda;
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const body_v = data.velocities[body.m_islandIndex].v;
            const delta = this.m_deltas[i];
            body_v.x += body.m_invMass * delta.y * 0.5 * lambda;
            body_v.y += body.m_invMass * -delta.x * 0.5 * lambda;
        }
    }
    SolvePositionConstraints(data) {
        let perimeter = 0;
        let area = 0;
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const next = this.m_bodies[(i + 1) % this.m_bodies.length];
            const body_c = data.positions[body.m_islandIndex].c;
            const next_c = data.positions[next.m_islandIndex].c;
            const delta = Vec2.SubVV(next_c, body_c, this.m_delta);
            let dist = delta.Length();
            if (dist < epsilon) {
                dist = 1;
            }
            this.m_normals[i].x = delta.y / dist;
            this.m_normals[i].y = -delta.x / dist;
            perimeter += dist;
            area += Vec2.CrossVV(body_c, next_c);
        }
        area *= 0.5;
        const deltaArea = this.m_targetArea - area;
        const toExtrude = (0.5 * deltaArea) / perimeter;
        let done = true;
        for (let i = 0; i < this.m_bodies.length; ++i) {
            const body = this.m_bodies[i];
            const body_c = data.positions[body.m_islandIndex].c;
            const next_i = (i + 1) % this.m_bodies.length;
            const delta = Vec2.AddVV(this.m_normals[i], this.m_normals[next_i], this.m_delta);
            delta.SelfMul(toExtrude);
            const norm_sq = delta.LengthSquared();
            if (norm_sq > Sq(maxLinearCorrection)) {
                delta.SelfMul(maxLinearCorrection / Sqrt(norm_sq));
            }
            if (norm_sq > Sq(linearSlop)) {
                done = false;
            }
            body_c.x += delta.x;
            body_c.y += delta.y;
        }
        return done;
    }
}
//# sourceMappingURL=AreaJoint.js.map