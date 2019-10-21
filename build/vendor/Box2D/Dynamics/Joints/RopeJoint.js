/*
 * Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
import { Clamp, Min, Rot, Vec2 } from '../../Common/Math';
import { linearSlop, maxLinearCorrection, Maybe } from '../../Common/Settings';
import { Joint, JointDef, JointType, LimitState } from './Joint';
/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in JointDef.
export class RopeJointDef extends JointDef {
    constructor() {
        super(JointType.e_ropeJoint);
        this.localAnchorA = new Vec2(-1, 0);
        this.localAnchorB = new Vec2(1, 0);
        this.maxLength = 0;
    }
}
export class RopeJoint extends Joint {
    constructor(def) {
        super(def);
        // Solver shared
        this.m_localAnchorA = new Vec2();
        this.m_localAnchorB = new Vec2();
        this.m_maxLength = 0;
        this.m_length = 0;
        this.m_impulse = 0;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_u = new Vec2();
        this.m_rA = new Vec2();
        this.m_rB = new Vec2();
        this.m_localCenterA = new Vec2();
        this.m_localCenterB = new Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_mass = 0;
        this.m_state = LimitState.e_inactiveLimit;
        this.m_qA = new Rot();
        this.m_qB = new Rot();
        this.m_lalcA = new Vec2();
        this.m_lalcB = new Vec2();
        this.m_localAnchorA.Copy(Maybe(def.localAnchorA, new Vec2(-1, 0)));
        this.m_localAnchorB.Copy(Maybe(def.localAnchorB, new Vec2(1, 0)));
        this.m_maxLength = Maybe(def.maxLength, 0);
    }
    InitVelocityConstraints(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_invMassA = this.m_bodyA.m_invMass;
        this.m_invMassB = this.m_bodyB.m_invMass;
        this.m_invIA = this.m_bodyA.m_invI;
        this.m_invIB = this.m_bodyB.m_invI;
        const cA = data.positions[this.m_indexA].c;
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const cB = data.positions[this.m_indexB].c;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const qA = this.m_qA.SetAngle(aA);
        const qB = this.m_qB.SetAngle(aB);
        // this.m_rA = Mul(qA, this.m_localAnchorA - this.m_localCenterA);
        Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // this.m_rB = Mul(qB, this.m_localAnchorB - this.m_localCenterB);
        Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // this.m_u = cB + this.m_rB - cA - this.m_rA;
        this.m_u
            .Copy(cB)
            .SelfAdd(this.m_rB)
            .SelfSub(cA)
            .SelfSub(this.m_rA);
        this.m_length = this.m_u.Length();
        const C = this.m_length - this.m_maxLength;
        this.m_state =
            C > 0 ? LimitState.e_atUpperLimit : LimitState.e_inactiveLimit;
        if (this.m_length > linearSlop) {
            this.m_u.SelfMul(1 / this.m_length);
        }
        else {
            this.m_u.SetZero();
            this.m_mass = 0;
            this.m_impulse = 0;
            return;
        }
        // Compute effective mass.
        const crA = Vec2.CrossVV(this.m_rA, this.m_u);
        const crB = Vec2.CrossVV(this.m_rB, this.m_u);
        const invMass = this.m_invMassA +
            this.m_invIA * crA * crA +
            this.m_invMassB +
            this.m_invIB * crB * crB;
        this.m_mass = invMass !== 0 ? 1 / invMass : 0;
        if (data.step.warmStarting) {
            // Scale the impulse to support a variable time step.
            this.m_impulse *= data.step.dtRatio;
            // Vec2 P = m_impulse * m_u;
            const P = Vec2.MulSV(this.m_impulse, this.m_u, RopeJoint.InitVelocityConstraints_s_P);
            // vA -= m_invMassA * P;
            vA.SelfMulSub(this.m_invMassA, P);
            wA -= this.m_invIA * Vec2.CrossVV(this.m_rA, P);
            // vB += m_invMassB * P;
            vB.SelfMulAdd(this.m_invMassB, P);
            wB += this.m_invIB * Vec2.CrossVV(this.m_rB, P);
        }
        else {
            this.m_impulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolveVelocityConstraints(data) {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        // Cdot = dot(u, v + cross(w, r))
        // Vec2 vpA = vA + Cross(wA, m_rA);
        const vpA = Vec2.AddVCrossSV(vA, wA, this.m_rA, RopeJoint.SolveVelocityConstraints_s_vpA);
        // Vec2 vpB = vB + Cross(wB, m_rB);
        const vpB = Vec2.AddVCrossSV(vB, wB, this.m_rB, RopeJoint.SolveVelocityConstraints_s_vpB);
        // float32 C = m_length - m_maxLength;
        const C = this.m_length - this.m_maxLength;
        // float32 Cdot = Dot(m_u, vpB - vpA);
        let Cdot = Vec2.DotVV(this.m_u, Vec2.SubVV(vpB, vpA, Vec2.s_t0));
        // Predictive constraint.
        if (C < 0) {
            Cdot += data.step.inv_dt * C;
        }
        let impulse = -this.m_mass * Cdot;
        const oldImpulse = this.m_impulse;
        this.m_impulse = Min(0, this.m_impulse + impulse);
        impulse = this.m_impulse - oldImpulse;
        // Vec2 P = impulse * m_u;
        const P = Vec2.MulSV(impulse, this.m_u, RopeJoint.SolveVelocityConstraints_s_P);
        // vA -= m_invMassA * P;
        vA.SelfMulSub(this.m_invMassA, P);
        wA -= this.m_invIA * Vec2.CrossVV(this.m_rA, P);
        // vB += m_invMassB * P;
        vB.SelfMulAdd(this.m_invMassB, P);
        wB += this.m_invIB * Vec2.CrossVV(this.m_rB, P);
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
    }
    SolvePositionConstraints(data) {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;
        const qA = this.m_qA.SetAngle(aA);
        const qB = this.m_qB.SetAngle(aB);
        // Vec2 rA = Mul(qA, this.m_localAnchorA - this.m_localCenterA);
        Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // Vec2 rB = Mul(qB, this.m_localAnchorB - this.m_localCenterB);
        Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // Vec2 u = cB + rB - cA - rA;
        const u = this.m_u
            .Copy(cB)
            .SelfAdd(rB)
            .SelfSub(cA)
            .SelfSub(rA);
        const length = u.Normalize();
        let C = length - this.m_maxLength;
        C = Clamp(C, 0, maxLinearCorrection);
        const impulse = -this.m_mass * C;
        // Vec2 P = impulse * u;
        const P = Vec2.MulSV(impulse, u, RopeJoint.SolvePositionConstraints_s_P);
        // cA -= m_invMassA * P;
        cA.SelfMulSub(this.m_invMassA, P);
        aA -= this.m_invIA * Vec2.CrossVV(rA, P);
        // cB += m_invMassB * P;
        cB.SelfMulAdd(this.m_invMassB, P);
        aB += this.m_invIB * Vec2.CrossVV(rB, P);
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return length - this.m_maxLength < linearSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // return out.Set(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
        return Vec2.MulSV(inv_dt * this.m_impulse, this.m_u, out);
    }
    GetReactionTorque(inv_dt) {
        return 0;
    }
    GetLocalAnchorA() {
        return this.m_localAnchorA;
    }
    GetLocalAnchorB() {
        return this.m_localAnchorB;
    }
    SetMaxLength(length) {
        this.m_maxLength = length;
    }
    GetMaxLength() {
        return this.m_maxLength;
    }
    GetLimitState() {
        return this.m_state;
    }
    Dump(log) {
        const indexA = this.m_bodyA.m_islandIndex;
        const indexB = this.m_bodyB.m_islandIndex;
        log('  const jd: RopeJointDef = new RopeJointDef();\n');
        log('  jd.bodyA = bodies[%d];\n', indexA);
        log('  jd.bodyB = bodies[%d];\n', indexB);
        log('  jd.collideConnected = %s;\n', this.m_collideConnected ? 'true' : 'false');
        log('  jd.localAnchorA.Set(%.15f, %.15f);\n', this.m_localAnchorA.x, this.m_localAnchorA.y);
        log('  jd.localAnchorB.Set(%.15f, %.15f);\n', this.m_localAnchorB.x, this.m_localAnchorB.y);
        log('  jd.maxLength = %.15f;\n', this.m_maxLength);
        log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index);
    }
}
RopeJoint.InitVelocityConstraints_s_P = new Vec2();
RopeJoint.SolveVelocityConstraints_s_vpA = new Vec2();
RopeJoint.SolveVelocityConstraints_s_vpB = new Vec2();
RopeJoint.SolveVelocityConstraints_s_P = new Vec2();
RopeJoint.SolvePositionConstraints_s_P = new Vec2();
//# sourceMappingURL=RopeJoint.js.map