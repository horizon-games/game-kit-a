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
// DEBUG: import { Assert } from "../../Common/Settings";
// DEBUG: import { IsValid } from "../../Common/Math";
import { Rot, Vec2 } from '../../Common/Math';
import { linearSlop, Maybe } from '../../Common/Settings';
import { Joint, JointDef, JointType } from './Joint';
/// Gear joint definition. This definition requires two existing
/// revolute or prismatic joints (any combination will work).
export class GearJointDef extends JointDef {
    constructor() {
        super(JointType.e_gearJoint);
        this.ratio = 1;
    }
}
export class GearJoint extends Joint {
    constructor(def) {
        super(def);
        this.m_typeA = JointType.e_unknownJoint;
        this.m_typeB = JointType.e_unknownJoint;
        // Solver shared
        this.m_localAnchorA = new Vec2();
        this.m_localAnchorB = new Vec2();
        this.m_localAnchorC = new Vec2();
        this.m_localAnchorD = new Vec2();
        this.m_localAxisC = new Vec2();
        this.m_localAxisD = new Vec2();
        this.m_referenceAngleA = 0;
        this.m_referenceAngleB = 0;
        this.m_constant = 0;
        this.m_ratio = 0;
        this.m_impulse = 0;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_indexC = 0;
        this.m_indexD = 0;
        this.m_lcA = new Vec2();
        this.m_lcB = new Vec2();
        this.m_lcC = new Vec2();
        this.m_lcD = new Vec2();
        this.m_mA = 0;
        this.m_mB = 0;
        this.m_mC = 0;
        this.m_mD = 0;
        this.m_iA = 0;
        this.m_iB = 0;
        this.m_iC = 0;
        this.m_iD = 0;
        this.m_JvAC = new Vec2();
        this.m_JvBD = new Vec2();
        this.m_JwA = 0;
        this.m_JwB = 0;
        this.m_JwC = 0;
        this.m_JwD = 0;
        this.m_mass = 0;
        this.m_qA = new Rot();
        this.m_qB = new Rot();
        this.m_qC = new Rot();
        this.m_qD = new Rot();
        this.m_lalcA = new Vec2();
        this.m_lalcB = new Vec2();
        this.m_lalcC = new Vec2();
        this.m_lalcD = new Vec2();
        this.m_joint1 = def.joint1;
        this.m_joint2 = def.joint2;
        this.m_typeA = this.m_joint1.GetType();
        this.m_typeB = this.m_joint2.GetType();
        // DEBUG: Assert(this.m_typeA === JointType.e_revoluteJoint || this.m_typeA === JointType.e_prismaticJoint);
        // DEBUG: Assert(this.m_typeB === JointType.e_revoluteJoint || this.m_typeB === JointType.e_prismaticJoint);
        let coordinateA;
        let coordinateB;
        // TODO_ERIN there might be some problem with the joint edges in Joint.
        this.m_bodyC = this.m_joint1.GetBodyA();
        this.m_bodyA = this.m_joint1.GetBodyB();
        // Get geometry of joint1
        const xfA = this.m_bodyA.m_xf;
        const aA = this.m_bodyA.m_sweep.a;
        const xfC = this.m_bodyC.m_xf;
        const aC = this.m_bodyC.m_sweep.a;
        if (this.m_typeA === JointType.e_revoluteJoint) {
            const revolute = def.joint1;
            this.m_localAnchorC.Copy(revolute.m_localAnchorA);
            this.m_localAnchorA.Copy(revolute.m_localAnchorB);
            this.m_referenceAngleA = revolute.m_referenceAngle;
            this.m_localAxisC.SetZero();
            coordinateA = aA - aC - this.m_referenceAngleA;
        }
        else {
            const prismatic = def.joint1;
            this.m_localAnchorC.Copy(prismatic.m_localAnchorA);
            this.m_localAnchorA.Copy(prismatic.m_localAnchorB);
            this.m_referenceAngleA = prismatic.m_referenceAngle;
            this.m_localAxisC.Copy(prismatic.m_localXAxisA);
            // Vec2 pC = m_localAnchorC;
            const pC = this.m_localAnchorC;
            // Vec2 pA = MulT(xfC.q, Mul(xfA.q, m_localAnchorA) + (xfA.p - xfC.p));
            const pA = Rot.MulTRV(xfC.q, Vec2.AddVV(Rot.MulRV(xfA.q, this.m_localAnchorA, Vec2.s_t0), Vec2.SubVV(xfA.p, xfC.p, Vec2.s_t1), Vec2.s_t0), Vec2.s_t0); // pA uses s_t0
            // coordinateA = Dot(pA - pC, m_localAxisC);
            coordinateA = Vec2.DotVV(Vec2.SubVV(pA, pC, Vec2.s_t0), this.m_localAxisC);
        }
        this.m_bodyD = this.m_joint2.GetBodyA();
        this.m_bodyB = this.m_joint2.GetBodyB();
        // Get geometry of joint2
        const xfB = this.m_bodyB.m_xf;
        const aB = this.m_bodyB.m_sweep.a;
        const xfD = this.m_bodyD.m_xf;
        const aD = this.m_bodyD.m_sweep.a;
        if (this.m_typeB === JointType.e_revoluteJoint) {
            const revolute = def.joint2;
            this.m_localAnchorD.Copy(revolute.m_localAnchorA);
            this.m_localAnchorB.Copy(revolute.m_localAnchorB);
            this.m_referenceAngleB = revolute.m_referenceAngle;
            this.m_localAxisD.SetZero();
            coordinateB = aB - aD - this.m_referenceAngleB;
        }
        else {
            const prismatic = def.joint2;
            this.m_localAnchorD.Copy(prismatic.m_localAnchorA);
            this.m_localAnchorB.Copy(prismatic.m_localAnchorB);
            this.m_referenceAngleB = prismatic.m_referenceAngle;
            this.m_localAxisD.Copy(prismatic.m_localXAxisA);
            // Vec2 pD = m_localAnchorD;
            const pD = this.m_localAnchorD;
            // Vec2 pB = MulT(xfD.q, Mul(xfB.q, m_localAnchorB) + (xfB.p - xfD.p));
            const pB = Rot.MulTRV(xfD.q, Vec2.AddVV(Rot.MulRV(xfB.q, this.m_localAnchorB, Vec2.s_t0), Vec2.SubVV(xfB.p, xfD.p, Vec2.s_t1), Vec2.s_t0), Vec2.s_t0); // pB uses s_t0
            // coordinateB = Dot(pB - pD, m_localAxisD);
            coordinateB = Vec2.DotVV(Vec2.SubVV(pB, pD, Vec2.s_t0), this.m_localAxisD);
        }
        this.m_ratio = Maybe(def.ratio, 1);
        this.m_constant = coordinateA + this.m_ratio * coordinateB;
        this.m_impulse = 0;
    }
    InitVelocityConstraints(data) {
        this.m_indexA = this.m_bodyA.m_islandIndex;
        this.m_indexB = this.m_bodyB.m_islandIndex;
        this.m_indexC = this.m_bodyC.m_islandIndex;
        this.m_indexD = this.m_bodyD.m_islandIndex;
        this.m_lcA.Copy(this.m_bodyA.m_sweep.localCenter);
        this.m_lcB.Copy(this.m_bodyB.m_sweep.localCenter);
        this.m_lcC.Copy(this.m_bodyC.m_sweep.localCenter);
        this.m_lcD.Copy(this.m_bodyD.m_sweep.localCenter);
        this.m_mA = this.m_bodyA.m_invMass;
        this.m_mB = this.m_bodyB.m_invMass;
        this.m_mC = this.m_bodyC.m_invMass;
        this.m_mD = this.m_bodyD.m_invMass;
        this.m_iA = this.m_bodyA.m_invI;
        this.m_iB = this.m_bodyB.m_invI;
        this.m_iC = this.m_bodyC.m_invI;
        this.m_iD = this.m_bodyD.m_invI;
        const aA = data.positions[this.m_indexA].a;
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const aB = data.positions[this.m_indexB].a;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const aC = data.positions[this.m_indexC].a;
        const vC = data.velocities[this.m_indexC].v;
        let wC = data.velocities[this.m_indexC].w;
        const aD = data.positions[this.m_indexD].a;
        const vD = data.velocities[this.m_indexD].v;
        let wD = data.velocities[this.m_indexD].w;
        // Rot qA(aA), qB(aB), qC(aC), qD(aD);
        const qA = this.m_qA.SetAngle(aA);
        const qB = this.m_qB.SetAngle(aB);
        const qC = this.m_qC.SetAngle(aC);
        const qD = this.m_qD.SetAngle(aD);
        this.m_mass = 0;
        if (this.m_typeA === JointType.e_revoluteJoint) {
            this.m_JvAC.SetZero();
            this.m_JwA = 1;
            this.m_JwC = 1;
            this.m_mass += this.m_iA + this.m_iC;
        }
        else {
            // Vec2 u = Mul(qC, m_localAxisC);
            const u = Rot.MulRV(qC, this.m_localAxisC, GearJoint.InitVelocityConstraints_s_u);
            // Vec2 rC = Mul(qC, m_localAnchorC - m_lcC);
            Vec2.SubVV(this.m_localAnchorC, this.m_lcC, this.m_lalcC);
            const rC = Rot.MulRV(qC, this.m_lalcC, GearJoint.InitVelocityConstraints_s_rC);
            // Vec2 rA = Mul(qA, m_localAnchorA - m_lcA);
            Vec2.SubVV(this.m_localAnchorA, this.m_lcA, this.m_lalcA);
            const rA = Rot.MulRV(qA, this.m_lalcA, GearJoint.InitVelocityConstraints_s_rA);
            // m_JvAC = u;
            this.m_JvAC.Copy(u);
            // m_JwC = Cross(rC, u);
            this.m_JwC = Vec2.CrossVV(rC, u);
            // m_JwA = Cross(rA, u);
            this.m_JwA = Vec2.CrossVV(rA, u);
            this.m_mass +=
                this.m_mC +
                    this.m_mA +
                    this.m_iC * this.m_JwC * this.m_JwC +
                    this.m_iA * this.m_JwA * this.m_JwA;
        }
        if (this.m_typeB === JointType.e_revoluteJoint) {
            this.m_JvBD.SetZero();
            this.m_JwB = this.m_ratio;
            this.m_JwD = this.m_ratio;
            this.m_mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);
        }
        else {
            // Vec2 u = Mul(qD, m_localAxisD);
            const u = Rot.MulRV(qD, this.m_localAxisD, GearJoint.InitVelocityConstraints_s_u);
            // Vec2 rD = Mul(qD, m_localAnchorD - m_lcD);
            Vec2.SubVV(this.m_localAnchorD, this.m_lcD, this.m_lalcD);
            const rD = Rot.MulRV(qD, this.m_lalcD, GearJoint.InitVelocityConstraints_s_rD);
            // Vec2 rB = Mul(qB, m_localAnchorB - m_lcB);
            Vec2.SubVV(this.m_localAnchorB, this.m_lcB, this.m_lalcB);
            const rB = Rot.MulRV(qB, this.m_lalcB, GearJoint.InitVelocityConstraints_s_rB);
            // m_JvBD = m_ratio * u;
            Vec2.MulSV(this.m_ratio, u, this.m_JvBD);
            // m_JwD = m_ratio * Cross(rD, u);
            this.m_JwD = this.m_ratio * Vec2.CrossVV(rD, u);
            // m_JwB = m_ratio * Cross(rB, u);
            this.m_JwB = this.m_ratio * Vec2.CrossVV(rB, u);
            this.m_mass +=
                this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) +
                    this.m_iD * this.m_JwD * this.m_JwD +
                    this.m_iB * this.m_JwB * this.m_JwB;
        }
        // Compute effective mass.
        this.m_mass = this.m_mass > 0 ? 1 / this.m_mass : 0;
        if (data.step.warmStarting) {
            // vA += (m_mA * m_impulse) * m_JvAC;
            vA.SelfMulAdd(this.m_mA * this.m_impulse, this.m_JvAC);
            wA += this.m_iA * this.m_impulse * this.m_JwA;
            // vB += (m_mB * m_impulse) * m_JvBD;
            vB.SelfMulAdd(this.m_mB * this.m_impulse, this.m_JvBD);
            wB += this.m_iB * this.m_impulse * this.m_JwB;
            // vC -= (m_mC * m_impulse) * m_JvAC;
            vC.SelfMulSub(this.m_mC * this.m_impulse, this.m_JvAC);
            wC -= this.m_iC * this.m_impulse * this.m_JwC;
            // vD -= (m_mD * m_impulse) * m_JvBD;
            vD.SelfMulSub(this.m_mD * this.m_impulse, this.m_JvBD);
            wD -= this.m_iD * this.m_impulse * this.m_JwD;
        }
        else {
            this.m_impulse = 0;
        }
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
        // data.velocities[this.m_indexC].v = vC;
        data.velocities[this.m_indexC].w = wC;
        // data.velocities[this.m_indexD].v = vD;
        data.velocities[this.m_indexD].w = wD;
    }
    SolveVelocityConstraints(data) {
        const vA = data.velocities[this.m_indexA].v;
        let wA = data.velocities[this.m_indexA].w;
        const vB = data.velocities[this.m_indexB].v;
        let wB = data.velocities[this.m_indexB].w;
        const vC = data.velocities[this.m_indexC].v;
        let wC = data.velocities[this.m_indexC].w;
        const vD = data.velocities[this.m_indexD].v;
        let wD = data.velocities[this.m_indexD].w;
        // float32 Cdot = Dot(m_JvAC, vA - vC) + Dot(m_JvBD, vB - vD);
        let Cdot = Vec2.DotVV(this.m_JvAC, Vec2.SubVV(vA, vC, Vec2.s_t0)) +
            Vec2.DotVV(this.m_JvBD, Vec2.SubVV(vB, vD, Vec2.s_t0));
        Cdot +=
            this.m_JwA * wA - this.m_JwC * wC + (this.m_JwB * wB - this.m_JwD * wD);
        const impulse = -this.m_mass * Cdot;
        this.m_impulse += impulse;
        // vA += (m_mA * impulse) * m_JvAC;
        vA.SelfMulAdd(this.m_mA * impulse, this.m_JvAC);
        wA += this.m_iA * impulse * this.m_JwA;
        // vB += (m_mB * impulse) * m_JvBD;
        vB.SelfMulAdd(this.m_mB * impulse, this.m_JvBD);
        wB += this.m_iB * impulse * this.m_JwB;
        // vC -= (m_mC * impulse) * m_JvAC;
        vC.SelfMulSub(this.m_mC * impulse, this.m_JvAC);
        wC -= this.m_iC * impulse * this.m_JwC;
        // vD -= (m_mD * impulse) * m_JvBD;
        vD.SelfMulSub(this.m_mD * impulse, this.m_JvBD);
        wD -= this.m_iD * impulse * this.m_JwD;
        // data.velocities[this.m_indexA].v = vA;
        data.velocities[this.m_indexA].w = wA;
        // data.velocities[this.m_indexB].v = vB;
        data.velocities[this.m_indexB].w = wB;
        // data.velocities[this.m_indexC].v = vC;
        data.velocities[this.m_indexC].w = wC;
        // data.velocities[this.m_indexD].v = vD;
        data.velocities[this.m_indexD].w = wD;
    }
    SolvePositionConstraints(data) {
        const cA = data.positions[this.m_indexA].c;
        let aA = data.positions[this.m_indexA].a;
        const cB = data.positions[this.m_indexB].c;
        let aB = data.positions[this.m_indexB].a;
        const cC = data.positions[this.m_indexC].c;
        let aC = data.positions[this.m_indexC].a;
        const cD = data.positions[this.m_indexD].c;
        let aD = data.positions[this.m_indexD].a;
        // Rot qA(aA), qB(aB), qC(aC), qD(aD);
        const qA = this.m_qA.SetAngle(aA);
        const qB = this.m_qB.SetAngle(aB);
        const qC = this.m_qC.SetAngle(aC);
        const qD = this.m_qD.SetAngle(aD);
        const linearError = 0;
        let coordinateA;
        let coordinateB;
        const JvAC = this.m_JvAC;
        const JvBD = this.m_JvBD;
        let JwA;
        let JwB;
        let JwC;
        let JwD;
        let mass = 0;
        if (this.m_typeA === JointType.e_revoluteJoint) {
            JvAC.SetZero();
            JwA = 1;
            JwC = 1;
            mass += this.m_iA + this.m_iC;
            coordinateA = aA - aC - this.m_referenceAngleA;
        }
        else {
            // Vec2 u = Mul(qC, m_localAxisC);
            const u = Rot.MulRV(qC, this.m_localAxisC, GearJoint.SolvePositionConstraints_s_u);
            // Vec2 rC = Mul(qC, m_localAnchorC - m_lcC);
            const rC = Rot.MulRV(qC, this.m_lalcC, GearJoint.SolvePositionConstraints_s_rC);
            // Vec2 rA = Mul(qA, m_localAnchorA - m_lcA);
            const rA = Rot.MulRV(qA, this.m_lalcA, GearJoint.SolvePositionConstraints_s_rA);
            // JvAC = u;
            JvAC.Copy(u);
            // JwC = Cross(rC, u);
            JwC = Vec2.CrossVV(rC, u);
            // JwA = Cross(rA, u);
            JwA = Vec2.CrossVV(rA, u);
            mass +=
                this.m_mC + this.m_mA + this.m_iC * JwC * JwC + this.m_iA * JwA * JwA;
            // Vec2 pC = m_localAnchorC - m_lcC;
            const pC = this.m_lalcC;
            // Vec2 pA = MulT(qC, rA + (cA - cC));
            const pA = Rot.MulTRV(qC, Vec2.AddVV(rA, Vec2.SubVV(cA, cC, Vec2.s_t0), Vec2.s_t0), Vec2.s_t0); // pA uses s_t0
            // coordinateA = Dot(pA - pC, m_localAxisC);
            coordinateA = Vec2.DotVV(Vec2.SubVV(pA, pC, Vec2.s_t0), this.m_localAxisC);
        }
        if (this.m_typeB === JointType.e_revoluteJoint) {
            JvBD.SetZero();
            JwB = this.m_ratio;
            JwD = this.m_ratio;
            mass += this.m_ratio * this.m_ratio * (this.m_iB + this.m_iD);
            coordinateB = aB - aD - this.m_referenceAngleB;
        }
        else {
            // Vec2 u = Mul(qD, m_localAxisD);
            const u = Rot.MulRV(qD, this.m_localAxisD, GearJoint.SolvePositionConstraints_s_u);
            // Vec2 rD = Mul(qD, m_localAnchorD - m_lcD);
            const rD = Rot.MulRV(qD, this.m_lalcD, GearJoint.SolvePositionConstraints_s_rD);
            // Vec2 rB = Mul(qB, m_localAnchorB - m_lcB);
            const rB = Rot.MulRV(qB, this.m_lalcB, GearJoint.SolvePositionConstraints_s_rB);
            // JvBD = m_ratio * u;
            Vec2.MulSV(this.m_ratio, u, JvBD);
            // JwD = m_ratio * Cross(rD, u);
            JwD = this.m_ratio * Vec2.CrossVV(rD, u);
            // JwB = m_ratio * Cross(rB, u);
            JwB = this.m_ratio * Vec2.CrossVV(rB, u);
            mass +=
                this.m_ratio * this.m_ratio * (this.m_mD + this.m_mB) +
                    this.m_iD * JwD * JwD +
                    this.m_iB * JwB * JwB;
            // Vec2 pD = m_localAnchorD - m_lcD;
            const pD = this.m_lalcD;
            // Vec2 pB = MulT(qD, rB + (cB - cD));
            const pB = Rot.MulTRV(qD, Vec2.AddVV(rB, Vec2.SubVV(cB, cD, Vec2.s_t0), Vec2.s_t0), Vec2.s_t0); // pB uses s_t0
            // coordinateB = Dot(pB - pD, m_localAxisD);
            coordinateB = Vec2.DotVV(Vec2.SubVV(pB, pD, Vec2.s_t0), this.m_localAxisD);
        }
        const C = coordinateA + this.m_ratio * coordinateB - this.m_constant;
        let impulse = 0;
        if (mass > 0) {
            impulse = -C / mass;
        }
        // cA += m_mA * impulse * JvAC;
        cA.SelfMulAdd(this.m_mA * impulse, JvAC);
        aA += this.m_iA * impulse * JwA;
        // cB += m_mB * impulse * JvBD;
        cB.SelfMulAdd(this.m_mB * impulse, JvBD);
        aB += this.m_iB * impulse * JwB;
        // cC -= m_mC * impulse * JvAC;
        cC.SelfMulSub(this.m_mC * impulse, JvAC);
        aC -= this.m_iC * impulse * JwC;
        // cD -= m_mD * impulse * JvBD;
        cD.SelfMulSub(this.m_mD * impulse, JvBD);
        aD -= this.m_iD * impulse * JwD;
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        // data.positions[this.m_indexC].c = cC;
        data.positions[this.m_indexC].a = aC;
        // data.positions[this.m_indexD].c = cD;
        data.positions[this.m_indexD].a = aD;
        // TODO_ERIN not implemented
        return linearError < linearSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // Vec2 P = m_impulse * m_JvAC;
        // return inv_dt * P;
        return Vec2.MulSV(inv_dt * this.m_impulse, this.m_JvAC, out);
    }
    GetReactionTorque(inv_dt) {
        // float32 L = m_impulse * m_JwA;
        // return inv_dt * L;
        return inv_dt * this.m_impulse * this.m_JwA;
    }
    GetJoint1() {
        return this.m_joint1;
    }
    GetJoint2() {
        return this.m_joint2;
    }
    GetRatio() {
        return this.m_ratio;
    }
    SetRatio(ratio) {
        // DEBUG: Assert(IsValid(ratio));
        this.m_ratio = ratio;
    }
    Dump(log) {
        const indexA = this.m_bodyA.m_islandIndex;
        const indexB = this.m_bodyB.m_islandIndex;
        const index1 = this.m_joint1.m_index;
        const index2 = this.m_joint2.m_index;
        log('  const jd: GearJointDef = new GearJointDef();\n');
        log('  jd.bodyA = bodies[%d];\n', indexA);
        log('  jd.bodyB = bodies[%d];\n', indexB);
        log('  jd.collideConnected = %s;\n', this.m_collideConnected ? 'true' : 'false');
        log('  jd.joint1 = joints[%d];\n', index1);
        log('  jd.joint2 = joints[%d];\n', index2);
        log('  jd.ratio = %.15f;\n', this.m_ratio);
        log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index);
    }
}
GearJoint.InitVelocityConstraints_s_u = new Vec2();
GearJoint.InitVelocityConstraints_s_rA = new Vec2();
GearJoint.InitVelocityConstraints_s_rB = new Vec2();
GearJoint.InitVelocityConstraints_s_rC = new Vec2();
GearJoint.InitVelocityConstraints_s_rD = new Vec2();
GearJoint.SolvePositionConstraints_s_u = new Vec2();
GearJoint.SolvePositionConstraints_s_rA = new Vec2();
GearJoint.SolvePositionConstraints_s_rB = new Vec2();
GearJoint.SolvePositionConstraints_s_rC = new Vec2();
GearJoint.SolvePositionConstraints_s_rD = new Vec2();
//# sourceMappingURL=GearJoint.js.map