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
import { Abs, Clamp, Mat22, Mat33, Max, Min, Rot, Vec2, Vec3 } from '../../Common/Math';
import { angularSlop, linearSlop, maxLinearCorrection, Maybe } from '../../Common/Settings';
import { Joint, JointDef, JointType, LimitState } from './Joint';
/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
export class PrismaticJointDef extends JointDef {
    constructor() {
        super(JointType.e_prismaticJoint);
        this.localAnchorA = new Vec2();
        this.localAnchorB = new Vec2();
        this.localAxisA = new Vec2(1, 0);
        this.referenceAngle = 0;
        this.enableLimit = false;
        this.lowerTranslation = 0;
        this.upperTranslation = 0;
        this.enableMotor = false;
        this.maxMotorForce = 0;
        this.motorSpeed = 0;
    }
    Initialize(bA, bB, anchor, axis) {
        this.bodyA = bA;
        this.bodyB = bB;
        this.bodyA.GetLocalPoint(anchor, this.localAnchorA);
        this.bodyB.GetLocalPoint(anchor, this.localAnchorB);
        this.bodyA.GetLocalVector(axis, this.localAxisA);
        this.referenceAngle = this.bodyB.GetAngle() - this.bodyA.GetAngle();
    }
}
export class PrismaticJoint extends Joint {
    constructor(def) {
        super(def);
        // Solver shared
        this.m_localAnchorA = new Vec2();
        this.m_localAnchorB = new Vec2();
        this.m_localXAxisA = new Vec2();
        this.m_localYAxisA = new Vec2();
        this.m_referenceAngle = 0;
        this.m_impulse = new Vec3(0, 0, 0);
        this.m_motorImpulse = 0;
        this.m_lowerTranslation = 0;
        this.m_upperTranslation = 0;
        this.m_maxMotorForce = 0;
        this.m_motorSpeed = 0;
        this.m_enableLimit = false;
        this.m_enableMotor = false;
        this.m_limitState = LimitState.e_inactiveLimit;
        // Solver temp
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_localCenterA = new Vec2();
        this.m_localCenterB = new Vec2();
        this.m_invMassA = 0;
        this.m_invMassB = 0;
        this.m_invIA = 0;
        this.m_invIB = 0;
        this.m_axis = new Vec2(0, 0);
        this.m_perp = new Vec2(0, 0);
        this.m_s1 = 0;
        this.m_s2 = 0;
        this.m_a1 = 0;
        this.m_a2 = 0;
        this.m_K = new Mat33();
        this.m_K3 = new Mat33();
        this.m_K2 = new Mat22();
        this.m_motorMass = 0;
        this.m_qA = new Rot();
        this.m_qB = new Rot();
        this.m_lalcA = new Vec2();
        this.m_lalcB = new Vec2();
        this.m_rA = new Vec2();
        this.m_rB = new Vec2();
        this.m_localAnchorA.Copy(Maybe(def.localAnchorA, Vec2.ZERO));
        this.m_localAnchorB.Copy(Maybe(def.localAnchorB, Vec2.ZERO));
        this.m_localXAxisA
            .Copy(Maybe(def.localAxisA, new Vec2(1, 0)))
            .SelfNormalize();
        Vec2.CrossOneV(this.m_localXAxisA, this.m_localYAxisA);
        this.m_referenceAngle = Maybe(def.referenceAngle, 0);
        this.m_lowerTranslation = Maybe(def.lowerTranslation, 0);
        this.m_upperTranslation = Maybe(def.upperTranslation, 0);
        this.m_maxMotorForce = Maybe(def.maxMotorForce, 0);
        this.m_motorSpeed = Maybe(def.motorSpeed, 0);
        this.m_enableLimit = Maybe(def.enableLimit, false);
        this.m_enableMotor = Maybe(def.enableMotor, false);
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
        // Compute the effective masses.
        // Vec2 rA = Mul(qA, m_localAnchorA - m_localCenterA);
        Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA);
        const rA = Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // Vec2 rB = Mul(qB, m_localAnchorB - m_localCenterB);
        Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB);
        const rB = Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // Vec2 d = (cB - cA) + rB - rA;
        const d = Vec2.AddVV(Vec2.SubVV(cB, cA, Vec2.s_t0), Vec2.SubVV(rB, rA, Vec2.s_t1), PrismaticJoint.InitVelocityConstraints_s_d);
        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;
        // Compute motor Jacobian and effective mass.
        {
            // m_axis = Mul(qA, m_localXAxisA);
            Rot.MulRV(qA, this.m_localXAxisA, this.m_axis);
            // m_a1 = Cross(d + rA, m_axis);
            this.m_a1 = Vec2.CrossVV(Vec2.AddVV(d, rA, Vec2.s_t0), this.m_axis);
            // m_a2 = Cross(rB, m_axis);
            this.m_a2 = Vec2.CrossVV(rB, this.m_axis);
            this.m_motorMass =
                mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
            if (this.m_motorMass > 0) {
                this.m_motorMass = 1 / this.m_motorMass;
            }
        }
        // Prismatic constraint.
        {
            // m_perp = Mul(qA, m_localYAxisA);
            Rot.MulRV(qA, this.m_localYAxisA, this.m_perp);
            // m_s1 = Cross(d + rA, m_perp);
            this.m_s1 = Vec2.CrossVV(Vec2.AddVV(d, rA, Vec2.s_t0), this.m_perp);
            // m_s2 = Cross(rB, m_perp);
            this.m_s2 = Vec2.CrossVV(rB, this.m_perp);
            // float32 k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
            this.m_K.ex.x =
                mA + mB + iA * this.m_s1 * this.m_s1 + iB * this.m_s2 * this.m_s2;
            // float32 k12 = iA * m_s1 + iB * m_s2;
            this.m_K.ex.y = iA * this.m_s1 + iB * this.m_s2;
            // float32 k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
            this.m_K.ex.z = iA * this.m_s1 * this.m_a1 + iB * this.m_s2 * this.m_a2;
            this.m_K.ey.x = this.m_K.ex.y;
            // float32 k22 = iA + iB;
            this.m_K.ey.y = iA + iB;
            if (this.m_K.ey.y === 0) {
                // For bodies with fixed rotation.
                this.m_K.ey.y = 1;
            }
            // float32 k23 = iA * m_a1 + iB * m_a2;
            this.m_K.ey.z = iA * this.m_a1 + iB * this.m_a2;
            this.m_K.ez.x = this.m_K.ex.z;
            this.m_K.ez.y = this.m_K.ey.z;
            // float32 k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
            this.m_K.ez.z =
                mA + mB + iA * this.m_a1 * this.m_a1 + iB * this.m_a2 * this.m_a2;
            // m_K.ex.Set(k11, k12, k13);
            // m_K.ey.Set(k12, k22, k23);
            // m_K.ez.Set(k13, k23, k33);
        }
        // Compute motor and limit terms.
        if (this.m_enableLimit) {
            // float32 jointTranslation = Dot(m_axis, d);
            const jointTranslation = Vec2.DotVV(this.m_axis, d);
            if (Abs(this.m_upperTranslation - this.m_lowerTranslation) <
                2 * linearSlop) {
                this.m_limitState = LimitState.e_equalLimits;
            }
            else if (jointTranslation <= this.m_lowerTranslation) {
                if (this.m_limitState !== LimitState.e_atLowerLimit) {
                    this.m_limitState = LimitState.e_atLowerLimit;
                    this.m_impulse.z = 0;
                }
            }
            else if (jointTranslation >= this.m_upperTranslation) {
                if (this.m_limitState !== LimitState.e_atUpperLimit) {
                    this.m_limitState = LimitState.e_atUpperLimit;
                    this.m_impulse.z = 0;
                }
            }
            else {
                this.m_limitState = LimitState.e_inactiveLimit;
                this.m_impulse.z = 0;
            }
        }
        else {
            this.m_limitState = LimitState.e_inactiveLimit;
            this.m_impulse.z = 0;
        }
        if (!this.m_enableMotor) {
            this.m_motorImpulse = 0;
        }
        if (data.step.warmStarting) {
            // Account for variable time step.
            // m_impulse *= data.step.dtRatio;
            this.m_impulse.SelfMul(data.step.dtRatio);
            this.m_motorImpulse *= data.step.dtRatio;
            // Vec2 P = m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis;
            const P = Vec2.AddVV(Vec2.MulSV(this.m_impulse.x, this.m_perp, Vec2.s_t0), Vec2.MulSV(this.m_motorImpulse + this.m_impulse.z, this.m_axis, Vec2.s_t1), PrismaticJoint.InitVelocityConstraints_s_P);
            // float32 LA = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
            const LA = this.m_impulse.x * this.m_s1 +
                this.m_impulse.y +
                (this.m_motorImpulse + this.m_impulse.z) * this.m_a1;
            // float32 LB = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;
            const LB = this.m_impulse.x * this.m_s2 +
                this.m_impulse.y +
                (this.m_motorImpulse + this.m_impulse.z) * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        else {
            this.m_impulse.SetZero();
            this.m_motorImpulse = 0;
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
        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;
        // Solve linear motor constraint.
        if (this.m_enableMotor && this.m_limitState !== LimitState.e_equalLimits) {
            // float32 Cdot = Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
            const Cdot = Vec2.DotVV(this.m_axis, Vec2.SubVV(vB, vA, Vec2.s_t0)) +
                this.m_a2 * wB -
                this.m_a1 * wA;
            let impulse = this.m_motorMass * (this.m_motorSpeed - Cdot);
            const oldImpulse = this.m_motorImpulse;
            const maxImpulse = data.step.dt * this.m_maxMotorForce;
            this.m_motorImpulse = Clamp(this.m_motorImpulse + impulse, -maxImpulse, maxImpulse);
            impulse = this.m_motorImpulse - oldImpulse;
            // Vec2 P = impulse * m_axis;
            const P = Vec2.MulSV(impulse, this.m_axis, PrismaticJoint.SolveVelocityConstraints_s_P);
            const LA = impulse * this.m_a1;
            const LB = impulse * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        // Vec2 Cdot1;
        // Cdot1.x = Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
        const Cdot1_x = Vec2.DotVV(this.m_perp, Vec2.SubVV(vB, vA, Vec2.s_t0)) +
            this.m_s2 * wB -
            this.m_s1 * wA;
        // Cdot1.y = wB - wA;
        const Cdot1_y = wB - wA;
        if (this.m_enableLimit &&
            this.m_limitState !== LimitState.e_inactiveLimit) {
            // Solve prismatic and limit constraint in block form.
            // float32 Cdot2;
            // Cdot2 = Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
            const Cdot2 = Vec2.DotVV(this.m_axis, Vec2.SubVV(vB, vA, Vec2.s_t0)) +
                this.m_a2 * wB -
                this.m_a1 * wA;
            // Vec3 Cdot(Cdot1.x, Cdot1.y, Cdot2);
            // Vec3 f1 = m_impulse;
            const f1 = PrismaticJoint.SolveVelocityConstraints_s_f1.Copy(this.m_impulse);
            // Vec3 df =  m_K.Solve33(-Cdot);
            const df3 = this.m_K.Solve33(-Cdot1_x, -Cdot1_y, -Cdot2, PrismaticJoint.SolveVelocityConstraints_s_df3);
            // m_impulse += df;
            this.m_impulse.SelfAdd(df3);
            if (this.m_limitState === LimitState.e_atLowerLimit) {
                this.m_impulse.z = Max(this.m_impulse.z, 0);
            }
            else if (this.m_limitState === LimitState.e_atUpperLimit) {
                this.m_impulse.z = Min(this.m_impulse.z, 0);
            }
            // f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
            // Vec2 b = -Cdot1 - (m_impulse.z - f1.z) * Vec2(m_K.ez.x, m_K.ez.y);
            const b_x = -Cdot1_x - (this.m_impulse.z - f1.z) * this.m_K.ez.x;
            const b_y = -Cdot1_y - (this.m_impulse.z - f1.z) * this.m_K.ez.y;
            // Vec2 f2r = m_K.Solve22(b) + Vec2(f1.x, f1.y);
            const f2r = this.m_K.Solve22(b_x, b_y, PrismaticJoint.SolveVelocityConstraints_s_f2r);
            f2r.x += f1.x;
            f2r.y += f1.y;
            // m_impulse.x = f2r.x;
            this.m_impulse.x = f2r.x;
            // m_impulse.y = f2r.y;
            this.m_impulse.y = f2r.y;
            // df = m_impulse - f1;
            df3.x = this.m_impulse.x - f1.x;
            df3.y = this.m_impulse.y - f1.y;
            df3.z = this.m_impulse.z - f1.z;
            // Vec2 P = df.x * m_perp + df.z * m_axis;
            const P = Vec2.AddVV(Vec2.MulSV(df3.x, this.m_perp, Vec2.s_t0), Vec2.MulSV(df3.z, this.m_axis, Vec2.s_t1), PrismaticJoint.SolveVelocityConstraints_s_P);
            // float32 LA = df.x * m_s1 + df.y + df.z * m_a1;
            const LA = df3.x * this.m_s1 + df3.y + df3.z * this.m_a1;
            // float32 LB = df.x * m_s2 + df.y + df.z * m_a2;
            const LB = df3.x * this.m_s2 + df3.y + df3.z * this.m_a2;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
        else {
            // Limit is inactive, just solve the prismatic constraint in block form.
            // Vec2 df = m_K.Solve22(-Cdot1);
            const df2 = this.m_K.Solve22(-Cdot1_x, -Cdot1_y, PrismaticJoint.SolveVelocityConstraints_s_df2);
            this.m_impulse.x += df2.x;
            this.m_impulse.y += df2.y;
            // Vec2 P = df.x * m_perp;
            const P = Vec2.MulSV(df2.x, this.m_perp, PrismaticJoint.SolveVelocityConstraints_s_P);
            // float32 LA = df.x * m_s1 + df.y;
            const LA = df2.x * this.m_s1 + df2.y;
            // float32 LB = df.x * m_s2 + df.y;
            const LB = df2.x * this.m_s2 + df2.y;
            // vA -= mA * P;
            vA.SelfMulSub(mA, P);
            wA -= iA * LA;
            // vB += mB * P;
            vB.SelfMulAdd(mB, P);
            wB += iB * LB;
        }
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
        const mA = this.m_invMassA;
        const mB = this.m_invMassB;
        const iA = this.m_invIA;
        const iB = this.m_invIB;
        // Vec2 rA = Mul(qA, m_localAnchorA - m_localCenterA);
        const rA = Rot.MulRV(qA, this.m_lalcA, this.m_rA);
        // Vec2 rB = Mul(qB, m_localAnchorB - m_localCenterB);
        const rB = Rot.MulRV(qB, this.m_lalcB, this.m_rB);
        // Vec2 d = cB + rB - cA - rA;
        const d = Vec2.SubVV(Vec2.AddVV(cB, rB, Vec2.s_t0), Vec2.AddVV(cA, rA, Vec2.s_t1), PrismaticJoint.SolvePositionConstraints_s_d);
        // Vec2 axis = Mul(qA, m_localXAxisA);
        const axis = Rot.MulRV(qA, this.m_localXAxisA, this.m_axis);
        // float32 a1 = Cross(d + rA, axis);
        const a1 = Vec2.CrossVV(Vec2.AddVV(d, rA, Vec2.s_t0), axis);
        // float32 a2 = Cross(rB, axis);
        const a2 = Vec2.CrossVV(rB, axis);
        // Vec2 perp = Mul(qA, m_localYAxisA);
        const perp = Rot.MulRV(qA, this.m_localYAxisA, this.m_perp);
        // float32 s1 = Cross(d + rA, perp);
        const s1 = Vec2.CrossVV(Vec2.AddVV(d, rA, Vec2.s_t0), perp);
        // float32 s2 = Cross(rB, perp);
        const s2 = Vec2.CrossVV(rB, perp);
        // Vec3 impulse;
        let impulse = PrismaticJoint.SolvePositionConstraints_s_impulse;
        // Vec2 C1;
        // C1.x = Dot(perp, d);
        const C1_x = Vec2.DotVV(perp, d);
        // C1.y = aB - aA - m_referenceAngle;
        const C1_y = aB - aA - this.m_referenceAngle;
        let linearError = Abs(C1_x);
        const angularError = Abs(C1_y);
        let active = false;
        let C2 = 0;
        if (this.m_enableLimit) {
            // float32 translation = Dot(axis, d);
            const translation = Vec2.DotVV(axis, d);
            if (Abs(this.m_upperTranslation - this.m_lowerTranslation) <
                2 * linearSlop) {
                // Prevent large angular corrections
                C2 = Clamp(translation, -maxLinearCorrection, maxLinearCorrection);
                linearError = Max(linearError, Abs(translation));
                active = true;
            }
            else if (translation <= this.m_lowerTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = Clamp(translation - this.m_lowerTranslation + linearSlop, -maxLinearCorrection, 0);
                linearError = Max(linearError, this.m_lowerTranslation - translation);
                active = true;
            }
            else if (translation >= this.m_upperTranslation) {
                // Prevent large linear corrections and allow some slop.
                C2 = Clamp(translation - this.m_upperTranslation - linearSlop, 0, maxLinearCorrection);
                linearError = Max(linearError, translation - this.m_upperTranslation);
                active = true;
            }
        }
        if (active) {
            // float32 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            const k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            // float32 k12 = iA * s1 + iB * s2;
            const k12 = iA * s1 + iB * s2;
            // float32 k13 = iA * s1 * a1 + iB * s2 * a2;
            const k13 = iA * s1 * a1 + iB * s2 * a2;
            // float32 k22 = iA + iB;
            let k22 = iA + iB;
            if (k22 === 0) {
                // For fixed rotation
                k22 = 1;
            }
            // float32 k23 = iA * a1 + iB * a2;
            const k23 = iA * a1 + iB * a2;
            // float32 k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            const k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;
            // Mat33 K;
            const K = this.m_K3;
            // K.ex.Set(k11, k12, k13);
            K.ex.SetXYZ(k11, k12, k13);
            // K.ey.Set(k12, k22, k23);
            K.ey.SetXYZ(k12, k22, k23);
            // K.ez.Set(k13, k23, k33);
            K.ez.SetXYZ(k13, k23, k33);
            // Vec3 C;
            // C.x = C1.x;
            // C.y = C1.y;
            // C.z = C2;
            // impulse = K.Solve33(-C);
            impulse = K.Solve33(-C1_x, -C1_y, -C2, impulse);
        }
        else {
            // float32 k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            const k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
            // float32 k12 = iA * s1 + iB * s2;
            const k12 = iA * s1 + iB * s2;
            // float32 k22 = iA + iB;
            let k22 = iA + iB;
            if (k22 === 0) {
                k22 = 1;
            }
            // Mat22 K;
            const K2 = this.m_K2;
            // K.ex.Set(k11, k12);
            K2.ex.Set(k11, k12);
            // K.ey.Set(k12, k22);
            K2.ey.Set(k12, k22);
            // Vec2 impulse1 = K.Solve(-C1);
            const impulse1 = K2.Solve(-C1_x, -C1_y, PrismaticJoint.SolvePositionConstraints_s_impulse1);
            impulse.x = impulse1.x;
            impulse.y = impulse1.y;
            impulse.z = 0;
        }
        // Vec2 P = impulse.x * perp + impulse.z * axis;
        const P = Vec2.AddVV(Vec2.MulSV(impulse.x, perp, Vec2.s_t0), Vec2.MulSV(impulse.z, axis, Vec2.s_t1), PrismaticJoint.SolvePositionConstraints_s_P);
        // float32 LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        const LA = impulse.x * s1 + impulse.y + impulse.z * a1;
        // float32 LB = impulse.x * s2 + impulse.y + impulse.z * a2;
        const LB = impulse.x * s2 + impulse.y + impulse.z * a2;
        // cA -= mA * P;
        cA.SelfMulSub(mA, P);
        aA -= iA * LA;
        // cB += mB * P;
        cB.SelfMulAdd(mB, P);
        aB += iB * LB;
        // data.positions[this.m_indexA].c = cA;
        data.positions[this.m_indexA].a = aA;
        // data.positions[this.m_indexB].c = cB;
        data.positions[this.m_indexB].a = aB;
        return linearError <= linearSlop && angularError <= angularSlop;
    }
    GetAnchorA(out) {
        return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out);
    }
    GetAnchorB(out) {
        return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out);
    }
    GetReactionForce(inv_dt, out) {
        // return inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis);
        out.x =
            inv_dt *
                (this.m_impulse.x * this.m_perp.x +
                    (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.x);
        out.y =
            inv_dt *
                (this.m_impulse.x * this.m_perp.y +
                    (this.m_motorImpulse + this.m_impulse.z) * this.m_axis.y);
        return out;
    }
    GetReactionTorque(inv_dt) {
        return inv_dt * this.m_impulse.y;
    }
    GetLocalAnchorA() {
        return this.m_localAnchorA;
    }
    GetLocalAnchorB() {
        return this.m_localAnchorB;
    }
    GetLocalAxisA() {
        return this.m_localXAxisA;
    }
    GetReferenceAngle() {
        return this.m_referenceAngle;
    }
    GetJointTranslation() {
        // Vec2 pA = m_bodyA.GetWorldPoint(m_localAnchorA);
        const pA = this.m_bodyA.GetWorldPoint(this.m_localAnchorA, PrismaticJoint.GetJointTranslation_s_pA);
        // Vec2 pB = m_bodyB.GetWorldPoint(m_localAnchorB);
        const pB = this.m_bodyB.GetWorldPoint(this.m_localAnchorB, PrismaticJoint.GetJointTranslation_s_pB);
        // Vec2 d = pB - pA;
        const d = Vec2.SubVV(pB, pA, PrismaticJoint.GetJointTranslation_s_d);
        // Vec2 axis = m_bodyA.GetWorldVector(m_localXAxisA);
        const axis = this.m_bodyA.GetWorldVector(this.m_localXAxisA, PrismaticJoint.GetJointTranslation_s_axis);
        // float32 translation = Dot(d, axis);
        const translation = Vec2.DotVV(d, axis);
        return translation;
    }
    GetJointSpeed() {
        const bA = this.m_bodyA;
        const bB = this.m_bodyB;
        // Vec2 rA = Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
        Vec2.SubVV(this.m_localAnchorA, bA.m_sweep.localCenter, this.m_lalcA);
        const rA = Rot.MulRV(bA.m_xf.q, this.m_lalcA, this.m_rA);
        // Vec2 rB = Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
        Vec2.SubVV(this.m_localAnchorB, bB.m_sweep.localCenter, this.m_lalcB);
        const rB = Rot.MulRV(bB.m_xf.q, this.m_lalcB, this.m_rB);
        // Vec2 pA = bA->m_sweep.c + rA;
        const pA = Vec2.AddVV(bA.m_sweep.c, rA, Vec2.s_t0); // pA uses s_t0
        // Vec2 pB = bB->m_sweep.c + rB;
        const pB = Vec2.AddVV(bB.m_sweep.c, rB, Vec2.s_t1); // pB uses s_t1
        // Vec2 d = pB - pA;
        const d = Vec2.SubVV(pB, pA, Vec2.s_t2); // d uses s_t2
        // Vec2 axis = Mul(bA.m_xf.q, m_localXAxisA);
        const axis = bA.GetWorldVector(this.m_localXAxisA, this.m_axis);
        const vA = bA.m_linearVelocity;
        const vB = bB.m_linearVelocity;
        const wA = bA.m_angularVelocity;
        const wB = bB.m_angularVelocity;
        // float32 speed = Dot(d, Cross(wA, axis)) + Dot(axis, vB + Cross(wB, rB) - vA - Cross(wA, rA));
        const speed = Vec2.DotVV(d, Vec2.CrossSV(wA, axis, Vec2.s_t0)) +
            Vec2.DotVV(axis, Vec2.SubVV(Vec2.AddVCrossSV(vB, wB, rB, Vec2.s_t0), Vec2.AddVCrossSV(vA, wA, rA, Vec2.s_t1), Vec2.s_t0));
        return speed;
    }
    IsLimitEnabled() {
        return this.m_enableLimit;
    }
    EnableLimit(flag) {
        if (flag !== this.m_enableLimit) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableLimit = flag;
            this.m_impulse.z = 0;
        }
    }
    GetLowerLimit() {
        return this.m_lowerTranslation;
    }
    GetUpperLimit() {
        return this.m_upperTranslation;
    }
    SetLimits(lower, upper) {
        if (lower !== this.m_lowerTranslation ||
            upper !== this.m_upperTranslation) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_lowerTranslation = lower;
            this.m_upperTranslation = upper;
            this.m_impulse.z = 0;
        }
    }
    IsMotorEnabled() {
        return this.m_enableMotor;
    }
    EnableMotor(flag) {
        if (flag !== this.m_enableMotor) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_enableMotor = flag;
        }
    }
    SetMotorSpeed(speed) {
        if (speed !== this.m_motorSpeed) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_motorSpeed = speed;
        }
    }
    GetMotorSpeed() {
        return this.m_motorSpeed;
    }
    SetMaxMotorForce(force) {
        if (force !== this.m_maxMotorForce) {
            this.m_bodyA.SetAwake(true);
            this.m_bodyB.SetAwake(true);
            this.m_maxMotorForce = force;
        }
    }
    GetMaxMotorForce() {
        return this.m_maxMotorForce;
    }
    GetMotorForce(inv_dt) {
        return inv_dt * this.m_motorImpulse;
    }
    Dump(log) {
        const indexA = this.m_bodyA.m_islandIndex;
        const indexB = this.m_bodyB.m_islandIndex;
        log('  const jd: PrismaticJointDef = new PrismaticJointDef();\n');
        log('  jd.bodyA = bodies[%d];\n', indexA);
        log('  jd.bodyB = bodies[%d];\n', indexB);
        log('  jd.collideConnected = %s;\n', this.m_collideConnected ? 'true' : 'false');
        log('  jd.localAnchorA.Set(%.15f, %.15f);\n', this.m_localAnchorA.x, this.m_localAnchorA.y);
        log('  jd.localAnchorB.Set(%.15f, %.15f);\n', this.m_localAnchorB.x, this.m_localAnchorB.y);
        log('  jd.localAxisA.Set(%.15f, %.15f);\n', this.m_localXAxisA.x, this.m_localXAxisA.y);
        log('  jd.referenceAngle = %.15f;\n', this.m_referenceAngle);
        log('  jd.enableLimit = %s;\n', this.m_enableLimit ? 'true' : 'false');
        log('  jd.lowerTranslation = %.15f;\n', this.m_lowerTranslation);
        log('  jd.upperTranslation = %.15f;\n', this.m_upperTranslation);
        log('  jd.enableMotor = %s;\n', this.m_enableMotor ? 'true' : 'false');
        log('  jd.motorSpeed = %.15f;\n', this.m_motorSpeed);
        log('  jd.maxMotorForce = %.15f;\n', this.m_maxMotorForce);
        log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index);
    }
}
PrismaticJoint.InitVelocityConstraints_s_d = new Vec2();
PrismaticJoint.InitVelocityConstraints_s_P = new Vec2();
PrismaticJoint.SolveVelocityConstraints_s_P = new Vec2();
PrismaticJoint.SolveVelocityConstraints_s_f2r = new Vec2();
PrismaticJoint.SolveVelocityConstraints_s_f1 = new Vec3();
PrismaticJoint.SolveVelocityConstraints_s_df3 = new Vec3();
PrismaticJoint.SolveVelocityConstraints_s_df2 = new Vec2();
// A velocity based solver computes reaction forces(impulses) using the velocity constraint solver.Under this context,
// the position solver is not there to resolve forces.It is only there to cope with integration error.
//
// Therefore, the pseudo impulses in the position solver do not have any physical meaning.Thus it is okay if they suck.
//
// We could take the active state from the velocity solver.However, the joint might push past the limit when the velocity
// solver indicates the limit is inactive.
PrismaticJoint.SolvePositionConstraints_s_d = new Vec2();
PrismaticJoint.SolvePositionConstraints_s_impulse = new Vec3();
PrismaticJoint.SolvePositionConstraints_s_impulse1 = new Vec2();
PrismaticJoint.SolvePositionConstraints_s_P = new Vec2();
PrismaticJoint.GetJointTranslation_s_pA = new Vec2();
PrismaticJoint.GetJointTranslation_s_pB = new Vec2();
PrismaticJoint.GetJointTranslation_s_d = new Vec2();
PrismaticJoint.GetJointTranslation_s_axis = new Vec2();
//# sourceMappingURL=PrismaticJoint.js.map