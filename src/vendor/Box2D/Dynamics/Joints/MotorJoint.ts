/*
 * Copyright (c) 2006-2012 Erin Catto http://www.box2d.org
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
import { Clamp, Mat22, Rot, Vec2, XY } from '../../Common/Math'
import { Maybe } from '../../Common/Settings'
import { Body } from '../Body'
import { SolverData } from '../TimeStep'

import { IJointDef, Joint, JointDef, JointType } from './Joint'

// Point-to-point constraint
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)
//
// r1 = offset - c1
// r2 = -c2

// Angle constraint
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

export interface IMotorJointDef extends IJointDef {
  linearOffset?: XY

  angularOffset?: number

  maxForce?: number

  maxTorque?: number

  correctionFactor?: number
}

export class MotorJointDef extends JointDef implements IMotorJointDef {
  readonly linearOffset: Vec2 = new Vec2(0, 0)

  angularOffset: number = 0

  maxForce: number = 1

  maxTorque: number = 1

  correctionFactor: number = 0.3

  constructor() {
    super(JointType.e_motorJoint)
  }

  Initialize(bA: Body, bB: Body): void {
    this.bodyA = bA
    this.bodyB = bB
    // Vec2 xB = bodyB->GetPosition();
    // linearOffset = bodyA->GetLocalPoint(xB);
    this.bodyA.GetLocalPoint(this.bodyB.GetPosition(), this.linearOffset)

    const angleA: number = this.bodyA.GetAngle()
    const angleB: number = this.bodyB.GetAngle()
    this.angularOffset = angleB - angleA
  }
}

export class MotorJoint extends Joint {
  private static SolveVelocityConstraints_s_Cdot_v2 = new Vec2()
  private static SolveVelocityConstraints_s_impulse_v2 = new Vec2()
  private static SolveVelocityConstraints_s_oldImpulse_v2 = new Vec2()
  // Solver shared
  readonly m_linearOffset: Vec2 = new Vec2()
  m_angularOffset: number = 0
  readonly m_linearImpulse: Vec2 = new Vec2()
  m_angularImpulse: number = 0
  m_maxForce: number = 0
  m_maxTorque: number = 0
  m_correctionFactor: number = 0.3

  // Solver temp
  m_indexA: number = 0
  m_indexB: number = 0
  readonly m_rA: Vec2 = new Vec2()
  readonly m_rB: Vec2 = new Vec2()
  readonly m_localCenterA: Vec2 = new Vec2()
  readonly m_localCenterB: Vec2 = new Vec2()
  readonly m_linearError: Vec2 = new Vec2()
  m_angularError: number = 0
  m_invMassA: number = 0
  m_invMassB: number = 0
  m_invIA: number = 0
  m_invIB: number = 0
  readonly m_linearMass: Mat22 = new Mat22()
  m_angularMass: number = 0

  readonly m_qA: Rot = new Rot()
  readonly m_qB: Rot = new Rot()
  readonly m_K: Mat22 = new Mat22()

  constructor(def: IMotorJointDef) {
    super(def)

    this.m_linearOffset.Copy(Maybe(def.linearOffset, Vec2.ZERO))
    this.m_linearImpulse.SetZero()
    this.m_maxForce = Maybe(def.maxForce, 0)
    this.m_maxTorque = Maybe(def.maxTorque, 0)
    this.m_correctionFactor = Maybe(def.correctionFactor, 0.3)
  }

  GetAnchorA<T extends XY>(out: T): T {
    const pos: Readonly<Vec2> = this.m_bodyA.GetPosition()
    out.x = pos.x
    out.y = pos.y
    return out
  }
  GetAnchorB<T extends XY>(out: T): T {
    const pos: Readonly<Vec2> = this.m_bodyB.GetPosition()
    out.x = pos.x
    out.y = pos.y
    return out
  }

  GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
    // return inv_dt * m_linearImpulse;
    return Vec2.MulSV(inv_dt, this.m_linearImpulse, out)
  }

  GetReactionTorque(inv_dt: number): number {
    return inv_dt * this.m_angularImpulse
  }

  SetLinearOffset(linearOffset: Vec2): void {
    if (!Vec2.IsEqualToV(linearOffset, this.m_linearOffset)) {
      this.m_bodyA.SetAwake(true)
      this.m_bodyB.SetAwake(true)
      this.m_linearOffset.Copy(linearOffset)
    }
  }
  GetLinearOffset() {
    return this.m_linearOffset
  }

  SetAngularOffset(angularOffset: number): void {
    if (angularOffset !== this.m_angularOffset) {
      this.m_bodyA.SetAwake(true)
      this.m_bodyB.SetAwake(true)
      this.m_angularOffset = angularOffset
    }
  }
  GetAngularOffset() {
    return this.m_angularOffset
  }

  SetMaxForce(force: number): void {
    // DEBUG: Assert(IsValid(force) && force >= 0);
    this.m_maxForce = force
  }

  GetMaxForce() {
    return this.m_maxForce
  }

  SetMaxTorque(torque: number): void {
    // DEBUG: Assert(IsValid(torque) && torque >= 0);
    this.m_maxTorque = torque
  }

  GetMaxTorque() {
    return this.m_maxTorque
  }

  InitVelocityConstraints(data: SolverData): void {
    this.m_indexA = this.m_bodyA.m_islandIndex
    this.m_indexB = this.m_bodyB.m_islandIndex
    this.m_localCenterA.Copy(this.m_bodyA.m_sweep.localCenter)
    this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter)
    this.m_invMassA = this.m_bodyA.m_invMass
    this.m_invMassB = this.m_bodyB.m_invMass
    this.m_invIA = this.m_bodyA.m_invI
    this.m_invIB = this.m_bodyB.m_invI

    const cA: Vec2 = data.positions[this.m_indexA].c
    const aA: number = data.positions[this.m_indexA].a
    const vA: Vec2 = data.velocities[this.m_indexA].v
    let wA: number = data.velocities[this.m_indexA].w

    const cB: Vec2 = data.positions[this.m_indexB].c
    const aB: number = data.positions[this.m_indexB].a
    const vB: Vec2 = data.velocities[this.m_indexB].v
    let wB: number = data.velocities[this.m_indexB].w

    const qA: Rot = this.m_qA.SetAngle(aA)
    const qB: Rot = this.m_qB.SetAngle(aB)

    // Compute the effective mass matrix.
    // this.m_rA = Mul(qA, m_linearOffset - this.m_localCenterA);
    const rA: Vec2 = Rot.MulRV(
      qA,
      Vec2.SubVV(this.m_linearOffset, this.m_localCenterA, Vec2.s_t0),
      this.m_rA
    )
    // this.m_rB = Mul(qB, -this.m_localCenterB);
    const rB: Vec2 = Rot.MulRV(
      qB,
      Vec2.NegV(this.m_localCenterB, Vec2.s_t0),
      this.m_rB
    )

    // J = [-I -r1_skew I r2_skew]
    // r_skew = [-ry; rx]

    // Matlab
    // K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
    //     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
    //     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

    const mA: number = this.m_invMassA
    const mB: number = this.m_invMassB
    const iA: number = this.m_invIA
    const iB: number = this.m_invIB

    // Upper 2 by 2 of K for point to point
    const K: Mat22 = this.m_K
    K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y
    K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y
    K.ey.x = K.ex.y
    K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x

    // this.m_linearMass = K.GetInverse();
    K.GetInverse(this.m_linearMass)

    this.m_angularMass = iA + iB
    if (this.m_angularMass > 0) {
      this.m_angularMass = 1 / this.m_angularMass
    }

    // this.m_linearError = cB + rB - cA - rA;
    Vec2.SubVV(
      Vec2.AddVV(cB, rB, Vec2.s_t0),
      Vec2.AddVV(cA, rA, Vec2.s_t1),
      this.m_linearError
    )
    this.m_angularError = aB - aA - this.m_angularOffset

    if (data.step.warmStarting) {
      // Scale impulses to support a variable time step.
      // this.m_linearImpulse *= data.step.dtRatio;
      this.m_linearImpulse.SelfMul(data.step.dtRatio)
      this.m_angularImpulse *= data.step.dtRatio

      // Vec2 P(this.m_linearImpulse.x, this.m_linearImpulse.y);
      const P: Vec2 = this.m_linearImpulse
      // vA -= mA * P;
      vA.SelfMulSub(mA, P)
      wA -= iA * (Vec2.CrossVV(rA, P) + this.m_angularImpulse)
      // vB += mB * P;
      vB.SelfMulAdd(mB, P)
      wB += iB * (Vec2.CrossVV(rB, P) + this.m_angularImpulse)
    } else {
      this.m_linearImpulse.SetZero()
      this.m_angularImpulse = 0
    }

    // data.velocities[this.m_indexA].v = vA; // vA is a reference
    data.velocities[this.m_indexA].w = wA
    // data.velocities[this.m_indexB].v = vB; // vB is a reference
    data.velocities[this.m_indexB].w = wB
  }
  SolveVelocityConstraints(data: SolverData): void {
    const vA: Vec2 = data.velocities[this.m_indexA].v
    let wA: number = data.velocities[this.m_indexA].w
    const vB: Vec2 = data.velocities[this.m_indexB].v
    let wB: number = data.velocities[this.m_indexB].w

    const mA: number = this.m_invMassA
    const mB: number = this.m_invMassB
    const iA: number = this.m_invIA
    const iB: number = this.m_invIB

    const h: number = data.step.dt
    const inv_h: number = data.step.inv_dt

    // Solve angular friction
    {
      const Cdot: number =
        wB - wA + inv_h * this.m_correctionFactor * this.m_angularError
      let impulse: number = -this.m_angularMass * Cdot

      const oldImpulse: number = this.m_angularImpulse
      const maxImpulse: number = h * this.m_maxTorque
      this.m_angularImpulse = Clamp(
        this.m_angularImpulse + impulse,
        -maxImpulse,
        maxImpulse
      )
      impulse = this.m_angularImpulse - oldImpulse

      wA -= iA * impulse
      wB += iB * impulse
    }

    // Solve linear friction
    {
      const rA = this.m_rA
      const rB = this.m_rB

      // Vec2 Cdot = vB + Vec2.CrossSV(wB, rB) - vA - Vec2.CrossSV(wA, rA) + inv_h * this.m_correctionFactor * this.m_linearError;
      const Cdot_v2 = Vec2.AddVV(
        Vec2.SubVV(
          Vec2.AddVV(vB, Vec2.CrossSV(wB, rB, Vec2.s_t0), Vec2.s_t0),
          Vec2.AddVV(vA, Vec2.CrossSV(wA, rA, Vec2.s_t1), Vec2.s_t1),
          Vec2.s_t2
        ),
        Vec2.MulSV(
          inv_h * this.m_correctionFactor,
          this.m_linearError,
          Vec2.s_t3
        ),
        MotorJoint.SolveVelocityConstraints_s_Cdot_v2
      )

      // Vec2 impulse = -Mul(this.m_linearMass, Cdot);
      const impulse_v2: Vec2 = Mat22.MulMV(
        this.m_linearMass,
        Cdot_v2,
        MotorJoint.SolveVelocityConstraints_s_impulse_v2
      ).SelfNeg()
      // Vec2 oldImpulse = this.m_linearImpulse;
      const oldImpulse_v2 = MotorJoint.SolveVelocityConstraints_s_oldImpulse_v2.Copy(
        this.m_linearImpulse
      )
      // this.m_linearImpulse += impulse;
      this.m_linearImpulse.SelfAdd(impulse_v2)

      const maxImpulse: number = h * this.m_maxForce

      if (this.m_linearImpulse.LengthSquared() > maxImpulse * maxImpulse) {
        this.m_linearImpulse.Normalize()
        // this.m_linearImpulse *= maxImpulse;
        this.m_linearImpulse.SelfMul(maxImpulse)
      }

      // impulse = this.m_linearImpulse - oldImpulse;
      Vec2.SubVV(this.m_linearImpulse, oldImpulse_v2, impulse_v2)

      // vA -= mA * impulse;
      vA.SelfMulSub(mA, impulse_v2)
      // wA -= iA * Vec2.CrossVV(rA, impulse);
      wA -= iA * Vec2.CrossVV(rA, impulse_v2)

      // vB += mB * impulse;
      vB.SelfMulAdd(mB, impulse_v2)
      // wB += iB * Vec2.CrossVV(rB, impulse);
      wB += iB * Vec2.CrossVV(rB, impulse_v2)
    }

    // data.velocities[this.m_indexA].v = vA; // vA is a reference
    data.velocities[this.m_indexA].w = wA
    // data.velocities[this.m_indexB].v = vB; // vB is a reference
    data.velocities[this.m_indexB].w = wB
  }

  SolvePositionConstraints(data: SolverData): boolean {
    return true
  }

  Dump(log: (format: string, ...args: any[]) => void) {
    const indexA = this.m_bodyA.m_islandIndex
    const indexB = this.m_bodyB.m_islandIndex

    log('  const jd: MotorJointDef = new MotorJointDef();\n')

    log('  jd.bodyA = bodies[%d];\n', indexA)
    log('  jd.bodyB = bodies[%d];\n', indexB)
    log(
      '  jd.collideConnected = %s;\n',
      this.m_collideConnected ? 'true' : 'false'
    )

    log(
      '  jd.linearOffset.Set(%.15f, %.15f);\n',
      this.m_linearOffset.x,
      this.m_linearOffset.y
    )
    log('  jd.angularOffset = %.15f;\n', this.m_angularOffset)
    log('  jd.maxForce = %.15f;\n', this.m_maxForce)
    log('  jd.maxTorque = %.15f;\n', this.m_maxTorque)
    log('  jd.correctionFactor = %.15f;\n', this.m_correctionFactor)
    log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index)
  }
}
