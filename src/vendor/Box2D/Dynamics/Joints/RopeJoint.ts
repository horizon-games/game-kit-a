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

import { Clamp, Min, Rot, Vec2, XY } from '../../Common/Math'
import { linearSlop, maxLinearCorrection, Maybe } from '../../Common/Settings'
import { SolverData } from '../TimeStep'

import { IJointDef, Joint, JointDef, JointType, LimitState } from './Joint'

export interface IRopeJointDef extends IJointDef {
  localAnchorA?: XY

  localAnchorB?: XY

  maxLength?: number
}

/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in JointDef.
export class RopeJointDef extends JointDef implements IRopeJointDef {
  readonly localAnchorA: Vec2 = new Vec2(-1, 0)

  readonly localAnchorB: Vec2 = new Vec2(1, 0)

  maxLength: number = 0

  constructor() {
    super(JointType.e_ropeJoint)
  }
}

export class RopeJoint extends Joint {
  private static InitVelocityConstraints_s_P = new Vec2()

  private static SolveVelocityConstraints_s_vpA = new Vec2()
  private static SolveVelocityConstraints_s_vpB = new Vec2()
  private static SolveVelocityConstraints_s_P = new Vec2()

  private static SolvePositionConstraints_s_P = new Vec2()
  // Solver shared
  readonly m_localAnchorA: Vec2 = new Vec2()
  readonly m_localAnchorB: Vec2 = new Vec2()
  m_maxLength: number = 0
  m_length: number = 0
  m_impulse: number = 0

  // Solver temp
  m_indexA: number = 0
  m_indexB: number = 0
  readonly m_u: Vec2 = new Vec2()
  readonly m_rA: Vec2 = new Vec2()
  readonly m_rB: Vec2 = new Vec2()
  readonly m_localCenterA: Vec2 = new Vec2()
  readonly m_localCenterB: Vec2 = new Vec2()
  m_invMassA: number = 0
  m_invMassB: number = 0
  m_invIA: number = 0
  m_invIB: number = 0
  m_mass: number = 0
  m_state = LimitState.e_inactiveLimit

  readonly m_qA: Rot = new Rot()
  readonly m_qB: Rot = new Rot()
  readonly m_lalcA: Vec2 = new Vec2()
  readonly m_lalcB: Vec2 = new Vec2()

  constructor(def: IRopeJointDef) {
    super(def)

    this.m_localAnchorA.Copy(Maybe(def.localAnchorA, new Vec2(-1, 0)))
    this.m_localAnchorB.Copy(Maybe(def.localAnchorB, new Vec2(1, 0)))
    this.m_maxLength = Maybe(def.maxLength, 0)
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

    // this.m_rA = Mul(qA, this.m_localAnchorA - this.m_localCenterA);
    Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA)
    Rot.MulRV(qA, this.m_lalcA, this.m_rA)
    // this.m_rB = Mul(qB, this.m_localAnchorB - this.m_localCenterB);
    Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB)
    Rot.MulRV(qB, this.m_lalcB, this.m_rB)
    // this.m_u = cB + this.m_rB - cA - this.m_rA;
    this.m_u
      .Copy(cB)
      .SelfAdd(this.m_rB)
      .SelfSub(cA)
      .SelfSub(this.m_rA)

    this.m_length = this.m_u.Length()

    const C: number = this.m_length - this.m_maxLength
    this.m_state =
      C > 0 ? LimitState.e_atUpperLimit : LimitState.e_inactiveLimit

    if (this.m_length > linearSlop) {
      this.m_u.SelfMul(1 / this.m_length)
    } else {
      this.m_u.SetZero()
      this.m_mass = 0
      this.m_impulse = 0
      return
    }

    // Compute effective mass.
    const crA: number = Vec2.CrossVV(this.m_rA, this.m_u)
    const crB: number = Vec2.CrossVV(this.m_rB, this.m_u)
    const invMass: number =
      this.m_invMassA +
      this.m_invIA * crA * crA +
      this.m_invMassB +
      this.m_invIB * crB * crB

    this.m_mass = invMass !== 0 ? 1 / invMass : 0

    if (data.step.warmStarting) {
      // Scale the impulse to support a variable time step.
      this.m_impulse *= data.step.dtRatio

      // Vec2 P = m_impulse * m_u;
      const P: Vec2 = Vec2.MulSV(
        this.m_impulse,
        this.m_u,
        RopeJoint.InitVelocityConstraints_s_P
      )
      // vA -= m_invMassA * P;
      vA.SelfMulSub(this.m_invMassA, P)
      wA -= this.m_invIA * Vec2.CrossVV(this.m_rA, P)
      // vB += m_invMassB * P;
      vB.SelfMulAdd(this.m_invMassB, P)
      wB += this.m_invIB * Vec2.CrossVV(this.m_rB, P)
    } else {
      this.m_impulse = 0
    }

    // data.velocities[this.m_indexA].v = vA;
    data.velocities[this.m_indexA].w = wA
    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB
  }
  SolveVelocityConstraints(data: SolverData): void {
    const vA: Vec2 = data.velocities[this.m_indexA].v
    let wA: number = data.velocities[this.m_indexA].w
    const vB: Vec2 = data.velocities[this.m_indexB].v
    let wB: number = data.velocities[this.m_indexB].w

    // Cdot = dot(u, v + cross(w, r))
    // Vec2 vpA = vA + Cross(wA, m_rA);
    const vpA: Vec2 = Vec2.AddVCrossSV(
      vA,
      wA,
      this.m_rA,
      RopeJoint.SolveVelocityConstraints_s_vpA
    )
    // Vec2 vpB = vB + Cross(wB, m_rB);
    const vpB: Vec2 = Vec2.AddVCrossSV(
      vB,
      wB,
      this.m_rB,
      RopeJoint.SolveVelocityConstraints_s_vpB
    )
    // float32 C = m_length - m_maxLength;
    const C: number = this.m_length - this.m_maxLength
    // float32 Cdot = Dot(m_u, vpB - vpA);
    let Cdot: number = Vec2.DotVV(this.m_u, Vec2.SubVV(vpB, vpA, Vec2.s_t0))

    // Predictive constraint.
    if (C < 0) {
      Cdot += data.step.inv_dt * C
    }

    let impulse: number = -this.m_mass * Cdot
    const oldImpulse: number = this.m_impulse
    this.m_impulse = Min(0, this.m_impulse + impulse)
    impulse = this.m_impulse - oldImpulse

    // Vec2 P = impulse * m_u;
    const P: Vec2 = Vec2.MulSV(
      impulse,
      this.m_u,
      RopeJoint.SolveVelocityConstraints_s_P
    )
    // vA -= m_invMassA * P;
    vA.SelfMulSub(this.m_invMassA, P)
    wA -= this.m_invIA * Vec2.CrossVV(this.m_rA, P)
    // vB += m_invMassB * P;
    vB.SelfMulAdd(this.m_invMassB, P)
    wB += this.m_invIB * Vec2.CrossVV(this.m_rB, P)

    // data.velocities[this.m_indexA].v = vA;
    data.velocities[this.m_indexA].w = wA
    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB
  }
  SolvePositionConstraints(data: SolverData): boolean {
    const cA: Vec2 = data.positions[this.m_indexA].c
    let aA: number = data.positions[this.m_indexA].a
    const cB: Vec2 = data.positions[this.m_indexB].c
    let aB: number = data.positions[this.m_indexB].a

    const qA: Rot = this.m_qA.SetAngle(aA)
    const qB: Rot = this.m_qB.SetAngle(aB)

    // Vec2 rA = Mul(qA, this.m_localAnchorA - this.m_localCenterA);
    Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA)
    const rA: Vec2 = Rot.MulRV(qA, this.m_lalcA, this.m_rA)
    // Vec2 rB = Mul(qB, this.m_localAnchorB - this.m_localCenterB);
    Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB)
    const rB: Vec2 = Rot.MulRV(qB, this.m_lalcB, this.m_rB)
    // Vec2 u = cB + rB - cA - rA;
    const u: Vec2 = this.m_u
      .Copy(cB)
      .SelfAdd(rB)
      .SelfSub(cA)
      .SelfSub(rA)

    const length: number = u.Normalize()
    let C: number = length - this.m_maxLength

    C = Clamp(C, 0, maxLinearCorrection)

    const impulse: number = -this.m_mass * C
    // Vec2 P = impulse * u;
    const P: Vec2 = Vec2.MulSV(
      impulse,
      u,
      RopeJoint.SolvePositionConstraints_s_P
    )

    // cA -= m_invMassA * P;
    cA.SelfMulSub(this.m_invMassA, P)
    aA -= this.m_invIA * Vec2.CrossVV(rA, P)
    // cB += m_invMassB * P;
    cB.SelfMulAdd(this.m_invMassB, P)
    aB += this.m_invIB * Vec2.CrossVV(rB, P)

    // data.positions[this.m_indexA].c = cA;
    data.positions[this.m_indexA].a = aA
    // data.positions[this.m_indexB].c = cB;
    data.positions[this.m_indexB].a = aB

    return length - this.m_maxLength < linearSlop
  }

  GetAnchorA<T extends XY>(out: T): T {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out)
  }

  GetAnchorB<T extends XY>(out: T): T {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out)
  }

  GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
    // return out.Set(inv_dt * this.m_linearImpulse.x, inv_dt * this.m_linearImpulse.y);
    return Vec2.MulSV(inv_dt * this.m_impulse, this.m_u, out)
  }

  GetReactionTorque(inv_dt: number): number {
    return 0
  }

  GetLocalAnchorA(): Readonly<Vec2> {
    return this.m_localAnchorA
  }

  GetLocalAnchorB(): Readonly<Vec2> {
    return this.m_localAnchorB
  }

  SetMaxLength(length: number): void {
    this.m_maxLength = length
  }
  GetMaxLength(): number {
    return this.m_maxLength
  }

  GetLimitState(): LimitState {
    return this.m_state
  }

  Dump(log: (format: string, ...args: any[]) => void): void {
    const indexA = this.m_bodyA.m_islandIndex
    const indexB = this.m_bodyB.m_islandIndex

    log('  const jd: RopeJointDef = new RopeJointDef();\n')
    log('  jd.bodyA = bodies[%d];\n', indexA)
    log('  jd.bodyB = bodies[%d];\n', indexB)
    log(
      '  jd.collideConnected = %s;\n',
      this.m_collideConnected ? 'true' : 'false'
    )
    log(
      '  jd.localAnchorA.Set(%.15f, %.15f);\n',
      this.m_localAnchorA.x,
      this.m_localAnchorA.y
    )
    log(
      '  jd.localAnchorB.Set(%.15f, %.15f);\n',
      this.m_localAnchorB.x,
      this.m_localAnchorB.y
    )
    log('  jd.maxLength = %.15f;\n', this.m_maxLength)
    log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index)
  }
}
