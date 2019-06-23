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
import { Abs, Clamp, Rot, Vec2, XY } from '../../Common/Math'
import { linearSlop, Maybe, pi } from '../../Common/Settings'
import { Body } from '../Body'
import { SolverData } from '../TimeStep'

import { IJointDef, Joint, JointDef, JointType } from './Joint'

export interface IWheelJointDef extends IJointDef {
  localAnchorA?: XY

  localAnchorB?: XY

  localAxisA?: XY

  enableMotor?: boolean

  maxMotorTorque?: number

  motorSpeed?: number

  frequencyHz?: number

  dampingRatio?: number
}

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
export class WheelJointDef extends JointDef implements IWheelJointDef {
  readonly localAnchorA: Vec2 = new Vec2(0, 0)

  readonly localAnchorB: Vec2 = new Vec2(0, 0)

  readonly localAxisA: Vec2 = new Vec2(1, 0)

  enableMotor = false

  maxMotorTorque: number = 0

  motorSpeed: number = 0

  frequencyHz: number = 2

  dampingRatio: number = 0.7

  constructor() {
    super(JointType.e_wheelJoint)
  }

  Initialize(bA: Body, bB: Body, anchor: Vec2, axis: Vec2): void {
    this.bodyA = bA
    this.bodyB = bB
    this.bodyA.GetLocalPoint(anchor, this.localAnchorA)
    this.bodyB.GetLocalPoint(anchor, this.localAnchorB)
    this.bodyA.GetLocalVector(axis, this.localAxisA)
  }
}

export class WheelJoint extends Joint {
  private static InitVelocityConstraints_s_d = new Vec2()
  private static InitVelocityConstraints_s_P = new Vec2()

  private static SolveVelocityConstraints_s_P = new Vec2()

  private static SolvePositionConstraints_s_d = new Vec2()
  private static SolvePositionConstraints_s_P = new Vec2()
  m_frequencyHz: number = 0
  m_dampingRatio: number = 0

  // Solver shared
  readonly m_localAnchorA: Vec2 = new Vec2()
  readonly m_localAnchorB: Vec2 = new Vec2()
  readonly m_localXAxisA: Vec2 = new Vec2()
  readonly m_localYAxisA: Vec2 = new Vec2()

  m_impulse: number = 0
  m_motorImpulse: number = 0
  m_springImpulse: number = 0

  m_maxMotorTorque: number = 0
  m_motorSpeed: number = 0
  m_enableMotor = false

  // Solver temp
  m_indexA: number = 0
  m_indexB: number = 0
  readonly m_localCenterA: Vec2 = new Vec2()
  readonly m_localCenterB: Vec2 = new Vec2()
  m_invMassA: number = 0
  m_invMassB: number = 0
  m_invIA: number = 0
  m_invIB: number = 0

  readonly m_ax: Vec2 = new Vec2()
  readonly m_ay: Vec2 = new Vec2()
  m_sAx: number = 0
  m_sBx: number = 0
  m_sAy: number = 0
  m_sBy: number = 0

  m_mass: number = 0
  m_motorMass: number = 0
  m_springMass: number = 0

  m_bias: number = 0
  m_gamma: number = 0

  readonly m_qA: Rot = new Rot()
  readonly m_qB: Rot = new Rot()
  readonly m_lalcA: Vec2 = new Vec2()
  readonly m_lalcB: Vec2 = new Vec2()
  readonly m_rA: Vec2 = new Vec2()
  readonly m_rB: Vec2 = new Vec2()

  constructor(def: IWheelJointDef) {
    super(def)

    this.m_frequencyHz = Maybe(def.frequencyHz, 2)
    this.m_dampingRatio = Maybe(def.dampingRatio, 0.7)

    this.m_localAnchorA.Copy(Maybe(def.localAnchorA, Vec2.ZERO))
    this.m_localAnchorB.Copy(Maybe(def.localAnchorB, Vec2.ZERO))
    this.m_localXAxisA.Copy(Maybe(def.localAxisA, Vec2.UNITX))
    Vec2.CrossOneV(this.m_localXAxisA, this.m_localYAxisA)

    this.m_maxMotorTorque = Maybe(def.maxMotorTorque, 0)
    this.m_motorSpeed = Maybe(def.motorSpeed, 0)
    this.m_enableMotor = Maybe(def.enableMotor, false)

    this.m_ax.SetZero()
    this.m_ay.SetZero()
  }

  GetMotorSpeed(): number {
    return this.m_motorSpeed
  }

  GetMaxMotorTorque(): number {
    return this.m_maxMotorTorque
  }

  SetSpringFrequencyHz(hz: number): void {
    this.m_frequencyHz = hz
  }

  GetSpringFrequencyHz(): number {
    return this.m_frequencyHz
  }

  SetSpringDampingRatio(ratio: number): void {
    this.m_dampingRatio = ratio
  }

  GetSpringDampingRatio(): number {
    return this.m_dampingRatio
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

    const mA: number = this.m_invMassA
    const mB: number = this.m_invMassB
    const iA: number = this.m_invIA
    const iB: number = this.m_invIB

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

    // Compute the effective masses.
    // Vec2 rA = Mul(qA, m_localAnchorA - m_localCenterA);
    Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA)
    const rA: Vec2 = Rot.MulRV(qA, this.m_lalcA, this.m_rA)
    // Vec2 rB = Mul(qB, m_localAnchorB - m_localCenterB);
    Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB)
    const rB: Vec2 = Rot.MulRV(qB, this.m_lalcB, this.m_rB)
    // Vec2 d = cB + rB - cA - rA;
    const d: Vec2 = Vec2.SubVV(
      Vec2.AddVV(cB, rB, Vec2.s_t0),
      Vec2.AddVV(cA, rA, Vec2.s_t1),
      WheelJoint.InitVelocityConstraints_s_d
    )

    // Point to line constraint
    {
      // m_ay = Mul(qA, m_localYAxisA);
      Rot.MulRV(qA, this.m_localYAxisA, this.m_ay)
      // m_sAy = Cross(d + rA, m_ay);
      this.m_sAy = Vec2.CrossVV(Vec2.AddVV(d, rA, Vec2.s_t0), this.m_ay)
      // m_sBy = Cross(rB, m_ay);
      this.m_sBy = Vec2.CrossVV(rB, this.m_ay)

      this.m_mass =
        mA + mB + iA * this.m_sAy * this.m_sAy + iB * this.m_sBy * this.m_sBy

      if (this.m_mass > 0) {
        this.m_mass = 1 / this.m_mass
      }
    }

    // Spring constraint
    this.m_springMass = 0
    this.m_bias = 0
    this.m_gamma = 0
    if (this.m_frequencyHz > 0) {
      // m_ax = Mul(qA, m_localXAxisA);
      Rot.MulRV(qA, this.m_localXAxisA, this.m_ax)
      // m_sAx = Cross(d + rA, m_ax);
      this.m_sAx = Vec2.CrossVV(Vec2.AddVV(d, rA, Vec2.s_t0), this.m_ax)
      // m_sBx = Cross(rB, m_ax);
      this.m_sBx = Vec2.CrossVV(rB, this.m_ax)

      const invMass: number =
        mA + mB + iA * this.m_sAx * this.m_sAx + iB * this.m_sBx * this.m_sBx

      if (invMass > 0) {
        this.m_springMass = 1 / invMass

        const C: number = Vec2.DotVV(d, this.m_ax)

        // Frequency
        const omega: number = 2 * pi * this.m_frequencyHz

        // Damping coefficient
        const damp: number = 2 * this.m_springMass * this.m_dampingRatio * omega

        // Spring stiffness
        const k: number = this.m_springMass * omega * omega

        // magic formulas
        const h: number = data.step.dt
        this.m_gamma = h * (damp + h * k)
        if (this.m_gamma > 0) {
          this.m_gamma = 1 / this.m_gamma
        }

        this.m_bias = C * h * k * this.m_gamma

        this.m_springMass = invMass + this.m_gamma
        if (this.m_springMass > 0) {
          this.m_springMass = 1 / this.m_springMass
        }
      }
    } else {
      this.m_springImpulse = 0
    }

    // Rotational motor
    if (this.m_enableMotor) {
      this.m_motorMass = iA + iB
      if (this.m_motorMass > 0) {
        this.m_motorMass = 1 / this.m_motorMass
      }
    } else {
      this.m_motorMass = 0
      this.m_motorImpulse = 0
    }

    if (data.step.warmStarting) {
      // Account for variable time step.
      this.m_impulse *= data.step.dtRatio
      this.m_springImpulse *= data.step.dtRatio
      this.m_motorImpulse *= data.step.dtRatio

      // Vec2 P = m_impulse * m_ay + m_springImpulse * m_ax;
      const P: Vec2 = Vec2.AddVV(
        Vec2.MulSV(this.m_impulse, this.m_ay, Vec2.s_t0),
        Vec2.MulSV(this.m_springImpulse, this.m_ax, Vec2.s_t1),
        WheelJoint.InitVelocityConstraints_s_P
      )
      // float32 LA = m_impulse * m_sAy + m_springImpulse * m_sAx + m_motorImpulse;
      const LA: number =
        this.m_impulse * this.m_sAy +
        this.m_springImpulse * this.m_sAx +
        this.m_motorImpulse
      // float32 LB = m_impulse * m_sBy + m_springImpulse * m_sBx + m_motorImpulse;
      const LB: number =
        this.m_impulse * this.m_sBy +
        this.m_springImpulse * this.m_sBx +
        this.m_motorImpulse

      // vA -= m_invMassA * P;
      vA.SelfMulSub(this.m_invMassA, P)
      wA -= this.m_invIA * LA

      // vB += m_invMassB * P;
      vB.SelfMulAdd(this.m_invMassB, P)
      wB += this.m_invIB * LB
    } else {
      this.m_impulse = 0
      this.m_springImpulse = 0
      this.m_motorImpulse = 0
    }

    // data.velocities[this.m_indexA].v = vA;
    data.velocities[this.m_indexA].w = wA
    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB
  }
  SolveVelocityConstraints(data: SolverData): void {
    const mA: number = this.m_invMassA
    const mB: number = this.m_invMassB
    const iA: number = this.m_invIA
    const iB: number = this.m_invIB

    const vA: Vec2 = data.velocities[this.m_indexA].v
    let wA: number = data.velocities[this.m_indexA].w
    const vB: Vec2 = data.velocities[this.m_indexB].v
    let wB: number = data.velocities[this.m_indexB].w

    // Solve spring constraint
    {
      const Cdot: number =
        Vec2.DotVV(this.m_ax, Vec2.SubVV(vB, vA, Vec2.s_t0)) +
        this.m_sBx * wB -
        this.m_sAx * wA
      const impulse: number =
        -this.m_springMass *
        (Cdot + this.m_bias + this.m_gamma * this.m_springImpulse)
      this.m_springImpulse += impulse

      // Vec2 P = impulse * m_ax;
      const P: Vec2 = Vec2.MulSV(
        impulse,
        this.m_ax,
        WheelJoint.SolveVelocityConstraints_s_P
      )
      const LA: number = impulse * this.m_sAx
      const LB: number = impulse * this.m_sBx

      // vA -= mA * P;
      vA.SelfMulSub(mA, P)
      wA -= iA * LA

      // vB += mB * P;
      vB.SelfMulAdd(mB, P)
      wB += iB * LB
    }

    // Solve rotational motor constraint
    {
      const Cdot: number = wB - wA - this.m_motorSpeed
      let impulse: number = -this.m_motorMass * Cdot

      const oldImpulse: number = this.m_motorImpulse
      const maxImpulse: number = data.step.dt * this.m_maxMotorTorque
      this.m_motorImpulse = Clamp(
        this.m_motorImpulse + impulse,
        -maxImpulse,
        maxImpulse
      )
      impulse = this.m_motorImpulse - oldImpulse

      wA -= iA * impulse
      wB += iB * impulse
    }

    // Solve point to line constraint
    {
      const Cdot: number =
        Vec2.DotVV(this.m_ay, Vec2.SubVV(vB, vA, Vec2.s_t0)) +
        this.m_sBy * wB -
        this.m_sAy * wA
      const impulse: number = -this.m_mass * Cdot
      this.m_impulse += impulse

      // Vec2 P = impulse * m_ay;
      const P: Vec2 = Vec2.MulSV(
        impulse,
        this.m_ay,
        WheelJoint.SolveVelocityConstraints_s_P
      )
      const LA: number = impulse * this.m_sAy
      const LB: number = impulse * this.m_sBy

      // vA -= mA * P;
      vA.SelfMulSub(mA, P)
      wA -= iA * LA

      // vB += mB * P;
      vB.SelfMulAdd(mB, P)
      wB += iB * LB
    }

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

    // Vec2 rA = Mul(qA, m_localAnchorA - m_localCenterA);
    Vec2.SubVV(this.m_localAnchorA, this.m_localCenterA, this.m_lalcA)
    const rA: Vec2 = Rot.MulRV(qA, this.m_lalcA, this.m_rA)
    // Vec2 rB = Mul(qB, m_localAnchorB - m_localCenterB);
    Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB)
    const rB: Vec2 = Rot.MulRV(qB, this.m_lalcB, this.m_rB)
    // Vec2 d = (cB - cA) + rB - rA;
    const d: Vec2 = Vec2.AddVV(
      Vec2.SubVV(cB, cA, Vec2.s_t0),
      Vec2.SubVV(rB, rA, Vec2.s_t1),
      WheelJoint.SolvePositionConstraints_s_d
    )

    // Vec2 ay = Mul(qA, m_localYAxisA);
    const ay: Vec2 = Rot.MulRV(qA, this.m_localYAxisA, this.m_ay)

    // float32 sAy = Cross(d + rA, ay);
    const sAy = Vec2.CrossVV(Vec2.AddVV(d, rA, Vec2.s_t0), ay)
    // float32 sBy = Cross(rB, ay);
    const sBy = Vec2.CrossVV(rB, ay)

    // float32 C = Dot(d, ay);
    const C: number = Vec2.DotVV(d, this.m_ay)

    const k: number =
      this.m_invMassA +
      this.m_invMassB +
      this.m_invIA * this.m_sAy * this.m_sAy +
      this.m_invIB * this.m_sBy * this.m_sBy

    const impulse: number = k !== 0 ? -C / k : 0

    // Vec2 P = impulse * ay;
    const P: Vec2 = Vec2.MulSV(
      impulse,
      ay,
      WheelJoint.SolvePositionConstraints_s_P
    )
    const LA: number = impulse * sAy
    const LB: number = impulse * sBy

    // cA -= m_invMassA * P;
    cA.SelfMulSub(this.m_invMassA, P)
    aA -= this.m_invIA * LA
    // cB += m_invMassB * P;
    cB.SelfMulAdd(this.m_invMassB, P)
    aB += this.m_invIB * LB

    // data.positions[this.m_indexA].c = cA;
    data.positions[this.m_indexA].a = aA
    // data.positions[this.m_indexB].c = cB;
    data.positions[this.m_indexB].a = aB

    return Abs(C) <= linearSlop
  }

  GetDefinition(def: WheelJointDef): WheelJointDef {
    // DEBUG: Assert(false); // TODO
    return def
  }

  GetAnchorA<T extends XY>(out: T): T {
    return this.m_bodyA.GetWorldPoint(this.m_localAnchorA, out)
  }

  GetAnchorB<T extends XY>(out: T): T {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out)
  }

  GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
    // return inv_dt * (m_impulse * m_ay + m_springImpulse * m_ax);
    out.x =
      inv_dt *
      (this.m_impulse * this.m_ay.x + this.m_springImpulse * this.m_ax.x)
    out.y =
      inv_dt *
      (this.m_impulse * this.m_ay.y + this.m_springImpulse * this.m_ax.y)
    return out
  }

  GetReactionTorque(inv_dt: number): number {
    return inv_dt * this.m_motorImpulse
  }

  GetLocalAnchorA(): Readonly<Vec2> {
    return this.m_localAnchorA
  }

  GetLocalAnchorB(): Readonly<Vec2> {
    return this.m_localAnchorB
  }

  GetLocalAxisA(): Readonly<Vec2> {
    return this.m_localXAxisA
  }

  GetJointTranslation(): number {
    return this.GetPrismaticJointTranslation()
  }

  GetJointLinearSpeed(): number {
    return this.GetPrismaticJointSpeed()
  }

  GetJointAngle(): number {
    return this.GetRevoluteJointAngle()
  }

  GetJointAngularSpeed(): number {
    return this.GetRevoluteJointSpeed()
  }

  GetPrismaticJointTranslation(): number {
    const bA: Body = this.m_bodyA
    const bB: Body = this.m_bodyB

    const pA: Vec2 = bA.GetWorldPoint(this.m_localAnchorA, new Vec2())
    const pB: Vec2 = bB.GetWorldPoint(this.m_localAnchorB, new Vec2())
    const d: Vec2 = Vec2.SubVV(pB, pA, new Vec2())
    const axis: Vec2 = bA.GetWorldVector(this.m_localXAxisA, new Vec2())

    const translation: number = Vec2.DotVV(d, axis)
    return translation
  }

  GetPrismaticJointSpeed(): number {
    const bA: Body = this.m_bodyA
    const bB: Body = this.m_bodyB

    // Vec2 rA = Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
    Vec2.SubVV(this.m_localAnchorA, bA.m_sweep.localCenter, this.m_lalcA)
    const rA = Rot.MulRV(bA.m_xf.q, this.m_lalcA, this.m_rA)
    // Vec2 rB = Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
    Vec2.SubVV(this.m_localAnchorB, bB.m_sweep.localCenter, this.m_lalcB)
    const rB = Rot.MulRV(bB.m_xf.q, this.m_lalcB, this.m_rB)
    // Vec2 pA = bA->m_sweep.c + rA;
    const pA = Vec2.AddVV(bA.m_sweep.c, rA, Vec2.s_t0) // pA uses s_t0
    // Vec2 pB = bB->m_sweep.c + rB;
    const pB = Vec2.AddVV(bB.m_sweep.c, rB, Vec2.s_t1) // pB uses s_t1
    // Vec2 d = pB - pA;
    const d = Vec2.SubVV(pB, pA, Vec2.s_t2) // d uses s_t2
    // Vec2 axis = Mul(bA.m_xf.q, m_localXAxisA);
    const axis = bA.GetWorldVector(this.m_localXAxisA, new Vec2())

    const vA = bA.m_linearVelocity
    const vB = bB.m_linearVelocity
    const wA = bA.m_angularVelocity
    const wB = bB.m_angularVelocity

    // float32 speed = Dot(d, Cross(wA, axis)) + Dot(axis, vB + Cross(wB, rB) - vA - Cross(wA, rA));
    const speed =
      Vec2.DotVV(d, Vec2.CrossSV(wA, axis, Vec2.s_t0)) +
      Vec2.DotVV(
        axis,
        Vec2.SubVV(
          Vec2.AddVCrossSV(vB, wB, rB, Vec2.s_t0),
          Vec2.AddVCrossSV(vA, wA, rA, Vec2.s_t1),
          Vec2.s_t0
        )
      )
    return speed
  }

  GetRevoluteJointAngle(): number {
    // Body* bA = this.m_bodyA;
    // Body* bB = this.m_bodyB;
    // return bB->this.m_sweep.a - bA->this.m_sweep.a;
    return this.m_bodyB.m_sweep.a - this.m_bodyA.m_sweep.a
  }

  GetRevoluteJointSpeed(): number {
    const wA: number = this.m_bodyA.m_angularVelocity
    const wB: number = this.m_bodyB.m_angularVelocity
    return wB - wA
  }

  IsMotorEnabled(): boolean {
    return this.m_enableMotor
  }

  EnableMotor(flag: boolean): void {
    if (flag !== this.m_enableMotor) {
      this.m_bodyA.SetAwake(true)
      this.m_bodyB.SetAwake(true)
      this.m_enableMotor = flag
    }
  }

  SetMotorSpeed(speed: number): void {
    if (speed !== this.m_motorSpeed) {
      this.m_bodyA.SetAwake(true)
      this.m_bodyB.SetAwake(true)
      this.m_motorSpeed = speed
    }
  }

  SetMaxMotorTorque(force: number): void {
    if (force !== this.m_maxMotorTorque) {
      this.m_bodyA.SetAwake(true)
      this.m_bodyB.SetAwake(true)
      this.m_maxMotorTorque = force
    }
  }

  GetMotorTorque(inv_dt: number): number {
    return inv_dt * this.m_motorImpulse
  }

  Dump(log: (format: string, ...args: any[]) => void): void {
    const indexA = this.m_bodyA.m_islandIndex
    const indexB = this.m_bodyB.m_islandIndex

    log('  const jd: WheelJointDef = new WheelJointDef();\n')
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
    log(
      '  jd.localAxisA.Set(%.15f, %.15f);\n',
      this.m_localXAxisA.x,
      this.m_localXAxisA.y
    )
    log('  jd.enableMotor = %s;\n', this.m_enableMotor ? 'true' : 'false')
    log('  jd.motorSpeed = %.15f;\n', this.m_motorSpeed)
    log('  jd.maxMotorTorque = %.15f;\n', this.m_maxMotorTorque)
    log('  jd.frequencyHz = %.15f;\n', this.m_frequencyHz)
    log('  jd.dampingRatio = %.15f;\n', this.m_dampingRatio)
    log('  joints[%d] = this.m_world.CreateJoint(jd);\n', this.m_index)
  }
}
