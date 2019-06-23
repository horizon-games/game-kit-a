/*
 * Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

// DEBUG: import { Assert, epsilon } from "../../Common/Settings";
// DEBUG: import { IsValid } from "../../Common/Math";
import { Mat22, Rot, Transform, Vec2, XY } from '../../Common/Math'
import { Maybe, pi } from '../../Common/Settings'
import { SolverData } from '../TimeStep'

import { IJointDef, Joint, JointDef, JointType } from './Joint'

export interface IMouseJointDef extends IJointDef {
  target?: XY

  maxForce?: number

  frequencyHz?: number

  dampingRatio?: number
}

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
export class MouseJointDef extends JointDef implements IMouseJointDef {
  readonly target: Vec2 = new Vec2()

  maxForce: number = 0

  frequencyHz: number = 5

  dampingRatio: number = 0.7

  constructor() {
    super(JointType.e_mouseJoint)
  }
}

export class MouseJoint extends Joint {
  private static SolveVelocityConstraints_s_Cdot = new Vec2()
  private static SolveVelocityConstraints_s_impulse = new Vec2()
  private static SolveVelocityConstraints_s_oldImpulse = new Vec2()
  readonly m_localAnchorB: Vec2 = new Vec2()
  readonly m_targetA: Vec2 = new Vec2()
  m_frequencyHz: number = 0
  m_dampingRatio: number = 0
  m_beta: number = 0

  // Solver shared
  readonly m_impulse: Vec2 = new Vec2()
  m_maxForce: number = 0
  m_gamma: number = 0

  // Solver temp
  m_indexA: number = 0
  m_indexB: number = 0
  readonly m_rB: Vec2 = new Vec2()
  readonly m_localCenterB: Vec2 = new Vec2()
  m_invMassB: number = 0
  m_invIB: number = 0
  readonly m_mass: Mat22 = new Mat22()
  readonly m_C: Vec2 = new Vec2()
  readonly m_qB: Rot = new Rot()
  readonly m_lalcB: Vec2 = new Vec2()
  readonly m_K: Mat22 = new Mat22()

  constructor(def: IMouseJointDef) {
    super(def)

    this.m_targetA.Copy(Maybe(def.target, Vec2.ZERO))
    // DEBUG: Assert(this.m_targetA.IsValid());
    Transform.MulTXV(
      this.m_bodyB.GetTransform(),
      this.m_targetA,
      this.m_localAnchorB
    )

    this.m_maxForce = Maybe(def.maxForce, 0)
    // DEBUG: Assert(IsValid(this.m_maxForce) && this.m_maxForce >= 0);
    this.m_impulse.SetZero()

    this.m_frequencyHz = Maybe(def.frequencyHz, 0)
    // DEBUG: Assert(IsValid(this.m_frequencyHz) && this.m_frequencyHz >= 0);
    this.m_dampingRatio = Maybe(def.dampingRatio, 0)
    // DEBUG: Assert(IsValid(this.m_dampingRatio) && this.m_dampingRatio >= 0);

    this.m_beta = 0
    this.m_gamma = 0
  }

  SetTarget(target: Vec2): void {
    if (!this.m_bodyB.IsAwake()) {
      this.m_bodyB.SetAwake(true)
    }
    this.m_targetA.Copy(target)
  }

  GetTarget() {
    return this.m_targetA
  }

  SetMaxForce(maxForce: number): void {
    this.m_maxForce = maxForce
  }

  GetMaxForce() {
    return this.m_maxForce
  }

  SetFrequency(hz: number): void {
    this.m_frequencyHz = hz
  }

  GetFrequency() {
    return this.m_frequencyHz
  }

  SetDampingRatio(ratio: number) {
    this.m_dampingRatio = ratio
  }

  GetDampingRatio() {
    return this.m_dampingRatio
  }

  InitVelocityConstraints(data: SolverData): void {
    this.m_indexB = this.m_bodyB.m_islandIndex
    this.m_localCenterB.Copy(this.m_bodyB.m_sweep.localCenter)
    this.m_invMassB = this.m_bodyB.m_invMass
    this.m_invIB = this.m_bodyB.m_invI

    const cB: Vec2 = data.positions[this.m_indexB].c
    const aB: number = data.positions[this.m_indexB].a
    const vB: Vec2 = data.velocities[this.m_indexB].v
    let wB: number = data.velocities[this.m_indexB].w

    const qB = this.m_qB.SetAngle(aB)

    const mass: number = this.m_bodyB.GetMass()

    // Frequency
    const omega: number = 2 * pi * this.m_frequencyHz

    // Damping coefficient
    const d: number = 2 * mass * this.m_dampingRatio * omega

    // Spring stiffness
    const k: number = mass * (omega * omega)

    // magic formulas
    // gamma has units of inverse mass.
    // beta has units of inverse time.
    const h: number = data.step.dt
    // DEBUG: Assert(d + h * k > epsilon);
    this.m_gamma = h * (d + h * k)
    if (this.m_gamma !== 0) {
      this.m_gamma = 1 / this.m_gamma
    }
    this.m_beta = h * k * this.m_gamma

    // Compute the effective mass matrix.
    Vec2.SubVV(this.m_localAnchorB, this.m_localCenterB, this.m_lalcB)
    Rot.MulRV(qB, this.m_lalcB, this.m_rB)

    // K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    //      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    //        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
    const K = this.m_K
    K.ex.x =
      this.m_invMassB + this.m_invIB * this.m_rB.y * this.m_rB.y + this.m_gamma
    K.ex.y = -this.m_invIB * this.m_rB.x * this.m_rB.y
    K.ey.x = K.ex.y
    K.ey.y =
      this.m_invMassB + this.m_invIB * this.m_rB.x * this.m_rB.x + this.m_gamma

    K.GetInverse(this.m_mass)

    // m_C = cB + m_rB - m_targetA;
    this.m_C.x = cB.x + this.m_rB.x - this.m_targetA.x
    this.m_C.y = cB.y + this.m_rB.y - this.m_targetA.y
    // m_C *= m_beta;
    this.m_C.SelfMul(this.m_beta)

    // Cheat with some damping
    wB *= 0.98

    if (data.step.warmStarting) {
      this.m_impulse.SelfMul(data.step.dtRatio)
      // vB += m_invMassB * m_impulse;
      vB.x += this.m_invMassB * this.m_impulse.x
      vB.y += this.m_invMassB * this.m_impulse.y
      wB += this.m_invIB * Vec2.CrossVV(this.m_rB, this.m_impulse)
    } else {
      this.m_impulse.SetZero()
    }

    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB
  }
  SolveVelocityConstraints(data: SolverData): void {
    const vB: Vec2 = data.velocities[this.m_indexB].v
    let wB: number = data.velocities[this.m_indexB].w

    // Cdot = v + cross(w, r)
    // Vec2 Cdot = vB + Cross(wB, m_rB);
    const Cdot: Vec2 = Vec2.AddVCrossSV(
      vB,
      wB,
      this.m_rB,
      MouseJoint.SolveVelocityConstraints_s_Cdot
    )
    //  Vec2 impulse = Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));
    const impulse: Vec2 = Mat22.MulMV(
      this.m_mass,
      Vec2.AddVV(
        Cdot,
        Vec2.AddVV(
          this.m_C,
          Vec2.MulSV(this.m_gamma, this.m_impulse, Vec2.s_t0),
          Vec2.s_t0
        ),
        Vec2.s_t0
      ).SelfNeg(),
      MouseJoint.SolveVelocityConstraints_s_impulse
    )

    // Vec2 oldImpulse = m_impulse;
    const oldImpulse = MouseJoint.SolveVelocityConstraints_s_oldImpulse.Copy(
      this.m_impulse
    )
    // m_impulse += impulse;
    this.m_impulse.SelfAdd(impulse)
    const maxImpulse: number = data.step.dt * this.m_maxForce
    if (this.m_impulse.LengthSquared() > maxImpulse * maxImpulse) {
      this.m_impulse.SelfMul(maxImpulse / this.m_impulse.Length())
    }
    // impulse = m_impulse - oldImpulse;
    Vec2.SubVV(this.m_impulse, oldImpulse, impulse)

    // vB += m_invMassB * impulse;
    vB.SelfMulAdd(this.m_invMassB, impulse)
    wB += this.m_invIB * Vec2.CrossVV(this.m_rB, impulse)

    // data.velocities[this.m_indexB].v = vB;
    data.velocities[this.m_indexB].w = wB
  }

  SolvePositionConstraints(data: SolverData): boolean {
    return true
  }

  GetAnchorA<T extends XY>(out: T): T {
    out.x = this.m_targetA.x
    out.y = this.m_targetA.y
    return out
  }

  GetAnchorB<T extends XY>(out: T): T {
    return this.m_bodyB.GetWorldPoint(this.m_localAnchorB, out)
  }

  GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
    return Vec2.MulSV(inv_dt, this.m_impulse, out)
  }

  GetReactionTorque(inv_dt: number): number {
    return 0
  }

  Dump(log: (format: string, ...args: any[]) => void) {
    log('Mouse joint dumping is not supported.\n')
  }

  ShiftOrigin(newOrigin: Vec2) {
    this.m_targetA.SelfSub(newOrigin)
  }
}
