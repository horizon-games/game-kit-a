// DEBUG: import { Assert } from "../../Common/Settings";
import { Sq, Sqrt, Vec2, XY } from '../../Common/Math'
import {
  epsilon,
  linearSlop,
  MakeNumberArray,
  maxLinearCorrection,
  Maybe
} from '../../Common/Settings'
import { Body } from '../Body'
import { SolverData } from '../TimeStep'

import { DistanceJoint, DistanceJointDef } from './DistanceJoint'
import { IJointDef, Joint, JointDef, JointType } from './Joint'

export interface IAreaJointDef extends IJointDef {
  // world: World;

  bodies: Body[]

  frequencyHz?: number

  dampingRatio?: number
}

export class AreaJointDef extends JointDef implements IAreaJointDef {
  bodies: Body[] = []

  frequencyHz: number = 0

  dampingRatio: number = 0

  constructor() {
    super(JointType.e_areaJoint)
  }

  AddBody(body: Body): void {
    this.bodies.push(body)

    if (this.bodies.length === 1) {
      this.bodyA = body
    } else if (this.bodies.length === 2) {
      this.bodyB = body
    }
  }
}

export class AreaJoint extends Joint {
  m_bodies: Body[]
  m_frequencyHz: number = 0
  m_dampingRatio: number = 0

  // Solver shared
  m_impulse: number = 0

  // Solver temp
  m_targetLengths: number[]
  m_targetArea: number = 0
  m_normals: Vec2[]
  m_joints: DistanceJoint[]
  m_deltas: Vec2[]
  m_delta: Vec2

  constructor(def: IAreaJointDef) {
    super(def)

    // DEBUG: Assert(def.bodies.length >= 3, "You cannot create an area joint with less than three bodies.");

    this.m_bodies = def.bodies
    this.m_frequencyHz = Maybe(def.frequencyHz, 0)
    this.m_dampingRatio = Maybe(def.dampingRatio, 0)

    this.m_targetLengths = MakeNumberArray(def.bodies.length)
    this.m_normals = Vec2.MakeArray(def.bodies.length)
    this.m_joints = [] // MakeNullArray(def.bodies.length);
    this.m_deltas = Vec2.MakeArray(def.bodies.length)
    this.m_delta = new Vec2()

    const djd: DistanceJointDef = new DistanceJointDef()
    djd.frequencyHz = this.m_frequencyHz
    djd.dampingRatio = this.m_dampingRatio

    this.m_targetArea = 0

    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const body: Body = this.m_bodies[i]
      const next: Body = this.m_bodies[(i + 1) % this.m_bodies.length]

      const body_c: Vec2 = body.GetWorldCenter()
      const next_c: Vec2 = next.GetWorldCenter()

      this.m_targetLengths[i] = Vec2.DistanceVV(body_c, next_c)

      this.m_targetArea += Vec2.CrossVV(body_c, next_c)

      djd.Initialize(body, next, body_c, next_c)
      this.m_joints[i] = body.GetWorld().CreateJoint(djd)
    }

    this.m_targetArea *= 0.5
  }

  GetAnchorA<T extends XY>(out: T): T {
    return out
  }

  GetAnchorB<T extends XY>(out: T): T {
    return out
  }

  GetReactionForce<T extends XY>(inv_dt: number, out: T): T {
    return out
  }

  GetReactionTorque(inv_dt: number): number {
    return 0
  }

  SetFrequency(hz: number): void {
    this.m_frequencyHz = hz

    for (const joint of this.m_joints) {
      joint.SetFrequency(hz)
    }
  }

  GetFrequency() {
    return this.m_frequencyHz
  }

  SetDampingRatio(ratio: number): void {
    this.m_dampingRatio = ratio

    for (const joint of this.m_joints) {
      joint.SetDampingRatio(ratio)
    }
  }

  GetDampingRatio() {
    return this.m_dampingRatio
  }

  Dump(log: (format: string, ...args: any[]) => void) {
    log('Area joint dumping is not supported.\n')
  }

  InitVelocityConstraints(data: SolverData): void {
    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const prev: Body = this.m_bodies[
        (i + this.m_bodies.length - 1) % this.m_bodies.length
      ]
      const next: Body = this.m_bodies[(i + 1) % this.m_bodies.length]
      const prev_c: Vec2 = data.positions[prev.m_islandIndex].c
      const next_c: Vec2 = data.positions[next.m_islandIndex].c
      const delta: Vec2 = this.m_deltas[i]

      Vec2.SubVV(next_c, prev_c, delta)
    }

    if (data.step.warmStarting) {
      this.m_impulse *= data.step.dtRatio

      for (let i: number = 0; i < this.m_bodies.length; ++i) {
        const body: Body = this.m_bodies[i]
        const body_v: Vec2 = data.velocities[body.m_islandIndex].v
        const delta: Vec2 = this.m_deltas[i]

        body_v.x += body.m_invMass * delta.y * 0.5 * this.m_impulse
        body_v.y += body.m_invMass * -delta.x * 0.5 * this.m_impulse
      }
    } else {
      this.m_impulse = 0
    }
  }

  SolveVelocityConstraints(data: SolverData): void {
    let dotMassSum: number = 0
    let crossMassSum: number = 0

    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const body: Body = this.m_bodies[i]
      const body_v: Vec2 = data.velocities[body.m_islandIndex].v
      const delta: Vec2 = this.m_deltas[i]

      dotMassSum += delta.LengthSquared() / body.GetMass()
      crossMassSum += Vec2.CrossVV(body_v, delta)
    }

    const lambda: number = (-2 * crossMassSum) / dotMassSum
    // lambda = Clamp(lambda, -maxLinearCorrection, maxLinearCorrection);

    this.m_impulse += lambda

    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const body: Body = this.m_bodies[i]
      const body_v: Vec2 = data.velocities[body.m_islandIndex].v
      const delta: Vec2 = this.m_deltas[i]

      body_v.x += body.m_invMass * delta.y * 0.5 * lambda
      body_v.y += body.m_invMass * -delta.x * 0.5 * lambda
    }
  }

  SolvePositionConstraints(data: SolverData): boolean {
    let perimeter: number = 0
    let area: number = 0

    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const body: Body = this.m_bodies[i]
      const next: Body = this.m_bodies[(i + 1) % this.m_bodies.length]
      const body_c: Vec2 = data.positions[body.m_islandIndex].c
      const next_c: Vec2 = data.positions[next.m_islandIndex].c

      const delta: Vec2 = Vec2.SubVV(next_c, body_c, this.m_delta)

      let dist: number = delta.Length()
      if (dist < epsilon) {
        dist = 1
      }

      this.m_normals[i].x = delta.y / dist
      this.m_normals[i].y = -delta.x / dist

      perimeter += dist

      area += Vec2.CrossVV(body_c, next_c)
    }

    area *= 0.5

    const deltaArea: number = this.m_targetArea - area
    const toExtrude: number = (0.5 * deltaArea) / perimeter
    let done: boolean = true

    for (let i: number = 0; i < this.m_bodies.length; ++i) {
      const body: Body = this.m_bodies[i]
      const body_c: Vec2 = data.positions[body.m_islandIndex].c
      const next_i: number = (i + 1) % this.m_bodies.length

      const delta: Vec2 = Vec2.AddVV(
        this.m_normals[i],
        this.m_normals[next_i],
        this.m_delta
      )
      delta.SelfMul(toExtrude)

      const norm_sq: number = delta.LengthSquared()
      if (norm_sq > Sq(maxLinearCorrection)) {
        delta.SelfMul(maxLinearCorrection / Sqrt(norm_sq))
      }
      if (norm_sq > Sq(linearSlop)) {
        done = false
      }

      body_c.x += delta.x
      body_c.y += delta.y
    }

    return done
  }
}
