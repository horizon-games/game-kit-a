/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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
import {
  Manifold,
  ManifoldPoint,
  ManifoldType,
  WorldManifold
} from '../../Collision/Collision'
import { Shape } from '../../Collision/Shapes/Shape'
import { Clamp, Mat22, Max, Min, Rot, Transform, Vec2 } from '../../Common/Math'
import {
  baumgarte,
  linearSlop,
  MakeArray,
  maxLinearCorrection,
  maxManifoldPoints,
  toiBaumgarte,
  velocityThreshold
} from '../../Common/Settings'
import { Body } from '../Body'
import { Fixture } from '../Fixture'
import { Position, TimeStep, Velocity } from '../TimeStep'

import { Contact } from './Contact'

// Solver debugging is normally disabled because the block solver sometimes has to deal with a poorly conditioned effective mass matrix.
// #define DEBUG_SOLVER 0

export let g_blockSolve: boolean = false

export class VelocityConstraintPoint {
  static MakeArray(length: number): VelocityConstraintPoint[] {
    return MakeArray(length, (i: number) => new VelocityConstraintPoint())
  }
  readonly rA: Vec2 = new Vec2()
  readonly rB: Vec2 = new Vec2()
  normalImpulse: number = 0
  tangentImpulse: number = 0
  normalMass: number = 0
  tangentMass: number = 0
  velocityBias: number = 0
}

export class ContactVelocityConstraint {
  static MakeArray(length: number): ContactVelocityConstraint[] {
    return MakeArray(length, (i: number) => new ContactVelocityConstraint())
  }
  points: VelocityConstraintPoint[] = VelocityConstraintPoint.MakeArray(
    maxManifoldPoints
  )
  readonly normal: Vec2 = new Vec2()
  readonly tangent: Vec2 = new Vec2()
  readonly normalMass: Mat22 = new Mat22()
  readonly K: Mat22 = new Mat22()
  indexA: number = 0
  indexB: number = 0
  invMassA: number = 0
  invMassB: number = 0
  invIA: number = 0
  invIB: number = 0
  friction: number = 0
  restitution: number = 0
  tangentSpeed: number = 0
  pointCount: number = 0
  contactIndex: number = 0
}

export class ContactPositionConstraint {
  static MakeArray(length: number): ContactPositionConstraint[] {
    return MakeArray(length, (i: number) => new ContactPositionConstraint())
  }
  localPoints: Vec2[] = Vec2.MakeArray(maxManifoldPoints)
  readonly localNormal: Vec2 = new Vec2()
  readonly localPoint: Vec2 = new Vec2()
  indexA: number = 0
  indexB: number = 0
  invMassA: number = 0
  invMassB: number = 0
  readonly localCenterA: Vec2 = new Vec2()
  readonly localCenterB: Vec2 = new Vec2()
  invIA: number = 0
  invIB: number = 0
  type: ManifoldType = ManifoldType.e_unknown
  radiusA: number = 0
  radiusB: number = 0
  pointCount: number = 0
}

export class ContactSolverDef {
  readonly step: TimeStep = new TimeStep()
  contacts!: Contact[]
  count: number = 0
  positions!: Position[]
  velocities!: Velocity[]
  allocator: any = null
}

export class PositionSolverManifold {
  private static Initialize_s_pointA = new Vec2()
  private static Initialize_s_pointB = new Vec2()
  private static Initialize_s_planePoint = new Vec2()
  private static Initialize_s_clipPoint = new Vec2()
  readonly normal: Vec2 = new Vec2()
  readonly point: Vec2 = new Vec2()
  separation: number = 0
  Initialize(
    pc: ContactPositionConstraint,
    xfA: Transform,
    xfB: Transform,
    index: number
  ): void {
    const pointA: Vec2 = PositionSolverManifold.Initialize_s_pointA
    const pointB: Vec2 = PositionSolverManifold.Initialize_s_pointB
    const planePoint: Vec2 = PositionSolverManifold.Initialize_s_planePoint
    const clipPoint: Vec2 = PositionSolverManifold.Initialize_s_clipPoint

    // DEBUG: Assert(pc.pointCount > 0);

    switch (pc.type) {
      case ManifoldType.e_circles: {
        // Vec2 pointA = Mul(xfA, pc->localPoint);
        Transform.MulXV(xfA, pc.localPoint, pointA)
        // Vec2 pointB = Mul(xfB, pc->localPoints[0]);
        Transform.MulXV(xfB, pc.localPoints[0], pointB)
        // normal = pointB - pointA;
        // normal.Normalize();
        Vec2.SubVV(pointB, pointA, this.normal).SelfNormalize()
        // point = 0.5f * (pointA + pointB);
        Vec2.MidVV(pointA, pointB, this.point)
        // separation = Dot(pointB - pointA, normal) - pc->radius;
        this.separation =
          Vec2.DotVV(Vec2.SubVV(pointB, pointA, Vec2.s_t0), this.normal) -
          pc.radiusA -
          pc.radiusB
        break
      }

      case ManifoldType.e_faceA: {
        // normal = Mul(xfA.q, pc->localNormal);
        Rot.MulRV(xfA.q, pc.localNormal, this.normal)
        // Vec2 planePoint = Mul(xfA, pc->localPoint);
        Transform.MulXV(xfA, pc.localPoint, planePoint)

        // Vec2 clipPoint = Mul(xfB, pc->localPoints[index]);
        Transform.MulXV(xfB, pc.localPoints[index], clipPoint)
        // separation = Dot(clipPoint - planePoint, normal) - pc->radius;
        this.separation =
          Vec2.DotVV(
            Vec2.SubVV(clipPoint, planePoint, Vec2.s_t0),
            this.normal
          ) -
          pc.radiusA -
          pc.radiusB
        // point = clipPoint;
        this.point.Copy(clipPoint)
        break
      }

      case ManifoldType.e_faceB: {
        // normal = Mul(xfB.q, pc->localNormal);
        Rot.MulRV(xfB.q, pc.localNormal, this.normal)
        // Vec2 planePoint = Mul(xfB, pc->localPoint);
        Transform.MulXV(xfB, pc.localPoint, planePoint)

        // Vec2 clipPoint = Mul(xfA, pc->localPoints[index]);
        Transform.MulXV(xfA, pc.localPoints[index], clipPoint)
        // separation = Dot(clipPoint - planePoint, normal) - pc->radius;
        this.separation =
          Vec2.DotVV(
            Vec2.SubVV(clipPoint, planePoint, Vec2.s_t0),
            this.normal
          ) -
          pc.radiusA -
          pc.radiusB
        // point = clipPoint;
        this.point.Copy(clipPoint)

        // Ensure normal points from A to B
        // normal = -normal;
        this.normal.SelfNeg()
        break
      }
    }
  }
}

export class ContactSolver {
  private static InitializeVelocityConstraints_s_xfA = new Transform()
  private static InitializeVelocityConstraints_s_xfB = new Transform()
  private static InitializeVelocityConstraints_s_worldManifold = new WorldManifold()

  private static WarmStart_s_P = new Vec2()

  private static SolveVelocityConstraints_s_dv = new Vec2()
  private static SolveVelocityConstraints_s_dv1 = new Vec2()
  private static SolveVelocityConstraints_s_dv2 = new Vec2()
  private static SolveVelocityConstraints_s_P = new Vec2()
  private static SolveVelocityConstraints_s_a = new Vec2()
  private static SolveVelocityConstraints_s_b = new Vec2()
  private static SolveVelocityConstraints_s_x = new Vec2()
  private static SolveVelocityConstraints_s_d = new Vec2()
  private static SolveVelocityConstraints_s_P1 = new Vec2()
  private static SolveVelocityConstraints_s_P2 = new Vec2()
  private static SolveVelocityConstraints_s_P1P2 = new Vec2()

  private static SolvePositionConstraints_s_xfA = new Transform()
  private static SolvePositionConstraints_s_xfB = new Transform()
  private static SolvePositionConstraints_s_psm = new PositionSolverManifold()
  private static SolvePositionConstraints_s_rA = new Vec2()
  private static SolvePositionConstraints_s_rB = new Vec2()
  private static SolvePositionConstraints_s_P = new Vec2()

  private static SolveTOIPositionConstraints_s_xfA = new Transform()
  private static SolveTOIPositionConstraints_s_xfB = new Transform()
  private static SolveTOIPositionConstraints_s_psm = new PositionSolverManifold()
  private static SolveTOIPositionConstraints_s_rA = new Vec2()
  private static SolveTOIPositionConstraints_s_rB = new Vec2()
  private static SolveTOIPositionConstraints_s_P = new Vec2()
  readonly m_step: TimeStep = new TimeStep()
  m_positions!: Position[]
  m_velocities!: Velocity[]
  m_allocator: any = null
  m_positionConstraints: ContactPositionConstraint[] = ContactPositionConstraint.MakeArray(
    1024
  ) // TODO: Settings
  m_velocityConstraints: ContactVelocityConstraint[] = ContactVelocityConstraint.MakeArray(
    1024
  ) // TODO: Settings
  m_contacts!: Contact[]
  m_count: number = 0

  Initialize(def: ContactSolverDef): ContactSolver {
    this.m_step.Copy(def.step)
    this.m_allocator = def.allocator
    this.m_count = def.count
    // TODO:
    if (this.m_positionConstraints.length < this.m_count) {
      const new_length: number = Max(
        this.m_positionConstraints.length * 2,
        this.m_count
      )
      while (this.m_positionConstraints.length < new_length) {
        this.m_positionConstraints[
          this.m_positionConstraints.length
        ] = new ContactPositionConstraint()
      }
    }
    // TODO:
    if (this.m_velocityConstraints.length < this.m_count) {
      const new_length: number = Max(
        this.m_velocityConstraints.length * 2,
        this.m_count
      )
      while (this.m_velocityConstraints.length < new_length) {
        this.m_velocityConstraints[
          this.m_velocityConstraints.length
        ] = new ContactVelocityConstraint()
      }
    }
    this.m_positions = def.positions
    this.m_velocities = def.velocities
    this.m_contacts = def.contacts

    // Initialize position independent portions of the constraints.
    for (let i: number = 0; i < this.m_count; ++i) {
      const contact: Contact = this.m_contacts[i]

      const fixtureA: Fixture = contact.m_fixtureA
      const fixtureB: Fixture = contact.m_fixtureB
      const shapeA: Shape = fixtureA.GetShape()
      const shapeB: Shape = fixtureB.GetShape()
      const radiusA: number = shapeA.m_radius
      const radiusB: number = shapeB.m_radius
      const bodyA: Body = fixtureA.GetBody()
      const bodyB: Body = fixtureB.GetBody()
      const manifold: Manifold = contact.GetManifold()

      const pointCount: number = manifold.pointCount
      // DEBUG: Assert(pointCount > 0);

      const vc: ContactVelocityConstraint = this.m_velocityConstraints[i]
      vc.friction = contact.m_friction
      vc.restitution = contact.m_restitution
      vc.tangentSpeed = contact.m_tangentSpeed
      vc.indexA = bodyA.m_islandIndex
      vc.indexB = bodyB.m_islandIndex
      vc.invMassA = bodyA.m_invMass
      vc.invMassB = bodyB.m_invMass
      vc.invIA = bodyA.m_invI
      vc.invIB = bodyB.m_invI
      vc.contactIndex = i
      vc.pointCount = pointCount
      vc.K.SetZero()
      vc.normalMass.SetZero()

      const pc: ContactPositionConstraint = this.m_positionConstraints[i]
      pc.indexA = bodyA.m_islandIndex
      pc.indexB = bodyB.m_islandIndex
      pc.invMassA = bodyA.m_invMass
      pc.invMassB = bodyB.m_invMass
      pc.localCenterA.Copy(bodyA.m_sweep.localCenter)
      pc.localCenterB.Copy(bodyB.m_sweep.localCenter)
      pc.invIA = bodyA.m_invI
      pc.invIB = bodyB.m_invI
      pc.localNormal.Copy(manifold.localNormal)
      pc.localPoint.Copy(manifold.localPoint)
      pc.pointCount = pointCount
      pc.radiusA = radiusA
      pc.radiusB = radiusB
      pc.type = manifold.type

      for (let j: number = 0; j < pointCount; ++j) {
        const cp: ManifoldPoint = manifold.points[j]
        const vcp: VelocityConstraintPoint = vc.points[j]

        if (this.m_step.warmStarting) {
          vcp.normalImpulse = this.m_step.dtRatio * cp.normalImpulse
          vcp.tangentImpulse = this.m_step.dtRatio * cp.tangentImpulse
        } else {
          vcp.normalImpulse = 0
          vcp.tangentImpulse = 0
        }

        vcp.rA.SetZero()
        vcp.rB.SetZero()
        vcp.normalMass = 0
        vcp.tangentMass = 0
        vcp.velocityBias = 0

        pc.localPoints[j].Copy(cp.localPoint)
      }
    }

    return this
  }
  InitializeVelocityConstraints(): void {
    const xfA: Transform = ContactSolver.InitializeVelocityConstraints_s_xfA
    const xfB: Transform = ContactSolver.InitializeVelocityConstraints_s_xfB
    const worldManifold: WorldManifold =
      ContactSolver.InitializeVelocityConstraints_s_worldManifold

    const k_maxConditionNumber: number = 1000

    for (let i: number = 0; i < this.m_count; ++i) {
      const vc: ContactVelocityConstraint = this.m_velocityConstraints[i]
      const pc: ContactPositionConstraint = this.m_positionConstraints[i]

      const radiusA: number = pc.radiusA
      const radiusB: number = pc.radiusB
      const manifold: Manifold = this.m_contacts[vc.contactIndex].GetManifold()

      const indexA: number = vc.indexA
      const indexB: number = vc.indexB

      const mA: number = vc.invMassA
      const mB: number = vc.invMassB
      const iA: number = vc.invIA
      const iB: number = vc.invIB
      const localCenterA: Vec2 = pc.localCenterA
      const localCenterB: Vec2 = pc.localCenterB

      const cA: Vec2 = this.m_positions[indexA].c
      const aA: number = this.m_positions[indexA].a
      const vA: Vec2 = this.m_velocities[indexA].v
      const wA: number = this.m_velocities[indexA].w

      const cB: Vec2 = this.m_positions[indexB].c
      const aB: number = this.m_positions[indexB].a
      const vB: Vec2 = this.m_velocities[indexB].v
      const wB: number = this.m_velocities[indexB].w

      // DEBUG: Assert(manifold.pointCount > 0);

      xfA.q.SetAngle(aA)
      xfB.q.SetAngle(aB)
      Vec2.SubVV(cA, Rot.MulRV(xfA.q, localCenterA, Vec2.s_t0), xfA.p)
      Vec2.SubVV(cB, Rot.MulRV(xfB.q, localCenterB, Vec2.s_t0), xfB.p)

      worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB)

      vc.normal.Copy(worldManifold.normal)
      Vec2.CrossVOne(vc.normal, vc.tangent) // compute from normal

      const pointCount: number = vc.pointCount
      for (let j: number = 0; j < pointCount; ++j) {
        const vcp: VelocityConstraintPoint = vc.points[j]

        // vcp->rA = worldManifold.points[j] - cA;
        Vec2.SubVV(worldManifold.points[j], cA, vcp.rA)
        // vcp->rB = worldManifold.points[j] - cB;
        Vec2.SubVV(worldManifold.points[j], cB, vcp.rB)

        const rnA: number = Vec2.CrossVV(vcp.rA, vc.normal)
        const rnB: number = Vec2.CrossVV(vcp.rB, vc.normal)

        const kNormal: number = mA + mB + iA * rnA * rnA + iB * rnB * rnB

        vcp.normalMass = kNormal > 0 ? 1 / kNormal : 0

        // Vec2 tangent = Cross(vc->normal, 1.0f);
        const tangent: Vec2 = vc.tangent // precomputed from normal

        const rtA: number = Vec2.CrossVV(vcp.rA, tangent)
        const rtB: number = Vec2.CrossVV(vcp.rB, tangent)

        const kTangent: number = mA + mB + iA * rtA * rtA + iB * rtB * rtB

        vcp.tangentMass = kTangent > 0 ? 1 / kTangent : 0

        // Setup a velocity bias for restitution.
        vcp.velocityBias = 0
        // float32 vRel = Dot(vc->normal, vB + Cross(wB, vcp->rB) - vA - Cross(wA, vcp->rA));
        const vRel: number = Vec2.DotVV(
          vc.normal,
          Vec2.SubVV(
            Vec2.AddVCrossSV(vB, wB, vcp.rB, Vec2.s_t0),
            Vec2.AddVCrossSV(vA, wA, vcp.rA, Vec2.s_t1),
            Vec2.s_t0
          )
        )
        if (vRel < -velocityThreshold) {
          vcp.velocityBias += -vc.restitution * vRel
        }
      }

      // If we have two points, then prepare the block solver.
      if (vc.pointCount === 2 && g_blockSolve) {
        const vcp1: VelocityConstraintPoint = vc.points[0]
        const vcp2: VelocityConstraintPoint = vc.points[1]

        const rn1A: number = Vec2.CrossVV(vcp1.rA, vc.normal)
        const rn1B: number = Vec2.CrossVV(vcp1.rB, vc.normal)
        const rn2A: number = Vec2.CrossVV(vcp2.rA, vc.normal)
        const rn2B: number = Vec2.CrossVV(vcp2.rB, vc.normal)

        const k11: number = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B
        const k22: number = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B
        const k12: number = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B

        // Ensure a reasonable condition number.
        // float32 k_maxConditionNumber = 1000.0f;
        if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12)) {
          // K is safe to invert.
          vc.K.ex.Set(k11, k12)
          vc.K.ey.Set(k12, k22)
          vc.K.GetInverse(vc.normalMass)
        } else {
          // The constraints are redundant, just use one.
          // TODO_ERIN use deepest?
          vc.pointCount = 1
        }
      }
    }
  }
  WarmStart(): void {
    const P: Vec2 = ContactSolver.WarmStart_s_P

    // Warm start.
    for (let i: number = 0; i < this.m_count; ++i) {
      const vc: ContactVelocityConstraint = this.m_velocityConstraints[i]

      const indexA: number = vc.indexA
      const indexB: number = vc.indexB
      const mA: number = vc.invMassA
      const iA: number = vc.invIA
      const mB: number = vc.invMassB
      const iB: number = vc.invIB
      const pointCount: number = vc.pointCount

      const vA: Vec2 = this.m_velocities[indexA].v
      let wA: number = this.m_velocities[indexA].w
      const vB: Vec2 = this.m_velocities[indexB].v
      let wB: number = this.m_velocities[indexB].w

      const normal: Vec2 = vc.normal
      // Vec2 tangent = Cross(normal, 1.0f);
      const tangent: Vec2 = vc.tangent // precomputed from normal

      for (let j: number = 0; j < pointCount; ++j) {
        const vcp: VelocityConstraintPoint = vc.points[j]
        // Vec2 P = vcp->normalImpulse * normal + vcp->tangentImpulse * tangent;
        Vec2.AddVV(
          Vec2.MulSV(vcp.normalImpulse, normal, Vec2.s_t0),
          Vec2.MulSV(vcp.tangentImpulse, tangent, Vec2.s_t1),
          P
        )
        // wA -= iA * Cross(vcp->rA, P);
        wA -= iA * Vec2.CrossVV(vcp.rA, P)
        // vA -= mA * P;
        vA.SelfMulSub(mA, P)
        // wB += iB * Cross(vcp->rB, P);
        wB += iB * Vec2.CrossVV(vcp.rB, P)
        // vB += mB * P;
        vB.SelfMulAdd(mB, P)
      }

      // this.m_velocities[indexA].v = vA;
      this.m_velocities[indexA].w = wA
      // this.m_velocities[indexB].v = vB;
      this.m_velocities[indexB].w = wB
    }
  }
  SolveVelocityConstraints(): void {
    const dv: Vec2 = ContactSolver.SolveVelocityConstraints_s_dv
    const dv1: Vec2 = ContactSolver.SolveVelocityConstraints_s_dv1
    const dv2: Vec2 = ContactSolver.SolveVelocityConstraints_s_dv2
    const P: Vec2 = ContactSolver.SolveVelocityConstraints_s_P
    const a: Vec2 = ContactSolver.SolveVelocityConstraints_s_a
    const b: Vec2 = ContactSolver.SolveVelocityConstraints_s_b
    const x: Vec2 = ContactSolver.SolveVelocityConstraints_s_x
    const d: Vec2 = ContactSolver.SolveVelocityConstraints_s_d
    const P1: Vec2 = ContactSolver.SolveVelocityConstraints_s_P1
    const P2: Vec2 = ContactSolver.SolveVelocityConstraints_s_P2
    const P1P2: Vec2 = ContactSolver.SolveVelocityConstraints_s_P1P2

    for (let i: number = 0; i < this.m_count; ++i) {
      const vc: ContactVelocityConstraint = this.m_velocityConstraints[i]

      const indexA: number = vc.indexA
      const indexB: number = vc.indexB
      const mA: number = vc.invMassA
      const iA: number = vc.invIA
      const mB: number = vc.invMassB
      const iB: number = vc.invIB
      const pointCount: number = vc.pointCount

      const vA: Vec2 = this.m_velocities[indexA].v
      let wA: number = this.m_velocities[indexA].w
      const vB: Vec2 = this.m_velocities[indexB].v
      let wB: number = this.m_velocities[indexB].w

      // Vec2 normal = vc->normal;
      const normal: Vec2 = vc.normal
      // Vec2 tangent = Cross(normal, 1.0f);
      const tangent: Vec2 = vc.tangent // precomputed from normal
      const friction: number = vc.friction

      // DEBUG: Assert(pointCount === 1 || pointCount === 2);

      // Solve tangent constraints first because non-penetration is more important
      // than friction.
      for (let j: number = 0; j < pointCount; ++j) {
        const vcp: VelocityConstraintPoint = vc.points[j]

        // Relative velocity at contact
        // Vec2 dv = vB + Cross(wB, vcp->rB) - vA - Cross(wA, vcp->rA);
        Vec2.SubVV(
          Vec2.AddVCrossSV(vB, wB, vcp.rB, Vec2.s_t0),
          Vec2.AddVCrossSV(vA, wA, vcp.rA, Vec2.s_t1),
          dv
        )

        // Compute tangent force
        // float32 vt = Dot(dv, tangent) - vc->tangentSpeed;
        const vt: number = Vec2.DotVV(dv, tangent) - vc.tangentSpeed
        let lambda: number = vcp.tangentMass * -vt

        // Clamp the accumulated force
        const maxFriction: number = friction * vcp.normalImpulse
        const newImpulse: number = Clamp(
          vcp.tangentImpulse + lambda,
          -maxFriction,
          maxFriction
        )
        lambda = newImpulse - vcp.tangentImpulse
        vcp.tangentImpulse = newImpulse

        // Apply contact impulse
        // Vec2 P = lambda * tangent;
        Vec2.MulSV(lambda, tangent, P)

        // vA -= mA * P;
        vA.SelfMulSub(mA, P)
        // wA -= iA * Cross(vcp->rA, P);
        wA -= iA * Vec2.CrossVV(vcp.rA, P)

        // vB += mB * P;
        vB.SelfMulAdd(mB, P)
        // wB += iB * Cross(vcp->rB, P);
        wB += iB * Vec2.CrossVV(vcp.rB, P)
      }

      // Solve normal constraints
      if (vc.pointCount === 1 || g_blockSolve === false) {
        for (let j = 0; j < pointCount; ++j) {
          const vcp: VelocityConstraintPoint = vc.points[j]

          // Relative velocity at contact
          // Vec2 dv = vB + Cross(wB, vcp->rB) - vA - Cross(wA, vcp->rA);
          Vec2.SubVV(
            Vec2.AddVCrossSV(vB, wB, vcp.rB, Vec2.s_t0),
            Vec2.AddVCrossSV(vA, wA, vcp.rA, Vec2.s_t1),
            dv
          )

          // Compute normal impulse
          // float32 vn = Dot(dv, normal);
          const vn: number = Vec2.DotVV(dv, normal)
          let lambda: number = -vcp.normalMass * (vn - vcp.velocityBias)

          // Clamp the accumulated impulse
          // float32 newImpulse = Max(vcp->normalImpulse + lambda, 0.0f);
          const newImpulse: number = Max(vcp.normalImpulse + lambda, 0)
          lambda = newImpulse - vcp.normalImpulse
          vcp.normalImpulse = newImpulse

          // Apply contact impulse
          // Vec2 P = lambda * normal;
          Vec2.MulSV(lambda, normal, P)
          // vA -= mA * P;
          vA.SelfMulSub(mA, P)
          // wA -= iA * Cross(vcp->rA, P);
          wA -= iA * Vec2.CrossVV(vcp.rA, P)

          // vB += mB * P;
          vB.SelfMulAdd(mB, P)
          // wB += iB * Cross(vcp->rB, P);
          wB += iB * Vec2.CrossVV(vcp.rB, P)
        }
      } else {
        // Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
        // Build the mini LCP for this contact patch
        //
        // vn = A * x + b, vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
        //
        // A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
        // b = vn0 - velocityBias
        //
        // The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
        // implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
        // vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
        // solution that satisfies the problem is chosen.
        //
        // In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
        // that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
        //
        // Substitute:
        //
        // x = a + d
        //
        // a := old total impulse
        // x := new total impulse
        // d := incremental impulse
        //
        // For the current iteration we extend the formula for the incremental impulse
        // to compute the new total impulse:
        //
        // vn = A * d + b
        //    = A * (x - a) + b
        //    = A * x + b - A * a
        //    = A * x + b'
        // b' = b - A * a;

        const cp1: VelocityConstraintPoint = vc.points[0]
        const cp2: VelocityConstraintPoint = vc.points[1]

        // Vec2 a(cp1->normalImpulse, cp2->normalImpulse);
        a.Set(cp1.normalImpulse, cp2.normalImpulse)
        // DEBUG: Assert(a.x >= 0 && a.y >= 0);

        // Relative velocity at contact
        // Vec2 dv1 = vB + Cross(wB, cp1->rB) - vA - Cross(wA, cp1->rA);
        Vec2.SubVV(
          Vec2.AddVCrossSV(vB, wB, cp1.rB, Vec2.s_t0),
          Vec2.AddVCrossSV(vA, wA, cp1.rA, Vec2.s_t1),
          dv1
        )
        // Vec2 dv2 = vB + Cross(wB, cp2->rB) - vA - Cross(wA, cp2->rA);
        Vec2.SubVV(
          Vec2.AddVCrossSV(vB, wB, cp2.rB, Vec2.s_t0),
          Vec2.AddVCrossSV(vA, wA, cp2.rA, Vec2.s_t1),
          dv2
        )

        // Compute normal velocity
        // float32 vn1 = Dot(dv1, normal);
        let vn1: number = Vec2.DotVV(dv1, normal)
        // float32 vn2 = Dot(dv2, normal);
        let vn2: number = Vec2.DotVV(dv2, normal)

        // Vec2 b;
        b.x = vn1 - cp1.velocityBias
        b.y = vn2 - cp2.velocityBias

        // Compute b'
        // b -= Mul(vc->K, a);
        b.SelfSub(Mat22.MulMV(vc.K, a, Vec2.s_t0))

        /*
        #if DEBUG_SOLVER === 1
        const k_errorTol: number = 0.001;
        #endif
        */

        for (;;) {
          //
          // Case 1: vn = 0
          //
          // 0 = A * x + b'
          //
          // Solve for x:
          //
          // x = - inv(A) * b'
          //
          // Vec2 x = - Mul(vc->normalMass, b);
          Mat22.MulMV(vc.normalMass, b, x).SelfNeg()

          if (x.x >= 0 && x.y >= 0) {
            // Get the incremental impulse
            // Vec2 d = x - a;
            Vec2.SubVV(x, a, d)

            // Apply incremental impulse
            // Vec2 P1 = d.x * normal;
            Vec2.MulSV(d.x, normal, P1)
            // Vec2 P2 = d.y * normal;
            Vec2.MulSV(d.y, normal, P2)
            Vec2.AddVV(P1, P2, P1P2)
            // vA -= mA * (P1 + P2);
            vA.SelfMulSub(mA, P1P2)
            // wA -= iA * (Cross(cp1->rA, P1) + Cross(cp2->rA, P2));
            wA -= iA * (Vec2.CrossVV(cp1.rA, P1) + Vec2.CrossVV(cp2.rA, P2))

            // vB += mB * (P1 + P2);
            vB.SelfMulAdd(mB, P1P2)
            // wB += iB * (Cross(cp1->rB, P1) + Cross(cp2->rB, P2));
            wB += iB * (Vec2.CrossVV(cp1.rB, P1) + Vec2.CrossVV(cp2.rB, P2))

            // Accumulate
            cp1.normalImpulse = x.x
            cp2.normalImpulse = x.y

            /*
            #if DEBUG_SOLVER === 1
            // Postconditions
            dv1 = vB + Cross(wB, cp1->rB) - vA - Cross(wA, cp1->rA);
            dv2 = vB + Cross(wB, cp2->rB) - vA - Cross(wA, cp2->rA);

            // Compute normal velocity
            vn1 = Dot(dv1, normal);
            vn2 = Dot(dv2, normal);

            Assert(Abs(vn1 - cp1->velocityBias) < k_errorTol);
            Assert(Abs(vn2 - cp2->velocityBias) < k_errorTol);
            #endif
            */
            break
          }

          //
          // Case 2: vn1 = 0 and x2 = 0
          //
          //   0 = a11 * x1 + a12 * 0 + b1'
          // vn2 = a21 * x1 + a22 * 0 + '
          //
          x.x = -cp1.normalMass * b.x
          x.y = 0
          vn1 = 0
          vn2 = vc.K.ex.y * x.x + b.y

          if (x.x >= 0 && vn2 >= 0) {
            // Get the incremental impulse
            // Vec2 d = x - a;
            Vec2.SubVV(x, a, d)

            // Apply incremental impulse
            // Vec2 P1 = d.x * normal;
            Vec2.MulSV(d.x, normal, P1)
            // Vec2 P2 = d.y * normal;
            Vec2.MulSV(d.y, normal, P2)
            Vec2.AddVV(P1, P2, P1P2)
            // vA -= mA * (P1 + P2);
            vA.SelfMulSub(mA, P1P2)
            // wA -= iA * (Cross(cp1->rA, P1) + Cross(cp2->rA, P2));
            wA -= iA * (Vec2.CrossVV(cp1.rA, P1) + Vec2.CrossVV(cp2.rA, P2))

            // vB += mB * (P1 + P2);
            vB.SelfMulAdd(mB, P1P2)
            // wB += iB * (Cross(cp1->rB, P1) + Cross(cp2->rB, P2));
            wB += iB * (Vec2.CrossVV(cp1.rB, P1) + Vec2.CrossVV(cp2.rB, P2))

            // Accumulate
            cp1.normalImpulse = x.x
            cp2.normalImpulse = x.y

            /*
            #if DEBUG_SOLVER === 1
            // Postconditions
            dv1 = vB + Cross(wB, cp1->rB) - vA - Cross(wA, cp1->rA);

            // Compute normal velocity
            vn1 = Dot(dv1, normal);

            Assert(Abs(vn1 - cp1->velocityBias) < k_errorTol);
            #endif
            */
            break
          }

          //
          // Case 3: vn2 = 0 and x1 = 0
          //
          // vn1 = a11 * 0 + a12 * x2 + b1'
          //   0 = a21 * 0 + a22 * x2 + '
          //
          x.x = 0
          x.y = -cp2.normalMass * b.y
          vn1 = vc.K.ey.x * x.y + b.x
          vn2 = 0

          if (x.y >= 0 && vn1 >= 0) {
            // Resubstitute for the incremental impulse
            // Vec2 d = x - a;
            Vec2.SubVV(x, a, d)

            // Apply incremental impulse
            // Vec2 P1 = d.x * normal;
            Vec2.MulSV(d.x, normal, P1)
            // Vec2 P2 = d.y * normal;
            Vec2.MulSV(d.y, normal, P2)
            Vec2.AddVV(P1, P2, P1P2)
            // vA -= mA * (P1 + P2);
            vA.SelfMulSub(mA, P1P2)
            // wA -= iA * (Cross(cp1->rA, P1) + Cross(cp2->rA, P2));
            wA -= iA * (Vec2.CrossVV(cp1.rA, P1) + Vec2.CrossVV(cp2.rA, P2))

            // vB += mB * (P1 + P2);
            vB.SelfMulAdd(mB, P1P2)
            // wB += iB * (Cross(cp1->rB, P1) + Cross(cp2->rB, P2));
            wB += iB * (Vec2.CrossVV(cp1.rB, P1) + Vec2.CrossVV(cp2.rB, P2))

            // Accumulate
            cp1.normalImpulse = x.x
            cp2.normalImpulse = x.y

            /*
            #if DEBUG_SOLVER === 1
            // Postconditions
            dv2 = vB + Cross(wB, cp2->rB) - vA - Cross(wA, cp2->rA);

            // Compute normal velocity
            vn2 = Dot(dv2, normal);

            Assert(Abs(vn2 - cp2->velocityBias) < k_errorTol);
            #endif
            */
            break
          }

          //
          // Case 4: x1 = 0 and x2 = 0
          //
          // vn1 = b1
          // vn2 = ;
          x.x = 0
          x.y = 0
          vn1 = b.x
          vn2 = b.y

          if (vn1 >= 0 && vn2 >= 0) {
            // Resubstitute for the incremental impulse
            // Vec2 d = x - a;
            Vec2.SubVV(x, a, d)

            // Apply incremental impulse
            // Vec2 P1 = d.x * normal;
            Vec2.MulSV(d.x, normal, P1)
            // Vec2 P2 = d.y * normal;
            Vec2.MulSV(d.y, normal, P2)
            Vec2.AddVV(P1, P2, P1P2)
            // vA -= mA * (P1 + P2);
            vA.SelfMulSub(mA, P1P2)
            // wA -= iA * (Cross(cp1->rA, P1) + Cross(cp2->rA, P2));
            wA -= iA * (Vec2.CrossVV(cp1.rA, P1) + Vec2.CrossVV(cp2.rA, P2))

            // vB += mB * (P1 + P2);
            vB.SelfMulAdd(mB, P1P2)
            // wB += iB * (Cross(cp1->rB, P1) + Cross(cp2->rB, P2));
            wB += iB * (Vec2.CrossVV(cp1.rB, P1) + Vec2.CrossVV(cp2.rB, P2))

            // Accumulate
            cp1.normalImpulse = x.x
            cp2.normalImpulse = x.y

            break
          }

          // No solution, give up. This is hit sometimes, but it doesn't seem to matter.
          break
        }
      }

      // this.m_velocities[indexA].v = vA;
      this.m_velocities[indexA].w = wA
      // this.m_velocities[indexB].v = vB;
      this.m_velocities[indexB].w = wB
    }
  }

  StoreImpulses(): void {
    for (let i: number = 0; i < this.m_count; ++i) {
      const vc: ContactVelocityConstraint = this.m_velocityConstraints[i]
      const manifold: Manifold = this.m_contacts[vc.contactIndex].GetManifold()

      for (let j: number = 0; j < vc.pointCount; ++j) {
        manifold.points[j].normalImpulse = vc.points[j].normalImpulse
        manifold.points[j].tangentImpulse = vc.points[j].tangentImpulse
      }
    }
  }
  SolvePositionConstraints(): boolean {
    const xfA: Transform = ContactSolver.SolvePositionConstraints_s_xfA
    const xfB: Transform = ContactSolver.SolvePositionConstraints_s_xfB
    const psm: PositionSolverManifold =
      ContactSolver.SolvePositionConstraints_s_psm
    const rA: Vec2 = ContactSolver.SolvePositionConstraints_s_rA
    const rB: Vec2 = ContactSolver.SolvePositionConstraints_s_rB
    const P: Vec2 = ContactSolver.SolvePositionConstraints_s_P

    let minSeparation: number = 0

    for (let i: number = 0; i < this.m_count; ++i) {
      const pc: ContactPositionConstraint = this.m_positionConstraints[i]

      const indexA: number = pc.indexA
      const indexB: number = pc.indexB
      const localCenterA: Vec2 = pc.localCenterA
      const mA: number = pc.invMassA
      const iA: number = pc.invIA
      const localCenterB: Vec2 = pc.localCenterB
      const mB: number = pc.invMassB
      const iB: number = pc.invIB
      const pointCount: number = pc.pointCount

      const cA: Vec2 = this.m_positions[indexA].c
      let aA: number = this.m_positions[indexA].a

      const cB: Vec2 = this.m_positions[indexB].c
      let aB: number = this.m_positions[indexB].a

      // Solve normal constraints
      for (let j: number = 0; j < pointCount; ++j) {
        xfA.q.SetAngle(aA)
        xfB.q.SetAngle(aB)
        Vec2.SubVV(cA, Rot.MulRV(xfA.q, localCenterA, Vec2.s_t0), xfA.p)
        Vec2.SubVV(cB, Rot.MulRV(xfB.q, localCenterB, Vec2.s_t0), xfB.p)

        psm.Initialize(pc, xfA, xfB, j)
        const normal: Vec2 = psm.normal

        const point: Vec2 = psm.point
        const separation: number = psm.separation

        // Vec2 rA = point - cA;
        Vec2.SubVV(point, cA, rA)
        // Vec2 rB = point - cB;
        Vec2.SubVV(point, cB, rB)

        // Track max constraint error.
        minSeparation = Min(minSeparation, separation)

        // Prevent large corrections and allow slop.
        const C: number = Clamp(
          baumgarte * (separation + linearSlop),
          -maxLinearCorrection,
          0
        )

        // Compute the effective mass.
        // float32 rnA = Cross(rA, normal);
        const rnA: number = Vec2.CrossVV(rA, normal)
        // float32 rnB = Cross(rB, normal);
        const rnB: number = Vec2.CrossVV(rB, normal)
        // float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
        const K: number = mA + mB + iA * rnA * rnA + iB * rnB * rnB

        // Compute normal impulse
        const impulse: number = K > 0 ? -C / K : 0

        // Vec2 P = impulse * normal;
        Vec2.MulSV(impulse, normal, P)

        // cA -= mA * P;
        cA.SelfMulSub(mA, P)
        // aA -= iA * Cross(rA, P);
        aA -= iA * Vec2.CrossVV(rA, P)

        // cB += mB * P;
        cB.SelfMulAdd(mB, P)
        // aB += iB * Cross(rB, P);
        aB += iB * Vec2.CrossVV(rB, P)
      }

      // this.m_positions[indexA].c = cA;
      this.m_positions[indexA].a = aA

      // this.m_positions[indexB].c = cB;
      this.m_positions[indexB].a = aB
    }

    // We can't expect minSpeparation >= -linearSlop because we don't
    // push the separation above -linearSlop.
    return minSeparation > -3 * linearSlop
  }
  SolveTOIPositionConstraints(toiIndexA: number, toiIndexB: number): boolean {
    const xfA: Transform = ContactSolver.SolveTOIPositionConstraints_s_xfA
    const xfB: Transform = ContactSolver.SolveTOIPositionConstraints_s_xfB
    const psm: PositionSolverManifold =
      ContactSolver.SolveTOIPositionConstraints_s_psm
    const rA: Vec2 = ContactSolver.SolveTOIPositionConstraints_s_rA
    const rB: Vec2 = ContactSolver.SolveTOIPositionConstraints_s_rB
    const P: Vec2 = ContactSolver.SolveTOIPositionConstraints_s_P

    let minSeparation: number = 0

    for (let i: number = 0; i < this.m_count; ++i) {
      const pc: ContactPositionConstraint = this.m_positionConstraints[i]

      const indexA: number = pc.indexA
      const indexB: number = pc.indexB
      const localCenterA: Vec2 = pc.localCenterA
      const localCenterB: Vec2 = pc.localCenterB
      const pointCount: number = pc.pointCount

      let mA: number = 0
      let iA: number = 0
      if (indexA === toiIndexA || indexA === toiIndexB) {
        mA = pc.invMassA
        iA = pc.invIA
      }

      let mB: number = 0
      let iB: number = 0
      if (indexB === toiIndexA || indexB === toiIndexB) {
        mB = pc.invMassB
        iB = pc.invIB
      }

      const cA: Vec2 = this.m_positions[indexA].c
      let aA: number = this.m_positions[indexA].a

      const cB: Vec2 = this.m_positions[indexB].c
      let aB: number = this.m_positions[indexB].a

      // Solve normal constraints
      for (let j: number = 0; j < pointCount; ++j) {
        xfA.q.SetAngle(aA)
        xfB.q.SetAngle(aB)
        Vec2.SubVV(cA, Rot.MulRV(xfA.q, localCenterA, Vec2.s_t0), xfA.p)
        Vec2.SubVV(cB, Rot.MulRV(xfB.q, localCenterB, Vec2.s_t0), xfB.p)

        psm.Initialize(pc, xfA, xfB, j)
        const normal: Vec2 = psm.normal

        const point: Vec2 = psm.point
        const separation: number = psm.separation

        // Vec2 rA = point - cA;
        Vec2.SubVV(point, cA, rA)
        // Vec2 rB = point - cB;
        Vec2.SubVV(point, cB, rB)

        // Track max constraint error.
        minSeparation = Min(minSeparation, separation)

        // Prevent large corrections and allow slop.
        const C: number = Clamp(
          toiBaumgarte * (separation + linearSlop),
          -maxLinearCorrection,
          0
        )

        // Compute the effective mass.
        // float32 rnA = Cross(rA, normal);
        const rnA: number = Vec2.CrossVV(rA, normal)
        // float32 rnB = Cross(rB, normal);
        const rnB: number = Vec2.CrossVV(rB, normal)
        // float32 K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;
        const K: number = mA + mB + iA * rnA * rnA + iB * rnB * rnB

        // Compute normal impulse
        const impulse: number = K > 0 ? -C / K : 0

        // Vec2 P = impulse * normal;
        Vec2.MulSV(impulse, normal, P)

        // cA -= mA * P;
        cA.SelfMulSub(mA, P)
        // aA -= iA * Cross(rA, P);
        aA -= iA * Vec2.CrossVV(rA, P)

        // cB += mB * P;
        cB.SelfMulAdd(mB, P)
        // aB += iB * Cross(rB, P);
        aB += iB * Vec2.CrossVV(rB, P)
      }

      // this.m_positions[indexA].c = cA;
      this.m_positions[indexA].a = aA

      // this.m_positions[indexB].c = cB;
      this.m_positions[indexB].a = aB
    }

    // We can't expect minSpeparation >= -linearSlop because we don't
    // push the separation above -linearSlop.
    return minSeparation >= -1.5 * linearSlop
  }
}
