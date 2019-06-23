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

import {
  ContactID,
  Manifold,
  ManifoldPoint,
  TestOverlapShape,
  WorldManifold
} from '../../Collision/Collision'
import { Shape } from '../../Collision/Shapes/Shape'
import { TimeOfImpact, TOIInput, TOIOutput } from '../../Collision/TimeOfImpact'
import { Sqrt, Sweep, Transform } from '../../Common/Math'
import { linearSlop } from '../../Common/Settings'
import { Body } from '../Body'
import { Fixture } from '../Fixture'
import { ContactListener } from '../WorldCallbacks'

/// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
/// For example, anything slides on ice.
export function MixFriction(friction1: number, friction2: number): number {
  return Sqrt(friction1 * friction2)
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
export function MixRestitution(
  restitution1: number,
  restitution2: number
): number {
  return restitution1 > restitution2 ? restitution1 : restitution2
}

export class ContactEdge {
  other!: Body ///< provides quick access to the other body attached.
  contact: Contact ///< the contact
  prev: ContactEdge | null = null ///< the previous contact edge in the body's contact list
  next: ContactEdge | null = null ///< the next contact edge in the body's contact list
  constructor(contact: Contact) {
    this.contact = contact
  }
}

export abstract class Contact {
  private static ComputeTOI_s_input = new TOIInput()
  private static ComputeTOI_s_output = new TOIOutput()
  m_islandFlag: boolean = false /// Used when crawling contact graph when forming islands.
  m_touchingFlag: boolean = false /// Set when the shapes are touching.
  m_enabledFlag: boolean = false /// This contact can be disabled (by user)
  m_filterFlag: boolean = false /// This contact needs filtering because a fixture filter was changed.
  m_bulletHitFlag: boolean = false /// This bullet contact had a TOI event
  m_toiFlag: boolean = false /// This contact has a valid TOI in m_toi

  m_prev: Contact | null = null
  m_next: Contact | null = null

  readonly m_nodeA: ContactEdge // = new ContactEdge(this);
  readonly m_nodeB: ContactEdge // = new ContactEdge(this);

  m_fixtureA!: Fixture
  m_fixtureB!: Fixture

  m_indexA: number = 0
  m_indexB: number = 0

  m_manifold: Manifold = new Manifold() // TODO: readonly

  m_toiCount: number = 0
  m_toi: number = 0

  m_friction: number = 0
  m_restitution: number = 0

  m_tangentSpeed: number = 0

  m_oldManifold: Manifold = new Manifold() // TODO: readonly

  constructor() {
    this.m_nodeA = new ContactEdge(this)
    this.m_nodeB = new ContactEdge(this)
  }

  GetManifold() {
    return this.m_manifold
  }

  GetWorldManifold(worldManifold: WorldManifold): void {
    const bodyA: Body = this.m_fixtureA.GetBody()
    const bodyB: Body = this.m_fixtureB.GetBody()
    const shapeA: Shape = this.m_fixtureA.GetShape()
    const shapeB: Shape = this.m_fixtureB.GetShape()
    worldManifold.Initialize(
      this.m_manifold,
      bodyA.GetTransform(),
      shapeA.m_radius,
      bodyB.GetTransform(),
      shapeB.m_radius
    )
  }

  IsTouching(): boolean {
    return this.m_touchingFlag
  }

  SetEnabled(flag: boolean): void {
    this.m_enabledFlag = flag
  }

  IsEnabled(): boolean {
    return this.m_enabledFlag
  }

  GetNext(): Contact | null {
    return this.m_next
  }

  GetFixtureA(): Fixture {
    return this.m_fixtureA
  }

  GetChildIndexA(): number {
    return this.m_indexA
  }

  GetFixtureB(): Fixture {
    return this.m_fixtureB
  }

  GetChildIndexB(): number {
    return this.m_indexB
  }

  abstract Evaluate(manifold: Manifold, xfA: Transform, xfB: Transform): void

  FlagForFiltering(): void {
    this.m_filterFlag = true
  }

  SetFriction(friction: number): void {
    this.m_friction = friction
  }

  GetFriction(): number {
    return this.m_friction
  }

  ResetFriction(): void {
    this.m_friction = MixFriction(
      this.m_fixtureA.m_friction,
      this.m_fixtureB.m_friction
    )
  }

  SetRestitution(restitution: number): void {
    this.m_restitution = restitution
  }

  GetRestitution(): number {
    return this.m_restitution
  }

  ResetRestitution(): void {
    this.m_restitution = MixRestitution(
      this.m_fixtureA.m_restitution,
      this.m_fixtureB.m_restitution
    )
  }

  SetTangentSpeed(speed: number): void {
    this.m_tangentSpeed = speed
  }

  GetTangentSpeed(): number {
    return this.m_tangentSpeed
  }

  Reset(
    fixtureA: Fixture,
    indexA: number,
    fixtureB: Fixture,
    indexB: number
  ): void {
    this.m_islandFlag = false
    this.m_touchingFlag = false
    this.m_enabledFlag = true
    this.m_filterFlag = false
    this.m_bulletHitFlag = false
    this.m_toiFlag = false

    this.m_fixtureA = fixtureA
    this.m_fixtureB = fixtureB

    this.m_indexA = indexA
    this.m_indexB = indexB

    this.m_manifold.pointCount = 0

    this.m_prev = null
    this.m_next = null

    delete this.m_nodeA.contact // = null;
    this.m_nodeA.prev = null
    this.m_nodeA.next = null
    delete this.m_nodeA.other // = null;

    delete this.m_nodeB.contact // = null;
    this.m_nodeB.prev = null
    this.m_nodeB.next = null
    delete this.m_nodeB.other // = null;

    this.m_toiCount = 0

    this.m_friction = MixFriction(
      this.m_fixtureA.m_friction,
      this.m_fixtureB.m_friction
    )
    this.m_restitution = MixRestitution(
      this.m_fixtureA.m_restitution,
      this.m_fixtureB.m_restitution
    )
  }

  Update(listener: ContactListener): void {
    const tManifold: Manifold = this.m_oldManifold
    this.m_oldManifold = this.m_manifold
    this.m_manifold = tManifold

    // Re-enable this contact.
    this.m_enabledFlag = true

    let touching: boolean = false
    const wasTouching: boolean = this.m_touchingFlag

    const sensorA: boolean = this.m_fixtureA.IsSensor()
    const sensorB: boolean = this.m_fixtureB.IsSensor()
    const sensor: boolean = sensorA || sensorB

    const bodyA: Body = this.m_fixtureA.GetBody()
    const bodyB: Body = this.m_fixtureB.GetBody()
    const xfA: Transform = bodyA.GetTransform()
    const xfB: Transform = bodyB.GetTransform()

    ///const aabbOverlap = TestOverlapAABB(this.m_fixtureA.GetAABB(0), this.m_fixtureB.GetAABB(0));

    // Is this contact a sensor?
    if (sensor) {
      ///if (aabbOverlap)
      ///{
      const shapeA: Shape = this.m_fixtureA.GetShape()
      const shapeB: Shape = this.m_fixtureB.GetShape()
      touching = TestOverlapShape(
        shapeA,
        this.m_indexA,
        shapeB,
        this.m_indexB,
        xfA,
        xfB
      )
      ///}

      // Sensors don't generate manifolds.
      this.m_manifold.pointCount = 0
    } else {
      ///if (aabbOverlap)
      ///{
      this.Evaluate(this.m_manifold, xfA, xfB)
      touching = this.m_manifold.pointCount > 0

      // Match old contact ids to new contact ids and copy the
      // stored impulses to warm start the solver.
      for (let i: number = 0; i < this.m_manifold.pointCount; ++i) {
        const mp2: ManifoldPoint = this.m_manifold.points[i]
        mp2.normalImpulse = 0
        mp2.tangentImpulse = 0
        const id2: ContactID = mp2.id

        for (let j: number = 0; j < this.m_oldManifold.pointCount; ++j) {
          const mp1: ManifoldPoint = this.m_oldManifold.points[j]

          if (mp1.id.key === id2.key) {
            mp2.normalImpulse = mp1.normalImpulse
            mp2.tangentImpulse = mp1.tangentImpulse
            break
          }
        }
      }
      ///}
      ///else
      ///{
      ///  this.m_manifold.pointCount = 0;
      ///}

      if (touching !== wasTouching) {
        bodyA.SetAwake(true)
        bodyB.SetAwake(true)
      }
    }

    this.m_touchingFlag = touching

    if (!wasTouching && touching && listener) {
      listener.BeginContact(this)
    }

    if (wasTouching && !touching && listener) {
      listener.EndContact(this)
    }

    if (!sensor && touching && listener) {
      listener.PreSolve(this, this.m_oldManifold)
    }
  }
  ComputeTOI(sweepA: Sweep, sweepB: Sweep): number {
    const input: TOIInput = Contact.ComputeTOI_s_input
    input.proxyA.SetShape(this.m_fixtureA.GetShape(), this.m_indexA)
    input.proxyB.SetShape(this.m_fixtureB.GetShape(), this.m_indexB)
    input.sweepA.Copy(sweepA)
    input.sweepB.Copy(sweepB)
    input.tMax = linearSlop

    const output: TOIOutput = Contact.ComputeTOI_s_output

    TimeOfImpact(output, input)

    return output.t
  }
}
