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

// DEBUG: import { Assert } from "../../Common/Settings";
import { Vec2, XY } from '../../Common/Math'
import { Maybe } from '../../Common/Settings'
import { Body } from '../Body'
import { SolverData } from '../TimeStep'

export enum JointType {
  e_unknownJoint = 0,
  e_revoluteJoint = 1,
  e_prismaticJoint = 2,
  e_distanceJoint = 3,
  e_pulleyJoint = 4,
  e_mouseJoint = 5,
  e_gearJoint = 6,
  e_wheelJoint = 7,
  e_weldJoint = 8,
  e_frictionJoint = 9,
  e_ropeJoint = 10,
  e_motorJoint = 11,
  e_areaJoint = 12
}

export enum LimitState {
  e_inactiveLimit = 0,
  e_atLowerLimit = 1,
  e_atUpperLimit = 2,
  e_equalLimits = 3
}

export class Jacobian {
  readonly linear: Vec2 = new Vec2()
  angularA: number = 0
  angularB: number = 0

  SetZero(): Jacobian {
    this.linear.SetZero()
    this.angularA = 0
    this.angularB = 0
    return this
  }

  Set(x: XY, a1: number, a2: number): Jacobian {
    this.linear.Copy(x)
    this.angularA = a1
    this.angularB = a2
    return this
  }
}

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
export class JointEdge {
  readonly other: Body ///< provides quick access to the other body attached.
  readonly joint: Joint ///< the joint
  prev: JointEdge | null = null ///< the previous joint edge in the body's joint list
  next: JointEdge | null = null ///< the next joint edge in the body's joint list
  constructor(joint: Joint, other: Body) {
    this.joint = joint
    this.other = other
  }
}

/// Joint definitions are used to construct joints.
export interface IJointDef {
  /// The joint type is set automatically for concrete joint types.
  type: JointType

  /// Use this to attach application specific data to your joints.
  userData?: any

  /// The first attached body.
  bodyA: Body

  /// The second attached body.
  bodyB: Body

  /// Set this flag to true if the attached bodies should collide.
  collideConnected?: boolean
}

/// Joint definitions are used to construct joints.
export class JointDef {
  /// The joint type is set automatically for concrete joint types.
  readonly type: JointType = JointType.e_unknownJoint

  /// Use this to attach application specific data to your joints.
  userData: any = null

  /// The first attached body.
  bodyA!: Body

  /// The second attached body.
  bodyB!: Body

  /// Set this flag to true if the attached bodies should collide.
  collideConnected: boolean = false

  constructor(type: JointType) {
    this.type = type
  }
}

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
export abstract class Joint {
  readonly m_type: JointType = JointType.e_unknownJoint
  m_prev: Joint | null = null
  m_next: Joint | null = null
  readonly m_edgeA: JointEdge
  readonly m_edgeB: JointEdge
  m_bodyA: Body
  m_bodyB: Body

  m_index: number = 0

  m_islandFlag: boolean = false
  m_collideConnected: boolean = false

  m_userData: any = null

  constructor(def: IJointDef) {
    // DEBUG: Assert(def.bodyA !== def.bodyB);

    this.m_type = def.type
    this.m_edgeA = new JointEdge(this, def.bodyB)
    this.m_edgeB = new JointEdge(this, def.bodyA)
    this.m_bodyA = def.bodyA
    this.m_bodyB = def.bodyB

    this.m_collideConnected = Maybe(def.collideConnected, false)

    this.m_userData = def.userData
  }

  /// Get the type of the concrete joint.
  GetType(): JointType {
    return this.m_type
  }

  /// Get the first body attached to this joint.
  GetBodyA(): Body {
    return this.m_bodyA
  }

  /// Get the second body attached to this joint.
  GetBodyB(): Body {
    return this.m_bodyB
  }

  /// Get the anchor point on bodyA in world coordinates.
  abstract GetAnchorA<T extends XY>(out: T): T

  /// Get the anchor point on bodyB in world coordinates.
  abstract GetAnchorB<T extends XY>(out: T): T

  /// Get the reaction force on bodyB at the joint anchor in Newtons.
  abstract GetReactionForce<T extends XY>(inv_dt: number, out: T): T

  /// Get the reaction torque on bodyB in N*m.
  abstract GetReactionTorque(inv_dt: number): number

  /// Get the next joint the world joint list.
  GetNext(): Joint | null {
    return this.m_next
  }

  /// Get the user data pointer.
  GetUserData(): any {
    return this.m_userData
  }

  /// Set the user data pointer.
  SetUserData(data: any): void {
    this.m_userData = data
  }

  /// Short-cut function to determine if either body is inactive.
  IsActive(): boolean {
    return this.m_bodyA.IsActive() && this.m_bodyB.IsActive()
  }

  /// Get collide connected.
  /// Note: modifying the collide connect flag won't work correctly because
  /// the flag is only checked when fixture AABBs begin to overlap.
  GetCollideConnected(): boolean {
    return this.m_collideConnected
  }

  /// Dump this joint to the log file.
  Dump(log: (format: string, ...args: any[]) => void): void {
    log('// Dump is not supported for this joint type.\n')
  }

  /// Shift the origin for any points stored in world coordinates.
  // tslint:disable-next-line
  ShiftOrigin(newOrigin: XY): void {}

  abstract InitVelocityConstraints(data: SolverData): void

  abstract SolveVelocityConstraints(data: SolverData): void

  // This returns true if the position errors are within tolerance.
  abstract SolvePositionConstraints(data: SolverData): boolean
}
