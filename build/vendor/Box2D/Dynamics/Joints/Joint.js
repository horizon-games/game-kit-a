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
import { Vec2 } from '../../Common/Math';
import { Maybe } from '../../Common/Settings';
export var JointType;
(function (JointType) {
    JointType[JointType["e_unknownJoint"] = 0] = "e_unknownJoint";
    JointType[JointType["e_revoluteJoint"] = 1] = "e_revoluteJoint";
    JointType[JointType["e_prismaticJoint"] = 2] = "e_prismaticJoint";
    JointType[JointType["e_distanceJoint"] = 3] = "e_distanceJoint";
    JointType[JointType["e_pulleyJoint"] = 4] = "e_pulleyJoint";
    JointType[JointType["e_mouseJoint"] = 5] = "e_mouseJoint";
    JointType[JointType["e_gearJoint"] = 6] = "e_gearJoint";
    JointType[JointType["e_wheelJoint"] = 7] = "e_wheelJoint";
    JointType[JointType["e_weldJoint"] = 8] = "e_weldJoint";
    JointType[JointType["e_frictionJoint"] = 9] = "e_frictionJoint";
    JointType[JointType["e_ropeJoint"] = 10] = "e_ropeJoint";
    JointType[JointType["e_motorJoint"] = 11] = "e_motorJoint";
    JointType[JointType["e_areaJoint"] = 12] = "e_areaJoint";
})(JointType || (JointType = {}));
export var LimitState;
(function (LimitState) {
    LimitState[LimitState["e_inactiveLimit"] = 0] = "e_inactiveLimit";
    LimitState[LimitState["e_atLowerLimit"] = 1] = "e_atLowerLimit";
    LimitState[LimitState["e_atUpperLimit"] = 2] = "e_atUpperLimit";
    LimitState[LimitState["e_equalLimits"] = 3] = "e_equalLimits";
})(LimitState || (LimitState = {}));
export class Jacobian {
    constructor() {
        this.linear = new Vec2();
        this.angularA = 0;
        this.angularB = 0;
    }
    SetZero() {
        this.linear.SetZero();
        this.angularA = 0;
        this.angularB = 0;
        return this;
    }
    Set(x, a1, a2) {
        this.linear.Copy(x);
        this.angularA = a1;
        this.angularB = a2;
        return this;
    }
}
/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
export class JointEdge {
    constructor(joint, other) {
        this.prev = null; ///< the previous joint edge in the body's joint list
        this.next = null; ///< the next joint edge in the body's joint list
        this.joint = joint;
        this.other = other;
    }
}
/// Joint definitions are used to construct joints.
export class JointDef {
    constructor(type) {
        /// The joint type is set automatically for concrete joint types.
        this.type = JointType.e_unknownJoint;
        /// Use this to attach application specific data to your joints.
        this.userData = null;
        /// Set this flag to true if the attached bodies should collide.
        this.collideConnected = false;
        this.type = type;
    }
}
/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
export class Joint {
    constructor(def) {
        // DEBUG: Assert(def.bodyA !== def.bodyB);
        this.m_type = JointType.e_unknownJoint;
        this.m_prev = null;
        this.m_next = null;
        this.m_index = 0;
        this.m_islandFlag = false;
        this.m_collideConnected = false;
        this.m_userData = null;
        this.m_type = def.type;
        this.m_edgeA = new JointEdge(this, def.bodyB);
        this.m_edgeB = new JointEdge(this, def.bodyA);
        this.m_bodyA = def.bodyA;
        this.m_bodyB = def.bodyB;
        this.m_collideConnected = Maybe(def.collideConnected, false);
        this.m_userData = def.userData;
    }
    /// Get the type of the concrete joint.
    GetType() {
        return this.m_type;
    }
    /// Get the first body attached to this joint.
    GetBodyA() {
        return this.m_bodyA;
    }
    /// Get the second body attached to this joint.
    GetBodyB() {
        return this.m_bodyB;
    }
    /// Get the next joint the world joint list.
    GetNext() {
        return this.m_next;
    }
    /// Get the user data pointer.
    GetUserData() {
        return this.m_userData;
    }
    /// Set the user data pointer.
    SetUserData(data) {
        this.m_userData = data;
    }
    /// Short-cut function to determine if either body is inactive.
    IsActive() {
        return this.m_bodyA.IsActive() && this.m_bodyB.IsActive();
    }
    /// Get collide connected.
    /// Note: modifying the collide connect flag won't work correctly because
    /// the flag is only checked when fixture AABBs begin to overlap.
    GetCollideConnected() {
        return this.m_collideConnected;
    }
    /// Dump this joint to the log file.
    Dump(log) {
        log('// Dump is not supported for this joint type.\n');
    }
    /// Shift the origin for any points stored in world coordinates.
    // tslint:disable-next-line
    ShiftOrigin(newOrigin) { }
}
//# sourceMappingURL=Joint.js.map