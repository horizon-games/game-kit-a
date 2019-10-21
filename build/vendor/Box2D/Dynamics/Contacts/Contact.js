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
import { Manifold, TestOverlapShape } from '../../Collision/Collision';
import { TimeOfImpact, TOIInput, TOIOutput } from '../../Collision/TimeOfImpact';
import { Sqrt } from '../../Common/Math';
import { linearSlop } from '../../Common/Settings';
/// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
/// For example, anything slides on ice.
export function MixFriction(friction1, friction2) {
    return Sqrt(friction1 * friction2);
}
/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
export function MixRestitution(restitution1, restitution2) {
    return restitution1 > restitution2 ? restitution1 : restitution2;
}
export class ContactEdge {
    constructor(contact) {
        this.prev = null; ///< the previous contact edge in the body's contact list
        this.next = null; ///< the next contact edge in the body's contact list
        this.contact = contact;
    }
}
export class Contact {
    constructor() {
        this.m_islandFlag = false; /// Used when crawling contact graph when forming islands.
        this.m_touchingFlag = false; /// Set when the shapes are touching.
        this.m_enabledFlag = false; /// This contact can be disabled (by user)
        this.m_filterFlag = false; /// This contact needs filtering because a fixture filter was changed.
        this.m_bulletHitFlag = false; /// This bullet contact had a TOI event
        this.m_toiFlag = false; /// This contact has a valid TOI in m_toi
        this.m_prev = null;
        this.m_next = null;
        this.m_indexA = 0;
        this.m_indexB = 0;
        this.m_manifold = new Manifold(); // TODO: readonly
        this.m_toiCount = 0;
        this.m_toi = 0;
        this.m_friction = 0;
        this.m_restitution = 0;
        this.m_tangentSpeed = 0;
        this.m_oldManifold = new Manifold(); // TODO: readonly
        this.m_nodeA = new ContactEdge(this);
        this.m_nodeB = new ContactEdge(this);
    }
    GetManifold() {
        return this.m_manifold;
    }
    GetWorldManifold(worldManifold) {
        const bodyA = this.m_fixtureA.GetBody();
        const bodyB = this.m_fixtureB.GetBody();
        const shapeA = this.m_fixtureA.GetShape();
        const shapeB = this.m_fixtureB.GetShape();
        worldManifold.Initialize(this.m_manifold, bodyA.GetTransform(), shapeA.m_radius, bodyB.GetTransform(), shapeB.m_radius);
    }
    IsTouching() {
        return this.m_touchingFlag;
    }
    SetEnabled(flag) {
        this.m_enabledFlag = flag;
    }
    IsEnabled() {
        return this.m_enabledFlag;
    }
    GetNext() {
        return this.m_next;
    }
    GetFixtureA() {
        return this.m_fixtureA;
    }
    GetChildIndexA() {
        return this.m_indexA;
    }
    GetFixtureB() {
        return this.m_fixtureB;
    }
    GetChildIndexB() {
        return this.m_indexB;
    }
    FlagForFiltering() {
        this.m_filterFlag = true;
    }
    SetFriction(friction) {
        this.m_friction = friction;
    }
    GetFriction() {
        return this.m_friction;
    }
    ResetFriction() {
        this.m_friction = MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
    }
    SetRestitution(restitution) {
        this.m_restitution = restitution;
    }
    GetRestitution() {
        return this.m_restitution;
    }
    ResetRestitution() {
        this.m_restitution = MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
    }
    SetTangentSpeed(speed) {
        this.m_tangentSpeed = speed;
    }
    GetTangentSpeed() {
        return this.m_tangentSpeed;
    }
    Reset(fixtureA, indexA, fixtureB, indexB) {
        this.m_islandFlag = false;
        this.m_touchingFlag = false;
        this.m_enabledFlag = true;
        this.m_filterFlag = false;
        this.m_bulletHitFlag = false;
        this.m_toiFlag = false;
        this.m_fixtureA = fixtureA;
        this.m_fixtureB = fixtureB;
        this.m_indexA = indexA;
        this.m_indexB = indexB;
        this.m_manifold.pointCount = 0;
        this.m_prev = null;
        this.m_next = null;
        delete this.m_nodeA.contact; // = null;
        this.m_nodeA.prev = null;
        this.m_nodeA.next = null;
        delete this.m_nodeA.other; // = null;
        delete this.m_nodeB.contact; // = null;
        this.m_nodeB.prev = null;
        this.m_nodeB.next = null;
        delete this.m_nodeB.other; // = null;
        this.m_toiCount = 0;
        this.m_friction = MixFriction(this.m_fixtureA.m_friction, this.m_fixtureB.m_friction);
        this.m_restitution = MixRestitution(this.m_fixtureA.m_restitution, this.m_fixtureB.m_restitution);
    }
    Update(listener) {
        const tManifold = this.m_oldManifold;
        this.m_oldManifold = this.m_manifold;
        this.m_manifold = tManifold;
        // Re-enable this contact.
        this.m_enabledFlag = true;
        let touching = false;
        const wasTouching = this.m_touchingFlag;
        const sensorA = this.m_fixtureA.IsSensor();
        const sensorB = this.m_fixtureB.IsSensor();
        const sensor = sensorA || sensorB;
        const bodyA = this.m_fixtureA.GetBody();
        const bodyB = this.m_fixtureB.GetBody();
        const xfA = bodyA.GetTransform();
        const xfB = bodyB.GetTransform();
        ///const aabbOverlap = TestOverlapAABB(this.m_fixtureA.GetAABB(0), this.m_fixtureB.GetAABB(0));
        // Is this contact a sensor?
        if (sensor) {
            ///if (aabbOverlap)
            ///{
            const shapeA = this.m_fixtureA.GetShape();
            const shapeB = this.m_fixtureB.GetShape();
            touching = TestOverlapShape(shapeA, this.m_indexA, shapeB, this.m_indexB, xfA, xfB);
            ///}
            // Sensors don't generate manifolds.
            this.m_manifold.pointCount = 0;
        }
        else {
            ///if (aabbOverlap)
            ///{
            this.Evaluate(this.m_manifold, xfA, xfB);
            touching = this.m_manifold.pointCount > 0;
            // Match old contact ids to new contact ids and copy the
            // stored impulses to warm start the solver.
            for (let i = 0; i < this.m_manifold.pointCount; ++i) {
                const mp2 = this.m_manifold.points[i];
                mp2.normalImpulse = 0;
                mp2.tangentImpulse = 0;
                const id2 = mp2.id;
                for (let j = 0; j < this.m_oldManifold.pointCount; ++j) {
                    const mp1 = this.m_oldManifold.points[j];
                    if (mp1.id.key === id2.key) {
                        mp2.normalImpulse = mp1.normalImpulse;
                        mp2.tangentImpulse = mp1.tangentImpulse;
                        break;
                    }
                }
            }
            ///}
            ///else
            ///{
            ///  this.m_manifold.pointCount = 0;
            ///}
            if (touching !== wasTouching) {
                bodyA.SetAwake(true);
                bodyB.SetAwake(true);
            }
        }
        this.m_touchingFlag = touching;
        if (!wasTouching && touching && listener) {
            listener.BeginContact(this);
        }
        if (wasTouching && !touching && listener) {
            listener.EndContact(this);
        }
        if (!sensor && touching && listener) {
            listener.PreSolve(this, this.m_oldManifold);
        }
    }
    ComputeTOI(sweepA, sweepB) {
        const input = Contact.ComputeTOI_s_input;
        input.proxyA.SetShape(this.m_fixtureA.GetShape(), this.m_indexA);
        input.proxyB.SetShape(this.m_fixtureB.GetShape(), this.m_indexB);
        input.sweepA.Copy(sweepA);
        input.sweepB.Copy(sweepB);
        input.tMax = linearSlop;
        const output = Contact.ComputeTOI_s_output;
        TimeOfImpact(output, input);
        return output.t;
    }
}
Contact.ComputeTOI_s_input = new TOIInput();
Contact.ComputeTOI_s_output = new TOIOutput();
//# sourceMappingURL=Contact.js.map