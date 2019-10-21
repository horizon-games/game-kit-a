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
import { AABB } from '../Collision/Collision';
import { MassData } from '../Collision/Shapes/Shape';
import { Vec2 } from '../Common/Math';
import { MakeArray, Maybe } from '../Common/Settings';
/// This holds contact filtering data.
export class Filter {
    constructor() {
        /// The collision category bits. Normally you would just set one bit.
        this.categoryBits = 0x0001;
        /// The collision mask bits. This states the categories that this
        /// shape would accept for collision.
        this.maskBits = 0xffff;
        /// Collision groups allow a certain group of objects to never collide (negative)
        /// or always collide (positive). Zero means no collision group. Non-zero group
        /// filtering always wins against the mask bits.
        this.groupIndex = 0;
    }
    Clone() {
        return new Filter().Copy(this);
    }
    Copy(other) {
        // DEBUG: Assert(this !== other);
        this.categoryBits = other.categoryBits;
        this.maskBits = other.maskBits;
        this.groupIndex = other.groupIndex || 0;
        return this;
    }
}
Filter.DEFAULT = new Filter();
/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
export class FixtureDef {
    constructor() {
        /// Use this to store application specific fixture data.
        this.userData = null;
        /// The friction coefficient, usually in the range [0,1].
        this.friction = 0.2;
        /// The restitution (elasticity) usually in the range [0,1].
        this.restitution = 0;
        /// The density, usually in kg/m^2.
        this.density = 0;
        /// A sensor shape collects contact information but never generates a collision
        /// response.
        this.isSensor = false;
        /// Contact filtering data.
        this.filter = new Filter();
    }
}
/// This proxy is used internally to connect fixtures to the broad-phase.
export class FixtureProxy {
    constructor(fixture) {
        this.aabb = new AABB();
        this.childIndex = 0;
        this.fixture = fixture;
    }
}
/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via Body::CreateFixture.
/// @warning you cannot reuse fixtures.
export class Fixture {
    constructor(def, body) {
        this.m_density = 0;
        this.m_next = null;
        this.m_friction = 0;
        this.m_restitution = 0;
        this.m_proxies = [];
        this.m_proxyCount = 0;
        this.m_filter = new Filter();
        this.m_isSensor = false;
        this.m_userData = null;
        this.m_body = body;
        this.m_shape = def.shape.Clone();
    }
    /// Get the type of the child shape. You can use this to down cast to the concrete shape.
    /// @return the shape type.
    GetType() {
        return this.m_shape.GetType();
    }
    /// Get the child shape. You can modify the child shape, however you should not change the
    /// number of vertices because this will crash some collision caching mechanisms.
    /// Manipulating the shape may lead to non-physical behavior.
    GetShape() {
        return this.m_shape;
    }
    /// Set if this fixture is a sensor.
    SetSensor(sensor) {
        if (sensor !== this.m_isSensor) {
            this.m_body.SetAwake(true);
            this.m_isSensor = sensor;
        }
    }
    /// Is this fixture a sensor (non-solid)?
    /// @return the true if the shape is a sensor.
    IsSensor() {
        return this.m_isSensor;
    }
    /// Set the contact filtering data. This will not update contacts until the next time
    /// step when either parent body is active and awake.
    /// This automatically calls Refilter.
    SetFilterData(filter) {
        this.m_filter.Copy(filter);
        this.Refilter();
    }
    /// Get the contact filtering data.
    GetFilterData() {
        return this.m_filter;
    }
    /// Call this if you want to establish collision that was previously disabled by ContactFilter::ShouldCollide.
    Refilter() {
        // Flag associated contacts for filtering.
        let edge = this.m_body.GetContactList();
        while (edge) {
            const contact = edge.contact;
            const fixtureA = contact.GetFixtureA();
            const fixtureB = contact.GetFixtureB();
            if (fixtureA === this || fixtureB === this) {
                contact.FlagForFiltering();
            }
            edge = edge.next;
        }
        const world = this.m_body.GetWorld();
        if (world === null) {
            return;
        }
        // Touch each proxy so that new pairs may be created
        const broadPhase = world.m_contactManager.m_broadPhase;
        for (let i = 0; i < this.m_proxyCount; ++i) {
            broadPhase.TouchProxy(this.m_proxies[i].treeNode);
        }
    }
    /// Get the parent body of this fixture. This is NULL if the fixture is not attached.
    /// @return the parent body.
    GetBody() {
        return this.m_body;
    }
    /// Get the next fixture in the parent body's fixture list.
    /// @return the next shape.
    GetNext() {
        return this.m_next;
    }
    /// Get the user data that was assigned in the fixture definition. Use this to
    /// store your application specific data.
    GetUserData() {
        return this.m_userData;
    }
    /// Set the user data. Use this to store your application specific data.
    SetUserData(data) {
        this.m_userData = data;
    }
    /// Test a point for containment in this fixture.
    /// @param p a point in world coordinates.
    TestPoint(p) {
        return this.m_shape.TestPoint(this.m_body.GetTransform(), p);
    }
    // #if ENABLE_PARTICLE
    ComputeDistance(p, normal, childIndex) {
        return this.m_shape.ComputeDistance(this.m_body.GetTransform(), p, normal, childIndex);
    }
    // #endif
    /// Cast a ray against this shape.
    /// @param output the ray-cast results.
    /// @param input the ray-cast input parameters.
    RayCast(output, input, childIndex) {
        return this.m_shape.RayCast(output, input, this.m_body.GetTransform(), childIndex);
    }
    /// Get the mass data for this fixture. The mass data is based on the density and
    /// the shape. The rotational inertia is about the shape's origin. This operation
    /// may be expensive.
    GetMassData(massData = new MassData()) {
        this.m_shape.ComputeMass(massData, this.m_density);
        return massData;
    }
    /// Set the density of this fixture. This will _not_ automatically adjust the mass
    /// of the body. You must call Body::ResetMassData to update the body's mass.
    SetDensity(density) {
        this.m_density = density;
    }
    /// Get the density of this fixture.
    GetDensity() {
        return this.m_density;
    }
    /// Get the coefficient of friction.
    GetFriction() {
        return this.m_friction;
    }
    /// Set the coefficient of friction. This will _not_ change the friction of
    /// existing contacts.
    SetFriction(friction) {
        this.m_friction = friction;
    }
    /// Get the coefficient of restitution.
    GetRestitution() {
        return this.m_restitution;
    }
    /// Set the coefficient of restitution. This will _not_ change the restitution of
    /// existing contacts.
    SetRestitution(restitution) {
        this.m_restitution = restitution;
    }
    /// Get the fixture's AABB. This AABB may be enlarge and/or stale.
    /// If you need a more accurate AABB, compute it using the shape and
    /// the body transform.
    GetAABB(childIndex) {
        // DEBUG: Assert(0 <= childIndex && childIndex < this.m_proxyCount);
        return this.m_proxies[childIndex].aabb;
    }
    /// Dump this fixture to the log file.
    Dump(log, bodyIndex) {
        log('    const fd: FixtureDef = new FixtureDef();\n');
        log('    fd.friction = %.15f;\n', this.m_friction);
        log('    fd.restitution = %.15f;\n', this.m_restitution);
        log('    fd.density = %.15f;\n', this.m_density);
        log('    fd.isSensor = %s;\n', this.m_isSensor ? 'true' : 'false');
        log('    fd.filter.categoryBits = %d;\n', this.m_filter.categoryBits);
        log('    fd.filter.maskBits = %d;\n', this.m_filter.maskBits);
        log('    fd.filter.groupIndex = %d;\n', this.m_filter.groupIndex);
        this.m_shape.Dump(log);
        log('\n');
        log('    fd.shape = shape;\n');
        log('\n');
        log('    bodies[%d].CreateFixture(fd);\n', bodyIndex);
    }
    // We need separation create/destroy functions from the constructor/destructor because
    // the destructor cannot access the allocator (no destructor arguments allowed by C++).
    Create(def) {
        this.m_userData = def.userData;
        this.m_friction = Maybe(def.friction, 0.2);
        this.m_restitution = Maybe(def.restitution, 0);
        // this.m_body = body;
        this.m_next = null;
        this.m_filter.Copy(Maybe(def.filter, Filter.DEFAULT));
        this.m_isSensor = Maybe(def.isSensor, false);
        // Reserve proxy space
        // const childCount = m_shape->GetChildCount();
        // m_proxies = (FixtureProxy*)allocator->Allocate(childCount * sizeof(FixtureProxy));
        // for (int32 i = 0; i < childCount; ++i)
        // {
        //   m_proxies[i].fixture = NULL;
        //   m_proxies[i].proxyId = BroadPhase::e_nullProxy;
        // }
        // this.m_proxies = FixtureProxy.MakeArray(this.m_shape.GetChildCount());
        this.m_proxies = MakeArray(this.m_shape.GetChildCount(), i => new FixtureProxy(this));
        this.m_proxyCount = 0;
        this.m_density = Maybe(def.density, 0);
    }
    Destroy() {
        // The proxies must be destroyed before calling this.
        // DEBUG: Assert(this.m_proxyCount === 0);
        // Free the proxy array.
        // int32 childCount = m_shape->GetChildCount();
        // allocator->Free(m_proxies, childCount * sizeof(FixtureProxy));
        // m_proxies = NULL;
        // this.m_shape = null;
    }
    // These support body activation/deactivation.
    CreateProxies(xf) {
        const broadPhase = this.m_body.m_world
            .m_contactManager.m_broadPhase;
        // DEBUG: Assert(this.m_proxyCount === 0);
        // Create proxies in the broad-phase.
        this.m_proxyCount = this.m_shape.GetChildCount();
        for (let i = 0; i < this.m_proxyCount; ++i) {
            const proxy = (this.m_proxies[i] = new FixtureProxy(this));
            this.m_shape.ComputeAABB(proxy.aabb, xf, i);
            proxy.treeNode = broadPhase.CreateProxy(proxy.aabb, proxy);
            proxy.childIndex = i;
        }
    }
    DestroyProxies() {
        const broadPhase = this.m_body.m_world
            .m_contactManager.m_broadPhase;
        // Destroy proxies in the broad-phase.
        for (let i = 0; i < this.m_proxyCount; ++i) {
            const proxy = this.m_proxies[i];
            delete proxy.treeNode.userData;
            broadPhase.DestroyProxy(proxy.treeNode);
            delete proxy.treeNode;
        }
        this.m_proxyCount = 0;
    }
    TouchProxies() {
        const broadPhase = this.m_body.m_world
            .m_contactManager.m_broadPhase;
        const proxyCount = this.m_proxyCount;
        for (let i = 0; i < proxyCount; ++i) {
            broadPhase.TouchProxy(this.m_proxies[i].treeNode);
        }
    }
    Synchronize(transform1, transform2) {
        if (this.m_proxyCount === 0) {
            return;
        }
        const broadPhase = this.m_body.m_world
            .m_contactManager.m_broadPhase;
        for (let i = 0; i < this.m_proxyCount; ++i) {
            const proxy = this.m_proxies[i];
            // Compute an AABB that covers the swept shape (may miss some rotation effect).
            const aabb1 = Fixture.Synchronize_s_aabb1;
            const aabb2 = Fixture.Synchronize_s_aabb2;
            this.m_shape.ComputeAABB(aabb1, transform1, i);
            this.m_shape.ComputeAABB(aabb2, transform2, i);
            proxy.aabb.Combine2(aabb1, aabb2);
            const displacement = Vec2.SubVV(transform2.p, transform1.p, Fixture.Synchronize_s_displacement);
            broadPhase.MoveProxy(proxy.treeNode, proxy.aabb, displacement);
        }
    }
}
Fixture.Synchronize_s_aabb1 = new AABB();
Fixture.Synchronize_s_aabb2 = new AABB();
Fixture.Synchronize_s_displacement = new Vec2();
//# sourceMappingURL=Fixture.js.map