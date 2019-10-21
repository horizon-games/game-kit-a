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
// DEBUG: import { Assert } from "../Common/Settings";
import { Abs, Max, Min, Rot, Transform, Vec2 } from '../Common/Math';
import { epsilon, epsilon_sq, MakeArray, MakeNumberArray, maxFloat, maxManifoldPoints } from '../Common/Settings';
import { Distance, DistanceInput, DistanceOutput, SimplexCache } from './Distance';
/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.
export var ContactFeatureType;
(function (ContactFeatureType) {
    ContactFeatureType[ContactFeatureType["e_vertex"] = 0] = "e_vertex";
    ContactFeatureType[ContactFeatureType["e_face"] = 1] = "e_face";
})(ContactFeatureType || (ContactFeatureType = {}));
/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
export class ContactFeature {
    constructor() {
        this._key = 0;
        this._key_invalid = false;
        this._indexA = 0;
        this._indexB = 0;
        this._typeA = 0;
        this._typeB = 0;
    }
    get key() {
        if (this._key_invalid) {
            this._key_invalid = false;
            this._key =
                this._indexA |
                    (this._indexB << 8) |
                    (this._typeA << 16) |
                    (this._typeB << 24);
        }
        return this._key;
    }
    set key(value) {
        this._key = value;
        this._key_invalid = false;
        this._indexA = this._key & 0xff;
        this._indexB = (this._key >> 8) & 0xff;
        this._typeA = (this._key >> 16) & 0xff;
        this._typeB = (this._key >> 24) & 0xff;
    }
    get indexA() {
        return this._indexA;
    }
    set indexA(value) {
        this._indexA = value;
        this._key_invalid = true;
    }
    get indexB() {
        return this._indexB;
    }
    set indexB(value) {
        this._indexB = value;
        this._key_invalid = true;
    }
    get typeA() {
        return this._typeA;
    }
    set typeA(value) {
        this._typeA = value;
        this._key_invalid = true;
    }
    get typeB() {
        return this._typeB;
    }
    set typeB(value) {
        this._typeB = value;
        this._key_invalid = true;
    }
}
/// Contact ids to facilitate warm starting.
export class ContactID {
    constructor() {
        this.cf = new ContactFeature();
    }
    Copy(o) {
        this.key = o.key;
        return this;
    }
    Clone() {
        return new ContactID().Copy(this);
    }
    get key() {
        return this.cf.key;
    }
    set key(value) {
        this.cf.key = value;
    }
}
/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
export class ManifoldPoint {
    constructor() {
        this.localPoint = new Vec2(); ///< usage depends on manifold type
        this.normalImpulse = 0; ///< the non-penetration impulse
        this.tangentImpulse = 0; ///< the friction impulse
        this.id = new ContactID(); // TODO: readonly  ///< uniquely identifies a contact point between two shapes
    }
    static MakeArray(length) {
        return MakeArray(length, (i) => new ManifoldPoint());
    }
    Reset() {
        this.localPoint.SetZero();
        this.normalImpulse = 0;
        this.tangentImpulse = 0;
        this.id.key = 0;
    }
    Copy(o) {
        this.localPoint.Copy(o.localPoint);
        this.normalImpulse = o.normalImpulse;
        this.tangentImpulse = o.tangentImpulse;
        this.id.Copy(o.id);
        return this;
    }
}
export var ManifoldType;
(function (ManifoldType) {
    ManifoldType[ManifoldType["e_unknown"] = -1] = "e_unknown";
    ManifoldType[ManifoldType["e_circles"] = 0] = "e_circles";
    ManifoldType[ManifoldType["e_faceA"] = 1] = "e_faceA";
    ManifoldType[ManifoldType["e_faceB"] = 2] = "e_faceB";
})(ManifoldType || (ManifoldType = {}));
/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
export class Manifold {
    constructor() {
        this.points = ManifoldPoint.MakeArray(maxManifoldPoints);
        this.localNormal = new Vec2();
        this.localPoint = new Vec2();
        this.type = ManifoldType.e_unknown;
        this.pointCount = 0;
    }
    Reset() {
        for (let i = 0; i < maxManifoldPoints; ++i) {
            // DEBUG: Assert(this.points[i] instanceof ManifoldPoint);
            this.points[i].Reset();
        }
        this.localNormal.SetZero();
        this.localPoint.SetZero();
        this.type = ManifoldType.e_unknown;
        this.pointCount = 0;
    }
    Copy(o) {
        this.pointCount = o.pointCount;
        for (let i = 0; i < maxManifoldPoints; ++i) {
            // DEBUG: Assert(this.points[i] instanceof ManifoldPoint);
            this.points[i].Copy(o.points[i]);
        }
        this.localNormal.Copy(o.localNormal);
        this.localPoint.Copy(o.localPoint);
        this.type = o.type;
        return this;
    }
    Clone() {
        return new Manifold().Copy(this);
    }
}
export class WorldManifold {
    constructor() {
        this.normal = new Vec2();
        this.points = Vec2.MakeArray(maxManifoldPoints);
        this.separations = MakeNumberArray(maxManifoldPoints);
    }
    Initialize(manifold, xfA, radiusA, xfB, radiusB) {
        if (manifold.pointCount === 0) {
            return;
        }
        switch (manifold.type) {
            case ManifoldType.e_circles: {
                this.normal.Set(1, 0);
                const pointA = Transform.MulXV(xfA, manifold.localPoint, WorldManifold.Initialize_s_pointA);
                const pointB = Transform.MulXV(xfB, manifold.points[0].localPoint, WorldManifold.Initialize_s_pointB);
                if (Vec2.DistanceSquaredVV(pointA, pointB) > epsilon_sq) {
                    Vec2.SubVV(pointB, pointA, this.normal).SelfNormalize();
                }
                const cA = Vec2.AddVMulSV(pointA, radiusA, this.normal, WorldManifold.Initialize_s_cA);
                const cB = Vec2.SubVMulSV(pointB, radiusB, this.normal, WorldManifold.Initialize_s_cB);
                Vec2.MidVV(cA, cB, this.points[0]);
                this.separations[0] = Vec2.DotVV(Vec2.SubVV(cB, cA, Vec2.s_t0), this.normal); // Dot(cB - cA, normal);
                break;
            }
            case ManifoldType.e_faceA: {
                Rot.MulRV(xfA.q, manifold.localNormal, this.normal);
                const planePoint = Transform.MulXV(xfA, manifold.localPoint, WorldManifold.Initialize_s_planePoint);
                for (let i = 0; i < manifold.pointCount; ++i) {
                    const clipPoint = Transform.MulXV(xfB, manifold.points[i].localPoint, WorldManifold.Initialize_s_clipPoint);
                    const s = radiusA -
                        Vec2.DotVV(Vec2.SubVV(clipPoint, planePoint, Vec2.s_t0), this.normal);
                    const cA = Vec2.AddVMulSV(clipPoint, s, this.normal, WorldManifold.Initialize_s_cA);
                    const cB = Vec2.SubVMulSV(clipPoint, radiusB, this.normal, WorldManifold.Initialize_s_cB);
                    Vec2.MidVV(cA, cB, this.points[i]);
                    this.separations[i] = Vec2.DotVV(Vec2.SubVV(cB, cA, Vec2.s_t0), this.normal); // Dot(cB - cA, normal);
                }
                break;
            }
            case ManifoldType.e_faceB: {
                Rot.MulRV(xfB.q, manifold.localNormal, this.normal);
                const planePoint = Transform.MulXV(xfB, manifold.localPoint, WorldManifold.Initialize_s_planePoint);
                for (let i = 0; i < manifold.pointCount; ++i) {
                    const clipPoint = Transform.MulXV(xfA, manifold.points[i].localPoint, WorldManifold.Initialize_s_clipPoint);
                    const s = radiusB -
                        Vec2.DotVV(Vec2.SubVV(clipPoint, planePoint, Vec2.s_t0), this.normal);
                    const cB = Vec2.AddVMulSV(clipPoint, s, this.normal, WorldManifold.Initialize_s_cB);
                    const cA = Vec2.SubVMulSV(clipPoint, radiusA, this.normal, WorldManifold.Initialize_s_cA);
                    Vec2.MidVV(cA, cB, this.points[i]);
                    this.separations[i] = Vec2.DotVV(Vec2.SubVV(cA, cB, Vec2.s_t0), this.normal); // Dot(cA - cB, normal);
                }
                // Ensure normal points from A to B.
                this.normal.SelfNeg();
                break;
            }
        }
    }
}
WorldManifold.Initialize_s_pointA = new Vec2();
WorldManifold.Initialize_s_pointB = new Vec2();
WorldManifold.Initialize_s_cA = new Vec2();
WorldManifold.Initialize_s_cB = new Vec2();
WorldManifold.Initialize_s_planePoint = new Vec2();
WorldManifold.Initialize_s_clipPoint = new Vec2();
/// This is used for determining the state of contact points.
export var PointState;
(function (PointState) {
    PointState[PointState["nullState"] = 0] = "nullState";
    PointState[PointState["addState"] = 1] = "addState";
    PointState[PointState["persistState"] = 2] = "persistState";
    PointState[PointState["removeState"] = 3] = "removeState"; ///< point was removed in the update
})(PointState || (PointState = {}));
/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
export function GetPointStates(state1, state2, manifold1, manifold2) {
    // Detect persists and removes.
    let i;
    for (i = 0; i < manifold1.pointCount; ++i) {
        const id = manifold1.points[i].id;
        const key = id.key;
        state1[i] = PointState.removeState;
        for (let j = 0, jct = manifold2.pointCount; j < jct; ++j) {
            if (manifold2.points[j].id.key === key) {
                state1[i] = PointState.persistState;
                break;
            }
        }
    }
    for (; i < maxManifoldPoints; ++i) {
        state1[i] = PointState.nullState;
    }
    // Detect persists and adds.
    for (i = 0; i < manifold2.pointCount; ++i) {
        const id = manifold2.points[i].id;
        const key = id.key;
        state2[i] = PointState.addState;
        for (let j = 0, jct = manifold1.pointCount; j < jct; ++j) {
            if (manifold1.points[j].id.key === key) {
                state2[i] = PointState.persistState;
                break;
            }
        }
    }
    for (; i < maxManifoldPoints; ++i) {
        state2[i] = PointState.nullState;
    }
}
/// Used for computing contact manifolds.
export class ClipVertex {
    constructor() {
        this.v = new Vec2();
        this.id = new ContactID();
    }
    static MakeArray(length) {
        return MakeArray(length, (i) => new ClipVertex());
    }
    Copy(other) {
        this.v.Copy(other.v);
        this.id.Copy(other.id);
        return this;
    }
}
/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
export class RayCastInput {
    constructor() {
        this.p1 = new Vec2();
        this.p2 = new Vec2();
        this.maxFraction = 1;
    }
    Copy(o) {
        this.p1.Copy(o.p1);
        this.p2.Copy(o.p2);
        this.maxFraction = o.maxFraction;
        return this;
    }
}
/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from RayCastInput.
export class RayCastOutput {
    constructor() {
        this.normal = new Vec2();
        this.fraction = 0;
    }
    Copy(o) {
        this.normal.Copy(o.normal);
        this.fraction = o.fraction;
        return this;
    }
}
/// An axis aligned bounding box.
export class AABB {
    constructor() {
        this.lowerBound = new Vec2(); ///< the lower vertex
        this.upperBound = new Vec2(); ///< the upper vertex
        this.m_cache_center = new Vec2(); // access using GetCenter()
        this.m_cache_extent = new Vec2(); // access using GetExtents()
    }
    static Combine(aabb1, aabb2, out) {
        out.Combine2(aabb1, aabb2);
        return out;
    }
    Copy(o) {
        this.lowerBound.Copy(o.lowerBound);
        this.upperBound.Copy(o.upperBound);
        return this;
    }
    /// Verify that the bounds are sorted.
    IsValid() {
        const d_x = this.upperBound.x - this.lowerBound.x;
        const d_y = this.upperBound.y - this.lowerBound.y;
        let valid = d_x >= 0 && d_y >= 0;
        valid = valid && this.lowerBound.IsValid() && this.upperBound.IsValid();
        return valid;
    }
    /// Get the center of the AABB.
    GetCenter() {
        return Vec2.MidVV(this.lowerBound, this.upperBound, this.m_cache_center);
    }
    /// Get the extents of the AABB (half-widths).
    GetExtents() {
        return Vec2.ExtVV(this.lowerBound, this.upperBound, this.m_cache_extent);
    }
    /// Get the perimeter length
    GetPerimeter() {
        const wx = this.upperBound.x - this.lowerBound.x;
        const wy = this.upperBound.y - this.lowerBound.y;
        return 2 * (wx + wy);
    }
    /// Combine an AABB into this one.
    Combine1(aabb) {
        this.lowerBound.x = Min(this.lowerBound.x, aabb.lowerBound.x);
        this.lowerBound.y = Min(this.lowerBound.y, aabb.lowerBound.y);
        this.upperBound.x = Max(this.upperBound.x, aabb.upperBound.x);
        this.upperBound.y = Max(this.upperBound.y, aabb.upperBound.y);
        return this;
    }
    /// Combine two AABBs into this one.
    Combine2(aabb1, aabb2) {
        this.lowerBound.x = Min(aabb1.lowerBound.x, aabb2.lowerBound.x);
        this.lowerBound.y = Min(aabb1.lowerBound.y, aabb2.lowerBound.y);
        this.upperBound.x = Max(aabb1.upperBound.x, aabb2.upperBound.x);
        this.upperBound.y = Max(aabb1.upperBound.y, aabb2.upperBound.y);
        return this;
    }
    /// Does this aabb contain the provided AABB.
    Contains(aabb) {
        let result = true;
        result = result && this.lowerBound.x <= aabb.lowerBound.x;
        result = result && this.lowerBound.y <= aabb.lowerBound.y;
        result = result && aabb.upperBound.x <= this.upperBound.x;
        result = result && aabb.upperBound.y <= this.upperBound.y;
        return result;
    }
    // From Real-time Collision Detection, p179.
    RayCast(output, input) {
        let tmin = -maxFloat;
        let tmax = maxFloat;
        const p_x = input.p1.x;
        const p_y = input.p1.y;
        const d_x = input.p2.x - input.p1.x;
        const d_y = input.p2.y - input.p1.y;
        const absD_x = Abs(d_x);
        const absD_y = Abs(d_y);
        const normal = output.normal;
        if (absD_x < epsilon) {
            // Parallel.
            if (p_x < this.lowerBound.x || this.upperBound.x < p_x) {
                return false;
            }
        }
        else {
            const inv_d = 1 / d_x;
            let t1 = (this.lowerBound.x - p_x) * inv_d;
            let t2 = (this.upperBound.x - p_x) * inv_d;
            // Sign of the normal vector.
            let s = -1;
            if (t1 > t2) {
                const t3 = t1;
                t1 = t2;
                t2 = t3;
                s = 1;
            }
            // Push the min up
            if (t1 > tmin) {
                normal.x = s;
                normal.y = 0;
                tmin = t1;
            }
            // Pull the max down
            tmax = Min(tmax, t2);
            if (tmin > tmax) {
                return false;
            }
        }
        if (absD_y < epsilon) {
            // Parallel.
            if (p_y < this.lowerBound.y || this.upperBound.y < p_y) {
                return false;
            }
        }
        else {
            const inv_d = 1 / d_y;
            let t1 = (this.lowerBound.y - p_y) * inv_d;
            let t2 = (this.upperBound.y - p_y) * inv_d;
            // Sign of the normal vector.
            let s = -1;
            if (t1 > t2) {
                const t3 = t1;
                t1 = t2;
                t2 = t3;
                s = 1;
            }
            // Push the min up
            if (t1 > tmin) {
                normal.x = 0;
                normal.y = s;
                tmin = t1;
            }
            // Pull the max down
            tmax = Min(tmax, t2);
            if (tmin > tmax) {
                return false;
            }
        }
        // Does the ray start inside the box?
        // Does the ray intersect beyond the max fraction?
        if (tmin < 0 || input.maxFraction < tmin) {
            return false;
        }
        // Intersection.
        output.fraction = tmin;
        return true;
    }
    TestContain(point) {
        if (point.x < this.lowerBound.x || this.upperBound.x < point.x) {
            return false;
        }
        if (point.y < this.lowerBound.y || this.upperBound.y < point.y) {
            return false;
        }
        return true;
    }
    TestOverlap(other) {
        const d1_x = other.lowerBound.x - this.upperBound.x;
        const d1_y = other.lowerBound.y - this.upperBound.y;
        const d2_x = this.lowerBound.x - other.upperBound.x;
        const d2_y = this.lowerBound.y - other.upperBound.y;
        if (d1_x > 0 || d1_y > 0) {
            return false;
        }
        if (d2_x > 0 || d2_y > 0) {
            return false;
        }
        return true;
    }
}
export function TestOverlapAABB(a, b) {
    const d1_x = b.lowerBound.x - a.upperBound.x;
    const d1_y = b.lowerBound.y - a.upperBound.y;
    const d2_x = a.lowerBound.x - b.upperBound.x;
    const d2_y = a.lowerBound.y - b.upperBound.y;
    if (d1_x > 0 || d1_y > 0) {
        return false;
    }
    if (d2_x > 0 || d2_y > 0) {
        return false;
    }
    return true;
}
/// Clipping for contact manifolds.
export function ClipSegmentToLine(vOut, vIn, normal, offset, vertexIndexA) {
    // Start with no output points
    let numOut = 0;
    const vIn0 = vIn[0];
    const vIn1 = vIn[1];
    // Calculate the distance of end points to the line
    const distance0 = Vec2.DotVV(normal, vIn0.v) - offset;
    const distance1 = Vec2.DotVV(normal, vIn1.v) - offset;
    // If the points are behind the plane
    if (distance0 <= 0) {
        vOut[numOut++].Copy(vIn0);
    }
    if (distance1 <= 0) {
        vOut[numOut++].Copy(vIn1);
    }
    // If the points are on different sides of the plane
    if (distance0 * distance1 < 0) {
        // Find intersection point of edge and plane
        const interp = distance0 / (distance0 - distance1);
        const v = vOut[numOut].v;
        v.x = vIn0.v.x + interp * (vIn1.v.x - vIn0.v.x);
        v.y = vIn0.v.y + interp * (vIn1.v.y - vIn0.v.y);
        // VertexA is hitting edgeB.
        const id = vOut[numOut].id;
        id.cf.indexA = vertexIndexA;
        id.cf.indexB = vIn0.id.cf.indexB;
        id.cf.typeA = ContactFeatureType.e_vertex;
        id.cf.typeB = ContactFeatureType.e_face;
        ++numOut;
    }
    return numOut;
}
/// Determine if two generic shapes overlap.
const TestOverlapShape_s_input = new DistanceInput();
const TestOverlapShape_s_simplexCache = new SimplexCache();
const TestOverlapShape_s_output = new DistanceOutput();
export function TestOverlapShape(shapeA, indexA, shapeB, indexB, xfA, xfB) {
    const input = TestOverlapShape_s_input.Reset();
    input.proxyA.SetShape(shapeA, indexA);
    input.proxyB.SetShape(shapeB, indexB);
    input.transformA.Copy(xfA);
    input.transformB.Copy(xfB);
    input.useRadii = true;
    const simplexCache = TestOverlapShape_s_simplexCache.Reset();
    simplexCache.count = 0;
    const output = TestOverlapShape_s_output.Reset();
    Distance(output, simplexCache, input);
    return output.distance < 10 * epsilon;
}
//# sourceMappingURL=Collision.js.map