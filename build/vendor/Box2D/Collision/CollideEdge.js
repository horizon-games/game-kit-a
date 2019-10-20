// DEBUG: import { Assert } from "../Common/Settings";
import { Min, Rot, Transform, Vec2 } from '../Common/Math';
import { angularSlop, maxFloat, maxManifoldPoints } from '../Common/Settings';
import { ClipSegmentToLine, ClipVertex, ContactFeatureType, ContactID, ManifoldType } from './Collision';
const CollideEdgeAndCircle_s_Q = new Vec2();
const CollideEdgeAndCircle_s_e = new Vec2();
const CollideEdgeAndCircle_s_d = new Vec2();
const CollideEdgeAndCircle_s_e1 = new Vec2();
const collideEdgeAndCircle_s_e2 = new Vec2();
const CollideEdgeAndCircle_s_P = new Vec2();
const CollideEdgeAndCircle_s_n = new Vec2();
const CollideEdgeAndCircle_s_id = new ContactID();
export function CollideEdgeAndCircle(manifold, edgeA, xfA, circleB, xfB) {
    manifold.pointCount = 0;
    // Compute circle in frame of edge
    const Q = Transform.MulTXV(xfA, Transform.MulXV(xfB, circleB.m_p, Vec2.s_t0), CollideEdgeAndCircle_s_Q);
    const A = edgeA.m_vertex1;
    const B = edgeA.m_vertex2;
    const e = Vec2.SubVV(B, A, CollideEdgeAndCircle_s_e);
    // Barycentric coordinates
    const u = Vec2.DotVV(e, Vec2.SubVV(B, Q, Vec2.s_t0));
    const v = Vec2.DotVV(e, Vec2.SubVV(Q, A, Vec2.s_t0));
    const radius = edgeA.m_radius + circleB.m_radius;
    // const cf: ContactFeature = new ContactFeature();
    const id = CollideEdgeAndCircle_s_id;
    id.cf.indexB = 0;
    id.cf.typeB = ContactFeatureType.e_vertex;
    // Region A
    if (v <= 0) {
        const P = A;
        const d = Vec2.SubVV(Q, P, CollideEdgeAndCircle_s_d);
        const dd = Vec2.DotVV(d, d);
        if (dd > radius * radius) {
            return;
        }
        // Is there an edge connected to A?
        if (edgeA.m_hasVertex0) {
            const A1 = edgeA.m_vertex0;
            const B1 = A;
            const e1 = Vec2.SubVV(B1, A1, CollideEdgeAndCircle_s_e1);
            const u1 = Vec2.DotVV(e1, Vec2.SubVV(B1, Q, Vec2.s_t0));
            // Is the circle in Region AB of the previous edge?
            if (u1 > 0) {
                return;
            }
        }
        id.cf.indexA = 0;
        id.cf.typeA = ContactFeatureType.e_vertex;
        manifold.pointCount = 1;
        manifold.type = ManifoldType.e_circles;
        manifold.localNormal.SetZero();
        manifold.localPoint.Copy(P);
        manifold.points[0].id.Copy(id);
        // manifold.points[0].id.key = 0;
        // manifold.points[0].id.cf = cf;
        manifold.points[0].localPoint.Copy(circleB.m_p);
        return;
    }
    // Region B
    if (u <= 0) {
        const P = B;
        const d = Vec2.SubVV(Q, P, CollideEdgeAndCircle_s_d);
        const dd = Vec2.DotVV(d, d);
        if (dd > radius * radius) {
            return;
        }
        // Is there an edge connected to B?
        if (edgeA.m_hasVertex3) {
            const B2 = edgeA.m_vertex3;
            const A2 = B;
            const e2 = Vec2.SubVV(B2, A2, collideEdgeAndCircle_s_e2);
            const v2 = Vec2.DotVV(e2, Vec2.SubVV(Q, A2, Vec2.s_t0));
            // Is the circle in Region AB of the next edge?
            if (v2 > 0) {
                return;
            }
        }
        id.cf.indexA = 1;
        id.cf.typeA = ContactFeatureType.e_vertex;
        manifold.pointCount = 1;
        manifold.type = ManifoldType.e_circles;
        manifold.localNormal.SetZero();
        manifold.localPoint.Copy(P);
        manifold.points[0].id.Copy(id);
        // manifold.points[0].id.key = 0;
        // manifold.points[0].id.cf = cf;
        manifold.points[0].localPoint.Copy(circleB.m_p);
        return;
    }
    // Region AB
    const den = Vec2.DotVV(e, e);
    // DEBUG: Assert(den > 0);
    const P = CollideEdgeAndCircle_s_P;
    P.x = (1 / den) * (u * A.x + v * B.x);
    P.y = (1 / den) * (u * A.y + v * B.y);
    const d = Vec2.SubVV(Q, P, CollideEdgeAndCircle_s_d);
    const dd = Vec2.DotVV(d, d);
    if (dd > radius * radius) {
        return;
    }
    const n = CollideEdgeAndCircle_s_n.Set(-e.y, e.x);
    if (Vec2.DotVV(n, Vec2.SubVV(Q, A, Vec2.s_t0)) < 0) {
        n.Set(-n.x, -n.y);
    }
    n.Normalize();
    id.cf.indexA = 0;
    id.cf.typeA = ContactFeatureType.e_face;
    manifold.pointCount = 1;
    manifold.type = ManifoldType.e_faceA;
    manifold.localNormal.Copy(n);
    manifold.localPoint.Copy(A);
    manifold.points[0].id.Copy(id);
    // manifold.points[0].id.key = 0;
    // manifold.points[0].id.cf = cf;
    manifold.points[0].localPoint.Copy(circleB.m_p);
}
var EPAxisType;
(function (EPAxisType) {
    EPAxisType[EPAxisType["e_unknown"] = 0] = "e_unknown";
    EPAxisType[EPAxisType["e_edgeA"] = 1] = "e_edgeA";
    EPAxisType[EPAxisType["e_edgeB"] = 2] = "e_edgeB";
})(EPAxisType || (EPAxisType = {}));
class EPAxis {
    constructor() {
        this.type = EPAxisType.e_unknown;
        this.index = 0;
        this.separation = 0;
    }
}
class TempPolygon {
    constructor() {
        this.vertices = [];
        this.normals = [];
        this.count = 0;
    }
}
class ReferenceFace {
    constructor() {
        this.i1 = 0;
        this.i2 = 0;
        this.v1 = new Vec2();
        this.v2 = new Vec2();
        this.normal = new Vec2();
        this.sideNormal1 = new Vec2();
        this.sideOffset1 = 0;
        this.sideNormal2 = new Vec2();
        this.sideOffset2 = 0;
    }
}
var EPColliderVertexType;
(function (EPColliderVertexType) {
    EPColliderVertexType[EPColliderVertexType["e_isolated"] = 0] = "e_isolated";
    EPColliderVertexType[EPColliderVertexType["e_concave"] = 1] = "e_concave";
    EPColliderVertexType[EPColliderVertexType["e_convex"] = 2] = "e_convex";
})(EPColliderVertexType || (EPColliderVertexType = {}));
class EPCollider {
    constructor() {
        this.m_polygonB = new TempPolygon();
        this.m_xf = new Transform();
        this.m_centroidB = new Vec2();
        this.m_v0 = new Vec2();
        this.m_v1 = new Vec2();
        this.m_v2 = new Vec2();
        this.m_v3 = new Vec2();
        this.m_normal0 = new Vec2();
        this.m_normal1 = new Vec2();
        this.m_normal2 = new Vec2();
        this.m_normal = new Vec2();
        this.m_type1 = EPColliderVertexType.e_isolated;
        this.m_type2 = EPColliderVertexType.e_isolated;
        this.m_lowerLimit = new Vec2();
        this.m_upperLimit = new Vec2();
        this.m_radius = 0;
        this.m_front = false;
    }
    Collide(manifold, edgeA, xfA, polygonB, xfB) {
        Transform.MulTXX(xfA, xfB, this.m_xf);
        Transform.MulXV(this.m_xf, polygonB.m_centroid, this.m_centroidB);
        this.m_v0.Copy(edgeA.m_vertex0);
        this.m_v1.Copy(edgeA.m_vertex1);
        this.m_v2.Copy(edgeA.m_vertex2);
        this.m_v3.Copy(edgeA.m_vertex3);
        const hasVertex0 = edgeA.m_hasVertex0;
        const hasVertex3 = edgeA.m_hasVertex3;
        const edge1 = Vec2.SubVV(this.m_v2, this.m_v1, EPCollider.s_edge1);
        edge1.Normalize();
        this.m_normal1.Set(edge1.y, -edge1.x);
        const offset1 = Vec2.DotVV(this.m_normal1, Vec2.SubVV(this.m_centroidB, this.m_v1, Vec2.s_t0));
        let offset0 = 0;
        let offset2 = 0;
        let convex1 = false;
        let convex2 = false;
        // Is there a preceding edge?
        if (hasVertex0) {
            const edge0 = Vec2.SubVV(this.m_v1, this.m_v0, EPCollider.s_edge0);
            edge0.Normalize();
            this.m_normal0.Set(edge0.y, -edge0.x);
            convex1 = Vec2.CrossVV(edge0, edge1) >= 0;
            offset0 = Vec2.DotVV(this.m_normal0, Vec2.SubVV(this.m_centroidB, this.m_v0, Vec2.s_t0));
        }
        // Is there a following edge?
        if (hasVertex3) {
            const edge2 = Vec2.SubVV(this.m_v3, this.m_v2, EPCollider.s_edge2);
            edge2.Normalize();
            this.m_normal2.Set(edge2.y, -edge2.x);
            convex2 = Vec2.CrossVV(edge1, edge2) > 0;
            offset2 = Vec2.DotVV(this.m_normal2, Vec2.SubVV(this.m_centroidB, this.m_v2, Vec2.s_t0));
        }
        // Determine front or back collision. Determine collision normal limits.
        if (hasVertex0 && hasVertex3) {
            if (convex1 && convex2) {
                this.m_front = offset0 >= 0 || offset1 >= 0 || offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal0);
                    this.m_upperLimit.Copy(this.m_normal2);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
            }
            else if (convex1) {
                this.m_front = offset0 >= 0 || (offset1 >= 0 && offset2 >= 0);
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal0);
                    this.m_upperLimit.Copy(this.m_normal1);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal2).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
            }
            else if (convex2) {
                this.m_front = offset2 >= 0 || (offset0 >= 0 && offset1 >= 0);
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal2);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal0).SelfNeg();
                }
            }
            else {
                this.m_front = offset0 >= 0 && offset1 >= 0 && offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal1);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal2).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal0).SelfNeg();
                }
            }
        }
        else if (hasVertex0) {
            if (convex1) {
                this.m_front = offset0 >= 0 || offset1 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal0);
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
            }
            else {
                this.m_front = offset0 >= 0 && offset1 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1);
                    this.m_upperLimit.Copy(this.m_normal0).SelfNeg();
                }
            }
        }
        else if (hasVertex3) {
            if (convex2) {
                this.m_front = offset1 >= 0 || offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal2);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1);
                }
            }
            else {
                this.m_front = offset1 >= 0 && offset2 >= 0;
                if (this.m_front) {
                    this.m_normal.Copy(this.m_normal1);
                    this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1);
                }
                else {
                    this.m_normal.Copy(this.m_normal1).SelfNeg();
                    this.m_lowerLimit.Copy(this.m_normal2).SelfNeg();
                    this.m_upperLimit.Copy(this.m_normal1);
                }
            }
        }
        else {
            this.m_front = offset1 >= 0;
            if (this.m_front) {
                this.m_normal.Copy(this.m_normal1);
                this.m_lowerLimit.Copy(this.m_normal1).SelfNeg();
                this.m_upperLimit.Copy(this.m_normal1).SelfNeg();
            }
            else {
                this.m_normal.Copy(this.m_normal1).SelfNeg();
                this.m_lowerLimit.Copy(this.m_normal1);
                this.m_upperLimit.Copy(this.m_normal1);
            }
        }
        // Get polygonB in frameA
        this.m_polygonB.count = polygonB.m_count;
        for (let i = 0; i < polygonB.m_count; ++i) {
            if (this.m_polygonB.vertices.length <= i) {
                this.m_polygonB.vertices.push(new Vec2());
            }
            if (this.m_polygonB.normals.length <= i) {
                this.m_polygonB.normals.push(new Vec2());
            }
            Transform.MulXV(this.m_xf, polygonB.m_vertices[i], this.m_polygonB.vertices[i]);
            Rot.MulRV(this.m_xf.q, polygonB.m_normals[i], this.m_polygonB.normals[i]);
        }
        this.m_radius = polygonB.m_radius + edgeA.m_radius;
        manifold.pointCount = 0;
        const edgeAxis = this.ComputeEdgeSeparation(EPCollider.s_edgeAxis);
        // If no valid normal can be found than this edge should not collide.
        if (edgeAxis.type === EPAxisType.e_unknown) {
            return;
        }
        if (edgeAxis.separation > this.m_radius) {
            return;
        }
        const polygonAxis = this.ComputePolygonSeparation(EPCollider.s_polygonAxis);
        if (polygonAxis.type !== EPAxisType.e_unknown &&
            polygonAxis.separation > this.m_radius) {
            return;
        }
        // Use hysteresis for jitter reduction.
        const k_relativeTol = 0.98;
        const k_absoluteTol = 0.001;
        let primaryAxis;
        if (polygonAxis.type === EPAxisType.e_unknown) {
            primaryAxis = edgeAxis;
        }
        else if (polygonAxis.separation >
            k_relativeTol * edgeAxis.separation + k_absoluteTol) {
            primaryAxis = polygonAxis;
        }
        else {
            primaryAxis = edgeAxis;
        }
        const ie = EPCollider.s_ie;
        const rf = EPCollider.s_rf;
        if (primaryAxis.type === EPAxisType.e_edgeA) {
            manifold.type = ManifoldType.e_faceA;
            // Search for the polygon normal that is most anti-parallel to the edge normal.
            let bestIndex = 0;
            let bestValue = Vec2.DotVV(this.m_normal, this.m_polygonB.normals[0]);
            for (let i = 1; i < this.m_polygonB.count; ++i) {
                const value = Vec2.DotVV(this.m_normal, this.m_polygonB.normals[i]);
                if (value < bestValue) {
                    bestValue = value;
                    bestIndex = i;
                }
            }
            const i1 = bestIndex;
            const i2 = (i1 + 1) % this.m_polygonB.count;
            const ie0 = ie[0];
            ie0.v.Copy(this.m_polygonB.vertices[i1]);
            ie0.id.cf.indexA = 0;
            ie0.id.cf.indexB = i1;
            ie0.id.cf.typeA = ContactFeatureType.e_face;
            ie0.id.cf.typeB = ContactFeatureType.e_vertex;
            const ie1 = ie[1];
            ie1.v.Copy(this.m_polygonB.vertices[i2]);
            ie1.id.cf.indexA = 0;
            ie1.id.cf.indexB = i2;
            ie1.id.cf.typeA = ContactFeatureType.e_face;
            ie1.id.cf.typeB = ContactFeatureType.e_vertex;
            if (this.m_front) {
                rf.i1 = 0;
                rf.i2 = 1;
                rf.v1.Copy(this.m_v1);
                rf.v2.Copy(this.m_v2);
                rf.normal.Copy(this.m_normal1);
            }
            else {
                rf.i1 = 1;
                rf.i2 = 0;
                rf.v1.Copy(this.m_v2);
                rf.v2.Copy(this.m_v1);
                rf.normal.Copy(this.m_normal1).SelfNeg();
            }
        }
        else {
            manifold.type = ManifoldType.e_faceB;
            const ie0 = ie[0];
            ie0.v.Copy(this.m_v1);
            ie0.id.cf.indexA = 0;
            ie0.id.cf.indexB = primaryAxis.index;
            ie0.id.cf.typeA = ContactFeatureType.e_vertex;
            ie0.id.cf.typeB = ContactFeatureType.e_face;
            const ie1 = ie[1];
            ie1.v.Copy(this.m_v2);
            ie1.id.cf.indexA = 0;
            ie1.id.cf.indexB = primaryAxis.index;
            ie1.id.cf.typeA = ContactFeatureType.e_vertex;
            ie1.id.cf.typeB = ContactFeatureType.e_face;
            rf.i1 = primaryAxis.index;
            rf.i2 = (rf.i1 + 1) % this.m_polygonB.count;
            rf.v1.Copy(this.m_polygonB.vertices[rf.i1]);
            rf.v2.Copy(this.m_polygonB.vertices[rf.i2]);
            rf.normal.Copy(this.m_polygonB.normals[rf.i1]);
        }
        rf.sideNormal1.Set(rf.normal.y, -rf.normal.x);
        rf.sideNormal2.Copy(rf.sideNormal1).SelfNeg();
        rf.sideOffset1 = Vec2.DotVV(rf.sideNormal1, rf.v1);
        rf.sideOffset2 = Vec2.DotVV(rf.sideNormal2, rf.v2);
        // Clip incident edge against extruded edge1 side edges.
        const clipPoints1 = EPCollider.s_clipPoints1;
        const clipPoints2 = EPCollider.s_clipPoints2;
        let np = 0;
        // Clip to box side 1
        np = ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
        if (np < maxManifoldPoints) {
            return;
        }
        // Clip to negative box side 1
        np = ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
        if (np < maxManifoldPoints) {
            return;
        }
        // Now clipPoints2 contains the clipped points.
        if (primaryAxis.type === EPAxisType.e_edgeA) {
            manifold.localNormal.Copy(rf.normal);
            manifold.localPoint.Copy(rf.v1);
        }
        else {
            manifold.localNormal.Copy(polygonB.m_normals[rf.i1]);
            manifold.localPoint.Copy(polygonB.m_vertices[rf.i1]);
        }
        let pointCount = 0;
        for (let i = 0; i < maxManifoldPoints; ++i) {
            let separation;
            separation = Vec2.DotVV(rf.normal, Vec2.SubVV(clipPoints2[i].v, rf.v1, Vec2.s_t0));
            if (separation <= this.m_radius) {
                const cp = manifold.points[pointCount];
                if (primaryAxis.type === EPAxisType.e_edgeA) {
                    Transform.MulTXV(this.m_xf, clipPoints2[i].v, cp.localPoint);
                    cp.id = clipPoints2[i].id;
                }
                else {
                    cp.localPoint.Copy(clipPoints2[i].v);
                    cp.id.cf.typeA = clipPoints2[i].id.cf.typeB;
                    cp.id.cf.typeB = clipPoints2[i].id.cf.typeA;
                    cp.id.cf.indexA = clipPoints2[i].id.cf.indexB;
                    cp.id.cf.indexB = clipPoints2[i].id.cf.indexA;
                }
                ++pointCount;
            }
        }
        manifold.pointCount = pointCount;
    }
    ComputeEdgeSeparation(out) {
        const axis = out;
        axis.type = EPAxisType.e_edgeA;
        axis.index = this.m_front ? 0 : 1;
        axis.separation = maxFloat;
        for (let i = 0; i < this.m_polygonB.count; ++i) {
            const s = Vec2.DotVV(this.m_normal, Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v1, Vec2.s_t0));
            if (s < axis.separation) {
                axis.separation = s;
            }
        }
        return axis;
    }
    ComputePolygonSeparation(out) {
        const axis = out;
        axis.type = EPAxisType.e_unknown;
        axis.index = -1;
        axis.separation = -maxFloat;
        const perp = EPCollider.s_perp.Set(-this.m_normal.y, this.m_normal.x);
        for (let i = 0; i < this.m_polygonB.count; ++i) {
            const n = Vec2.NegV(this.m_polygonB.normals[i], EPCollider.s_n);
            const s1 = Vec2.DotVV(n, Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v1, Vec2.s_t0));
            const s2 = Vec2.DotVV(n, Vec2.SubVV(this.m_polygonB.vertices[i], this.m_v2, Vec2.s_t0));
            const s = Min(s1, s2);
            if (s > this.m_radius) {
                // No collision
                axis.type = EPAxisType.e_edgeB;
                axis.index = i;
                axis.separation = s;
                return axis;
            }
            // Adjacency
            if (Vec2.DotVV(n, perp) >= 0) {
                if (Vec2.DotVV(Vec2.SubVV(n, this.m_upperLimit, Vec2.s_t0), this.m_normal) < -angularSlop) {
                    continue;
                }
            }
            else {
                if (Vec2.DotVV(Vec2.SubVV(n, this.m_lowerLimit, Vec2.s_t0), this.m_normal) < -angularSlop) {
                    continue;
                }
            }
            if (s > axis.separation) {
                axis.type = EPAxisType.e_edgeB;
                axis.index = i;
                axis.separation = s;
            }
        }
        return axis;
    }
}
EPCollider.s_edge1 = new Vec2();
EPCollider.s_edge0 = new Vec2();
EPCollider.s_edge2 = new Vec2();
EPCollider.s_ie = ClipVertex.MakeArray(2);
EPCollider.s_rf = new ReferenceFace();
EPCollider.s_clipPoints1 = ClipVertex.MakeArray(2);
EPCollider.s_clipPoints2 = ClipVertex.MakeArray(2);
EPCollider.s_edgeAxis = new EPAxis();
EPCollider.s_polygonAxis = new EPAxis();
EPCollider.s_n = new Vec2();
EPCollider.s_perp = new Vec2();
const CollideEdgeAndPolygon_s_collider = new EPCollider();
export function CollideEdgeAndPolygon(manifold, edgeA, xfA, polygonB, xfB) {
    const collider = CollideEdgeAndPolygon_s_collider;
    collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
}
//# sourceMappingURL=CollideEdge.js.map