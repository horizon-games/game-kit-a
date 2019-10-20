import { Transform, Vec2 } from '../Common/Math';
import { epsilon, maxFloat } from '../Common/Settings';
import { ManifoldType } from './Collision';
const CollideCircles_s_pA = new Vec2();
const CollideCircles_s_pB = new Vec2();
export function CollideCircles(manifold, circleA, xfA, circleB, xfB) {
    manifold.pointCount = 0;
    const pA = Transform.MulXV(xfA, circleA.m_p, CollideCircles_s_pA);
    const pB = Transform.MulXV(xfB, circleB.m_p, CollideCircles_s_pB);
    const distSqr = Vec2.DistanceSquaredVV(pA, pB);
    const radius = circleA.m_radius + circleB.m_radius;
    if (distSqr > radius * radius) {
        return;
    }
    manifold.type = ManifoldType.e_circles;
    manifold.localPoint.Copy(circleA.m_p);
    manifold.localNormal.SetZero();
    manifold.pointCount = 1;
    manifold.points[0].localPoint.Copy(circleB.m_p);
    manifold.points[0].id.key = 0;
}
const CollidePolygonAndCircle_s_c = new Vec2();
const CollidePolygonAndCircle_s_cLocal = new Vec2();
const CollidePolygonAndCircle_s_faceCenter = new Vec2();
export function CollidePolygonAndCircle(manifold, polygonA, xfA, circleB, xfB) {
    manifold.pointCount = 0;
    // Compute circle position in the frame of the polygon.
    const c = Transform.MulXV(xfB, circleB.m_p, CollidePolygonAndCircle_s_c);
    const cLocal = Transform.MulTXV(xfA, c, CollidePolygonAndCircle_s_cLocal);
    // Find the min separating edge.
    let normalIndex = 0;
    let separation = -maxFloat;
    const radius = polygonA.m_radius + circleB.m_radius;
    const vertexCount = polygonA.m_count;
    const vertices = polygonA.m_vertices;
    const normals = polygonA.m_normals;
    for (let i = 0; i < vertexCount; ++i) {
        const s = Vec2.DotVV(normals[i], Vec2.SubVV(cLocal, vertices[i], Vec2.s_t0));
        if (s > radius) {
            // Early out.
            return;
        }
        if (s > separation) {
            separation = s;
            normalIndex = i;
        }
    }
    // Vertices that subtend the incident face.
    const vertIndex1 = normalIndex;
    const vertIndex2 = (vertIndex1 + 1) % vertexCount;
    const v1 = vertices[vertIndex1];
    const v2 = vertices[vertIndex2];
    // If the center is inside the polygon ...
    if (separation < epsilon) {
        manifold.pointCount = 1;
        manifold.type = ManifoldType.e_faceA;
        manifold.localNormal.Copy(normals[normalIndex]);
        Vec2.MidVV(v1, v2, manifold.localPoint);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
        return;
    }
    // Compute barycentric coordinates
    const u1 = Vec2.DotVV(Vec2.SubVV(cLocal, v1, Vec2.s_t0), Vec2.SubVV(v2, v1, Vec2.s_t1));
    const u2 = Vec2.DotVV(Vec2.SubVV(cLocal, v2, Vec2.s_t0), Vec2.SubVV(v1, v2, Vec2.s_t1));
    if (u1 <= 0) {
        if (Vec2.DistanceSquaredVV(cLocal, v1) > radius * radius) {
            return;
        }
        manifold.pointCount = 1;
        manifold.type = ManifoldType.e_faceA;
        Vec2.SubVV(cLocal, v1, manifold.localNormal).SelfNormalize();
        manifold.localPoint.Copy(v1);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
    else if (u2 <= 0) {
        if (Vec2.DistanceSquaredVV(cLocal, v2) > radius * radius) {
            return;
        }
        manifold.pointCount = 1;
        manifold.type = ManifoldType.e_faceA;
        Vec2.SubVV(cLocal, v2, manifold.localNormal).SelfNormalize();
        manifold.localPoint.Copy(v2);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
    else {
        const faceCenter = Vec2.MidVV(v1, v2, CollidePolygonAndCircle_s_faceCenter);
        const separation = Vec2.DotVV(Vec2.SubVV(cLocal, faceCenter, Vec2.s_t1), normals[vertIndex1]);
        if (separation > radius) {
            return;
        }
        manifold.pointCount = 1;
        manifold.type = ManifoldType.e_faceA;
        manifold.localNormal.Copy(normals[vertIndex1]).SelfNormalize();
        manifold.localPoint.Copy(faceCenter);
        manifold.points[0].localPoint.Copy(circleB.m_p);
        manifold.points[0].id.key = 0;
    }
}
//# sourceMappingURL=CollideCircle.js.map