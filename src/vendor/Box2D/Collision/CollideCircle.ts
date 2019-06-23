import { Transform, Vec2 } from '../Common/Math'
import { epsilon, maxFloat } from '../Common/Settings'

import { Manifold, ManifoldType } from './Collision'
import { CircleShape } from './Shapes/CircleShape'
import { PolygonShape } from './Shapes/PolygonShape'

const CollideCircles_s_pA: Vec2 = new Vec2()
const CollideCircles_s_pB: Vec2 = new Vec2()
export function CollideCircles(
  manifold: Manifold,
  circleA: CircleShape,
  xfA: Transform,
  circleB: CircleShape,
  xfB: Transform
): void {
  manifold.pointCount = 0

  const pA: Vec2 = Transform.MulXV(xfA, circleA.m_p, CollideCircles_s_pA)
  const pB: Vec2 = Transform.MulXV(xfB, circleB.m_p, CollideCircles_s_pB)

  const distSqr: number = Vec2.DistanceSquaredVV(pA, pB)
  const radius: number = circleA.m_radius + circleB.m_radius
  if (distSqr > radius * radius) {
    return
  }

  manifold.type = ManifoldType.e_circles
  manifold.localPoint.Copy(circleA.m_p)
  manifold.localNormal.SetZero()
  manifold.pointCount = 1

  manifold.points[0].localPoint.Copy(circleB.m_p)
  manifold.points[0].id.key = 0
}

const CollidePolygonAndCircle_s_c: Vec2 = new Vec2()
const CollidePolygonAndCircle_s_cLocal: Vec2 = new Vec2()
const CollidePolygonAndCircle_s_faceCenter: Vec2 = new Vec2()
export function CollidePolygonAndCircle(
  manifold: Manifold,
  polygonA: PolygonShape,
  xfA: Transform,
  circleB: CircleShape,
  xfB: Transform
): void {
  manifold.pointCount = 0

  // Compute circle position in the frame of the polygon.
  const c: Vec2 = Transform.MulXV(xfB, circleB.m_p, CollidePolygonAndCircle_s_c)
  const cLocal: Vec2 = Transform.MulTXV(
    xfA,
    c,
    CollidePolygonAndCircle_s_cLocal
  )

  // Find the min separating edge.
  let normalIndex: number = 0
  let separation: number = -maxFloat
  const radius: number = polygonA.m_radius + circleB.m_radius
  const vertexCount: number = polygonA.m_count
  const vertices: Vec2[] = polygonA.m_vertices
  const normals: Vec2[] = polygonA.m_normals

  for (let i: number = 0; i < vertexCount; ++i) {
    const s: number = Vec2.DotVV(
      normals[i],
      Vec2.SubVV(cLocal, vertices[i], Vec2.s_t0)
    )

    if (s > radius) {
      // Early out.
      return
    }

    if (s > separation) {
      separation = s
      normalIndex = i
    }
  }

  // Vertices that subtend the incident face.
  const vertIndex1: number = normalIndex
  const vertIndex2: number = (vertIndex1 + 1) % vertexCount
  const v1: Vec2 = vertices[vertIndex1]
  const v2: Vec2 = vertices[vertIndex2]

  // If the center is inside the polygon ...
  if (separation < epsilon) {
    manifold.pointCount = 1
    manifold.type = ManifoldType.e_faceA
    manifold.localNormal.Copy(normals[normalIndex])
    Vec2.MidVV(v1, v2, manifold.localPoint)
    manifold.points[0].localPoint.Copy(circleB.m_p)
    manifold.points[0].id.key = 0
    return
  }

  // Compute barycentric coordinates
  const u1: number = Vec2.DotVV(
    Vec2.SubVV(cLocal, v1, Vec2.s_t0),
    Vec2.SubVV(v2, v1, Vec2.s_t1)
  )
  const u2: number = Vec2.DotVV(
    Vec2.SubVV(cLocal, v2, Vec2.s_t0),
    Vec2.SubVV(v1, v2, Vec2.s_t1)
  )
  if (u1 <= 0) {
    if (Vec2.DistanceSquaredVV(cLocal, v1) > radius * radius) {
      return
    }

    manifold.pointCount = 1
    manifold.type = ManifoldType.e_faceA
    Vec2.SubVV(cLocal, v1, manifold.localNormal).SelfNormalize()
    manifold.localPoint.Copy(v1)
    manifold.points[0].localPoint.Copy(circleB.m_p)
    manifold.points[0].id.key = 0
  } else if (u2 <= 0) {
    if (Vec2.DistanceSquaredVV(cLocal, v2) > radius * radius) {
      return
    }

    manifold.pointCount = 1
    manifold.type = ManifoldType.e_faceA
    Vec2.SubVV(cLocal, v2, manifold.localNormal).SelfNormalize()
    manifold.localPoint.Copy(v2)
    manifold.points[0].localPoint.Copy(circleB.m_p)
    manifold.points[0].id.key = 0
  } else {
    const faceCenter: Vec2 = Vec2.MidVV(
      v1,
      v2,
      CollidePolygonAndCircle_s_faceCenter
    )
    const separation = Vec2.DotVV(
      Vec2.SubVV(cLocal, faceCenter, Vec2.s_t1),
      normals[vertIndex1]
    )
    if (separation > radius) {
      return
    }

    manifold.pointCount = 1
    manifold.type = ManifoldType.e_faceA
    manifold.localNormal.Copy(normals[vertIndex1]).SelfNormalize()
    manifold.localPoint.Copy(faceCenter)
    manifold.points[0].localPoint.Copy(circleB.m_p)
    manifold.points[0].id.key = 0
  }
}
