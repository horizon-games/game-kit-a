// DEBUG: import { Assert } from "../Common/Settings";
import { Rot, Transform, Vec2 } from '../Common/Math'
import { maxFloat, maxManifoldPoints } from '../Common/Settings'

import {
  ClipSegmentToLine,
  ClipVertex,
  ContactFeature,
  ContactFeatureType,
  Manifold,
  ManifoldPoint,
  ManifoldType
} from './Collision'
import { PolygonShape } from './Shapes/PolygonShape'

const edgeSeparation_s_normal1World: Vec2 = new Vec2()
const edgeSeparation_s_normal1: Vec2 = new Vec2()
const edgeSeparation_s_v1: Vec2 = new Vec2()
const edgeSeparation_s_v2: Vec2 = new Vec2()
function EdgeSeparation(
  poly1: PolygonShape,
  xf1: Transform,
  edge1: number,
  poly2: PolygonShape,
  xf2: Transform
): number {
  // DEBUG: const count1: number = poly1.m_count;
  const vertices1: Vec2[] = poly1.m_vertices
  const normals1: Vec2[] = poly1.m_normals

  const count2: number = poly2.m_count
  const vertices2: Vec2[] = poly2.m_vertices

  // DEBUG: Assert(0 <= edge1 && edge1 < count1);

  // Convert normal from poly1's frame into poly2's frame.
  const normal1World: Vec2 = Rot.MulRV(
    xf1.q,
    normals1[edge1],
    edgeSeparation_s_normal1World
  )
  const normal1: Vec2 = Rot.MulTRV(
    xf2.q,
    normal1World,
    edgeSeparation_s_normal1
  )

  // Find support vertex on poly2 for -normal.
  let index: number = 0
  let minDot: number = maxFloat

  for (let i: number = 0; i < count2; ++i) {
    const dot: number = Vec2.DotVV(vertices2[i], normal1)
    if (dot < minDot) {
      minDot = dot
      index = i
    }
  }

  const v1: Vec2 = Transform.MulXV(xf1, vertices1[edge1], edgeSeparation_s_v1)
  const v2: Vec2 = Transform.MulXV(xf2, vertices2[index], edgeSeparation_s_v2)
  const separation: number = Vec2.DotVV(
    Vec2.SubVV(v2, v1, Vec2.s_t0),
    normal1World
  )
  return separation
}

const FindMaxSeparation_s_d: Vec2 = new Vec2()
const FindMaxSeparation_s_dLocal1: Vec2 = new Vec2()
function FindMaxSeparation(
  edgeIndex: number[],
  poly1: PolygonShape,
  xf1: Transform,
  poly2: PolygonShape,
  xf2: Transform
): number {
  const count1: number = poly1.m_count
  const normals1: Vec2[] = poly1.m_normals

  // Vector pointing from the centroid of poly1 to the centroid of poly2.
  const d: Vec2 = Vec2.SubVV(
    Transform.MulXV(xf2, poly2.m_centroid, Vec2.s_t0),
    Transform.MulXV(xf1, poly1.m_centroid, Vec2.s_t1),
    FindMaxSeparation_s_d
  )
  const dLocal1: Vec2 = Rot.MulTRV(xf1.q, d, FindMaxSeparation_s_dLocal1)

  // Find edge normal on poly1 that has the largest projection onto d.
  let edge: number = 0
  let maxDot: number = -maxFloat
  for (let i: number = 0; i < count1; ++i) {
    const dot: number = Vec2.DotVV(normals1[i], dLocal1)
    if (dot > maxDot) {
      maxDot = dot
      edge = i
    }
  }

  // Get the separation for the edge normal.
  let s: number = EdgeSeparation(poly1, xf1, edge, poly2, xf2)

  // Check the separation for the previous edge normal.
  const prevEdge = (edge + count1 - 1) % count1
  const sPrev = EdgeSeparation(poly1, xf1, prevEdge, poly2, xf2)

  // Check the separation for the next edge normal.
  const nextEdge = (edge + 1) % count1
  const sNext = EdgeSeparation(poly1, xf1, nextEdge, poly2, xf2)

  // Find the best edge and the search direction.
  let bestEdge: number = 0
  let bestSeparation: number = 0
  let increment: number = 0
  if (sPrev > s && sPrev > sNext) {
    increment = -1
    bestEdge = prevEdge
    bestSeparation = sPrev
  } else if (sNext > s) {
    increment = 1
    bestEdge = nextEdge
    bestSeparation = sNext
  } else {
    edgeIndex[0] = edge
    return s
  }

  // Perform a local search for the best edge normal.
  while (true) {
    edge =
      increment === -1
        ? (bestEdge + count1 - 1) % count1
        : (bestEdge + 1) % count1

    s = EdgeSeparation(poly1, xf1, edge, poly2, xf2)

    if (s > bestSeparation) {
      bestEdge = edge
      bestSeparation = s
    } else {
      break
    }
  }

  edgeIndex[0] = bestEdge
  return bestSeparation
}

const FindIncidentEdge_s_normal1: Vec2 = new Vec2()
function FindIncidentEdge(
  c: ClipVertex[],
  poly1: PolygonShape,
  xf1: Transform,
  edge1: number,
  poly2: PolygonShape,
  xf2: Transform
): void {
  // DEBUG: const count1: number = poly1.m_count;
  const normals1: Vec2[] = poly1.m_normals

  const count2: number = poly2.m_count
  const vertices2: Vec2[] = poly2.m_vertices
  const normals2: Vec2[] = poly2.m_normals

  // DEBUG: Assert(0 <= edge1 && edge1 < count1);

  // Get the normal of the reference edge in poly2's frame.
  const normal1: Vec2 = Rot.MulTRV(
    xf2.q,
    Rot.MulRV(xf1.q, normals1[edge1], Vec2.s_t0),
    FindIncidentEdge_s_normal1
  )

  // Find the incident edge on poly2.
  let index: number = 0
  let minDot: number = maxFloat
  for (let i: number = 0; i < count2; ++i) {
    const dot: number = Vec2.DotVV(normal1, normals2[i])
    if (dot < minDot) {
      minDot = dot
      index = i
    }
  }

  // Build the clip vertices for the incident edge.
  const i1: number = index
  const i2: number = (i1 + 1) % count2

  const c0: ClipVertex = c[0]
  Transform.MulXV(xf2, vertices2[i1], c0.v)
  const cf0: ContactFeature = c0.id.cf
  cf0.indexA = edge1
  cf0.indexB = i1
  cf0.typeA = ContactFeatureType.e_face
  cf0.typeB = ContactFeatureType.e_vertex

  const c1: ClipVertex = c[1]
  Transform.MulXV(xf2, vertices2[i2], c1.v)
  const cf1: ContactFeature = c1.id.cf
  cf1.indexA = edge1
  cf1.indexB = i2
  cf1.typeA = ContactFeatureType.e_face
  cf1.typeB = ContactFeatureType.e_vertex
}

const CollidePolygons_s_incidentEdge = ClipVertex.MakeArray(2)
const CollidePolygons_s_clipPoints1 = ClipVertex.MakeArray(2)
const CollidePolygons_s_clipPoints2 = ClipVertex.MakeArray(2)
const CollidePolygons_s_edgeA = [0]
const CollidePolygons_s_edgeB = [0]
const CollidePolygons_s_localTangent: Vec2 = new Vec2()
const CollidePolygons_s_localNormal: Vec2 = new Vec2()
const CollidePolygons_s_planePoint: Vec2 = new Vec2()
const CollidePolygons_s_normal: Vec2 = new Vec2()
const CollidePolygons_s_tangent: Vec2 = new Vec2()
const CollidePolygons_s_ntangent: Vec2 = new Vec2()
const CollidePolygons_s_v11: Vec2 = new Vec2()
const CollidePolygons_s_v12: Vec2 = new Vec2()
export function CollidePolygons(
  manifold: Manifold,
  polyA: PolygonShape,
  xfA: Transform,
  polyB: PolygonShape,
  xfB: Transform
): void {
  manifold.pointCount = 0
  const totalRadius: number = polyA.m_radius + polyB.m_radius

  const edgeA: number[] = CollidePolygons_s_edgeA
  edgeA[0] = 0
  const separationA: number = FindMaxSeparation(edgeA, polyA, xfA, polyB, xfB)
  if (separationA > totalRadius) {
    return
  }

  const edgeB: number[] = CollidePolygons_s_edgeB
  edgeB[0] = 0
  const separationB: number = FindMaxSeparation(edgeB, polyB, xfB, polyA, xfA)
  if (separationB > totalRadius) {
    return
  }

  let poly1: PolygonShape // reference polygon
  let poly2: PolygonShape // incident polygon
  let xf1: Transform
  let xf2: Transform
  let edge1: number = 0 // reference edge
  let flip: number = 0
  const k_relativeTol: number = 0.98
  const k_absoluteTol: number = 0.001

  if (separationB > k_relativeTol * separationA + k_absoluteTol) {
    poly1 = polyB
    poly2 = polyA
    xf1 = xfB
    xf2 = xfA
    edge1 = edgeB[0]
    manifold.type = ManifoldType.e_faceB
    flip = 1
  } else {
    poly1 = polyA
    poly2 = polyB
    xf1 = xfA
    xf2 = xfB
    edge1 = edgeA[0]
    manifold.type = ManifoldType.e_faceA
    flip = 0
  }

  const incidentEdge: ClipVertex[] = CollidePolygons_s_incidentEdge
  FindIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2)

  const count1: number = poly1.m_count
  const vertices1: Vec2[] = poly1.m_vertices

  const iv1: number = edge1
  const iv2: number = (edge1 + 1) % count1

  const local_v11: Vec2 = vertices1[iv1]
  const local_v12: Vec2 = vertices1[iv2]

  const localTangent: Vec2 = Vec2.SubVV(
    local_v12,
    local_v11,
    CollidePolygons_s_localTangent
  )
  localTangent.Normalize()

  const localNormal: Vec2 = Vec2.CrossVOne(
    localTangent,
    CollidePolygons_s_localNormal
  )
  const planePoint: Vec2 = Vec2.MidVV(
    local_v11,
    local_v12,
    CollidePolygons_s_planePoint
  )

  const tangent: Vec2 = Rot.MulRV(
    xf1.q,
    localTangent,
    CollidePolygons_s_tangent
  )
  const normal: Vec2 = Vec2.CrossVOne(tangent, CollidePolygons_s_normal)

  const v11: Vec2 = Transform.MulXV(xf1, local_v11, CollidePolygons_s_v11)
  const v12: Vec2 = Transform.MulXV(xf1, local_v12, CollidePolygons_s_v12)

  // Face offset.
  const frontOffset: number = Vec2.DotVV(normal, v11)

  // Side offsets, extended by polytope skin thickness.
  const sideOffset1: number = -Vec2.DotVV(tangent, v11) + totalRadius
  const sideOffset2: number = Vec2.DotVV(tangent, v12) + totalRadius

  // Clip incident edge against extruded edge1 side edges.
  const clipPoints1: ClipVertex[] = CollidePolygons_s_clipPoints1
  const clipPoints2: ClipVertex[] = CollidePolygons_s_clipPoints2
  let np: number

  // Clip to box side 1
  const ntangent: Vec2 = Vec2.NegV(tangent, CollidePolygons_s_ntangent)
  np = ClipSegmentToLine(clipPoints1, incidentEdge, ntangent, sideOffset1, iv1)

  if (np < 2) {
    return
  }

  // Clip to negative box side 1
  np = ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2)

  if (np < 2) {
    return
  }

  // Now clipPoints2 contains the clipped points.
  manifold.localNormal.Copy(localNormal)
  manifold.localPoint.Copy(planePoint)

  let pointCount: number = 0
  for (let i: number = 0; i < maxManifoldPoints; ++i) {
    const cv: ClipVertex = clipPoints2[i]
    const separation: number = Vec2.DotVV(normal, cv.v) - frontOffset

    if (separation <= totalRadius) {
      const cp: ManifoldPoint = manifold.points[pointCount]
      Transform.MulTXV(xf2, cv.v, cp.localPoint)
      cp.id.Copy(cv.id)
      if (flip) {
        // Swap features
        const cf: ContactFeature = cp.id.cf
        cp.id.cf.indexA = cf.indexB
        cp.id.cf.indexB = cf.indexA
        cp.id.cf.typeA = cf.typeB
        cp.id.cf.typeB = cf.typeA
      }
      ++pointCount
    }
  }

  manifold.pointCount = pointCount
}
