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
import { Abs, Max, Rot, Transform, Vec2 } from '../Common/Math'
import {
  epsilon,
  epsilon_sq,
  linearSlop,
  polygonRadius
} from '../Common/Settings'

import { Shape } from './Shapes/Shape'

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
export class DistanceProxy {
  readonly m_buffer: Vec2[] = Vec2.MakeArray(2)
  m_vertices: Vec2[] = this.m_buffer
  m_count: number = 0
  m_radius: number = 0

  Copy(other: Readonly<DistanceProxy>): this {
    if (other.m_vertices === other.m_buffer) {
      this.m_vertices = this.m_buffer
      this.m_buffer[0].Copy(other.m_buffer[0])
      this.m_buffer[1].Copy(other.m_buffer[1])
    } else {
      this.m_vertices = other.m_vertices
    }
    this.m_count = other.m_count
    this.m_radius = other.m_radius
    return this
  }

  Reset(): DistanceProxy {
    this.m_vertices = this.m_buffer
    this.m_count = 0
    this.m_radius = 0
    return this
  }

  SetShape(shape: Shape, index: number): void {
    shape.SetupDistanceProxy(this, index)
  }

  SetVerticesRadius(vertices: Vec2[], count: number, radius: number): void {
    this.m_vertices = vertices
    this.m_count = count
    this.m_radius = radius
  }

  GetSupport(d: Vec2): number {
    let bestIndex: number = 0
    let bestValue: number = Vec2.DotVV(this.m_vertices[0], d)
    for (let i: number = 1; i < this.m_count; ++i) {
      const value: number = Vec2.DotVV(this.m_vertices[i], d)
      if (value > bestValue) {
        bestIndex = i
        bestValue = value
      }
    }

    return bestIndex
  }

  GetSupportVertex(d: Vec2): Vec2 {
    let bestIndex: number = 0
    let bestValue: number = Vec2.DotVV(this.m_vertices[0], d)
    for (let i: number = 1; i < this.m_count; ++i) {
      const value: number = Vec2.DotVV(this.m_vertices[i], d)
      if (value > bestValue) {
        bestIndex = i
        bestValue = value
      }
    }

    return this.m_vertices[bestIndex]
  }

  GetVertexCount(): number {
    return this.m_count
  }

  GetVertex(index: number): Vec2 {
    // DEBUG: Assert(0 <= index && index < this.m_count);
    return this.m_vertices[index]
  }
}

export class SimplexCache {
  metric: number = 0
  count: number = 0
  readonly indexA: number[] = [0, 0, 0]
  readonly indexB: number[] = [0, 0, 0]

  Reset(): SimplexCache {
    this.metric = 0
    this.count = 0
    return this
  }
}

export class DistanceInput {
  readonly proxyA: DistanceProxy = new DistanceProxy()
  readonly proxyB: DistanceProxy = new DistanceProxy()
  readonly transformA: Transform = new Transform()
  readonly transformB: Transform = new Transform()
  useRadii: boolean = false

  Reset(): DistanceInput {
    this.proxyA.Reset()
    this.proxyB.Reset()
    this.transformA.SetIdentity()
    this.transformB.SetIdentity()
    this.useRadii = false
    return this
  }
}

export class DistanceOutput {
  readonly pointA: Vec2 = new Vec2()
  readonly pointB: Vec2 = new Vec2()
  distance: number = 0
  iterations: number = 0 ///< number of GJK iterations used

  Reset(): DistanceOutput {
    this.pointA.SetZero()
    this.pointB.SetZero()
    this.distance = 0
    this.iterations = 0
    return this
  }
}

/// Input parameters for ShapeCast
export class ShapeCastInput {
  readonly proxyA: DistanceProxy = new DistanceProxy()
  readonly proxyB: DistanceProxy = new DistanceProxy()
  readonly transformA: Transform = new Transform()
  readonly transformB: Transform = new Transform()
  readonly translationB: Vec2 = new Vec2()
}

/// Output results for ShapeCast
export class ShapeCastOutput {
  readonly point: Vec2 = new Vec2()
  readonly normal: Vec2 = new Vec2()
  lambda: number = 0.0
  iterations: number = 0
}

export let gjkCalls: number = 0
export let gjkIters: number = 0
export let gjkMaxIters: number = 0
export function _gjk_reset(): void {
  gjkCalls = 0
  gjkIters = 0
  gjkMaxIters = 0
}

export class SimplexVertex {
  readonly wA: Vec2 = new Vec2() // support point in proxyA
  readonly wB: Vec2 = new Vec2() // support point in proxyB
  readonly w: Vec2 = new Vec2() // wB - wA
  a: number = 0 // barycentric coordinate for closest point
  indexA: number = 0 // wA index
  indexB: number = 0 // wB index

  Copy(other: SimplexVertex): SimplexVertex {
    this.wA.Copy(other.wA) // support point in proxyA
    this.wB.Copy(other.wB) // support point in proxyB
    this.w.Copy(other.w) // wB - wA
    this.a = other.a // barycentric coordinate for closest point
    this.indexA = other.indexA // wA index
    this.indexB = other.indexB // wB index
    return this
  }
}

export class Simplex {
  private static s_e12: Vec2 = new Vec2()
  private static s_e13: Vec2 = new Vec2()
  private static s_e23: Vec2 = new Vec2()
  readonly m_v1: SimplexVertex = new SimplexVertex()
  readonly m_v2: SimplexVertex = new SimplexVertex()
  readonly m_v3: SimplexVertex = new SimplexVertex()
  readonly m_vertices: SimplexVertex[] = [
    /*3*/
  ]
  m_count: number = 0

  constructor() {
    this.m_vertices[0] = this.m_v1
    this.m_vertices[1] = this.m_v2
    this.m_vertices[2] = this.m_v3
  }

  ReadCache(
    cache: SimplexCache,
    proxyA: DistanceProxy,
    transformA: Transform,
    proxyB: DistanceProxy,
    transformB: Transform
  ): void {
    // DEBUG: Assert(0 <= cache.count && cache.count <= 3);

    // Copy data from cache.
    this.m_count = cache.count
    const vertices: SimplexVertex[] = this.m_vertices
    for (let i: number = 0; i < this.m_count; ++i) {
      const v: SimplexVertex = vertices[i]
      v.indexA = cache.indexA[i]
      v.indexB = cache.indexB[i]
      const wALocal: Vec2 = proxyA.GetVertex(v.indexA)
      const wBLocal: Vec2 = proxyB.GetVertex(v.indexB)
      Transform.MulXV(transformA, wALocal, v.wA)
      Transform.MulXV(transformB, wBLocal, v.wB)
      Vec2.SubVV(v.wB, v.wA, v.w)
      v.a = 0
    }

    // Compute the new simplex metric, if it is substantially different than
    // old metric then flush the simplex.
    if (this.m_count > 1) {
      const metric1: number = cache.metric
      const metric2: number = this.GetMetric()
      if (
        metric2 < 0.5 * metric1 ||
        2 * metric1 < metric2 ||
        metric2 < epsilon
      ) {
        // Reset the simplex.
        this.m_count = 0
      }
    }

    // If the cache is empty or invalid ...
    if (this.m_count === 0) {
      const v: SimplexVertex = vertices[0]
      v.indexA = 0
      v.indexB = 0
      const wALocal: Vec2 = proxyA.GetVertex(0)
      const wBLocal: Vec2 = proxyB.GetVertex(0)
      Transform.MulXV(transformA, wALocal, v.wA)
      Transform.MulXV(transformB, wBLocal, v.wB)
      Vec2.SubVV(v.wB, v.wA, v.w)
      v.a = 1
      this.m_count = 1
    }
  }

  WriteCache(cache: SimplexCache): void {
    cache.metric = this.GetMetric()
    cache.count = this.m_count
    const vertices: SimplexVertex[] = this.m_vertices
    for (let i: number = 0; i < this.m_count; ++i) {
      cache.indexA[i] = vertices[i].indexA
      cache.indexB[i] = vertices[i].indexB
    }
  }

  GetSearchDirection(out: Vec2): Vec2 {
    switch (this.m_count) {
      case 1:
        return Vec2.NegV(this.m_v1.w, out)

      case 2: {
        const e12: Vec2 = Vec2.SubVV(this.m_v2.w, this.m_v1.w, out)
        const sgn: number = Vec2.CrossVV(e12, Vec2.NegV(this.m_v1.w, Vec2.s_t0))
        if (sgn > 0) {
          // Origin is left of e12.
          return Vec2.CrossOneV(e12, out)
        } else {
          // Origin is right of e12.
          return Vec2.CrossVOne(e12, out)
        }
      }

      default:
        // DEBUG: Assert(false);
        return out.SetZero()
    }
  }

  GetClosestPoint(out: Vec2): Vec2 {
    switch (this.m_count) {
      case 0:
        // DEBUG: Assert(false);
        return out.SetZero()

      case 1:
        return out.Copy(this.m_v1.w)

      case 2:
        return out.Set(
          this.m_v1.a * this.m_v1.w.x + this.m_v2.a * this.m_v2.w.x,
          this.m_v1.a * this.m_v1.w.y + this.m_v2.a * this.m_v2.w.y
        )

      case 3:
        return out.SetZero()

      default:
        // DEBUG: Assert(false);
        return out.SetZero()
    }
  }

  GetWitnessPoints(pA: Vec2, pB: Vec2): void {
    switch (this.m_count) {
      case 0:
        // DEBUG: Assert(false);
        break

      case 1:
        pA.Copy(this.m_v1.wA)
        pB.Copy(this.m_v1.wB)
        break

      case 2:
        pA.x = this.m_v1.a * this.m_v1.wA.x + this.m_v2.a * this.m_v2.wA.x
        pA.y = this.m_v1.a * this.m_v1.wA.y + this.m_v2.a * this.m_v2.wA.y
        pB.x = this.m_v1.a * this.m_v1.wB.x + this.m_v2.a * this.m_v2.wB.x
        pB.y = this.m_v1.a * this.m_v1.wB.y + this.m_v2.a * this.m_v2.wB.y
        break

      case 3:
        pB.x = pA.x =
          this.m_v1.a * this.m_v1.wA.x +
          this.m_v2.a * this.m_v2.wA.x +
          this.m_v3.a * this.m_v3.wA.x
        pB.y = pA.y =
          this.m_v1.a * this.m_v1.wA.y +
          this.m_v2.a * this.m_v2.wA.y +
          this.m_v3.a * this.m_v3.wA.y
        break

      default:
        // DEBUG: Assert(false);
        break
    }
  }

  GetMetric(): number {
    switch (this.m_count) {
      case 0:
        // DEBUG: Assert(false);
        return 0

      case 1:
        return 0

      case 2:
        return Vec2.DistanceVV(this.m_v1.w, this.m_v2.w)

      case 3:
        return Vec2.CrossVV(
          Vec2.SubVV(this.m_v2.w, this.m_v1.w, Vec2.s_t0),
          Vec2.SubVV(this.m_v3.w, this.m_v1.w, Vec2.s_t1)
        )

      default:
        // DEBUG: Assert(false);
        return 0
    }
  }

  Solve2(): void {
    const w1: Vec2 = this.m_v1.w
    const w2: Vec2 = this.m_v2.w
    const e12: Vec2 = Vec2.SubVV(w2, w1, Simplex.s_e12)

    // w1 region
    const d12_2: number = -Vec2.DotVV(w1, e12)
    if (d12_2 <= 0) {
      // a2 <= 0, so we clamp it to 0
      this.m_v1.a = 1
      this.m_count = 1
      return
    }

    // w2 region
    const d12_1: number = Vec2.DotVV(w2, e12)
    if (d12_1 <= 0) {
      // a1 <= 0, so we clamp it to 0
      this.m_v2.a = 1
      this.m_count = 1
      this.m_v1.Copy(this.m_v2)
      return
    }

    // Must be in e12 region.
    const inv_d12: number = 1 / (d12_1 + d12_2)
    this.m_v1.a = d12_1 * inv_d12
    this.m_v2.a = d12_2 * inv_d12
    this.m_count = 2
  }

  Solve3(): void {
    const w1: Vec2 = this.m_v1.w
    const w2: Vec2 = this.m_v2.w
    const w3: Vec2 = this.m_v3.w

    // Edge12
    // [1      1     ][a1] = [1]
    // [w1.e12 w2.e12][a2] = [0]
    // a3 = 0
    const e12: Vec2 = Vec2.SubVV(w2, w1, Simplex.s_e12)
    const w1e12: number = Vec2.DotVV(w1, e12)
    const w2e12: number = Vec2.DotVV(w2, e12)
    const d12_1: number = w2e12
    const d12_2: number = -w1e12

    // Edge13
    // [1      1     ][a1] = [1]
    // [w1.e13 w3.e13][a3] = [0]
    // a2 = 0
    const e13: Vec2 = Vec2.SubVV(w3, w1, Simplex.s_e13)
    const w1e13: number = Vec2.DotVV(w1, e13)
    const w3e13: number = Vec2.DotVV(w3, e13)
    const d13_1: number = w3e13
    const d13_2: number = -w1e13

    // Edge23
    // [1      1     ][a2] = [1]
    // [w2.e23 w3.e23][a3] = [0]
    // a1 = 0
    const e23: Vec2 = Vec2.SubVV(w3, w2, Simplex.s_e23)
    const w2e23: number = Vec2.DotVV(w2, e23)
    const w3e23: number = Vec2.DotVV(w3, e23)
    const d23_1: number = w3e23
    const d23_2: number = -w2e23

    // Triangle123
    const n123: number = Vec2.CrossVV(e12, e13)

    const d123_1: number = n123 * Vec2.CrossVV(w2, w3)
    const d123_2: number = n123 * Vec2.CrossVV(w3, w1)
    const d123_3: number = n123 * Vec2.CrossVV(w1, w2)

    // w1 region
    if (d12_2 <= 0 && d13_2 <= 0) {
      this.m_v1.a = 1
      this.m_count = 1
      return
    }

    // e12
    if (d12_1 > 0 && d12_2 > 0 && d123_3 <= 0) {
      const inv_d12: number = 1 / (d12_1 + d12_2)
      this.m_v1.a = d12_1 * inv_d12
      this.m_v2.a = d12_2 * inv_d12
      this.m_count = 2
      return
    }

    // e13
    if (d13_1 > 0 && d13_2 > 0 && d123_2 <= 0) {
      const inv_d13: number = 1 / (d13_1 + d13_2)
      this.m_v1.a = d13_1 * inv_d13
      this.m_v3.a = d13_2 * inv_d13
      this.m_count = 2
      this.m_v2.Copy(this.m_v3)
      return
    }

    // w2 region
    if (d12_1 <= 0 && d23_2 <= 0) {
      this.m_v2.a = 1
      this.m_count = 1
      this.m_v1.Copy(this.m_v2)
      return
    }

    // w3 region
    if (d13_1 <= 0 && d23_1 <= 0) {
      this.m_v3.a = 1
      this.m_count = 1
      this.m_v1.Copy(this.m_v3)
      return
    }

    // e23
    if (d23_1 > 0 && d23_2 > 0 && d123_1 <= 0) {
      const inv_d23: number = 1 / (d23_1 + d23_2)
      this.m_v2.a = d23_1 * inv_d23
      this.m_v3.a = d23_2 * inv_d23
      this.m_count = 2
      this.m_v1.Copy(this.m_v3)
      return
    }

    // Must be in triangle123
    const inv_d123: number = 1 / (d123_1 + d123_2 + d123_3)
    this.m_v1.a = d123_1 * inv_d123
    this.m_v2.a = d123_2 * inv_d123
    this.m_v3.a = d123_3 * inv_d123
    this.m_count = 3
  }
}

const distance_s_simplex: Simplex = new Simplex()
const distance_s_saveA = [0, 0, 0]
const distance_s_saveB = [0, 0, 0]
const distance_s_p: Vec2 = new Vec2()
const distance_s_d: Vec2 = new Vec2()
const distance_s_normal: Vec2 = new Vec2()
const distance_s_supportA: Vec2 = new Vec2()
const distance_s_supportB: Vec2 = new Vec2()
export function Distance(
  output: DistanceOutput,
  cache: SimplexCache,
  input: DistanceInput
): void {
  ++gjkCalls

  const proxyA: DistanceProxy = input.proxyA
  const proxyB: DistanceProxy = input.proxyB

  const transformA: Transform = input.transformA
  const transformB: Transform = input.transformB

  // Initialize the simplex.
  const simplex: Simplex = distance_s_simplex
  simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB)

  // Get simplex vertices as an array.
  const vertices: SimplexVertex[] = simplex.m_vertices
  const k_maxIters: number = 20

  // These store the vertices of the last simplex so that we
  // can check for duplicates and prevent cycling.
  const saveA: number[] = distance_s_saveA
  const saveB: number[] = distance_s_saveB
  let saveCount: number = 0

  // Main iteration loop.
  let iter: number = 0
  while (iter < k_maxIters) {
    // Copy simplex so we can identify duplicates.
    saveCount = simplex.m_count
    for (let i: number = 0; i < saveCount; ++i) {
      saveA[i] = vertices[i].indexA
      saveB[i] = vertices[i].indexB
    }

    switch (simplex.m_count) {
      case 1:
        break

      case 2:
        simplex.Solve2()
        break

      case 3:
        simplex.Solve3()
        break

      default:
        // DEBUG: Assert(false);
        break
    }

    // If we have 3 points, then the origin is in the corresponding triangle.
    if (simplex.m_count === 3) {
      break
    }

    // Get search direction.
    const d: Vec2 = simplex.GetSearchDirection(distance_s_d)

    // Ensure the search direction is numerically fit.
    if (d.LengthSquared() < epsilon_sq) {
      // The origin is probably contained by a line segment
      // or triangle. Thus the shapes are overlapped.

      // We can't return zero here even though there may be overlap.
      // In case the simplex is a point, segment, or triangle it is difficult
      // to determine if the origin is contained in the CSO or very close to it.
      break
    }

    // Compute a tentative new simplex vertex using support points.
    const vertex: SimplexVertex = vertices[simplex.m_count]
    vertex.indexA = proxyA.GetSupport(
      Rot.MulTRV(transformA.q, Vec2.NegV(d, Vec2.s_t0), distance_s_supportA)
    )
    Transform.MulXV(transformA, proxyA.GetVertex(vertex.indexA), vertex.wA)
    vertex.indexB = proxyB.GetSupport(
      Rot.MulTRV(transformB.q, d, distance_s_supportB)
    )
    Transform.MulXV(transformB, proxyB.GetVertex(vertex.indexB), vertex.wB)
    Vec2.SubVV(vertex.wB, vertex.wA, vertex.w)

    // Iteration count is equated to the number of support point calls.
    ++iter
    ++gjkIters

    // Check for duplicate support points. This is the main termination criteria.
    let duplicate: boolean = false
    for (let i: number = 0; i < saveCount; ++i) {
      if (vertex.indexA === saveA[i] && vertex.indexB === saveB[i]) {
        duplicate = true
        break
      }
    }

    // If we found a duplicate support point we must exit to avoid cycling.
    if (duplicate) {
      break
    }

    // New vertex is ok and needed.
    ++simplex.m_count
  }

  gjkMaxIters = Max(gjkMaxIters, iter)

  // Prepare output.
  simplex.GetWitnessPoints(output.pointA, output.pointB)
  output.distance = Vec2.DistanceVV(output.pointA, output.pointB)
  output.iterations = iter

  // Cache the simplex.
  simplex.WriteCache(cache)

  // Apply radii if requested.
  if (input.useRadii) {
    const rA: number = proxyA.m_radius
    const rB: number = proxyB.m_radius

    if (output.distance > rA + rB && output.distance > epsilon) {
      // Shapes are still no overlapped.
      // Move the witness points to the outer surface.
      output.distance -= rA + rB
      const normal: Vec2 = Vec2.SubVV(
        output.pointB,
        output.pointA,
        distance_s_normal
      )
      normal.Normalize()
      output.pointA.SelfMulAdd(rA, normal)
      output.pointB.SelfMulSub(rB, normal)
    } else {
      // Shapes are overlapped when radii are considered.
      // Move the witness points to the middle.
      const p: Vec2 = Vec2.MidVV(output.pointA, output.pointB, distance_s_p)
      output.pointA.Copy(p)
      output.pointB.Copy(p)
      output.distance = 0
    }
  }
}

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.

// GJK-raycast
// Algorithm by Gino van den Bergen.
// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
// bool ShapeCast(ShapeCastOutput* output, const ShapeCastInput* input);
const ShapeCast_s_n = new Vec2()
const ShapeCast_s_simplex = new Simplex()
const ShapeCast_s_wA = new Vec2()
const ShapeCast_s_wB = new Vec2()
const ShapeCast_s_v = new Vec2()
const ShapeCast_s_p = new Vec2()
const ShapeCast_s_pointA = new Vec2()
const ShapeCast_s_pointB = new Vec2()
export function ShapeCast(
  output: ShapeCastOutput,
  input: ShapeCastInput
): boolean {
  output.iterations = 0
  output.lambda = 1.0
  output.normal.SetZero()
  output.point.SetZero()

  // const DistanceProxy* proxyA = &input.proxyA;
  const proxyA = input.proxyA
  // const DistanceProxy* proxyB = &input.proxyB;
  const proxyB = input.proxyB

  // float32 radiusA = Max(proxyA.m_radius, polygonRadius);
  const radiusA = Max(proxyA.m_radius, polygonRadius)
  // float32 radiusB = Max(proxyB.m_radius, polygonRadius);
  const radiusB = Max(proxyB.m_radius, polygonRadius)
  // float32 radius = radiusA + radiusB;
  const radius = radiusA + radiusB

  // Transform xfA = input.transformA;
  const xfA = input.transformA
  // Transform xfB = input.transformB;
  const xfB = input.transformB

  // Vec2 r = input.translationB;
  const r = input.translationB
  // Vec2 n(0.0f, 0.0f);
  const n = ShapeCast_s_n.Set(0.0, 0.0)
  // float32 lambda = 0.0f;
  let lambda = 0.0

  // Initial simplex
  const simplex = ShapeCast_s_simplex
  simplex.m_count = 0

  // Get simplex vertices as an array.
  // SimplexVertex* vertices = &simplex.m_v1;
  const vertices = simplex.m_vertices

  // Get support point in -r direction
  // int32 indexA = proxyA.GetSupport(MulT(xfA.q, -r));
  let indexA = proxyA.GetSupport(
    Rot.MulTRV(xfA.q, Vec2.NegV(r, Vec2.s_t1), Vec2.s_t0)
  )
  // Vec2 wA = Mul(xfA, proxyA.GetVertex(indexA));
  let wA = Transform.MulXV(xfA, proxyA.GetVertex(indexA), ShapeCast_s_wA)
  // int32 indexB = proxyB.GetSupport(MulT(xfB.q, r));
  let indexB = proxyB.GetSupport(Rot.MulTRV(xfB.q, r, Vec2.s_t0))
  // Vec2 wB = Mul(xfB, proxyB.GetVertex(indexB));
  let wB = Transform.MulXV(xfB, proxyB.GetVertex(indexB), ShapeCast_s_wB)
  // Vec2 v = wA - wB;
  const v = Vec2.SubVV(wA, wB, ShapeCast_s_v)

  // Sigma is the target distance between polygons
  // float32 sigma = Max(polygonRadius, radius - polygonRadius);
  const sigma = Max(polygonRadius, radius - polygonRadius)
  // const float32 tolerance = 0.5f * linearSlop;
  const tolerance = 0.5 * linearSlop

  // Main iteration loop.
  // const int32 k_maxIters = 20;
  const k_maxIters = 20
  // int32 iter = 0;
  let iter = 0
  // while (iter < k_maxIters && Abs(v.Length() - sigma) > tolerance)
  while (iter < k_maxIters && Abs(v.Length() - sigma) > tolerance) {
    // DEBUG: Assert(simplex.m_count < 3);

    output.iterations += 1

    // Support in direction -v (A - B)
    // indexA = proxyA.GetSupport(MulT(xfA.q, -v));
    indexA = proxyA.GetSupport(
      Rot.MulTRV(xfA.q, Vec2.NegV(v, Vec2.s_t1), Vec2.s_t0)
    )
    // wA = Mul(xfA, proxyA.GetVertex(indexA));
    wA = Transform.MulXV(xfA, proxyA.GetVertex(indexA), ShapeCast_s_wA)
    // indexB = proxyB.GetSupport(MulT(xfB.q, v));
    indexB = proxyB.GetSupport(Rot.MulTRV(xfB.q, v, Vec2.s_t0))
    // wB = Mul(xfB, proxyB.GetVertex(indexB));
    wB = Transform.MulXV(xfB, proxyB.GetVertex(indexB), ShapeCast_s_wB)
    // Vec2 p = wA - wB;
    const p = Vec2.SubVV(wA, wB, ShapeCast_s_p)

    // -v is a normal at p
    v.Normalize()

    // Intersect ray with plane
    const vp = Vec2.DotVV(v, p)
    const vr = Vec2.DotVV(v, r)
    if (vp - sigma > lambda * vr) {
      if (vr <= 0.0) {
        return false
      }

      lambda = (vp - sigma) / vr
      if (lambda > 1.0) {
        return false
      }

      // n = -v;
      n.Copy(v).SelfNeg()
      simplex.m_count = 0
    }

    // Reverse simplex since it works with B - A.
    // Shift by lambda * r because we want the closest point to the current clip point.
    // Note that the support point p is not shifted because we want the plane equation
    // to be formed in unshifted space.
    // SimplexVertex* vertex = vertices + simplex.m_count;
    const vertex: SimplexVertex = vertices[simplex.m_count]
    vertex.indexA = indexB
    // vertex.wA = wB + lambda * r;
    vertex.wA.Copy(wB).SelfMulAdd(lambda, r)
    vertex.indexB = indexA
    // vertex.wB = wA;
    vertex.wB.Copy(wA)
    // vertex.w = vertex.wB - vertex.wA;
    vertex.w.Copy(vertex.wB).SelfSub(vertex.wA)
    vertex.a = 1.0
    simplex.m_count += 1

    switch (simplex.m_count) {
      case 1:
        break

      case 2:
        simplex.Solve2()
        break

      case 3:
        simplex.Solve3()
        break

      default:
      // DEBUG: Assert(false);
    }

    // If we have 3 points, then the origin is in the corresponding triangle.
    if (simplex.m_count === 3) {
      // Overlap
      return false
    }

    // Get search direction.
    // v = simplex.GetClosestPoint();
    simplex.GetClosestPoint(v)

    // Iteration count is equated to the number of support point calls.
    ++iter
  }

  // Prepare output.
  const pointA = ShapeCast_s_pointA
  const pointB = ShapeCast_s_pointB
  simplex.GetWitnessPoints(pointA, pointB)

  if (v.LengthSquared() > 0.0) {
    // n = -v;
    n.Copy(v).SelfNeg()
    n.Normalize()
  }

  // output.point = pointA + radiusA * n;
  output.normal.Copy(n)
  output.lambda = lambda
  output.iterations = iter
  return true
}
