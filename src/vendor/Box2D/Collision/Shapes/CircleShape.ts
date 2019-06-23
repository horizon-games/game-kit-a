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

// DEBUG: import { Assert } from "../../Common/Settings";
import { Asin, Pow, Sq, Sqrt, Transform, Vec2, XY } from '../../Common/Math'
import { epsilon, pi } from '../../Common/Settings'
import { AABB, RayCastInput, RayCastOutput } from '../Collision'
import { DistanceProxy } from '../Distance'

import { MassData, Shape, ShapeType } from './Shape'

/// A circle shape.
export class CircleShape extends Shape {
  /// Implement Shape.
  private static TestPoint_s_center = new Vec2()
  private static TestPoint_s_d = new Vec2()

  // #if ENABLE_PARTICLE
  /// @see Shape::ComputeDistance
  private static ComputeDistance_s_center = new Vec2()
  // #endif

  /// Implement Shape.
  // Collision Detection in Interactive 3D Environments by Gino van den Bergen
  // From Section 3.1.2
  // x = s + a * r
  // norm(x) = radius
  private static RayCast_s_position = new Vec2()
  private static RayCast_s_s = new Vec2()
  private static RayCast_s_r = new Vec2()

  /// @see Shape::ComputeAABB
  private static ComputeAABB_s_p = new Vec2()
  readonly m_p: Vec2 = new Vec2()

  constructor(radius: number = 0) {
    super(ShapeType.e_circleShape, radius)
  }

  Set(position: XY, radius: number = this.m_radius): this {
    this.m_p.Copy(position)
    this.m_radius = radius
    return this
  }

  /// Implement Shape.
  Clone(): CircleShape {
    return new CircleShape().Copy(this)
  }

  Copy(other: CircleShape): CircleShape {
    super.Copy(other)

    // DEBUG: Assert(other instanceof CircleShape);

    this.m_p.Copy(other.m_p)
    return this
  }

  /// @see Shape::GetChildCount
  GetChildCount(): number {
    return 1
  }
  TestPoint(transform: Transform, p: XY): boolean {
    const center: Vec2 = Transform.MulXV(
      transform,
      this.m_p,
      CircleShape.TestPoint_s_center
    )
    const d: Vec2 = Vec2.SubVV(p, center, CircleShape.TestPoint_s_d)
    return Vec2.DotVV(d, d) <= Sq(this.m_radius)
  }
  ComputeDistance(
    xf: Transform,
    p: Vec2,
    normal: Vec2,
    childIndex: number
  ): number {
    const center = Transform.MulXV(
      xf,
      this.m_p,
      CircleShape.ComputeDistance_s_center
    )
    Vec2.SubVV(p, center, normal)
    return normal.Normalize() - this.m_radius
  }
  RayCast(
    output: RayCastOutput,
    input: RayCastInput,
    transform: Transform,
    childIndex: number
  ): boolean {
    const position: Vec2 = Transform.MulXV(
      transform,
      this.m_p,
      CircleShape.RayCast_s_position
    )
    const s: Vec2 = Vec2.SubVV(input.p1, position, CircleShape.RayCast_s_s)
    const b: number = Vec2.DotVV(s, s) - Sq(this.m_radius)

    // Solve quadratic equation.
    const r: Vec2 = Vec2.SubVV(input.p2, input.p1, CircleShape.RayCast_s_r)
    const c: number = Vec2.DotVV(s, r)
    const rr: number = Vec2.DotVV(r, r)
    const sigma = c * c - rr * b

    // Check for negative discriminant and short segment.
    if (sigma < 0 || rr < epsilon) {
      return false
    }

    // Find the point of intersection of the line with the circle.
    let a: number = -(c + Sqrt(sigma))

    // Is the intersection point on the segment?
    if (0 <= a && a <= input.maxFraction * rr) {
      a /= rr
      output.fraction = a
      Vec2.AddVMulSV(s, a, r, output.normal).SelfNormalize()
      return true
    }

    return false
  }
  ComputeAABB(aabb: AABB, transform: Transform, childIndex: number): void {
    const p: Vec2 = Transform.MulXV(
      transform,
      this.m_p,
      CircleShape.ComputeAABB_s_p
    )
    aabb.lowerBound.Set(p.x - this.m_radius, p.y - this.m_radius)
    aabb.upperBound.Set(p.x + this.m_radius, p.y + this.m_radius)
  }

  /// @see Shape::ComputeMass
  ComputeMass(massData: MassData, density: number): void {
    const radius_sq: number = Sq(this.m_radius)
    massData.mass = density * pi * radius_sq
    massData.center.Copy(this.m_p)

    // inertia about the local origin
    massData.I =
      massData.mass * (0.5 * radius_sq + Vec2.DotVV(this.m_p, this.m_p))
  }

  SetupDistanceProxy(proxy: DistanceProxy, index: number): void {
    proxy.m_vertices = proxy.m_buffer
    proxy.m_vertices[0].Copy(this.m_p)
    proxy.m_count = 1
    proxy.m_radius = this.m_radius
  }

  ComputeSubmergedArea(
    normal: Vec2,
    offset: number,
    xf: Transform,
    c: Vec2
  ): number {
    const p: Vec2 = Transform.MulXV(xf, this.m_p, new Vec2())
    const l: number = -(Vec2.DotVV(normal, p) - offset)

    if (l < -this.m_radius + epsilon) {
      // Completely dry
      return 0
    }
    if (l > this.m_radius) {
      // Completely wet
      c.Copy(p)
      return pi * this.m_radius * this.m_radius
    }

    // Magic
    const r2: number = this.m_radius * this.m_radius
    const l2: number = l * l
    const area: number =
      r2 * (Asin(l / this.m_radius) + pi / 2) + l * Sqrt(r2 - l2)
    const com: number = ((-2 / 3) * Pow(r2 - l2, 1.5)) / area

    c.x = p.x + normal.x * com
    c.y = p.y + normal.y * com

    return area
  }

  Dump(log: (format: string, ...args: any[]) => void): void {
    log('    const shape: CircleShape = new CircleShape();\n')
    log('    shape.m_radius = %.15f;\n', this.m_radius)
    log('    shape.m_p.Set(%.15f, %.15f);\n', this.m_p.x, this.m_p.y)
  }
}
