/*
 * Copyright (c) 2011 Erin Catto http://www.box2d.org
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
import { Color, Draw } from '../Common/Draw'
import { Atan2, Vec2 } from '../Common/Math'
import { MakeNumberArray, pi } from '../Common/Settings'

///
export class RopeDef {
  ///
  vertices: Vec2[] = []

  ///
  count: number = 0

  ///
  masses: number[] = []

  ///
  readonly gravity: Vec2 = new Vec2(0, 0)

  ///
  damping: number = 0.1

  /// Stretching stiffness
  k2: number = 0.9

  /// Bending stiffness. Values above 0.5 can make the simulation blow up.
  k3: number = 0.1
}

///
export class Rope {
  ///
  private static s_d = new Vec2()

  private static s_d1 = new Vec2()
  private static s_d2 = new Vec2()
  private static s_Jd1 = new Vec2()
  private static s_Jd2 = new Vec2()
  private static s_J1 = new Vec2()
  private static s_J2 = new Vec2()
  m_count: number = 0
  m_ps: Vec2[] = []
  m_p0s: Vec2[] = []
  m_vs: Vec2[] = []

  m_ims: number[] = []

  m_Ls: number[] = []
  m_as: number[] = []

  readonly m_gravity: Vec2 = new Vec2()
  m_damping: number = 0

  m_k2: number = 1
  m_k3: number = 0.1

  GetVertexCount(): number {
    return this.m_count
  }

  GetVertices(): Vec2[] {
    return this.m_ps
  }

  ///
  Initialize(def: RopeDef): void {
    // DEBUG: Assert(def.count >= 3);
    this.m_count = def.count
    // this.m_ps = (Vec2*)Alloc(this.m_count * sizeof(Vec2));
    this.m_ps = Vec2.MakeArray(this.m_count)
    // this.m_p0s = (Vec2*)Alloc(this.m_count * sizeof(Vec2));
    this.m_p0s = Vec2.MakeArray(this.m_count)
    // this.m_vs = (Vec2*)Alloc(this.m_count * sizeof(Vec2));
    this.m_vs = Vec2.MakeArray(this.m_count)
    // this.m_ims = (float32*)Alloc(this.m_count * sizeof(float32));
    this.m_ims = MakeNumberArray(this.m_count)

    for (let i: number = 0; i < this.m_count; ++i) {
      this.m_ps[i].Copy(def.vertices[i])
      this.m_p0s[i].Copy(def.vertices[i])
      this.m_vs[i].SetZero()

      const m: number = def.masses[i]
      this.m_ims[i] = m > 0 ? 1 / m : 0
    }

    const count2: number = this.m_count - 1
    const count3: number = this.m_count - 2
    // this.m_Ls = (float32*)be2Alloc(count2 * sizeof(float32));
    this.m_Ls = MakeNumberArray(count2)
    // this.m_as = (float32*)Alloc(count3 * sizeof(float32));
    this.m_as = MakeNumberArray(count3)

    for (let i: number = 0; i < count2; ++i) {
      const p1: Vec2 = this.m_ps[i]
      const p2: Vec2 = this.m_ps[i + 1]
      this.m_Ls[i] = Vec2.DistanceVV(p1, p2)
    }

    for (let i: number = 0; i < count3; ++i) {
      const p1: Vec2 = this.m_ps[i]
      const p2: Vec2 = this.m_ps[i + 1]
      const p3: Vec2 = this.m_ps[i + 2]

      const d1: Vec2 = Vec2.SubVV(p2, p1, Vec2.s_t0)
      const d2: Vec2 = Vec2.SubVV(p3, p2, Vec2.s_t1)

      const a: number = Vec2.CrossVV(d1, d2)
      const b: number = Vec2.DotVV(d1, d2)

      this.m_as[i] = Atan2(a, b)
    }

    this.m_gravity.Copy(def.gravity)
    this.m_damping = def.damping
    this.m_k2 = def.k2
    this.m_k3 = def.k3
  }

  ///
  Step(h: number, iterations: number): void {
    if (h === 0) {
      return
    }

    const d: number = Math.exp(-h * this.m_damping)

    for (let i: number = 0; i < this.m_count; ++i) {
      this.m_p0s[i].Copy(this.m_ps[i])
      if (this.m_ims[i] > 0) {
        this.m_vs[i].SelfMulAdd(h, this.m_gravity)
      }
      this.m_vs[i].SelfMul(d)
      this.m_ps[i].SelfMulAdd(h, this.m_vs[i])
    }

    for (let i: number = 0; i < iterations; ++i) {
      this.SolveC2()
      this.SolveC3()
      this.SolveC2()
    }

    const inv_h: number = 1 / h
    for (let i: number = 0; i < this.m_count; ++i) {
      Vec2.MulSV(
        inv_h,
        Vec2.SubVV(this.m_ps[i], this.m_p0s[i], Vec2.s_t0),
        this.m_vs[i]
      )
    }
  }
  SolveC2(): void {
    const count2: number = this.m_count - 1

    for (let i: number = 0; i < count2; ++i) {
      const p1: Vec2 = this.m_ps[i]
      const p2: Vec2 = this.m_ps[i + 1]

      const d: Vec2 = Vec2.SubVV(p2, p1, Rope.s_d)
      const L: number = d.Normalize()

      const im1: number = this.m_ims[i]
      const im2: number = this.m_ims[i + 1]

      if (im1 + im2 === 0) {
        continue
      }

      const s1: number = im1 / (im1 + im2)
      const s2: number = im2 / (im1 + im2)

      p1.SelfMulSub(this.m_k2 * s1 * (this.m_Ls[i] - L), d)
      p2.SelfMulAdd(this.m_k2 * s2 * (this.m_Ls[i] - L), d)

      // this.m_ps[i] = p1;
      // this.m_ps[i + 1] = p2;
    }
  }

  SetAngle(angle: number): void {
    const count3: number = this.m_count - 2
    for (let i: number = 0; i < count3; ++i) {
      this.m_as[i] = angle
    }
  }
  SolveC3(): void {
    const count3: number = this.m_count - 2

    for (let i: number = 0; i < count3; ++i) {
      const p1: Vec2 = this.m_ps[i]
      const p2: Vec2 = this.m_ps[i + 1]
      const p3: Vec2 = this.m_ps[i + 2]

      const m1: number = this.m_ims[i]
      const m2: number = this.m_ims[i + 1]
      const m3: number = this.m_ims[i + 2]

      const d1: Vec2 = Vec2.SubVV(p2, p1, Rope.s_d1)
      const d2: Vec2 = Vec2.SubVV(p3, p2, Rope.s_d2)

      const L1sqr: number = d1.LengthSquared()
      const L2sqr: number = d2.LengthSquared()

      if (L1sqr * L2sqr === 0) {
        continue
      }

      const a: number = Vec2.CrossVV(d1, d2)
      const b: number = Vec2.DotVV(d1, d2)

      let angle: number = Atan2(a, b)

      const Jd1: Vec2 = Vec2.MulSV(-1 / L1sqr, d1.SelfSkew(), Rope.s_Jd1)
      const Jd2: Vec2 = Vec2.MulSV(1 / L2sqr, d2.SelfSkew(), Rope.s_Jd2)

      const J1: Vec2 = Vec2.NegV(Jd1, Rope.s_J1)
      const J2: Vec2 = Vec2.SubVV(Jd1, Jd2, Rope.s_J2)
      const J3: Vec2 = Jd2

      let mass: number =
        m1 * Vec2.DotVV(J1, J1) +
        m2 * Vec2.DotVV(J2, J2) +
        m3 * Vec2.DotVV(J3, J3)
      if (mass === 0) {
        continue
      }

      mass = 1 / mass

      let C: number = angle - this.m_as[i]

      while (C > pi) {
        angle -= 2 * pi
        C = angle - this.m_as[i]
      }

      while (C < -pi) {
        angle += 2 * pi
        C = angle - this.m_as[i]
      }

      const impulse: number = -this.m_k3 * mass * C

      p1.SelfMulAdd(m1 * impulse, J1)
      p2.SelfMulAdd(m2 * impulse, J2)
      p3.SelfMulAdd(m3 * impulse, J3)

      // this.m_ps[i] = p1;
      // this.m_ps[i + 1] = p2;
      // this.m_ps[i + 2] = p3;
    }
  }

  Draw(draw: Draw): void {
    const c: Color = new Color(0.4, 0.5, 0.7)

    for (let i: number = 0; i < this.m_count - 1; ++i) {
      draw.DrawSegment(this.m_ps[i], this.m_ps[i + 1], c)
    }
  }
}
