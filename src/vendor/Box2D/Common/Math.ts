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

// DEBUG: import { Assert } from "./Settings";
import { epsilon, MakeArray, pi } from './Settings'

export const degToRad: number = pi / 180
export const radToDeg: number = 180 / pi
export const two_pi: number = 2 * pi

export const Abs = Math.abs

export const Min = Math.min
export const Max = Math.max

export function Clamp(a: number, lo: number, hi: number): number {
  return a < lo ? lo : a > hi ? hi : a
}

export function Swap<T>(a: T[], b: T[]): void {
  // DEBUG: Assert(false);
  const tmp: T = a[0]
  a[0] = b[0]
  b[0] = tmp
}

/// This function is used to ensure that a floating point number is
/// not a NaN or infinity.
export const IsValid = isFinite

export function Sq(n: number): number {
  return n * n
}

/// This is a approximate yet fast inverse square-root.
export function InvSqrt(n: number): number {
  return 1 / Math.sqrt(n)
}

export const Sqrt = Math.sqrt

export const Pow = Math.pow

export function DegToRad(degrees: number): number {
  return degrees * degToRad
}

export function RadToDeg(radians: number): number {
  return radians * radToDeg
}

export const Cos = Math.cos
export const Sin = Math.sin
export const Acos = Math.acos
export const Asin = Math.asin
export const Atan2 = Math.atan2

export function NextPowerOfTwo(x: number): number {
  x |= (x >> 1) & 0x7fffffff
  x |= (x >> 2) & 0x3fffffff
  x |= (x >> 4) & 0x0fffffff
  x |= (x >> 8) & 0x00ffffff
  x |= (x >> 16) & 0x0000ffff
  return x + 1
}

export function IsPowerOfTwo(x: number): boolean {
  return x > 0 && (x & (x - 1)) === 0
}

export function Random(): number {
  return Math.random() * 2 - 1
}

export function RandomRange(lo: number, hi: number): number {
  return (hi - lo) * Math.random() + lo
}

export interface XY {
  x: number
  y: number
}

/// A 2D column vector.
export class Vec2 implements XY {
  static readonly ZERO: Readonly<Vec2> = new Vec2(0, 0)
  static readonly UNITX: Readonly<Vec2> = new Vec2(1, 0)
  static readonly UNITY: Readonly<Vec2> = new Vec2(0, 1)

  static readonly s_t0: Vec2 = new Vec2()
  static readonly s_t1: Vec2 = new Vec2()
  static readonly s_t2: Vec2 = new Vec2()
  static readonly s_t3: Vec2 = new Vec2()

  static MakeArray(length: number): Vec2[] {
    return MakeArray(length, (i: number): Vec2 => new Vec2())
  }

  static AbsV<T extends XY>(v: XY, out: T): T {
    out.x = Abs(v.x)
    out.y = Abs(v.y)
    return out
  }

  static MinV<T extends XY>(a: XY, b: XY, out: T): T {
    out.x = Min(a.x, b.x)
    out.y = Min(a.y, b.y)
    return out
  }

  static MaxV<T extends XY>(a: XY, b: XY, out: T): T {
    out.x = Max(a.x, b.x)
    out.y = Max(a.y, b.y)
    return out
  }

  static ClampV<T extends XY>(v: XY, lo: XY, hi: XY, out: T): T {
    out.x = Clamp(v.x, lo.x, hi.x)
    out.y = Clamp(v.y, lo.y, hi.y)
    return out
  }

  static RotateV<T extends XY>(v: XY, radians: number, out: T): T {
    const v_x: number = v.x
    const v_y: number = v.y
    const c: number = Math.cos(radians)
    const s: number = Math.sin(radians)
    out.x = c * v_x - s * v_y
    out.y = s * v_x + c * v_y
    return out
  }

  static DotVV(a: XY, b: XY): number {
    return a.x * b.x + a.y * b.y
  }

  static CrossVV(a: XY, b: XY): number {
    return a.x * b.y - a.y * b.x
  }

  static CrossVS<T extends XY>(v: XY, s: number, out: T): T {
    const v_x: number = v.x
    out.x = s * v.y
    out.y = -s * v_x
    return out
  }

  static CrossVOne<T extends XY>(v: XY, out: T): T {
    const v_x: number = v.x
    out.x = v.y
    out.y = -v_x
    return out
  }

  static CrossSV<T extends XY>(s: number, v: XY, out: T): T {
    const v_x: number = v.x
    out.x = -s * v.y
    out.y = s * v_x
    return out
  }

  static CrossOneV<T extends XY>(v: XY, out: T): T {
    const v_x: number = v.x
    out.x = -v.y
    out.y = v_x
    return out
  }

  static AddVV<T extends XY>(a: XY, b: XY, out: T): T {
    out.x = a.x + b.x
    out.y = a.y + b.y
    return out
  }

  static SubVV<T extends XY>(a: XY, b: XY, out: T): T {
    out.x = a.x - b.x
    out.y = a.y - b.y
    return out
  }

  static MulSV<T extends XY>(s: number, v: XY, out: T): T {
    out.x = v.x * s
    out.y = v.y * s
    return out
  }
  static MulVS<T extends XY>(v: XY, s: number, out: T): T {
    out.x = v.x * s
    out.y = v.y * s
    return out
  }

  static AddVMulSV<T extends XY>(a: XY, s: number, b: XY, out: T): T {
    out.x = a.x + s * b.x
    out.y = a.y + s * b.y
    return out
  }
  static SubVMulSV<T extends XY>(a: XY, s: number, b: XY, out: T): T {
    out.x = a.x - s * b.x
    out.y = a.y - s * b.y
    return out
  }

  static AddVCrossSV<T extends XY>(a: XY, s: number, v: XY, out: T): T {
    const v_x: number = v.x
    out.x = a.x - s * v.y
    out.y = a.y + s * v_x
    return out
  }

  static MidVV<T extends XY>(a: XY, b: XY, out: T): T {
    out.x = (a.x + b.x) * 0.5
    out.y = (a.y + b.y) * 0.5
    return out
  }

  static ExtVV<T extends XY>(a: XY, b: XY, out: T): T {
    out.x = (b.x - a.x) * 0.5
    out.y = (b.y - a.y) * 0.5
    return out
  }

  static IsEqualToV(a: XY, b: XY): boolean {
    return a.x === b.x && a.y === b.y
  }

  static DistanceVV(a: XY, b: XY): number {
    const c_x: number = a.x - b.x
    const c_y: number = a.y - b.y
    return Math.sqrt(c_x * c_x + c_y * c_y)
  }

  static DistanceSquaredVV(a: XY, b: XY): number {
    const c_x: number = a.x - b.x
    const c_y: number = a.y - b.y
    return c_x * c_x + c_y * c_y
  }

  static NegV<T extends XY>(v: XY, out: T): T {
    out.x = -v.x
    out.y = -v.y
    return out
  }

  x: number
  y: number

  constructor(x: number = 0, y: number = 0) {
    this.x = x
    this.y = y
  }

  Clone(): Vec2 {
    return new Vec2(this.x, this.y)
  }

  SetZero(): this {
    this.x = 0
    this.y = 0
    return this
  }

  Set(x: number, y: number): this {
    this.x = x
    this.y = y
    return this
  }

  Copy(other: XY): this {
    this.x = other.x
    this.y = other.y
    return this
  }

  SelfAdd(v: XY): this {
    this.x += v.x
    this.y += v.y
    return this
  }

  SelfAddXY(x: number, y: number): this {
    this.x += x
    this.y += y
    return this
  }

  SelfSub(v: XY): this {
    this.x -= v.x
    this.y -= v.y
    return this
  }

  SelfSubXY(x: number, y: number): this {
    this.x -= x
    this.y -= y
    return this
  }

  SelfMul(s: number): this {
    this.x *= s
    this.y *= s
    return this
  }

  SelfMulAdd(s: number, v: XY): this {
    this.x += s * v.x
    this.y += s * v.y
    return this
  }

  SelfMulSub(s: number, v: XY): this {
    this.x -= s * v.x
    this.y -= s * v.y
    return this
  }

  Dot(v: XY): number {
    return this.x * v.x + this.y * v.y
  }

  Cross(v: XY): number {
    return this.x * v.y - this.y * v.x
  }

  Length(): number {
    const x: number = this.x
    const y: number = this.y
    return Math.sqrt(x * x + y * y)
  }

  LengthSquared(): number {
    const x: number = this.x
    const y: number = this.y
    return x * x + y * y
  }

  Normalize(): number {
    const length: number = this.Length()
    if (length >= epsilon) {
      const inv_length: number = 1 / length
      this.x *= inv_length
      this.y *= inv_length
    }
    return length
  }

  SelfNormalize(): this {
    const length: number = this.Length()
    if (length >= epsilon) {
      const inv_length: number = 1 / length
      this.x *= inv_length
      this.y *= inv_length
    }
    return this
  }

  SelfRotate(radians: number): this {
    const c: number = Math.cos(radians)
    const s: number = Math.sin(radians)
    const x: number = this.x
    this.x = c * x - s * this.y
    this.y = s * x + c * this.y
    return this
  }

  SelfRotateCosSin(c: number, s: number): this {
    const x: number = this.x
    this.x = c * x - s * this.y
    this.y = s * x + c * this.y
    return this
  }

  IsValid(): boolean {
    return isFinite(this.x) && isFinite(this.y)
  }

  SelfCrossVS(s: number): this {
    const x: number = this.x
    this.x = s * this.y
    this.y = -s * x
    return this
  }

  SelfCrossSV(s: number): this {
    const x: number = this.x
    this.x = -s * this.y
    this.y = s * x
    return this
  }

  SelfMinV(v: XY): this {
    this.x = Min(this.x, v.x)
    this.y = Min(this.y, v.y)
    return this
  }

  SelfMaxV(v: XY): this {
    this.x = Max(this.x, v.x)
    this.y = Max(this.y, v.y)
    return this
  }

  SelfAbs(): this {
    this.x = Abs(this.x)
    this.y = Abs(this.y)
    return this
  }

  SelfNeg(): this {
    this.x = -this.x
    this.y = -this.y
    return this
  }

  SelfSkew(): this {
    const x: number = this.x
    this.x = -this.y
    this.y = x
    return this
  }
}

export const Vec2_zero: Readonly<Vec2> = new Vec2(0, 0)

export interface XYZ extends XY {
  z: number
}

/// A 2D column vector with 3 elements.
export class Vec3 implements XYZ {
  static readonly ZERO: Readonly<Vec3> = new Vec3(0, 0, 0)

  static readonly s_t0: Vec3 = new Vec3()

  static DotV3V3(a: XYZ, b: XYZ): number {
    return a.x * b.x + a.y * b.y + a.z * b.z
  }

  static CrossV3V3<T extends XYZ>(a: XYZ, b: XYZ, out: T): T {
    const a_x: number = a.x
    const a_y = a.y
    const a_z = a.z
    const b_x: number = b.x
    const b_y = b.y
    const b_z = b.z
    out.x = a_y * b_z - a_z * b_y
    out.y = a_z * b_x - a_x * b_z
    out.z = a_x * b_y - a_y * b_x
    return out
  }

  x: number
  y: number
  z: number

  constructor(x: number = 0, y: number = 0, z: number = 0) {
    this.x = x
    this.y = y
    this.z = z
  }

  Clone(): Vec3 {
    return new Vec3(this.x, this.y, this.z)
  }

  SetZero(): this {
    this.x = 0
    this.y = 0
    this.z = 0
    return this
  }

  SetXYZ(x: number, y: number, z: number): this {
    this.x = x
    this.y = y
    this.z = z
    return this
  }

  Copy(other: XYZ): this {
    this.x = other.x
    this.y = other.y
    this.z = other.z
    return this
  }

  SelfNeg(): this {
    this.x = -this.x
    this.y = -this.y
    this.z = -this.z
    return this
  }

  SelfAdd(v: XYZ): this {
    this.x += v.x
    this.y += v.y
    this.z += v.z
    return this
  }

  SelfAddXYZ(x: number, y: number, z: number): this {
    this.x += x
    this.y += y
    this.z += z
    return this
  }

  SelfSub(v: XYZ): this {
    this.x -= v.x
    this.y -= v.y
    this.z -= v.z
    return this
  }

  SelfSubXYZ(x: number, y: number, z: number): this {
    this.x -= x
    this.y -= y
    this.z -= z
    return this
  }

  SelfMul(s: number): this {
    this.x *= s
    this.y *= s
    this.z *= s
    return this
  }
}

/// A 2-by-2 matrix. Stored in column-major order.
export class Mat22 {
  static readonly IDENTITY: Readonly<Mat22> = new Mat22()

  static FromVV(c1: XY, c2: XY): Mat22 {
    return new Mat22().SetVV(c1, c2)
  }

  static FromSSSS(
    r1c1: number,
    r1c2: number,
    r2c1: number,
    r2c2: number
  ): Mat22 {
    return new Mat22().SetSSSS(r1c1, r1c2, r2c1, r2c2)
  }

  static FromAngle(radians: number): Mat22 {
    return new Mat22().SetAngle(radians)
  }

  static AbsM(M: Mat22, out: Mat22): Mat22 {
    const M_ex: Vec2 = M.ex
    const M_ey: Vec2 = M.ey
    out.ex.x = Abs(M_ex.x)
    out.ex.y = Abs(M_ex.y)
    out.ey.x = Abs(M_ey.x)
    out.ey.y = Abs(M_ey.y)
    return out
  }

  static MulMV<T extends XY>(M: Mat22, v: XY, out: T): T {
    const M_ex: Vec2 = M.ex
    const M_ey: Vec2 = M.ey
    const v_x: number = v.x
    const v_y: number = v.y
    out.x = M_ex.x * v_x + M_ey.x * v_y
    out.y = M_ex.y * v_x + M_ey.y * v_y
    return out
  }

  static MulTMV<T extends XY>(M: Mat22, v: XY, out: T): T {
    const M_ex: Vec2 = M.ex
    const M_ey: Vec2 = M.ey
    const v_x: number = v.x
    const v_y: number = v.y
    out.x = M_ex.x * v_x + M_ex.y * v_y
    out.y = M_ey.x * v_x + M_ey.y * v_y
    return out
  }

  static AddMM(A: Mat22, B: Mat22, out: Mat22): Mat22 {
    const A_ex: Vec2 = A.ex
    const A_ey: Vec2 = A.ey
    const B_ex: Vec2 = B.ex
    const B_ey: Vec2 = B.ey
    out.ex.x = A_ex.x + B_ex.x
    out.ex.y = A_ex.y + B_ex.y
    out.ey.x = A_ey.x + B_ey.x
    out.ey.y = A_ey.y + B_ey.y
    return out
  }

  static MulMM(A: Mat22, B: Mat22, out: Mat22): Mat22 {
    const A_ex_x: number = A.ex.x
    const A_ex_y: number = A.ex.y
    const A_ey_x: number = A.ey.x
    const A_ey_y: number = A.ey.y
    const B_ex_x: number = B.ex.x
    const B_ex_y: number = B.ex.y
    const B_ey_x: number = B.ey.x
    const B_ey_y: number = B.ey.y
    out.ex.x = A_ex_x * B_ex_x + A_ey_x * B_ex_y
    out.ex.y = A_ex_y * B_ex_x + A_ey_y * B_ex_y
    out.ey.x = A_ex_x * B_ey_x + A_ey_x * B_ey_y
    out.ey.y = A_ex_y * B_ey_x + A_ey_y * B_ey_y
    return out
  }

  static MulTMM(A: Mat22, B: Mat22, out: Mat22): Mat22 {
    const A_ex_x: number = A.ex.x
    const A_ex_y: number = A.ex.y
    const A_ey_x: number = A.ey.x
    const A_ey_y: number = A.ey.y
    const B_ex_x: number = B.ex.x
    const B_ex_y: number = B.ex.y
    const B_ey_x: number = B.ey.x
    const B_ey_y: number = B.ey.y
    out.ex.x = A_ex_x * B_ex_x + A_ex_y * B_ex_y
    out.ex.y = A_ey_x * B_ex_x + A_ey_y * B_ex_y
    out.ey.x = A_ex_x * B_ey_x + A_ex_y * B_ey_y
    out.ey.y = A_ey_x * B_ey_x + A_ey_y * B_ey_y
    return out
  }

  readonly ex: Vec2 = new Vec2(1, 0)
  readonly ey: Vec2 = new Vec2(0, 1)

  Clone(): Mat22 {
    return new Mat22().Copy(this)
  }

  SetSSSS(r1c1: number, r1c2: number, r2c1: number, r2c2: number): this {
    this.ex.Set(r1c1, r2c1)
    this.ey.Set(r1c2, r2c2)
    return this
  }

  SetVV(c1: XY, c2: XY): this {
    this.ex.Copy(c1)
    this.ey.Copy(c2)
    return this
  }

  SetAngle(radians: number): this {
    const c: number = Math.cos(radians)
    const s: number = Math.sin(radians)
    this.ex.Set(c, s)
    this.ey.Set(-s, c)
    return this
  }

  Copy(other: Mat22): this {
    this.ex.Copy(other.ex)
    this.ey.Copy(other.ey)
    return this
  }

  SetIdentity(): this {
    this.ex.Set(1, 0)
    this.ey.Set(0, 1)
    return this
  }

  SetZero(): this {
    this.ex.SetZero()
    this.ey.SetZero()
    return this
  }

  GetAngle(): number {
    return Math.atan2(this.ex.y, this.ex.x)
  }

  GetInverse(out: Mat22): Mat22 {
    const a: number = this.ex.x
    const b: number = this.ey.x
    const c: number = this.ex.y
    const d: number = this.ey.y
    let det: number = a * d - b * c
    if (det !== 0) {
      det = 1 / det
    }
    out.ex.x = det * d
    out.ey.x = -det * b
    out.ex.y = -det * c
    out.ey.y = det * a
    return out
  }

  Solve<T extends XY>(b_x: number, b_y: number, out: T): T {
    const a11: number = this.ex.x
    const a12 = this.ey.x
    const a21: number = this.ex.y
    const a22 = this.ey.y
    let det: number = a11 * a22 - a12 * a21
    if (det !== 0) {
      det = 1 / det
    }
    out.x = det * (a22 * b_x - a12 * b_y)
    out.y = det * (a11 * b_y - a21 * b_x)
    return out
  }

  SelfAbs(): this {
    this.ex.SelfAbs()
    this.ey.SelfAbs()
    return this
  }

  SelfInv(): this {
    this.GetInverse(this)
    return this
  }

  SelfAddM(M: Mat22): this {
    this.ex.SelfAdd(M.ex)
    this.ey.SelfAdd(M.ey)
    return this
  }

  SelfSubM(M: Mat22): this {
    this.ex.SelfSub(M.ex)
    this.ey.SelfSub(M.ey)
    return this
  }
}

/// A 3-by-3 matrix. Stored in column-major order.
export class Mat33 {
  static readonly IDENTITY: Readonly<Mat33> = new Mat33()

  static MulM33V3<T extends XYZ>(A: Mat33, v: XYZ, out: T): T {
    const v_x: number = v.x
    const v_y: number = v.y
    const v_z: number = v.z
    out.x = A.ex.x * v_x + A.ey.x * v_y + A.ez.x * v_z
    out.y = A.ex.y * v_x + A.ey.y * v_y + A.ez.y * v_z
    out.z = A.ex.z * v_x + A.ey.z * v_y + A.ez.z * v_z
    return out
  }
  static MulM33XYZ<T extends XYZ>(
    A: Mat33,
    x: number,
    y: number,
    z: number,
    out: T
  ): T {
    out.x = A.ex.x * x + A.ey.x * y + A.ez.x * z
    out.y = A.ex.y * x + A.ey.y * y + A.ez.y * z
    out.z = A.ex.z * x + A.ey.z * y + A.ez.z * z
    return out
  }
  static MulM33V2<T extends XY>(A: Mat33, v: XY, out: T): T {
    const v_x: number = v.x
    const v_y: number = v.y
    out.x = A.ex.x * v_x + A.ey.x * v_y
    out.y = A.ex.y * v_x + A.ey.y * v_y
    return out
  }
  static MulM33XY<T extends XY>(A: Mat33, x: number, y: number, out: T): T {
    out.x = A.ex.x * x + A.ey.x * y
    out.y = A.ex.y * x + A.ey.y * y
    return out
  }

  readonly ex: Vec3 = new Vec3(1, 0, 0)
  readonly ey: Vec3 = new Vec3(0, 1, 0)
  readonly ez: Vec3 = new Vec3(0, 0, 1)

  Clone(): Mat33 {
    return new Mat33().Copy(this)
  }

  SetVVV(c1: XYZ, c2: XYZ, c3: XYZ): this {
    this.ex.Copy(c1)
    this.ey.Copy(c2)
    this.ez.Copy(c3)
    return this
  }

  Copy(other: Mat33): this {
    this.ex.Copy(other.ex)
    this.ey.Copy(other.ey)
    this.ez.Copy(other.ez)
    return this
  }

  SetIdentity(): this {
    this.ex.SetXYZ(1, 0, 0)
    this.ey.SetXYZ(0, 1, 0)
    this.ez.SetXYZ(0, 0, 1)
    return this
  }

  SetZero(): this {
    this.ex.SetZero()
    this.ey.SetZero()
    this.ez.SetZero()
    return this
  }

  SelfAddM(M: Mat33): this {
    this.ex.SelfAdd(M.ex)
    this.ey.SelfAdd(M.ey)
    this.ez.SelfAdd(M.ez)
    return this
  }

  Solve33<T extends XYZ>(b_x: number, b_y: number, b_z: number, out: T): T {
    const a11: number = this.ex.x
    const a21: number = this.ex.y
    const a31: number = this.ex.z
    const a12: number = this.ey.x
    const a22: number = this.ey.y
    const a32: number = this.ey.z
    const a13: number = this.ez.x
    const a23: number = this.ez.y
    const a33: number = this.ez.z
    let det: number =
      a11 * (a22 * a33 - a32 * a23) +
      a21 * (a32 * a13 - a12 * a33) +
      a31 * (a12 * a23 - a22 * a13)
    if (det !== 0) {
      det = 1 / det
    }
    out.x =
      det *
      (b_x * (a22 * a33 - a32 * a23) +
        b_y * (a32 * a13 - a12 * a33) +
        b_z * (a12 * a23 - a22 * a13))
    out.y =
      det *
      (a11 * (b_y * a33 - b_z * a23) +
        a21 * (b_z * a13 - b_x * a33) +
        a31 * (b_x * a23 - b_y * a13))
    out.z =
      det *
      (a11 * (a22 * b_z - a32 * b_y) +
        a21 * (a32 * b_x - a12 * b_z) +
        a31 * (a12 * b_y - a22 * b_x))
    return out
  }

  Solve22<T extends XY>(b_x: number, b_y: number, out: T): T {
    const a11: number = this.ex.x
    const a12: number = this.ey.x
    const a21: number = this.ex.y
    const a22: number = this.ey.y
    let det: number = a11 * a22 - a12 * a21
    if (det !== 0) {
      det = 1 / det
    }
    out.x = det * (a22 * b_x - a12 * b_y)
    out.y = det * (a11 * b_y - a21 * b_x)
    return out
  }

  GetInverse22(M: Mat33): void {
    const a: number = this.ex.x
    const b: number = this.ey.x
    const c: number = this.ex.y
    const d: number = this.ey.y
    let det: number = a * d - b * c
    if (det !== 0) {
      det = 1 / det
    }

    M.ex.x = det * d
    M.ey.x = -det * b
    M.ex.z = 0
    M.ex.y = -det * c
    M.ey.y = det * a
    M.ey.z = 0
    M.ez.x = 0
    M.ez.y = 0
    M.ez.z = 0
  }

  GetSymInverse33(M: Mat33): void {
    let det: number = Vec3.DotV3V3(
      this.ex,
      Vec3.CrossV3V3(this.ey, this.ez, Vec3.s_t0)
    )
    if (det !== 0) {
      det = 1 / det
    }

    const a11: number = this.ex.x
    const a12: number = this.ey.x
    const a13: number = this.ez.x
    const a22: number = this.ey.y
    const a23: number = this.ez.y
    const a33: number = this.ez.z

    M.ex.x = det * (a22 * a33 - a23 * a23)
    M.ex.y = det * (a13 * a23 - a12 * a33)
    M.ex.z = det * (a12 * a23 - a13 * a22)

    M.ey.x = M.ex.y
    M.ey.y = det * (a11 * a33 - a13 * a13)
    M.ey.z = det * (a13 * a12 - a11 * a23)

    M.ez.x = M.ex.z
    M.ez.y = M.ey.z
    M.ez.z = det * (a11 * a22 - a12 * a12)
  }
}

/// Rotation
export class Rot {
  static readonly IDENTITY: Readonly<Rot> = new Rot()

  static MulRR(q: Rot, r: Rot, out: Rot): Rot {
    // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
    // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
    // s = qs * rc + qc * rs
    // c = qc * rc - qs * rs
    const q_c: number = q.c
    const q_s: number = q.s
    const r_c: number = r.c
    const r_s: number = r.s
    out.s = q_s * r_c + q_c * r_s
    out.c = q_c * r_c - q_s * r_s
    return out
  }

  static MulTRR(q: Rot, r: Rot, out: Rot): Rot {
    // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
    // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
    // s = qc * rs - qs * rc
    // c = qc * rc + qs * rs
    const q_c: number = q.c
    const q_s: number = q.s
    const r_c: number = r.c
    const r_s: number = r.s
    out.s = q_c * r_s - q_s * r_c
    out.c = q_c * r_c + q_s * r_s
    return out
  }

  static MulRV<T extends XY>(q: Rot, v: XY, out: T): T {
    const q_c: number = q.c
    const q_s: number = q.s
    const v_x: number = v.x
    const v_y: number = v.y
    out.x = q_c * v_x - q_s * v_y
    out.y = q_s * v_x + q_c * v_y
    return out
  }

  static MulTRV<T extends XY>(q: Rot, v: XY, out: T): T {
    const q_c: number = q.c
    const q_s: number = q.s
    const v_x: number = v.x
    const v_y: number = v.y
    out.x = q_c * v_x + q_s * v_y
    out.y = -q_s * v_x + q_c * v_y
    return out
  }

  s: number = 0
  c: number = 1

  constructor(angle: number = 0) {
    if (angle) {
      this.s = Math.sin(angle)
      this.c = Math.cos(angle)
    }
  }

  Clone(): Rot {
    return new Rot().Copy(this)
  }

  Copy(other: Rot): this {
    this.s = other.s
    this.c = other.c
    return this
  }

  SetAngle(angle: number): this {
    this.s = Math.sin(angle)
    this.c = Math.cos(angle)
    return this
  }

  SetIdentity(): this {
    this.s = 0
    this.c = 1
    return this
  }

  GetAngle(): number {
    return Math.atan2(this.s, this.c)
  }

  GetXAxis<T extends XY>(out: T): T {
    out.x = this.c
    out.y = this.s
    return out
  }

  GetYAxis<T extends XY>(out: T): T {
    out.x = -this.s
    out.y = this.c
    return out
  }
}

/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
export class Transform {
  static readonly IDENTITY: Readonly<Transform> = new Transform()

  static MulXV<T extends XY>(T: Transform, v: XY, out: T): T {
    // float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
    // float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
    // return Vec2(x, y);
    const T_q_c: number = T.q.c
    const T_q_s: number = T.q.s
    const v_x: number = v.x
    const v_y: number = v.y
    out.x = T_q_c * v_x - T_q_s * v_y + T.p.x
    out.y = T_q_s * v_x + T_q_c * v_y + T.p.y
    return out
  }

  static MulTXV<T extends XY>(T: Transform, v: XY, out: T): T {
    // float32 px = v.x - T.p.x;
    // float32 py = v.y - T.p.y;
    // float32 x = (T.q.c * px + T.q.s * py);
    // float32 y = (-T.q.s * px + T.q.c * py);
    // return Vec2(x, y);
    const T_q_c: number = T.q.c
    const T_q_s: number = T.q.s
    const p_x: number = v.x - T.p.x
    const p_y: number = v.y - T.p.y
    out.x = T_q_c * p_x + T_q_s * p_y
    out.y = -T_q_s * p_x + T_q_c * p_y
    return out
  }

  static MulXX(A: Transform, B: Transform, out: Transform): Transform {
    Rot.MulRR(A.q, B.q, out.q)
    Vec2.AddVV(Rot.MulRV(A.q, B.p, out.p), A.p, out.p)
    return out
  }

  static MulTXX(A: Transform, B: Transform, out: Transform): Transform {
    Rot.MulTRR(A.q, B.q, out.q)
    Rot.MulTRV(A.q, Vec2.SubVV(B.p, A.p, out.p), out.p)
    return out
  }

  readonly p: Vec2 = new Vec2()
  readonly q: Rot = new Rot()

  Clone(): Transform {
    return new Transform().Copy(this)
  }

  Copy(other: Transform): this {
    this.p.Copy(other.p)
    this.q.Copy(other.q)
    return this
  }

  SetIdentity(): this {
    this.p.SetZero()
    this.q.SetIdentity()
    return this
  }

  SetPositionRotation(position: XY, q: Readonly<Rot>): this {
    this.p.Copy(position)
    this.q.Copy(q)
    return this
  }

  SetPositionAngle(pos: XY, a: number): this {
    this.p.Copy(pos)
    this.q.SetAngle(a)
    return this
  }

  SetPosition(position: XY): this {
    this.p.Copy(position)
    return this
  }

  SetPositionXY(x: number, y: number): this {
    this.p.Set(x, y)
    return this
  }

  SetRotation(rotation: Readonly<Rot>): this {
    this.q.Copy(rotation)
    return this
  }

  SetRotationAngle(radians: number): this {
    this.q.SetAngle(radians)
    return this
  }

  GetPosition(): Readonly<Vec2> {
    return this.p
  }

  GetRotation(): Readonly<Rot> {
    return this.q
  }

  GetRotationAngle(): number {
    return this.q.GetAngle()
  }

  GetAngle(): number {
    return this.q.GetAngle()
  }
}

/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
export class Sweep {
  readonly localCenter: Vec2 = new Vec2()
  readonly c0: Vec2 = new Vec2()
  readonly c: Vec2 = new Vec2()
  a0: number = 0
  a: number = 0
  alpha0: number = 0

  Clone(): Sweep {
    return new Sweep().Copy(this)
  }

  Copy(other: Sweep): this {
    this.localCenter.Copy(other.localCenter)
    this.c0.Copy(other.c0)
    this.c.Copy(other.c)
    this.a0 = other.a0
    this.a = other.a
    this.alpha0 = other.alpha0
    return this
  }

  GetTransform(xf: Transform, beta: number): Transform {
    const one_minus_beta: number = 1 - beta
    xf.p.x = one_minus_beta * this.c0.x + beta * this.c.x
    xf.p.y = one_minus_beta * this.c0.y + beta * this.c.y
    const angle: number = one_minus_beta * this.a0 + beta * this.a
    xf.q.SetAngle(angle)

    xf.p.SelfSub(Rot.MulRV(xf.q, this.localCenter, Vec2.s_t0))
    return xf
  }

  Advance(alpha: number): void {
    // DEBUG: Assert(this.alpha0 < 1);
    const beta: number = (alpha - this.alpha0) / (1 - this.alpha0)
    const one_minus_beta: number = 1 - beta
    this.c0.x = one_minus_beta * this.c0.x + beta * this.c.x
    this.c0.y = one_minus_beta * this.c0.y + beta * this.c.y
    this.a0 = one_minus_beta * this.a0 + beta * this.a
    this.alpha0 = alpha
  }

  Normalize(): void {
    const d: number = two_pi * Math.floor(this.a0 / two_pi)
    this.a0 -= d
    this.a -= d
  }
}
