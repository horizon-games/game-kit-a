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
import { epsilon, MakeArray, pi } from './Settings';
export const degToRad = pi / 180;
export const radToDeg = 180 / pi;
export const two_pi = 2 * pi;
export const Abs = Math.abs;
export const Min = Math.min;
export const Max = Math.max;
export function Clamp(a, lo, hi) {
    return a < lo ? lo : a > hi ? hi : a;
}
export function Swap(a, b) {
    // DEBUG: Assert(false);
    const tmp = a[0];
    a[0] = b[0];
    b[0] = tmp;
}
/// This function is used to ensure that a floating point number is
/// not a NaN or infinity.
export const IsValid = isFinite;
export function Sq(n) {
    return n * n;
}
/// This is a approximate yet fast inverse square-root.
export function InvSqrt(n) {
    return 1 / Math.sqrt(n);
}
export const Sqrt = Math.sqrt;
export const Pow = Math.pow;
export function DegToRad(degrees) {
    return degrees * degToRad;
}
export function RadToDeg(radians) {
    return radians * radToDeg;
}
export const Cos = Math.cos;
export const Sin = Math.sin;
export const Acos = Math.acos;
export const Asin = Math.asin;
export const Atan2 = Math.atan2;
export function NextPowerOfTwo(x) {
    x |= (x >> 1) & 0x7fffffff;
    x |= (x >> 2) & 0x3fffffff;
    x |= (x >> 4) & 0x0fffffff;
    x |= (x >> 8) & 0x00ffffff;
    x |= (x >> 16) & 0x0000ffff;
    return x + 1;
}
export function IsPowerOfTwo(x) {
    return x > 0 && (x & (x - 1)) === 0;
}
export function Random() {
    return Math.random() * 2 - 1;
}
export function RandomRange(lo, hi) {
    return (hi - lo) * Math.random() + lo;
}
/// A 2D column vector.
export class Vec2 {
    constructor(x = 0, y = 0) {
        this.x = x;
        this.y = y;
    }
    static MakeArray(length) {
        return MakeArray(length, (i) => new Vec2());
    }
    static AbsV(v, out) {
        out.x = Abs(v.x);
        out.y = Abs(v.y);
        return out;
    }
    static MinV(a, b, out) {
        out.x = Min(a.x, b.x);
        out.y = Min(a.y, b.y);
        return out;
    }
    static MaxV(a, b, out) {
        out.x = Max(a.x, b.x);
        out.y = Max(a.y, b.y);
        return out;
    }
    static ClampV(v, lo, hi, out) {
        out.x = Clamp(v.x, lo.x, hi.x);
        out.y = Clamp(v.y, lo.y, hi.y);
        return out;
    }
    static RotateV(v, radians, out) {
        const v_x = v.x;
        const v_y = v.y;
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        out.x = c * v_x - s * v_y;
        out.y = s * v_x + c * v_y;
        return out;
    }
    static DotVV(a, b) {
        return a.x * b.x + a.y * b.y;
    }
    static CrossVV(a, b) {
        return a.x * b.y - a.y * b.x;
    }
    static CrossVS(v, s, out) {
        const v_x = v.x;
        out.x = s * v.y;
        out.y = -s * v_x;
        return out;
    }
    static CrossVOne(v, out) {
        const v_x = v.x;
        out.x = v.y;
        out.y = -v_x;
        return out;
    }
    static CrossSV(s, v, out) {
        const v_x = v.x;
        out.x = -s * v.y;
        out.y = s * v_x;
        return out;
    }
    static CrossOneV(v, out) {
        const v_x = v.x;
        out.x = -v.y;
        out.y = v_x;
        return out;
    }
    static AddVV(a, b, out) {
        out.x = a.x + b.x;
        out.y = a.y + b.y;
        return out;
    }
    static SubVV(a, b, out) {
        out.x = a.x - b.x;
        out.y = a.y - b.y;
        return out;
    }
    static MulSV(s, v, out) {
        out.x = v.x * s;
        out.y = v.y * s;
        return out;
    }
    static MulVS(v, s, out) {
        out.x = v.x * s;
        out.y = v.y * s;
        return out;
    }
    static AddVMulSV(a, s, b, out) {
        out.x = a.x + s * b.x;
        out.y = a.y + s * b.y;
        return out;
    }
    static SubVMulSV(a, s, b, out) {
        out.x = a.x - s * b.x;
        out.y = a.y - s * b.y;
        return out;
    }
    static AddVCrossSV(a, s, v, out) {
        const v_x = v.x;
        out.x = a.x - s * v.y;
        out.y = a.y + s * v_x;
        return out;
    }
    static MidVV(a, b, out) {
        out.x = (a.x + b.x) * 0.5;
        out.y = (a.y + b.y) * 0.5;
        return out;
    }
    static ExtVV(a, b, out) {
        out.x = (b.x - a.x) * 0.5;
        out.y = (b.y - a.y) * 0.5;
        return out;
    }
    static IsEqualToV(a, b) {
        return a.x === b.x && a.y === b.y;
    }
    static DistanceVV(a, b) {
        const c_x = a.x - b.x;
        const c_y = a.y - b.y;
        return Math.sqrt(c_x * c_x + c_y * c_y);
    }
    static DistanceSquaredVV(a, b) {
        const c_x = a.x - b.x;
        const c_y = a.y - b.y;
        return c_x * c_x + c_y * c_y;
    }
    static NegV(v, out) {
        out.x = -v.x;
        out.y = -v.y;
        return out;
    }
    Clone() {
        return new Vec2(this.x, this.y);
    }
    SetZero() {
        this.x = 0;
        this.y = 0;
        return this;
    }
    Set(x, y) {
        this.x = x;
        this.y = y;
        return this;
    }
    Copy(other) {
        this.x = other.x;
        this.y = other.y;
        return this;
    }
    SelfAdd(v) {
        this.x += v.x;
        this.y += v.y;
        return this;
    }
    SelfAddXY(x, y) {
        this.x += x;
        this.y += y;
        return this;
    }
    SelfSub(v) {
        this.x -= v.x;
        this.y -= v.y;
        return this;
    }
    SelfSubXY(x, y) {
        this.x -= x;
        this.y -= y;
        return this;
    }
    SelfMul(s) {
        this.x *= s;
        this.y *= s;
        return this;
    }
    SelfMulAdd(s, v) {
        this.x += s * v.x;
        this.y += s * v.y;
        return this;
    }
    SelfMulSub(s, v) {
        this.x -= s * v.x;
        this.y -= s * v.y;
        return this;
    }
    Dot(v) {
        return this.x * v.x + this.y * v.y;
    }
    Cross(v) {
        return this.x * v.y - this.y * v.x;
    }
    Length() {
        const x = this.x;
        const y = this.y;
        return Math.sqrt(x * x + y * y);
    }
    LengthSquared() {
        const x = this.x;
        const y = this.y;
        return x * x + y * y;
    }
    Normalize() {
        const length = this.Length();
        if (length >= epsilon) {
            const inv_length = 1 / length;
            this.x *= inv_length;
            this.y *= inv_length;
        }
        return length;
    }
    SelfNormalize() {
        const length = this.Length();
        if (length >= epsilon) {
            const inv_length = 1 / length;
            this.x *= inv_length;
            this.y *= inv_length;
        }
        return this;
    }
    SelfRotate(radians) {
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        const x = this.x;
        this.x = c * x - s * this.y;
        this.y = s * x + c * this.y;
        return this;
    }
    SelfRotateCosSin(c, s) {
        const x = this.x;
        this.x = c * x - s * this.y;
        this.y = s * x + c * this.y;
        return this;
    }
    IsValid() {
        return isFinite(this.x) && isFinite(this.y);
    }
    SelfCrossVS(s) {
        const x = this.x;
        this.x = s * this.y;
        this.y = -s * x;
        return this;
    }
    SelfCrossSV(s) {
        const x = this.x;
        this.x = -s * this.y;
        this.y = s * x;
        return this;
    }
    SelfMinV(v) {
        this.x = Min(this.x, v.x);
        this.y = Min(this.y, v.y);
        return this;
    }
    SelfMaxV(v) {
        this.x = Max(this.x, v.x);
        this.y = Max(this.y, v.y);
        return this;
    }
    SelfAbs() {
        this.x = Abs(this.x);
        this.y = Abs(this.y);
        return this;
    }
    SelfNeg() {
        this.x = -this.x;
        this.y = -this.y;
        return this;
    }
    SelfSkew() {
        const x = this.x;
        this.x = -this.y;
        this.y = x;
        return this;
    }
}
Vec2.ZERO = new Vec2(0, 0);
Vec2.UNITX = new Vec2(1, 0);
Vec2.UNITY = new Vec2(0, 1);
Vec2.s_t0 = new Vec2();
Vec2.s_t1 = new Vec2();
Vec2.s_t2 = new Vec2();
Vec2.s_t3 = new Vec2();
export const Vec2_zero = new Vec2(0, 0);
/// A 2D column vector with 3 elements.
export class Vec3 {
    constructor(x = 0, y = 0, z = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }
    static DotV3V3(a, b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }
    static CrossV3V3(a, b, out) {
        const a_x = a.x;
        const a_y = a.y;
        const a_z = a.z;
        const b_x = b.x;
        const b_y = b.y;
        const b_z = b.z;
        out.x = a_y * b_z - a_z * b_y;
        out.y = a_z * b_x - a_x * b_z;
        out.z = a_x * b_y - a_y * b_x;
        return out;
    }
    Clone() {
        return new Vec3(this.x, this.y, this.z);
    }
    SetZero() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
        return this;
    }
    SetXYZ(x, y, z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }
    Copy(other) {
        this.x = other.x;
        this.y = other.y;
        this.z = other.z;
        return this;
    }
    SelfNeg() {
        this.x = -this.x;
        this.y = -this.y;
        this.z = -this.z;
        return this;
    }
    SelfAdd(v) {
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
        return this;
    }
    SelfAddXYZ(x, y, z) {
        this.x += x;
        this.y += y;
        this.z += z;
        return this;
    }
    SelfSub(v) {
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
        return this;
    }
    SelfSubXYZ(x, y, z) {
        this.x -= x;
        this.y -= y;
        this.z -= z;
        return this;
    }
    SelfMul(s) {
        this.x *= s;
        this.y *= s;
        this.z *= s;
        return this;
    }
}
Vec3.ZERO = new Vec3(0, 0, 0);
Vec3.s_t0 = new Vec3();
/// A 2-by-2 matrix. Stored in column-major order.
export class Mat22 {
    constructor() {
        this.ex = new Vec2(1, 0);
        this.ey = new Vec2(0, 1);
    }
    static FromVV(c1, c2) {
        return new Mat22().SetVV(c1, c2);
    }
    static FromSSSS(r1c1, r1c2, r2c1, r2c2) {
        return new Mat22().SetSSSS(r1c1, r1c2, r2c1, r2c2);
    }
    static FromAngle(radians) {
        return new Mat22().SetAngle(radians);
    }
    static AbsM(M, out) {
        const M_ex = M.ex;
        const M_ey = M.ey;
        out.ex.x = Abs(M_ex.x);
        out.ex.y = Abs(M_ex.y);
        out.ey.x = Abs(M_ey.x);
        out.ey.y = Abs(M_ey.y);
        return out;
    }
    static MulMV(M, v, out) {
        const M_ex = M.ex;
        const M_ey = M.ey;
        const v_x = v.x;
        const v_y = v.y;
        out.x = M_ex.x * v_x + M_ey.x * v_y;
        out.y = M_ex.y * v_x + M_ey.y * v_y;
        return out;
    }
    static MulTMV(M, v, out) {
        const M_ex = M.ex;
        const M_ey = M.ey;
        const v_x = v.x;
        const v_y = v.y;
        out.x = M_ex.x * v_x + M_ex.y * v_y;
        out.y = M_ey.x * v_x + M_ey.y * v_y;
        return out;
    }
    static AddMM(A, B, out) {
        const A_ex = A.ex;
        const A_ey = A.ey;
        const B_ex = B.ex;
        const B_ey = B.ey;
        out.ex.x = A_ex.x + B_ex.x;
        out.ex.y = A_ex.y + B_ex.y;
        out.ey.x = A_ey.x + B_ey.x;
        out.ey.y = A_ey.y + B_ey.y;
        return out;
    }
    static MulMM(A, B, out) {
        const A_ex_x = A.ex.x;
        const A_ex_y = A.ex.y;
        const A_ey_x = A.ey.x;
        const A_ey_y = A.ey.y;
        const B_ex_x = B.ex.x;
        const B_ex_y = B.ex.y;
        const B_ey_x = B.ey.x;
        const B_ey_y = B.ey.y;
        out.ex.x = A_ex_x * B_ex_x + A_ey_x * B_ex_y;
        out.ex.y = A_ex_y * B_ex_x + A_ey_y * B_ex_y;
        out.ey.x = A_ex_x * B_ey_x + A_ey_x * B_ey_y;
        out.ey.y = A_ex_y * B_ey_x + A_ey_y * B_ey_y;
        return out;
    }
    static MulTMM(A, B, out) {
        const A_ex_x = A.ex.x;
        const A_ex_y = A.ex.y;
        const A_ey_x = A.ey.x;
        const A_ey_y = A.ey.y;
        const B_ex_x = B.ex.x;
        const B_ex_y = B.ex.y;
        const B_ey_x = B.ey.x;
        const B_ey_y = B.ey.y;
        out.ex.x = A_ex_x * B_ex_x + A_ex_y * B_ex_y;
        out.ex.y = A_ey_x * B_ex_x + A_ey_y * B_ex_y;
        out.ey.x = A_ex_x * B_ey_x + A_ex_y * B_ey_y;
        out.ey.y = A_ey_x * B_ey_x + A_ey_y * B_ey_y;
        return out;
    }
    Clone() {
        return new Mat22().Copy(this);
    }
    SetSSSS(r1c1, r1c2, r2c1, r2c2) {
        this.ex.Set(r1c1, r2c1);
        this.ey.Set(r1c2, r2c2);
        return this;
    }
    SetVV(c1, c2) {
        this.ex.Copy(c1);
        this.ey.Copy(c2);
        return this;
    }
    SetAngle(radians) {
        const c = Math.cos(radians);
        const s = Math.sin(radians);
        this.ex.Set(c, s);
        this.ey.Set(-s, c);
        return this;
    }
    Copy(other) {
        this.ex.Copy(other.ex);
        this.ey.Copy(other.ey);
        return this;
    }
    SetIdentity() {
        this.ex.Set(1, 0);
        this.ey.Set(0, 1);
        return this;
    }
    SetZero() {
        this.ex.SetZero();
        this.ey.SetZero();
        return this;
    }
    GetAngle() {
        return Math.atan2(this.ex.y, this.ex.x);
    }
    GetInverse(out) {
        const a = this.ex.x;
        const b = this.ey.x;
        const c = this.ex.y;
        const d = this.ey.y;
        let det = a * d - b * c;
        if (det !== 0) {
            det = 1 / det;
        }
        out.ex.x = det * d;
        out.ey.x = -det * b;
        out.ex.y = -det * c;
        out.ey.y = det * a;
        return out;
    }
    Solve(b_x, b_y, out) {
        const a11 = this.ex.x;
        const a12 = this.ey.x;
        const a21 = this.ex.y;
        const a22 = this.ey.y;
        let det = a11 * a22 - a12 * a21;
        if (det !== 0) {
            det = 1 / det;
        }
        out.x = det * (a22 * b_x - a12 * b_y);
        out.y = det * (a11 * b_y - a21 * b_x);
        return out;
    }
    SelfAbs() {
        this.ex.SelfAbs();
        this.ey.SelfAbs();
        return this;
    }
    SelfInv() {
        this.GetInverse(this);
        return this;
    }
    SelfAddM(M) {
        this.ex.SelfAdd(M.ex);
        this.ey.SelfAdd(M.ey);
        return this;
    }
    SelfSubM(M) {
        this.ex.SelfSub(M.ex);
        this.ey.SelfSub(M.ey);
        return this;
    }
}
Mat22.IDENTITY = new Mat22();
/// A 3-by-3 matrix. Stored in column-major order.
export class Mat33 {
    constructor() {
        this.ex = new Vec3(1, 0, 0);
        this.ey = new Vec3(0, 1, 0);
        this.ez = new Vec3(0, 0, 1);
    }
    static MulM33V3(A, v, out) {
        const v_x = v.x;
        const v_y = v.y;
        const v_z = v.z;
        out.x = A.ex.x * v_x + A.ey.x * v_y + A.ez.x * v_z;
        out.y = A.ex.y * v_x + A.ey.y * v_y + A.ez.y * v_z;
        out.z = A.ex.z * v_x + A.ey.z * v_y + A.ez.z * v_z;
        return out;
    }
    static MulM33XYZ(A, x, y, z, out) {
        out.x = A.ex.x * x + A.ey.x * y + A.ez.x * z;
        out.y = A.ex.y * x + A.ey.y * y + A.ez.y * z;
        out.z = A.ex.z * x + A.ey.z * y + A.ez.z * z;
        return out;
    }
    static MulM33V2(A, v, out) {
        const v_x = v.x;
        const v_y = v.y;
        out.x = A.ex.x * v_x + A.ey.x * v_y;
        out.y = A.ex.y * v_x + A.ey.y * v_y;
        return out;
    }
    static MulM33XY(A, x, y, out) {
        out.x = A.ex.x * x + A.ey.x * y;
        out.y = A.ex.y * x + A.ey.y * y;
        return out;
    }
    Clone() {
        return new Mat33().Copy(this);
    }
    SetVVV(c1, c2, c3) {
        this.ex.Copy(c1);
        this.ey.Copy(c2);
        this.ez.Copy(c3);
        return this;
    }
    Copy(other) {
        this.ex.Copy(other.ex);
        this.ey.Copy(other.ey);
        this.ez.Copy(other.ez);
        return this;
    }
    SetIdentity() {
        this.ex.SetXYZ(1, 0, 0);
        this.ey.SetXYZ(0, 1, 0);
        this.ez.SetXYZ(0, 0, 1);
        return this;
    }
    SetZero() {
        this.ex.SetZero();
        this.ey.SetZero();
        this.ez.SetZero();
        return this;
    }
    SelfAddM(M) {
        this.ex.SelfAdd(M.ex);
        this.ey.SelfAdd(M.ey);
        this.ez.SelfAdd(M.ez);
        return this;
    }
    Solve33(b_x, b_y, b_z, out) {
        const a11 = this.ex.x;
        const a21 = this.ex.y;
        const a31 = this.ex.z;
        const a12 = this.ey.x;
        const a22 = this.ey.y;
        const a32 = this.ey.z;
        const a13 = this.ez.x;
        const a23 = this.ez.y;
        const a33 = this.ez.z;
        let det = a11 * (a22 * a33 - a32 * a23) +
            a21 * (a32 * a13 - a12 * a33) +
            a31 * (a12 * a23 - a22 * a13);
        if (det !== 0) {
            det = 1 / det;
        }
        out.x =
            det *
                (b_x * (a22 * a33 - a32 * a23) +
                    b_y * (a32 * a13 - a12 * a33) +
                    b_z * (a12 * a23 - a22 * a13));
        out.y =
            det *
                (a11 * (b_y * a33 - b_z * a23) +
                    a21 * (b_z * a13 - b_x * a33) +
                    a31 * (b_x * a23 - b_y * a13));
        out.z =
            det *
                (a11 * (a22 * b_z - a32 * b_y) +
                    a21 * (a32 * b_x - a12 * b_z) +
                    a31 * (a12 * b_y - a22 * b_x));
        return out;
    }
    Solve22(b_x, b_y, out) {
        const a11 = this.ex.x;
        const a12 = this.ey.x;
        const a21 = this.ex.y;
        const a22 = this.ey.y;
        let det = a11 * a22 - a12 * a21;
        if (det !== 0) {
            det = 1 / det;
        }
        out.x = det * (a22 * b_x - a12 * b_y);
        out.y = det * (a11 * b_y - a21 * b_x);
        return out;
    }
    GetInverse22(M) {
        const a = this.ex.x;
        const b = this.ey.x;
        const c = this.ex.y;
        const d = this.ey.y;
        let det = a * d - b * c;
        if (det !== 0) {
            det = 1 / det;
        }
        M.ex.x = det * d;
        M.ey.x = -det * b;
        M.ex.z = 0;
        M.ex.y = -det * c;
        M.ey.y = det * a;
        M.ey.z = 0;
        M.ez.x = 0;
        M.ez.y = 0;
        M.ez.z = 0;
    }
    GetSymInverse33(M) {
        let det = Vec3.DotV3V3(this.ex, Vec3.CrossV3V3(this.ey, this.ez, Vec3.s_t0));
        if (det !== 0) {
            det = 1 / det;
        }
        const a11 = this.ex.x;
        const a12 = this.ey.x;
        const a13 = this.ez.x;
        const a22 = this.ey.y;
        const a23 = this.ez.y;
        const a33 = this.ez.z;
        M.ex.x = det * (a22 * a33 - a23 * a23);
        M.ex.y = det * (a13 * a23 - a12 * a33);
        M.ex.z = det * (a12 * a23 - a13 * a22);
        M.ey.x = M.ex.y;
        M.ey.y = det * (a11 * a33 - a13 * a13);
        M.ey.z = det * (a13 * a12 - a11 * a23);
        M.ez.x = M.ex.z;
        M.ez.y = M.ey.z;
        M.ez.z = det * (a11 * a22 - a12 * a12);
    }
}
Mat33.IDENTITY = new Mat33();
/// Rotation
export class Rot {
    constructor(angle = 0) {
        this.s = 0;
        this.c = 1;
        if (angle) {
            this.s = Math.sin(angle);
            this.c = Math.cos(angle);
        }
    }
    static MulRR(q, r, out) {
        // [qc -qs] * [rc -rs] = [qc*rc-qs*rs -qc*rs-qs*rc]
        // [qs  qc]   [rs  rc]   [qs*rc+qc*rs -qs*rs+qc*rc]
        // s = qs * rc + qc * rs
        // c = qc * rc - qs * rs
        const q_c = q.c;
        const q_s = q.s;
        const r_c = r.c;
        const r_s = r.s;
        out.s = q_s * r_c + q_c * r_s;
        out.c = q_c * r_c - q_s * r_s;
        return out;
    }
    static MulTRR(q, r, out) {
        // [ qc qs] * [rc -rs] = [qc*rc+qs*rs -qc*rs+qs*rc]
        // [-qs qc]   [rs  rc]   [-qs*rc+qc*rs qs*rs+qc*rc]
        // s = qc * rs - qs * rc
        // c = qc * rc + qs * rs
        const q_c = q.c;
        const q_s = q.s;
        const r_c = r.c;
        const r_s = r.s;
        out.s = q_c * r_s - q_s * r_c;
        out.c = q_c * r_c + q_s * r_s;
        return out;
    }
    static MulRV(q, v, out) {
        const q_c = q.c;
        const q_s = q.s;
        const v_x = v.x;
        const v_y = v.y;
        out.x = q_c * v_x - q_s * v_y;
        out.y = q_s * v_x + q_c * v_y;
        return out;
    }
    static MulTRV(q, v, out) {
        const q_c = q.c;
        const q_s = q.s;
        const v_x = v.x;
        const v_y = v.y;
        out.x = q_c * v_x + q_s * v_y;
        out.y = -q_s * v_x + q_c * v_y;
        return out;
    }
    Clone() {
        return new Rot().Copy(this);
    }
    Copy(other) {
        this.s = other.s;
        this.c = other.c;
        return this;
    }
    SetAngle(angle) {
        this.s = Math.sin(angle);
        this.c = Math.cos(angle);
        return this;
    }
    SetIdentity() {
        this.s = 0;
        this.c = 1;
        return this;
    }
    GetAngle() {
        return Math.atan2(this.s, this.c);
    }
    GetXAxis(out) {
        out.x = this.c;
        out.y = this.s;
        return out;
    }
    GetYAxis(out) {
        out.x = -this.s;
        out.y = this.c;
        return out;
    }
}
Rot.IDENTITY = new Rot();
/// A transform contains translation and rotation. It is used to represent
/// the position and orientation of rigid frames.
export class Transform {
    constructor() {
        this.p = new Vec2();
        this.q = new Rot();
    }
    static MulXV(T, v, out) {
        // float32 x = (T.q.c * v.x - T.q.s * v.y) + T.p.x;
        // float32 y = (T.q.s * v.x + T.q.c * v.y) + T.p.y;
        // return Vec2(x, y);
        const T_q_c = T.q.c;
        const T_q_s = T.q.s;
        const v_x = v.x;
        const v_y = v.y;
        out.x = T_q_c * v_x - T_q_s * v_y + T.p.x;
        out.y = T_q_s * v_x + T_q_c * v_y + T.p.y;
        return out;
    }
    static MulTXV(T, v, out) {
        // float32 px = v.x - T.p.x;
        // float32 py = v.y - T.p.y;
        // float32 x = (T.q.c * px + T.q.s * py);
        // float32 y = (-T.q.s * px + T.q.c * py);
        // return Vec2(x, y);
        const T_q_c = T.q.c;
        const T_q_s = T.q.s;
        const p_x = v.x - T.p.x;
        const p_y = v.y - T.p.y;
        out.x = T_q_c * p_x + T_q_s * p_y;
        out.y = -T_q_s * p_x + T_q_c * p_y;
        return out;
    }
    static MulXX(A, B, out) {
        Rot.MulRR(A.q, B.q, out.q);
        Vec2.AddVV(Rot.MulRV(A.q, B.p, out.p), A.p, out.p);
        return out;
    }
    static MulTXX(A, B, out) {
        Rot.MulTRR(A.q, B.q, out.q);
        Rot.MulTRV(A.q, Vec2.SubVV(B.p, A.p, out.p), out.p);
        return out;
    }
    Clone() {
        return new Transform().Copy(this);
    }
    Copy(other) {
        this.p.Copy(other.p);
        this.q.Copy(other.q);
        return this;
    }
    SetIdentity() {
        this.p.SetZero();
        this.q.SetIdentity();
        return this;
    }
    SetPositionRotation(position, q) {
        this.p.Copy(position);
        this.q.Copy(q);
        return this;
    }
    SetPositionAngle(pos, a) {
        this.p.Copy(pos);
        this.q.SetAngle(a);
        return this;
    }
    SetPosition(position) {
        this.p.Copy(position);
        return this;
    }
    SetPositionXY(x, y) {
        this.p.Set(x, y);
        return this;
    }
    SetRotation(rotation) {
        this.q.Copy(rotation);
        return this;
    }
    SetRotationAngle(radians) {
        this.q.SetAngle(radians);
        return this;
    }
    GetPosition() {
        return this.p;
    }
    GetRotation() {
        return this.q;
    }
    GetRotationAngle() {
        return this.q.GetAngle();
    }
    GetAngle() {
        return this.q.GetAngle();
    }
}
Transform.IDENTITY = new Transform();
/// This describes the motion of a body/shape for TOI computation.
/// Shapes are defined with respect to the body origin, which may
/// no coincide with the center of mass. However, to support dynamics
/// we must interpolate the center of mass position.
export class Sweep {
    constructor() {
        this.localCenter = new Vec2();
        this.c0 = new Vec2();
        this.c = new Vec2();
        this.a0 = 0;
        this.a = 0;
        this.alpha0 = 0;
    }
    Clone() {
        return new Sweep().Copy(this);
    }
    Copy(other) {
        this.localCenter.Copy(other.localCenter);
        this.c0.Copy(other.c0);
        this.c.Copy(other.c);
        this.a0 = other.a0;
        this.a = other.a;
        this.alpha0 = other.alpha0;
        return this;
    }
    GetTransform(xf, beta) {
        const one_minus_beta = 1 - beta;
        xf.p.x = one_minus_beta * this.c0.x + beta * this.c.x;
        xf.p.y = one_minus_beta * this.c0.y + beta * this.c.y;
        const angle = one_minus_beta * this.a0 + beta * this.a;
        xf.q.SetAngle(angle);
        xf.p.SelfSub(Rot.MulRV(xf.q, this.localCenter, Vec2.s_t0));
        return xf;
    }
    Advance(alpha) {
        // DEBUG: Assert(this.alpha0 < 1);
        const beta = (alpha - this.alpha0) / (1 - this.alpha0);
        const one_minus_beta = 1 - beta;
        this.c0.x = one_minus_beta * this.c0.x + beta * this.c.x;
        this.c0.y = one_minus_beta * this.c0.y + beta * this.c.y;
        this.a0 = one_minus_beta * this.a0 + beta * this.a;
        this.alpha0 = alpha;
    }
    Normalize() {
        const d = two_pi * Math.floor(this.a0 / two_pi);
        this.a0 -= d;
        this.a -= d;
    }
}
//# sourceMappingURL=Math.js.map