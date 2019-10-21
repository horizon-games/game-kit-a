/*
 * Copyright (c) 2011 Erin Catto http://box2d.org
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
/// Color for debug drawing. Each value has the range [0,1].
export class Color {
    constructor(rr = 0.5, gg = 0.5, bb = 0.5, aa = 1.0) {
        this.r = rr;
        this.g = gg;
        this.b = bb;
        this.a = aa;
    }
    static MixColors(colorA, colorB, strength) {
        const dr = strength * (colorB.r - colorA.r);
        const dg = strength * (colorB.g - colorA.g);
        const db = strength * (colorB.b - colorA.b);
        const da = strength * (colorB.a - colorA.a);
        colorA.r += dr;
        colorA.g += dg;
        colorA.b += db;
        colorA.a += da;
        colorB.r -= dr;
        colorB.g -= dg;
        colorB.b -= db;
        colorB.a -= da;
    }
    static MakeStyleString(r, g, b, a = 1.0) {
        // function clamp(x: number, lo: number, hi: number) { return x < lo ? lo : hi < x ? hi : x; }
        r *= 255; // r = clamp(r, 0, 255);
        g *= 255; // g = clamp(g, 0, 255);
        b *= 255; // b = clamp(b, 0, 255);
        // a = clamp(a, 0, 1);
        if (a < 1) {
            return `rgba(${r},${g},${b},${a})`;
        }
        else {
            return `rgb(${r},${g},${b})`;
        }
    }
    Clone() {
        return new Color().Copy(this);
    }
    Copy(other) {
        this.r = other.r;
        this.g = other.g;
        this.b = other.b;
        this.a = other.a;
        return this;
    }
    IsEqual(color) {
        return (this.r === color.r &&
            this.g === color.g &&
            this.b === color.b &&
            this.a === color.a);
    }
    IsZero() {
        return this.r === 0 && this.g === 0 && this.b === 0 && this.a === 0;
    }
    Set(r, g, b, a = this.a) {
        this.SetRGBA(r, g, b, a);
    }
    SetByteRGB(r, g, b) {
        this.r = r / 0xff;
        this.g = g / 0xff;
        this.b = b / 0xff;
        return this;
    }
    SetByteRGBA(r, g, b, a) {
        this.r = r / 0xff;
        this.g = g / 0xff;
        this.b = b / 0xff;
        this.a = a / 0xff;
        return this;
    }
    SetRGB(rr, gg, bb) {
        this.r = rr;
        this.g = gg;
        this.b = bb;
        return this;
    }
    SetRGBA(rr, gg, bb, aa) {
        this.r = rr;
        this.g = gg;
        this.b = bb;
        this.a = aa;
        return this;
    }
    SelfAdd(color) {
        this.r += color.r;
        this.g += color.g;
        this.b += color.b;
        this.a += color.a;
        return this;
    }
    Add(color, out) {
        out.r = this.r + color.r;
        out.g = this.g + color.g;
        out.b = this.b + color.b;
        out.a = this.a + color.a;
        return out;
    }
    SelfSub(color) {
        this.r -= color.r;
        this.g -= color.g;
        this.b -= color.b;
        this.a -= color.a;
        return this;
    }
    Sub(color, out) {
        out.r = this.r - color.r;
        out.g = this.g - color.g;
        out.b = this.b - color.b;
        out.a = this.a - color.a;
        return out;
    }
    SelfMul(s) {
        this.r *= s;
        this.g *= s;
        this.b *= s;
        this.a *= s;
        return this;
    }
    Mul(s, out) {
        out.r = this.r * s;
        out.g = this.g * s;
        out.b = this.b * s;
        out.a = this.a * s;
        return out;
    }
    Mix(mixColor, strength) {
        Color.MixColors(this, mixColor, strength);
    }
    MakeStyleString(alpha = this.a) {
        return Color.MakeStyleString(this.r, this.g, this.b, alpha);
    }
}
Color.ZERO = new Color(0, 0, 0, 0);
Color.RED = new Color(1, 0, 0);
Color.GREEN = new Color(0, 1, 0);
Color.BLUE = new Color(0, 0, 1);
export var DrawFlags;
(function (DrawFlags) {
    DrawFlags[DrawFlags["e_none"] = 0] = "e_none";
    DrawFlags[DrawFlags["e_shapeBit"] = 1] = "e_shapeBit";
    DrawFlags[DrawFlags["e_jointBit"] = 2] = "e_jointBit";
    DrawFlags[DrawFlags["e_aabbBit"] = 4] = "e_aabbBit";
    DrawFlags[DrawFlags["e_pairBit"] = 8] = "e_pairBit";
    DrawFlags[DrawFlags["e_centerOfMassBit"] = 16] = "e_centerOfMassBit";
    // #if ENABLE_PARTICLE
    DrawFlags[DrawFlags["e_particleBit"] = 32] = "e_particleBit";
    // #endif
    DrawFlags[DrawFlags["e_controllerBit"] = 64] = "e_controllerBit";
    DrawFlags[DrawFlags["e_all"] = 63] = "e_all";
})(DrawFlags || (DrawFlags = {}));
/// Implement and register this class with a World to provide debug drawing of physics
/// entities in your game.
export class Draw {
    constructor() {
        this.m_drawFlags = 0;
    }
    SetFlags(flags) {
        this.m_drawFlags = flags;
    }
    GetFlags() {
        return this.m_drawFlags;
    }
    AppendFlags(flags) {
        this.m_drawFlags |= flags;
    }
    ClearFlags(flags) {
        this.m_drawFlags &= ~flags;
    }
}
//# sourceMappingURL=Draw.js.map