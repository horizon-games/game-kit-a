import { Transform, XY } from './Math';
export interface RGB {
    r: number;
    g: number;
    b: number;
}
export interface RGBA extends RGB {
    a: number;
}
export declare class Color implements RGBA {
    static readonly ZERO: Readonly<Color>;
    static readonly RED: Readonly<Color>;
    static readonly GREEN: Readonly<Color>;
    static readonly BLUE: Readonly<Color>;
    static MixColors(colorA: RGBA, colorB: RGBA, strength: number): void;
    static MakeStyleString(r: number, g: number, b: number, a?: number): string;
    r: number;
    g: number;
    b: number;
    a: number;
    constructor(rr?: number, gg?: number, bb?: number, aa?: number);
    Clone(): Color;
    Copy(other: RGBA): this;
    IsEqual(color: RGBA): boolean;
    IsZero(): boolean;
    Set(r: number, g: number, b: number, a?: number): void;
    SetByteRGB(r: number, g: number, b: number): this;
    SetByteRGBA(r: number, g: number, b: number, a: number): this;
    SetRGB(rr: number, gg: number, bb: number): this;
    SetRGBA(rr: number, gg: number, bb: number, aa: number): this;
    SelfAdd(color: RGBA): this;
    Add<T extends RGBA>(color: RGBA, out: T): T;
    SelfSub(color: RGBA): this;
    Sub<T extends RGBA>(color: RGBA, out: T): T;
    SelfMul(s: number): this;
    Mul<T extends RGBA>(s: number, out: T): T;
    Mix(mixColor: RGBA, strength: number): void;
    MakeStyleString(alpha?: number): string;
}
export declare enum DrawFlags {
    e_none = 0,
    e_shapeBit = 1,
    e_jointBit = 2,
    e_aabbBit = 4,
    e_pairBit = 8,
    e_centerOfMassBit = 16,
    e_particleBit = 32,
    e_controllerBit = 64,
    e_all = 63
}
export declare abstract class Draw {
    m_drawFlags: DrawFlags;
    SetFlags(flags: DrawFlags): void;
    GetFlags(): DrawFlags;
    AppendFlags(flags: DrawFlags): void;
    ClearFlags(flags: DrawFlags): void;
    abstract PushTransform(xf: Transform): void;
    abstract PopTransform(xf: Transform): void;
    abstract DrawPolygon(vertices: XY[], vertexCount: number, color: RGBA): void;
    abstract DrawSolidPolygon(vertices: XY[], vertexCount: number, color: RGBA): void;
    abstract DrawCircle(center: XY, radius: number, color: RGBA): void;
    abstract DrawSolidCircle(center: XY, radius: number, axis: XY, color: RGBA): void;
    abstract DrawParticles(centers: XY[], radius: number, colors: RGBA[] | null, count: number): void;
    abstract DrawSegment(p1: XY, p2: XY, color: RGBA): void;
    abstract DrawTransform(xf: Transform): void;
    abstract DrawPoint(p: XY, size: number, color: RGBA): void;
}
