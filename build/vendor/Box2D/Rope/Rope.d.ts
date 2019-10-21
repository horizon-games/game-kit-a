import { Draw } from '../Common/Draw';
import { Vec2 } from '../Common/Math';
export declare class RopeDef {
    vertices: Vec2[];
    count: number;
    masses: number[];
    readonly gravity: Vec2;
    damping: number;
    k2: number;
    k3: number;
}
export declare class Rope {
    private static s_d;
    private static s_d1;
    private static s_d2;
    private static s_Jd1;
    private static s_Jd2;
    private static s_J1;
    private static s_J2;
    m_count: number;
    m_ps: Vec2[];
    m_p0s: Vec2[];
    m_vs: Vec2[];
    m_ims: number[];
    m_Ls: number[];
    m_as: number[];
    readonly m_gravity: Vec2;
    m_damping: number;
    m_k2: number;
    m_k3: number;
    GetVertexCount(): number;
    GetVertices(): Vec2[];
    Initialize(def: RopeDef): void;
    Step(h: number, iterations: number): void;
    SolveC2(): void;
    SetAngle(angle: number): void;
    SolveC3(): void;
    Draw(draw: Draw): void;
}
