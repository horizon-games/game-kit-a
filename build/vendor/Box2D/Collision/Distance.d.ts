import { Transform, Vec2 } from '../Common/Math';
import { Shape } from './Shapes/Shape';
export declare class DistanceProxy {
    readonly m_buffer: Vec2[];
    m_vertices: Vec2[];
    m_count: number;
    m_radius: number;
    Copy(other: Readonly<DistanceProxy>): this;
    Reset(): DistanceProxy;
    SetShape(shape: Shape, index: number): void;
    SetVerticesRadius(vertices: Vec2[], count: number, radius: number): void;
    GetSupport(d: Vec2): number;
    GetSupportVertex(d: Vec2): Vec2;
    GetVertexCount(): number;
    GetVertex(index: number): Vec2;
}
export declare class SimplexCache {
    metric: number;
    count: number;
    readonly indexA: number[];
    readonly indexB: number[];
    Reset(): SimplexCache;
}
export declare class DistanceInput {
    readonly proxyA: DistanceProxy;
    readonly proxyB: DistanceProxy;
    readonly transformA: Transform;
    readonly transformB: Transform;
    useRadii: boolean;
    Reset(): DistanceInput;
}
export declare class DistanceOutput {
    readonly pointA: Vec2;
    readonly pointB: Vec2;
    distance: number;
    iterations: number;
    Reset(): DistanceOutput;
}
export declare class ShapeCastInput {
    readonly proxyA: DistanceProxy;
    readonly proxyB: DistanceProxy;
    readonly transformA: Transform;
    readonly transformB: Transform;
    readonly translationB: Vec2;
}
export declare class ShapeCastOutput {
    readonly point: Vec2;
    readonly normal: Vec2;
    lambda: number;
    iterations: number;
}
export declare let gjkCalls: number;
export declare let gjkIters: number;
export declare let gjkMaxIters: number;
export declare function _gjk_reset(): void;
export declare class SimplexVertex {
    readonly wA: Vec2;
    readonly wB: Vec2;
    readonly w: Vec2;
    a: number;
    indexA: number;
    indexB: number;
    Copy(other: SimplexVertex): SimplexVertex;
}
export declare class Simplex {
    private static s_e12;
    private static s_e13;
    private static s_e23;
    readonly m_v1: SimplexVertex;
    readonly m_v2: SimplexVertex;
    readonly m_v3: SimplexVertex;
    readonly m_vertices: SimplexVertex[];
    m_count: number;
    constructor();
    ReadCache(cache: SimplexCache, proxyA: DistanceProxy, transformA: Transform, proxyB: DistanceProxy, transformB: Transform): void;
    WriteCache(cache: SimplexCache): void;
    GetSearchDirection(out: Vec2): Vec2;
    GetClosestPoint(out: Vec2): Vec2;
    GetWitnessPoints(pA: Vec2, pB: Vec2): void;
    GetMetric(): number;
    Solve2(): void;
    Solve3(): void;
}
export declare function Distance(output: DistanceOutput, cache: SimplexCache, input: DistanceInput): void;
export declare function ShapeCast(output: ShapeCastOutput, input: ShapeCastInput): boolean;
