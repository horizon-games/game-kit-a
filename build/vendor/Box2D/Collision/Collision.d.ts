import { Transform, Vec2 } from '../Common/Math';
import { Shape } from './Shapes/Shape';
export declare enum ContactFeatureType {
    e_vertex = 0,
    e_face = 1
}
export declare class ContactFeature {
    private _key;
    private _key_invalid;
    private _indexA;
    private _indexB;
    private _typeA;
    private _typeB;
    key: number;
    indexA: number;
    indexB: number;
    typeA: number;
    typeB: number;
}
export declare class ContactID {
    readonly cf: ContactFeature;
    Copy(o: ContactID): ContactID;
    Clone(): ContactID;
    key: number;
}
export declare class ManifoldPoint {
    static MakeArray(length: number): ManifoldPoint[];
    readonly localPoint: Vec2;
    normalImpulse: number;
    tangentImpulse: number;
    id: ContactID;
    Reset(): void;
    Copy(o: ManifoldPoint): ManifoldPoint;
}
export declare enum ManifoldType {
    e_unknown = -1,
    e_circles = 0,
    e_faceA = 1,
    e_faceB = 2
}
export declare class Manifold {
    readonly points: ManifoldPoint[];
    readonly localNormal: Vec2;
    readonly localPoint: Vec2;
    type: ManifoldType;
    pointCount: number;
    Reset(): void;
    Copy(o: Manifold): Manifold;
    Clone(): Manifold;
}
export declare class WorldManifold {
    private static Initialize_s_pointA;
    private static Initialize_s_pointB;
    private static Initialize_s_cA;
    private static Initialize_s_cB;
    private static Initialize_s_planePoint;
    private static Initialize_s_clipPoint;
    readonly normal: Vec2;
    readonly points: Vec2[];
    readonly separations: number[];
    Initialize(manifold: Manifold, xfA: Transform, radiusA: number, xfB: Transform, radiusB: number): void;
}
export declare enum PointState {
    nullState = 0,
    addState = 1,
    persistState = 2,
    removeState = 3
}
export declare function GetPointStates(state1: PointState[], state2: PointState[], manifold1: Manifold, manifold2: Manifold): void;
export declare class ClipVertex {
    static MakeArray(length: number): ClipVertex[];
    readonly v: Vec2;
    readonly id: ContactID;
    Copy(other: ClipVertex): ClipVertex;
}
export declare class RayCastInput {
    readonly p1: Vec2;
    readonly p2: Vec2;
    maxFraction: number;
    Copy(o: RayCastInput): RayCastInput;
}
export declare class RayCastOutput {
    readonly normal: Vec2;
    fraction: number;
    Copy(o: RayCastOutput): RayCastOutput;
}
export declare class AABB {
    static Combine(aabb1: AABB, aabb2: AABB, out: AABB): AABB;
    readonly lowerBound: Vec2;
    readonly upperBound: Vec2;
    private readonly m_cache_center;
    private readonly m_cache_extent;
    Copy(o: AABB): AABB;
    IsValid(): boolean;
    GetCenter(): Vec2;
    GetExtents(): Vec2;
    GetPerimeter(): number;
    Combine1(aabb: AABB): AABB;
    Combine2(aabb1: AABB, aabb2: AABB): AABB;
    Contains(aabb: AABB): boolean;
    RayCast(output: RayCastOutput, input: RayCastInput): boolean;
    TestContain(point: Vec2): boolean;
    TestOverlap(other: AABB): boolean;
}
export declare function TestOverlapAABB(a: AABB, b: AABB): boolean;
export declare function ClipSegmentToLine(vOut: ClipVertex[], vIn: ClipVertex[], normal: Vec2, offset: number, vertexIndexA: number): number;
export declare function TestOverlapShape(shapeA: Shape, indexA: number, shapeB: Shape, indexB: number, xfA: Transform, xfB: Transform): boolean;
