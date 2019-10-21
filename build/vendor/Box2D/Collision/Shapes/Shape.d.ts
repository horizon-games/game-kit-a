import { Transform, Vec2 } from '../../Common/Math';
import { AABB, RayCastInput, RayCastOutput } from '../Collision';
import { DistanceProxy } from '../Distance';
export declare class MassData {
    mass: number;
    readonly center: Vec2;
    I: number;
}
export declare enum ShapeType {
    e_unknown = -1,
    e_circleShape = 0,
    e_edgeShape = 1,
    e_polygonShape = 2,
    e_chainShape = 3,
    e_shapeTypeCount = 4
}
export declare abstract class Shape {
    m_type: ShapeType;
    m_radius: number;
    constructor(type: ShapeType, radius: number);
    abstract Clone(): Shape;
    Copy(other: Shape): Shape;
    GetType(): ShapeType;
    abstract GetChildCount(): number;
    abstract TestPoint(xf: Transform, p: Vec2): boolean;
    abstract ComputeDistance(xf: Transform, p: Vec2, normal: Vec2, childIndex: number): number;
    abstract RayCast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: number): boolean;
    abstract ComputeAABB(aabb: AABB, xf: Transform, childIndex: number): void;
    abstract ComputeMass(massData: MassData, density: number): void;
    abstract SetupDistanceProxy(proxy: DistanceProxy, index: number): void;
    abstract ComputeSubmergedArea(normal: Vec2, offset: number, xf: Transform, c: Vec2): number;
    abstract Dump(log: (format: string, ...args: any[]) => void): void;
}
