import { Transform, Vec2, XY } from '../../Common/Math';
import { AABB, RayCastInput, RayCastOutput } from '../Collision';
import { DistanceProxy } from '../Distance';
import { MassData, Shape } from './Shape';
export declare class CircleShape extends Shape {
    private static TestPoint_s_center;
    private static TestPoint_s_d;
    private static ComputeDistance_s_center;
    private static RayCast_s_position;
    private static RayCast_s_s;
    private static RayCast_s_r;
    private static ComputeAABB_s_p;
    readonly m_p: Vec2;
    constructor(radius?: number);
    Set(position: XY, radius?: number): this;
    Clone(): CircleShape;
    Copy(other: CircleShape): CircleShape;
    GetChildCount(): number;
    TestPoint(transform: Transform, p: XY): boolean;
    ComputeDistance(xf: Transform, p: Vec2, normal: Vec2, childIndex: number): number;
    RayCast(output: RayCastOutput, input: RayCastInput, transform: Transform, childIndex: number): boolean;
    ComputeAABB(aabb: AABB, transform: Transform, childIndex: number): void;
    ComputeMass(massData: MassData, density: number): void;
    SetupDistanceProxy(proxy: DistanceProxy, index: number): void;
    ComputeSubmergedArea(normal: Vec2, offset: number, xf: Transform, c: Vec2): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
