import { AABB, RayCastInput, RayCastOutput } from '../Collision/Collision';
import { TreeNode } from '../Collision/DynamicTree';
import { MassData, Shape, ShapeType } from '../Collision/Shapes/Shape';
import { Transform, Vec2 } from '../Common/Math';
import { Body } from './Body';
export interface IFilter {
    categoryBits: number;
    maskBits: number;
    groupIndex?: number;
}
export declare class Filter implements IFilter {
    static readonly DEFAULT: Readonly<Filter>;
    categoryBits: number;
    maskBits: number;
    groupIndex: number;
    Clone(): Filter;
    Copy(other: IFilter): this;
}
export interface IFixtureDef {
    shape: Shape;
    userData?: any;
    friction?: number;
    restitution?: number;
    density?: number;
    isSensor?: boolean;
    filter?: IFilter;
}
export declare class FixtureDef implements IFixtureDef {
    shape: Shape;
    userData: any;
    friction: number;
    restitution: number;
    density: number;
    isSensor: boolean;
    readonly filter: Filter;
}
export declare class FixtureProxy {
    readonly aabb: AABB;
    fixture: Fixture;
    childIndex: number;
    treeNode: TreeNode<FixtureProxy>;
    constructor(fixture: Fixture);
}
export declare class Fixture {
    private static Synchronize_s_aabb1;
    private static Synchronize_s_aabb2;
    private static Synchronize_s_displacement;
    m_density: number;
    m_next: Fixture | null;
    readonly m_body: Body;
    readonly m_shape: Shape;
    m_friction: number;
    m_restitution: number;
    m_proxies: FixtureProxy[];
    m_proxyCount: number;
    readonly m_filter: Filter;
    m_isSensor: boolean;
    m_userData: any;
    constructor(def: IFixtureDef, body: Body);
    GetType(): ShapeType;
    GetShape(): Shape;
    SetSensor(sensor: boolean): void;
    IsSensor(): boolean;
    SetFilterData(filter: Filter): void;
    GetFilterData(): Readonly<Filter>;
    Refilter(): void;
    GetBody(): Body;
    GetNext(): Fixture | null;
    GetUserData(): any;
    SetUserData(data: any): void;
    TestPoint(p: Vec2): boolean;
    ComputeDistance(p: Vec2, normal: Vec2, childIndex: number): number;
    RayCast(output: RayCastOutput, input: RayCastInput, childIndex: number): boolean;
    GetMassData(massData?: MassData): MassData;
    SetDensity(density: number): void;
    GetDensity(): number;
    GetFriction(): number;
    SetFriction(friction: number): void;
    GetRestitution(): number;
    SetRestitution(restitution: number): void;
    GetAABB(childIndex: number): Readonly<AABB>;
    Dump(log: (format: string, ...args: any[]) => void, bodyIndex: number): void;
    Create(def: IFixtureDef): void;
    Destroy(): void;
    CreateProxies(xf: Transform): void;
    DestroyProxies(): void;
    TouchProxies(): void;
    Synchronize(transform1: Transform, transform2: Transform): void;
}
