import { Shape } from '../Collision/Shapes/Shape';
import { Color, RGBA } from '../Common/Draw';
import { Transform, Vec2, XY } from '../Common/Math';
import { ParticleFlag } from './Particle';
import { ParticleSystem } from './ParticleSystem';
export declare enum ParticleGroupFlag {
    solidParticleGroup = 1,
    rigidParticleGroup = 2,
    particleGroupCanBeEmpty = 4,
    particleGroupWillBeDestroyed = 8,
    particleGroupNeedsUpdateDepth = 16,
    particleGroupInternalMask = 24
}
export interface IParticleGroupDef {
    flags?: ParticleFlag;
    groupFlags?: ParticleGroupFlag;
    position?: XY;
    angle?: number;
    linearVelocity?: XY;
    angularVelocity?: number;
    color?: RGBA;
    strength?: number;
    shape?: Shape;
    shapes?: Shape[];
    shapeCount?: number;
    stride?: number;
    particleCount?: number;
    positionData?: XY[];
    lifetime?: number;
    userData?: any;
    group?: ParticleGroup | null;
}
export declare class ParticleGroupDef implements IParticleGroupDef {
    flags: ParticleFlag;
    groupFlags: ParticleGroupFlag;
    readonly position: Vec2;
    angle: number;
    readonly linearVelocity: Vec2;
    angularVelocity: number;
    readonly color: Color;
    strength: number;
    shape?: Shape;
    shapes?: Shape[];
    shapeCount: number;
    stride: number;
    particleCount: number;
    positionData?: Vec2[];
    lifetime: number;
    userData: any;
    group: ParticleGroup | null;
}
export declare class ParticleGroup {
    static readonly GetLinearVelocityFromWorldPoint_s_t0: Vec2;
    readonly m_system: ParticleSystem;
    m_firstIndex: number;
    m_lastIndex: number;
    m_groupFlags: ParticleGroupFlag;
    m_strength: number;
    m_prev: ParticleGroup | null;
    m_next: ParticleGroup | null;
    m_timestamp: number;
    m_mass: number;
    m_inertia: number;
    readonly m_center: Vec2;
    readonly m_linearVelocity: Vec2;
    m_angularVelocity: number;
    readonly m_transform: Transform;
    m_userData: any;
    constructor(system: ParticleSystem);
    GetNext(): ParticleGroup | null;
    GetParticleSystem(): ParticleSystem;
    GetParticleCount(): number;
    GetBufferIndex(): number;
    ContainsParticle(index: number): boolean;
    GetAllParticleFlags(): ParticleFlag;
    GetGroupFlags(): ParticleGroupFlag;
    SetGroupFlags(flags: number): void;
    GetMass(): number;
    GetInertia(): number;
    GetCenter(): Readonly<Vec2>;
    GetLinearVelocity(): Readonly<Vec2>;
    GetAngularVelocity(): number;
    GetTransform(): Readonly<Transform>;
    GetPosition(): Readonly<Vec2>;
    GetAngle(): number;
    GetLinearVelocityFromWorldPoint<T extends XY>(worldPoint: XY, out: T): T;
    GetUserData(): void;
    SetUserData(data: any): void;
    ApplyForce(force: XY): void;
    ApplyLinearImpulse(impulse: XY): void;
    DestroyParticles(callDestructionListener: boolean): void;
    UpdateStatistics(): void;
}
