import { ManifoldType } from '../../Collision/Collision';
import { Mat22, Transform, Vec2 } from '../../Common/Math';
import { Position, TimeStep, Velocity } from '../TimeStep';
import { Contact } from './Contact';
export declare let g_blockSolve: boolean;
export declare class VelocityConstraintPoint {
    static MakeArray(length: number): VelocityConstraintPoint[];
    readonly rA: Vec2;
    readonly rB: Vec2;
    normalImpulse: number;
    tangentImpulse: number;
    normalMass: number;
    tangentMass: number;
    velocityBias: number;
}
export declare class ContactVelocityConstraint {
    static MakeArray(length: number): ContactVelocityConstraint[];
    points: VelocityConstraintPoint[];
    readonly normal: Vec2;
    readonly tangent: Vec2;
    readonly normalMass: Mat22;
    readonly K: Mat22;
    indexA: number;
    indexB: number;
    invMassA: number;
    invMassB: number;
    invIA: number;
    invIB: number;
    friction: number;
    restitution: number;
    tangentSpeed: number;
    pointCount: number;
    contactIndex: number;
}
export declare class ContactPositionConstraint {
    static MakeArray(length: number): ContactPositionConstraint[];
    localPoints: Vec2[];
    readonly localNormal: Vec2;
    readonly localPoint: Vec2;
    indexA: number;
    indexB: number;
    invMassA: number;
    invMassB: number;
    readonly localCenterA: Vec2;
    readonly localCenterB: Vec2;
    invIA: number;
    invIB: number;
    type: ManifoldType;
    radiusA: number;
    radiusB: number;
    pointCount: number;
}
export declare class ContactSolverDef {
    readonly step: TimeStep;
    contacts: Contact[];
    count: number;
    positions: Position[];
    velocities: Velocity[];
    allocator: any;
}
export declare class PositionSolverManifold {
    private static Initialize_s_pointA;
    private static Initialize_s_pointB;
    private static Initialize_s_planePoint;
    private static Initialize_s_clipPoint;
    readonly normal: Vec2;
    readonly point: Vec2;
    separation: number;
    Initialize(pc: ContactPositionConstraint, xfA: Transform, xfB: Transform, index: number): void;
}
export declare class ContactSolver {
    private static InitializeVelocityConstraints_s_xfA;
    private static InitializeVelocityConstraints_s_xfB;
    private static InitializeVelocityConstraints_s_worldManifold;
    private static WarmStart_s_P;
    private static SolveVelocityConstraints_s_dv;
    private static SolveVelocityConstraints_s_dv1;
    private static SolveVelocityConstraints_s_dv2;
    private static SolveVelocityConstraints_s_P;
    private static SolveVelocityConstraints_s_a;
    private static SolveVelocityConstraints_s_b;
    private static SolveVelocityConstraints_s_x;
    private static SolveVelocityConstraints_s_d;
    private static SolveVelocityConstraints_s_P1;
    private static SolveVelocityConstraints_s_P2;
    private static SolveVelocityConstraints_s_P1P2;
    private static SolvePositionConstraints_s_xfA;
    private static SolvePositionConstraints_s_xfB;
    private static SolvePositionConstraints_s_psm;
    private static SolvePositionConstraints_s_rA;
    private static SolvePositionConstraints_s_rB;
    private static SolvePositionConstraints_s_P;
    private static SolveTOIPositionConstraints_s_xfA;
    private static SolveTOIPositionConstraints_s_xfB;
    private static SolveTOIPositionConstraints_s_psm;
    private static SolveTOIPositionConstraints_s_rA;
    private static SolveTOIPositionConstraints_s_rB;
    private static SolveTOIPositionConstraints_s_P;
    readonly m_step: TimeStep;
    m_positions: Position[];
    m_velocities: Velocity[];
    m_allocator: any;
    m_positionConstraints: ContactPositionConstraint[];
    m_velocityConstraints: ContactVelocityConstraint[];
    m_contacts: Contact[];
    m_count: number;
    Initialize(def: ContactSolverDef): ContactSolver;
    InitializeVelocityConstraints(): void;
    WarmStart(): void;
    SolveVelocityConstraints(): void;
    StoreImpulses(): void;
    SolvePositionConstraints(): boolean;
    SolveTOIPositionConstraints(toiIndexA: number, toiIndexB: number): boolean;
}
