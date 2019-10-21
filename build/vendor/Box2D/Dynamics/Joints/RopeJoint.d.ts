import { Rot, Vec2, XY } from '../../Common/Math';
import { SolverData } from '../TimeStep';
import { IJointDef, Joint, JointDef, LimitState } from './Joint';
export interface IRopeJointDef extends IJointDef {
    localAnchorA?: XY;
    localAnchorB?: XY;
    maxLength?: number;
}
export declare class RopeJointDef extends JointDef implements IRopeJointDef {
    readonly localAnchorA: Vec2;
    readonly localAnchorB: Vec2;
    maxLength: number;
    constructor();
}
export declare class RopeJoint extends Joint {
    private static InitVelocityConstraints_s_P;
    private static SolveVelocityConstraints_s_vpA;
    private static SolveVelocityConstraints_s_vpB;
    private static SolveVelocityConstraints_s_P;
    private static SolvePositionConstraints_s_P;
    readonly m_localAnchorA: Vec2;
    readonly m_localAnchorB: Vec2;
    m_maxLength: number;
    m_length: number;
    m_impulse: number;
    m_indexA: number;
    m_indexB: number;
    readonly m_u: Vec2;
    readonly m_rA: Vec2;
    readonly m_rB: Vec2;
    readonly m_localCenterA: Vec2;
    readonly m_localCenterB: Vec2;
    m_invMassA: number;
    m_invMassB: number;
    m_invIA: number;
    m_invIB: number;
    m_mass: number;
    m_state: LimitState;
    readonly m_qA: Rot;
    readonly m_qB: Rot;
    readonly m_lalcA: Vec2;
    readonly m_lalcB: Vec2;
    constructor(def: IRopeJointDef);
    InitVelocityConstraints(data: SolverData): void;
    SolveVelocityConstraints(data: SolverData): void;
    SolvePositionConstraints(data: SolverData): boolean;
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    GetLocalAnchorA(): Readonly<Vec2>;
    GetLocalAnchorB(): Readonly<Vec2>;
    SetMaxLength(length: number): void;
    GetMaxLength(): number;
    GetLimitState(): LimitState;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
