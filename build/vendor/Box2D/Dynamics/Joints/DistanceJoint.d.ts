import { Rot, Vec2, XY } from '../../Common/Math';
import { Body } from '../Body';
import { SolverData } from '../TimeStep';
import { IJointDef, Joint, JointDef } from './Joint';
export interface IDistanceJointDef extends IJointDef {
    localAnchorA: XY;
    localAnchorB: XY;
    length: number;
    frequencyHz?: number;
    dampingRatio?: number;
}
export declare class DistanceJointDef extends JointDef implements IDistanceJointDef {
    readonly localAnchorA: Vec2;
    readonly localAnchorB: Vec2;
    length: number;
    frequencyHz: number;
    dampingRatio: number;
    constructor();
    Initialize(b1: Body, b2: Body, anchor1: XY, anchor2: XY): void;
}
export declare class DistanceJoint extends Joint {
    private static InitVelocityConstraints_s_P;
    private static SolveVelocityConstraints_s_vpA;
    private static SolveVelocityConstraints_s_vpB;
    private static SolveVelocityConstraints_s_P;
    private static SolvePositionConstraints_s_P;
    m_frequencyHz: number;
    m_dampingRatio: number;
    m_bias: number;
    readonly m_localAnchorA: Vec2;
    readonly m_localAnchorB: Vec2;
    m_gamma: number;
    m_impulse: number;
    m_length: number;
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
    readonly m_qA: Rot;
    readonly m_qB: Rot;
    readonly m_lalcA: Vec2;
    readonly m_lalcB: Vec2;
    constructor(def: IDistanceJointDef);
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    GetLocalAnchorA(): Readonly<Vec2>;
    GetLocalAnchorB(): Readonly<Vec2>;
    SetLength(length: number): void;
    Length(): number;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
    InitVelocityConstraints(data: SolverData): void;
    SolveVelocityConstraints(data: SolverData): void;
    SolvePositionConstraints(data: SolverData): boolean;
}
