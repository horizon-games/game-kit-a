import { Mat22, Rot, Vec2, XY } from '../../Common/Math';
import { SolverData } from '../TimeStep';
import { IJointDef, Joint, JointDef } from './Joint';
export interface IMouseJointDef extends IJointDef {
    target?: XY;
    maxForce?: number;
    frequencyHz?: number;
    dampingRatio?: number;
}
export declare class MouseJointDef extends JointDef implements IMouseJointDef {
    readonly target: Vec2;
    maxForce: number;
    frequencyHz: number;
    dampingRatio: number;
    constructor();
}
export declare class MouseJoint extends Joint {
    private static SolveVelocityConstraints_s_Cdot;
    private static SolveVelocityConstraints_s_impulse;
    private static SolveVelocityConstraints_s_oldImpulse;
    readonly m_localAnchorB: Vec2;
    readonly m_targetA: Vec2;
    m_frequencyHz: number;
    m_dampingRatio: number;
    m_beta: number;
    readonly m_impulse: Vec2;
    m_maxForce: number;
    m_gamma: number;
    m_indexA: number;
    m_indexB: number;
    readonly m_rB: Vec2;
    readonly m_localCenterB: Vec2;
    m_invMassB: number;
    m_invIB: number;
    readonly m_mass: Mat22;
    readonly m_C: Vec2;
    readonly m_qB: Rot;
    readonly m_lalcB: Vec2;
    readonly m_K: Mat22;
    constructor(def: IMouseJointDef);
    SetTarget(target: Vec2): void;
    GetTarget(): Vec2;
    SetMaxForce(maxForce: number): void;
    GetMaxForce(): number;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
    InitVelocityConstraints(data: SolverData): void;
    SolveVelocityConstraints(data: SolverData): void;
    SolvePositionConstraints(data: SolverData): boolean;
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
    ShiftOrigin(newOrigin: Vec2): void;
}
