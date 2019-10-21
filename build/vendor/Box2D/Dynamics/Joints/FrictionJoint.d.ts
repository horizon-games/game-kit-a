import { Mat22, Rot, Vec2, XY } from '../../Common/Math';
import { Body } from '../Body';
import { SolverData } from '../TimeStep';
import { IJointDef, Joint, JointDef } from './Joint';
export interface IFrictionJointDef extends IJointDef {
    localAnchorA: XY;
    localAnchorB: XY;
    maxForce?: number;
    maxTorque?: number;
}
export declare class FrictionJointDef extends JointDef implements IFrictionJointDef {
    readonly localAnchorA: Vec2;
    readonly localAnchorB: Vec2;
    maxForce: number;
    maxTorque: number;
    constructor();
    Initialize(bA: Body, bB: Body, anchor: Vec2): void;
}
export declare class FrictionJoint extends Joint {
    private static SolveVelocityConstraints_s_Cdot_v2;
    private static SolveVelocityConstraints_s_impulseV;
    private static SolveVelocityConstraints_s_oldImpulseV;
    readonly m_localAnchorA: Vec2;
    readonly m_localAnchorB: Vec2;
    readonly m_linearImpulse: Vec2;
    m_angularImpulse: number;
    m_maxForce: number;
    m_maxTorque: number;
    m_indexA: number;
    m_indexB: number;
    readonly m_rA: Vec2;
    readonly m_rB: Vec2;
    readonly m_localCenterA: Vec2;
    readonly m_localCenterB: Vec2;
    m_invMassA: number;
    m_invMassB: number;
    m_invIA: number;
    m_invIB: number;
    readonly m_linearMass: Mat22;
    m_angularMass: number;
    readonly m_qA: Rot;
    readonly m_qB: Rot;
    readonly m_lalcA: Vec2;
    readonly m_lalcB: Vec2;
    readonly m_K: Mat22;
    constructor(def: IFrictionJointDef);
    InitVelocityConstraints(data: SolverData): void;
    SolveVelocityConstraints(data: SolverData): void;
    SolvePositionConstraints(data: SolverData): boolean;
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    GetLocalAnchorA(): Readonly<Vec2>;
    GetLocalAnchorB(): Readonly<Vec2>;
    SetMaxForce(force: number): void;
    GetMaxForce(): number;
    SetMaxTorque(torque: number): void;
    GetMaxTorque(): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
