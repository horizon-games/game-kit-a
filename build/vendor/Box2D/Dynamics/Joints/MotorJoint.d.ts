import { Mat22, Rot, Vec2, XY } from '../../Common/Math';
import { Body } from '../Body';
import { SolverData } from '../TimeStep';
import { IJointDef, Joint, JointDef } from './Joint';
export interface IMotorJointDef extends IJointDef {
    linearOffset?: XY;
    angularOffset?: number;
    maxForce?: number;
    maxTorque?: number;
    correctionFactor?: number;
}
export declare class MotorJointDef extends JointDef implements IMotorJointDef {
    readonly linearOffset: Vec2;
    angularOffset: number;
    maxForce: number;
    maxTorque: number;
    correctionFactor: number;
    constructor();
    Initialize(bA: Body, bB: Body): void;
}
export declare class MotorJoint extends Joint {
    private static SolveVelocityConstraints_s_Cdot_v2;
    private static SolveVelocityConstraints_s_impulse_v2;
    private static SolveVelocityConstraints_s_oldImpulse_v2;
    readonly m_linearOffset: Vec2;
    m_angularOffset: number;
    readonly m_linearImpulse: Vec2;
    m_angularImpulse: number;
    m_maxForce: number;
    m_maxTorque: number;
    m_correctionFactor: number;
    m_indexA: number;
    m_indexB: number;
    readonly m_rA: Vec2;
    readonly m_rB: Vec2;
    readonly m_localCenterA: Vec2;
    readonly m_localCenterB: Vec2;
    readonly m_linearError: Vec2;
    m_angularError: number;
    m_invMassA: number;
    m_invMassB: number;
    m_invIA: number;
    m_invIB: number;
    readonly m_linearMass: Mat22;
    m_angularMass: number;
    readonly m_qA: Rot;
    readonly m_qB: Rot;
    readonly m_K: Mat22;
    constructor(def: IMotorJointDef);
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    SetLinearOffset(linearOffset: Vec2): void;
    GetLinearOffset(): Vec2;
    SetAngularOffset(angularOffset: number): void;
    GetAngularOffset(): number;
    SetMaxForce(force: number): void;
    GetMaxForce(): number;
    SetMaxTorque(torque: number): void;
    GetMaxTorque(): number;
    InitVelocityConstraints(data: SolverData): void;
    SolveVelocityConstraints(data: SolverData): void;
    SolvePositionConstraints(data: SolverData): boolean;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
