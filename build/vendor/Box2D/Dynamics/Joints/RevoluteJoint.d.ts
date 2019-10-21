import { Mat22, Mat33, Rot, Vec2, Vec3, XY } from '../../Common/Math';
import { Body } from '../Body';
import { SolverData } from '../TimeStep';
import { IJointDef, Joint, JointDef, LimitState } from './Joint';
export interface IRevoluteJointDef extends IJointDef {
    localAnchorA?: XY;
    localAnchorB?: XY;
    referenceAngle?: number;
    enableLimit?: boolean;
    lowerAngle?: number;
    upperAngle?: number;
    enableMotor?: boolean;
    motorSpeed?: number;
    maxMotorTorque?: number;
}
export declare class RevoluteJointDef extends JointDef implements IRevoluteJointDef {
    readonly localAnchorA: Vec2;
    readonly localAnchorB: Vec2;
    referenceAngle: number;
    enableLimit: boolean;
    lowerAngle: number;
    upperAngle: number;
    enableMotor: boolean;
    motorSpeed: number;
    maxMotorTorque: number;
    constructor();
    Initialize(bA: Body, bB: Body, anchor: XY): void;
}
export declare class RevoluteJoint extends Joint {
    private static InitVelocityConstraints_s_P;
    private static SolveVelocityConstraints_s_P;
    private static SolveVelocityConstraints_s_Cdot_v2;
    private static SolveVelocityConstraints_s_Cdot1;
    private static SolveVelocityConstraints_s_impulse_v3;
    private static SolveVelocityConstraints_s_reduced_v2;
    private static SolveVelocityConstraints_s_impulse_v2;
    private static SolvePositionConstraints_s_C_v2;
    private static SolvePositionConstraints_s_impulse;
    readonly m_localAnchorA: Vec2;
    readonly m_localAnchorB: Vec2;
    readonly m_impulse: Vec3;
    m_motorImpulse: number;
    m_enableMotor: boolean;
    m_maxMotorTorque: number;
    m_motorSpeed: number;
    m_enableLimit: boolean;
    m_referenceAngle: number;
    m_lowerAngle: number;
    m_upperAngle: number;
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
    readonly m_mass: Mat33;
    m_motorMass: number;
    m_limitState: LimitState;
    readonly m_qA: Rot;
    readonly m_qB: Rot;
    readonly m_lalcA: Vec2;
    readonly m_lalcB: Vec2;
    readonly m_K: Mat22;
    constructor(def: IRevoluteJointDef);
    InitVelocityConstraints(data: SolverData): void;
    SolveVelocityConstraints(data: SolverData): void;
    SolvePositionConstraints(data: SolverData): boolean;
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    GetLocalAnchorA(): Readonly<Vec2>;
    GetLocalAnchorB(): Readonly<Vec2>;
    GetReferenceAngle(): number;
    GetJointAngle(): number;
    GetJointSpeed(): number;
    IsMotorEnabled(): boolean;
    EnableMotor(flag: boolean): void;
    GetMotorTorque(inv_dt: number): number;
    GetMotorSpeed(): number;
    SetMaxMotorTorque(torque: number): void;
    GetMaxMotorTorque(): number;
    IsLimitEnabled(): boolean;
    EnableLimit(flag: boolean): void;
    GetLowerLimit(): number;
    GetUpperLimit(): number;
    SetLimits(lower: number, upper: number): void;
    SetMotorSpeed(speed: number): void;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
