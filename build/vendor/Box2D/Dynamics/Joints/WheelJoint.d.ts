import { Rot, Vec2, XY } from '../../Common/Math';
import { Body } from '../Body';
import { SolverData } from '../TimeStep';
import { IJointDef, Joint, JointDef } from './Joint';
export interface IWheelJointDef extends IJointDef {
    localAnchorA?: XY;
    localAnchorB?: XY;
    localAxisA?: XY;
    enableMotor?: boolean;
    maxMotorTorque?: number;
    motorSpeed?: number;
    frequencyHz?: number;
    dampingRatio?: number;
}
export declare class WheelJointDef extends JointDef implements IWheelJointDef {
    readonly localAnchorA: Vec2;
    readonly localAnchorB: Vec2;
    readonly localAxisA: Vec2;
    enableMotor: boolean;
    maxMotorTorque: number;
    motorSpeed: number;
    frequencyHz: number;
    dampingRatio: number;
    constructor();
    Initialize(bA: Body, bB: Body, anchor: Vec2, axis: Vec2): void;
}
export declare class WheelJoint extends Joint {
    private static InitVelocityConstraints_s_d;
    private static InitVelocityConstraints_s_P;
    private static SolveVelocityConstraints_s_P;
    private static SolvePositionConstraints_s_d;
    private static SolvePositionConstraints_s_P;
    m_frequencyHz: number;
    m_dampingRatio: number;
    readonly m_localAnchorA: Vec2;
    readonly m_localAnchorB: Vec2;
    readonly m_localXAxisA: Vec2;
    readonly m_localYAxisA: Vec2;
    m_impulse: number;
    m_motorImpulse: number;
    m_springImpulse: number;
    m_maxMotorTorque: number;
    m_motorSpeed: number;
    m_enableMotor: boolean;
    m_indexA: number;
    m_indexB: number;
    readonly m_localCenterA: Vec2;
    readonly m_localCenterB: Vec2;
    m_invMassA: number;
    m_invMassB: number;
    m_invIA: number;
    m_invIB: number;
    readonly m_ax: Vec2;
    readonly m_ay: Vec2;
    m_sAx: number;
    m_sBx: number;
    m_sAy: number;
    m_sBy: number;
    m_mass: number;
    m_motorMass: number;
    m_springMass: number;
    m_bias: number;
    m_gamma: number;
    readonly m_qA: Rot;
    readonly m_qB: Rot;
    readonly m_lalcA: Vec2;
    readonly m_lalcB: Vec2;
    readonly m_rA: Vec2;
    readonly m_rB: Vec2;
    constructor(def: IWheelJointDef);
    GetMotorSpeed(): number;
    GetMaxMotorTorque(): number;
    SetSpringFrequencyHz(hz: number): void;
    GetSpringFrequencyHz(): number;
    SetSpringDampingRatio(ratio: number): void;
    GetSpringDampingRatio(): number;
    InitVelocityConstraints(data: SolverData): void;
    SolveVelocityConstraints(data: SolverData): void;
    SolvePositionConstraints(data: SolverData): boolean;
    GetDefinition(def: WheelJointDef): WheelJointDef;
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    GetLocalAnchorA(): Readonly<Vec2>;
    GetLocalAnchorB(): Readonly<Vec2>;
    GetLocalAxisA(): Readonly<Vec2>;
    GetJointTranslation(): number;
    GetJointLinearSpeed(): number;
    GetJointAngle(): number;
    GetJointAngularSpeed(): number;
    GetPrismaticJointTranslation(): number;
    GetPrismaticJointSpeed(): number;
    GetRevoluteJointAngle(): number;
    GetRevoluteJointSpeed(): number;
    IsMotorEnabled(): boolean;
    EnableMotor(flag: boolean): void;
    SetMotorSpeed(speed: number): void;
    SetMaxMotorTorque(force: number): void;
    GetMotorTorque(inv_dt: number): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
}
