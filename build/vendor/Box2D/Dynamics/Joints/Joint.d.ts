import { Vec2, XY } from '../../Common/Math';
import { Body } from '../Body';
import { SolverData } from '../TimeStep';
export declare enum JointType {
    e_unknownJoint = 0,
    e_revoluteJoint = 1,
    e_prismaticJoint = 2,
    e_distanceJoint = 3,
    e_pulleyJoint = 4,
    e_mouseJoint = 5,
    e_gearJoint = 6,
    e_wheelJoint = 7,
    e_weldJoint = 8,
    e_frictionJoint = 9,
    e_ropeJoint = 10,
    e_motorJoint = 11,
    e_areaJoint = 12
}
export declare enum LimitState {
    e_inactiveLimit = 0,
    e_atLowerLimit = 1,
    e_atUpperLimit = 2,
    e_equalLimits = 3
}
export declare class Jacobian {
    readonly linear: Vec2;
    angularA: number;
    angularB: number;
    SetZero(): Jacobian;
    Set(x: XY, a1: number, a2: number): Jacobian;
}
export declare class JointEdge {
    readonly other: Body;
    readonly joint: Joint;
    prev: JointEdge | null;
    next: JointEdge | null;
    constructor(joint: Joint, other: Body);
}
export interface IJointDef {
    type: JointType;
    userData?: any;
    bodyA: Body;
    bodyB: Body;
    collideConnected?: boolean;
}
export declare class JointDef {
    readonly type: JointType;
    userData: any;
    bodyA: Body;
    bodyB: Body;
    collideConnected: boolean;
    constructor(type: JointType);
}
export declare abstract class Joint {
    readonly m_type: JointType;
    m_prev: Joint | null;
    m_next: Joint | null;
    readonly m_edgeA: JointEdge;
    readonly m_edgeB: JointEdge;
    m_bodyA: Body;
    m_bodyB: Body;
    m_index: number;
    m_islandFlag: boolean;
    m_collideConnected: boolean;
    m_userData: any;
    constructor(def: IJointDef);
    GetType(): JointType;
    GetBodyA(): Body;
    GetBodyB(): Body;
    abstract GetAnchorA<T extends XY>(out: T): T;
    abstract GetAnchorB<T extends XY>(out: T): T;
    abstract GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    abstract GetReactionTorque(inv_dt: number): number;
    GetNext(): Joint | null;
    GetUserData(): any;
    SetUserData(data: any): void;
    IsActive(): boolean;
    GetCollideConnected(): boolean;
    Dump(log: (format: string, ...args: any[]) => void): void;
    ShiftOrigin(newOrigin: XY): void;
    abstract InitVelocityConstraints(data: SolverData): void;
    abstract SolveVelocityConstraints(data: SolverData): void;
    abstract SolvePositionConstraints(data: SolverData): boolean;
}
