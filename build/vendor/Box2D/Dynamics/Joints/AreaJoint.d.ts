import { Vec2, XY } from '../../Common/Math';
import { Body } from '../Body';
import { SolverData } from '../TimeStep';
import { DistanceJoint } from './DistanceJoint';
import { IJointDef, Joint, JointDef } from './Joint';
export interface IAreaJointDef extends IJointDef {
    bodies: Body[];
    frequencyHz?: number;
    dampingRatio?: number;
}
export declare class AreaJointDef extends JointDef implements IAreaJointDef {
    bodies: Body[];
    frequencyHz: number;
    dampingRatio: number;
    constructor();
    AddBody(body: Body): void;
}
export declare class AreaJoint extends Joint {
    m_bodies: Body[];
    m_frequencyHz: number;
    m_dampingRatio: number;
    m_impulse: number;
    m_targetLengths: number[];
    m_targetArea: number;
    m_normals: Vec2[];
    m_joints: DistanceJoint[];
    m_deltas: Vec2[];
    m_delta: Vec2;
    constructor(def: IAreaJointDef);
    GetAnchorA<T extends XY>(out: T): T;
    GetAnchorB<T extends XY>(out: T): T;
    GetReactionForce<T extends XY>(inv_dt: number, out: T): T;
    GetReactionTorque(inv_dt: number): number;
    SetFrequency(hz: number): void;
    GetFrequency(): number;
    SetDampingRatio(ratio: number): void;
    GetDampingRatio(): number;
    Dump(log: (format: string, ...args: any[]) => void): void;
    InitVelocityConstraints(data: SolverData): void;
    SolveVelocityConstraints(data: SolverData): void;
    SolvePositionConstraints(data: SolverData): boolean;
}
