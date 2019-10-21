import { Vec2 } from '../Common/Math';
export declare class Profile {
    step: number;
    collide: number;
    solve: number;
    solveInit: number;
    solveVelocity: number;
    solvePosition: number;
    broadphase: number;
    solveTOI: number;
    Reset(): this;
}
export declare class TimeStep {
    dt: number;
    inv_dt: number;
    dtRatio: number;
    velocityIterations: number;
    positionIterations: number;
    particleIterations: number;
    warmStarting: boolean;
    Copy(step: TimeStep): TimeStep;
}
export declare class Position {
    static MakeArray(length: number): Position[];
    readonly c: Vec2;
    a: number;
}
export declare class Velocity {
    static MakeArray(length: number): Velocity[];
    readonly v: Vec2;
    w: number;
}
export declare class SolverData {
    readonly step: TimeStep;
    positions: Position[];
    velocities: Velocity[];
}
