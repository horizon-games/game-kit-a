import { Sweep, Vec2 } from '../Common/Math';
import { DistanceProxy, SimplexCache } from './Distance';
export declare let toiTime: number;
export declare let toiMaxTime: number;
export declare let toiCalls: number;
export declare let toiIters: number;
export declare let toiMaxIters: number;
export declare let toiRootIters: number;
export declare let toiMaxRootIters: number;
export declare function _toi_reset(): void;
export declare class TOIInput {
    readonly proxyA: DistanceProxy;
    readonly proxyB: DistanceProxy;
    readonly sweepA: Sweep;
    readonly sweepB: Sweep;
    tMax: number;
}
export declare enum TOIOutputState {
    e_unknown = 0,
    e_failed = 1,
    e_overlapped = 2,
    e_touching = 3,
    e_separated = 4
}
export declare class TOIOutput {
    state: TOIOutputState;
    t: number;
}
export declare enum SeparationFunctionType {
    e_unknown = -1,
    e_points = 0,
    e_faceA = 1,
    e_faceB = 2
}
export declare class SeparationFunction {
    m_proxyA: DistanceProxy;
    m_proxyB: DistanceProxy;
    readonly m_sweepA: Sweep;
    readonly m_sweepB: Sweep;
    m_type: SeparationFunctionType;
    readonly m_localPoint: Vec2;
    readonly m_axis: Vec2;
    Initialize(cache: SimplexCache, proxyA: DistanceProxy, sweepA: Sweep, proxyB: DistanceProxy, sweepB: Sweep, t1: number): number;
    FindMinSeparation(indexA: [number], indexB: [number], t: number): number;
    Evaluate(indexA: number, indexB: number, t: number): number;
}
export declare function TimeOfImpact(output: TOIOutput, input: TOIInput): void;
