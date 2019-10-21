import { RawTweener } from './RawTweener';
export declare class RafTweener extends RawTweener {
    private discreteStepDuration;
    speed: number;
    rafTick: () => void;
    private requestStop;
    private paused;
    private ticking;
    private timeSnapshot;
    private update;
    constructor(discreteStepDuration?: number, autoStart?: boolean);
    discreteStepTick(delta: number): void;
    start(): void;
    stop(): void;
    pause(): void;
    unpause(): void;
    private _rafTick;
}
