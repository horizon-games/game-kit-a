export declare type NumberEaser = (v: number) => number;
interface Params extends Optional {
    target: any;
    propertyGoals: any;
}
interface Optional {
    delay?: number;
    duration?: number;
    easing?: NumberEaser;
    onUpdate?: () => void;
    onComplete?: () => void;
}
declare class AnimatedProperty {
    key: string;
    valueStart: number;
    valueEnd: number;
    constructor(key: string, valueStart: number, valueEnd: number);
}
declare class AnimatedObject {
    private tweener;
    readonly target: any;
    readonly easing: NumberEaser;
    readonly startTime: number;
    readonly endTime: number;
    readonly animatedProperties: AnimatedProperty[];
    readonly onUpdate?: (() => void) | undefined;
    readonly duration: number;
    onComplete: () => void;
    finished: Promise<void>;
    constructor(tweener: RawTweener, target: any, easing: NumberEaser, startTime: number, endTime: number, animatedProperties: AnimatedProperty[], onUpdate?: (() => void) | undefined, onComplete?: () => void);
    kill(): void;
}
export declare class RawTweener {
    protected _now: number;
    private _processingTick;
    private _animations;
    private _animationsToComplete;
    to(params: Params): AnimatedObject;
    tick(delta: number): void;
    killTweensOf(target: any): void;
    kill(animation: AnimatedObject, index?: number): void;
}
export {};
