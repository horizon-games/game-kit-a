import { NumberEaser } from '../animation/RawTweener';
declare type ProgressCallback = (val: number) => void;
export declare class AnimatedBool {
    private _onUpdate;
    private _durationMS;
    easing: NumberEaser;
    private _value;
    private _durationMSOut;
    private _animatedValue;
    constructor(_onUpdate: ProgressCallback, initVal?: boolean, _durationMS?: number, easing?: NumberEaser, durationMSOut?: number);
    pulse(): void;
    animatedValue: number;
    value: boolean;
}
export {};
