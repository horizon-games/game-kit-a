export declare const niceParameters: NiceParameter[];
declare type Listener = (value: number) => void;
export declare class NiceParameter {
    name: string;
    label: string;
    defaultValue: number;
    minValue: number;
    maxValue: number;
    step: number;
    sliderOrderPriority: number;
    distribution: (value: number) => number;
    valueTextConverter: (value: number) => string;
    private _availableToUserMethod;
    private _value;
    private _listeners;
    constructor(name: string, label: string, defaultValue: number, minValue: number, maxValue: number, distribution: (value: number) => number, valueTextConverter: (value: number) => string, availableToUser: (() => boolean) | boolean, forceDefault?: boolean, step?: number, sliderOrderPriority?: number);
    listen(callback: Listener): void;
    stopListening(callback: Listener): void;
    value: number;
    readonly availableToUser: boolean;
}
export {};
