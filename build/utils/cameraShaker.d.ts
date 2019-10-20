import { PerspectiveCamera } from 'three';
declare class CameraShaker {
    camera: PerspectiveCamera;
    private shakeStrength;
    shakeScale: number;
    shakyCamera: PerspectiveCamera;
    private _time;
    private _timers;
    private _shakeStrength;
    private _shakeScale;
    constructor(camera: PerspectiveCamera);
    updateProjection(): void;
    update(dt: number): void;
    add(duration: number): void;
}
export declare const cameraShaker: CameraShaker;
export {};
