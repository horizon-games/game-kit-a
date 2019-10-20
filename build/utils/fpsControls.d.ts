import { PerspectiveCamera } from 'three';
export declare class FPSControls {
    private _camera;
    private _active;
    private _cameraLocal;
    private _fpsController;
    constructor(_camera: PerspectiveCamera);
    toggle(state?: boolean): void;
    update(): void;
}
declare const fpsControls: FPSControls;
export default fpsControls;
