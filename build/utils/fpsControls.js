import { PerspectiveCamera } from 'three';
import FPSController from 'threejs-camera-controller-first-person-desktop';
import renderer from '../renderer';
import { cameraShaker } from './cameraShaker';
import { getUrlFloat } from './location';
import { clamp, lerp } from './math';
import UpdateManager from './UpdateManager';
const FPS_CAMERA_DAMPING = getUrlFloat('camDamping', 0, 0, 0.999);
const FPS_CAMERA_LERP_STRENGTH = 1 - clamp(FPS_CAMERA_DAMPING, 0, 1);
function copyCam(dst, src) {
    dst.position.copy(src.position);
    dst.scale.copy(src.scale);
    dst.quaternion.copy(src.quaternion);
    dst.fov = src.fov;
}
export class FPSControls {
    constructor(_camera) {
        this._camera = _camera;
        this._active = false;
        this._fpsController = undefined;
        //
    }
    toggle(state) {
        if (state === undefined) {
            state = !this._active;
        }
        if (!this._fpsController) {
            this._cameraLocal = new PerspectiveCamera();
            copyCam(this._cameraLocal, this._camera);
            this._camera.parent.add(this._cameraLocal);
            this._fpsController = new FPSController(this._cameraLocal, renderer.domElement, {
                movementSpeed: 0.01
            });
            UpdateManager.register(this);
            // setInterval(() => {
            //   cameraShaker.add(0.025)
            // }, 2000)
        }
        const sig = this._fpsController.onPointerLockAttainSignal;
        const origListener = sig._bindings[0]._listener;
        sig._bindings[0]._listener = () => {
            if (this._active) {
                origListener();
            }
        };
        if (!state) {
            this._fpsController.onPointerLockReleaseSignal.dispatch();
        }
        // debugger
        this._active = state;
    }
    update() {
        if (this._active && this._fpsController) {
            this._fpsController.update();
            this._camera.position.lerp(this._cameraLocal.position, FPS_CAMERA_LERP_STRENGTH);
            this._camera.quaternion.slerp(this._cameraLocal.quaternion, FPS_CAMERA_LERP_STRENGTH);
            this._camera.scale.lerp(this._cameraLocal.scale, FPS_CAMERA_LERP_STRENGTH);
            // this._camera.matrix.copy(this._cameraLocal.matrix)
            this._camera.fov = lerp(this._camera.fov, this._cameraLocal.fov, FPS_CAMERA_LERP_STRENGTH);
            cameraShaker.updateProjection();
        }
    }
}
const fpsControls = new FPSControls(cameraShaker.camera);
export default fpsControls;
//# sourceMappingURL=fpsControls.js.map