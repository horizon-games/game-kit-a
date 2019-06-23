/// <reference types="three" />

declare module 'threejs-camera-controller-first-person-desktop' {
  class FPSController {
    onPointerLockAttainSignal: any
    onPointerLockReleaseSignal: any
    update: () => void
    constructor(camera: THREE.PerspectiveCamera, element: any, options: any)
  }
  export default FPSController
}
