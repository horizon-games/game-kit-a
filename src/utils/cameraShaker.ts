import { Euler, PerspectiveCamera } from 'three'
import device from '~/device'
import { clamp } from '~/utils/math'

const SHAKE_CHANGE_DURATION = 0.1
const LOW_HERTZ = 200
const HIGH_HERTZ = 6000

const FOV = 35
const MOBILE_FOV = 28

class CameraShaker {
  private set shakeStrength(value: number) {
    value = clamp(value, 0, 1)
    if (value === this._shakeStrength) {
      return
    }
    this._shakeStrength = value
  }
  set shakeScale(value: number) {
    value = clamp(value, 0, 10)
    if (value === this._shakeScale) {
      return
    }
    this._shakeScale = value
  }
  shakyCamera: PerspectiveCamera
  private _time: number
  private _timers: number[]
  private _shakeStrength: number
  private _shakeScale: number
  constructor(public camera: PerspectiveCamera) {
    this._time = 0
    this._timers = []
    this._shakeScale = 0.00025
    this._shakeStrength = 0
    const shakyCamera = camera.clone()
    shakyCamera.position.set(0, 0, 0)
    shakyCamera.quaternion.setFromEuler(new Euler(0, 0, 0))
    shakyCamera.scale.set(1, 1, 1)
    camera.add(shakyCamera)
    this.shakyCamera = shakyCamera
  }
  updateProjection() {
    this.shakyCamera.fov = this.camera.fov
    this.shakyCamera.aspect = this.camera.aspect
    this.shakyCamera.updateProjectionMatrix()
  }
  update(dt: number) {
    this._time += dt
    this.shakeStrength =
      this._shakeStrength +
      (dt / SHAKE_CHANGE_DURATION) * (this._timers.length > 0 ? 12 : -1)
    if (this._shakeStrength > 0) {
      this.shakyCamera.position.set(0, 0, 0)
      let octave = 1
      for (let i = LOW_HERTZ; i < HIGH_HERTZ; i *= 1.674, octave *= 0.8) {
        const overallStrength =
          (1 / octave) * this._shakeScale * this._shakeStrength
        this.shakyCamera.position.x +=
          Math.sin(this._time * i) * overallStrength
        this.shakyCamera.position.y +=
          Math.sin(this._time * i * 1.235) * overallStrength
        this.shakyCamera.position.z +=
          Math.sin(this._time * i * 1.67235) * overallStrength
      }
    }
    while (this._timers.length > 0 && this._timers[0] <= this._time) {
      this._timers.shift()
    }
  }
  add(duration: number) {
    this._timers.push(this._time + duration)
    this._timers.sort()
  }
}

const camera = new PerspectiveCamera(
  device.isMobile ? MOBILE_FOV : FOV,
  device.aspect,
  0.01,
  10
)

export const cameraShaker = new CameraShaker(camera)

device.onChange(() => {
  camera.fov = device.isMobile ? MOBILE_FOV : FOV
  camera.aspect = device.aspect
  camera.updateProjectionMatrix()
  cameraShaker.updateProjection()
})
