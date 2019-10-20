import { Clock, Color, Vector3 } from 'three'

import { simpleTweener } from './animation/tweeners'
import { BaseTestScene } from './helpers/scenes/BaseTestScene'
import renderer from './renderer'
import { timeUniform } from './uniforms'
import { cameraShaker } from './utils/cameraShaker'
import { nextFrameUpdate } from './utils/onNextFrame'
import { taskTimer } from './utils/taskTimer'
import UpdateManager from './utils/UpdateManager'

document.addEventListener('gesturestart', e => e.preventDefault()) // disable zooming on mobile

const clock = new Clock()
renderer.setClearColor(new Color(0x344556), 1.0)
cameraShaker.camera.position.set(0, 0.5, 0.5)
cameraShaker.camera.lookAt(new Vector3())

export default class View {
  constructor(private _scene: BaseTestScene) {
    // Start loop
    requestAnimationFrame(this._loop)
  }
  private _loop = () => {
    const dt = Math.min(clock.getDelta(), 0.1) * simpleTweener.speed

    nextFrameUpdate()
    simpleTweener.rafTick()
    UpdateManager.update(dt)
    taskTimer.update(dt)
    timeUniform.value += dt

    this._scene.update(dt)
    this._scene.render(renderer, dt)

    requestAnimationFrame(this._loop)
  }
}
