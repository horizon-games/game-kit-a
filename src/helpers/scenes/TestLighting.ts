import {
  BoxBufferGeometry,
  Mesh,
  MeshStandardMaterial,
  PerspectiveCamera,
  PlaneBufferGeometry,
  SphereBufferGeometry,
  WebGLRenderer
} from 'three'
import { FPSControls } from '../../utils/fpsControls'
import { getUrlFlag } from '../../utils/location'

import { addPrettyLights } from '../utils/lights'

import BaseTestScene from './BaseTestScene'

export default class TestLightingScene extends BaseTestScene {
  constructor() {
    super()
    addPrettyLights(this.scene, this.bgColor)
    const fps = new FPSControls(this.camera as PerspectiveCamera)
    if (getUrlFlag('fpsCam')) {
      fps.toggle(true)
    }
    const init = async () => {
      const unitSize = 0.06
      const radius = unitSize * 0.5
      const basicMaterial = new MeshStandardMaterial({
        color: 0xaaddee,
        roughness: 0.7
      })
      const floor = new Mesh(new PlaneBufferGeometry(1, 1, 1, 1), basicMaterial)
      floor.castShadow = false
      floor.receiveShadow = true
      this.scene.add(floor)
      floor.rotation.x = Math.PI * -0.5
      const sphere = new Mesh(
        new SphereBufferGeometry(radius, 32, 16),
        basicMaterial
      )
      sphere.castShadow = true
      sphere.receiveShadow = true
      sphere.position.x = -unitSize * 0.5
      sphere.position.y = radius
      sphere.name = 'Sphere'
      this.scene.add(sphere)
      const box = new Mesh(
        new BoxBufferGeometry(unitSize, unitSize, unitSize),
        basicMaterial
      )
      box.castShadow = true
      box.receiveShadow = true
      box.position.x = unitSize * 0.5
      box.position.y = radius
      box.name = 'Cube'
      this.scene.add(box)
    }
    init()
  }
  update(dt: number) {
    super.update(dt)
  }
  render(renderer: WebGLRenderer, dt: number) {
    super.render(renderer, dt)
  }
}
