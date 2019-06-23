import {
  CameraHelper,
  Color,
  DirectionalLight,
  DirectionalLightHelper,
  Fog,
  HemisphereLight,
  Scene,
  Vector3
} from 'three'
import { getUrlFlag } from '~/utils/location'

const __defaultGroundColor = new Color(0x4f3f2f)
const __defaultSkyColor = new Color(0xafbfef)

const s = 1

const __defaultShadowBoxSize = new Vector3(0.8 * s, 0.2 * s, 0.6 * s)
const __defaultShadowBoxCenter = new Vector3(0 * s, 0.05 * s, 0.04 * s)

const ZERO = new Vector3()
export function addPrettyLights(scene: Scene, bgColor: Color) {
  const ambientLight = new HemisphereLight(
    __defaultSkyColor,
    __defaultGroundColor
  )
  scene.add(ambientLight)

  const sunLight = new DirectionalLight(0xffffaf, 0.75)
  scene.add(sunLight)

  sunLight.name = 'sunlight'
  sunLight.castShadow = true
  sunLight.shadow.camera.near = 0.01
  sunLight.shadow.camera.far = 1
  sunLight.shadow.camera.left = -0.5
  sunLight.shadow.camera.right = 0.5
  sunLight.shadow.camera.top = -0.3
  sunLight.shadow.camera.bottom = 0.3
  sunLight.shadow.camera.updateProjectionMatrix()
  sunLight.shadow.mapSize.width = sunLight.shadow.mapSize.height = 1024

  const distance = 0.5
  const angle = 0.6
  sunLight.position.set(
    Math.cos(angle) * distance,
    Math.sin(angle) * distance,
    0
  )
  sunLight.lookAt(ZERO)
  sunLight.updateMatrix()
  sunLight.updateMatrixWorld(true)

  const shadowCam = sunLight.shadow.camera
  shadowCam.updateMatrix()
  shadowCam.updateMatrixWorld(true)
  shadowCam.top = -Infinity
  shadowCam.bottom = Infinity
  shadowCam.left = Infinity
  shadowCam.right = -Infinity
  shadowCam.near = Infinity
  shadowCam.far = -Infinity
  const __tempVec = new Vector3()
  for (let i = 0; i < 8; i++) {
    __tempVec.x = (i % 2) - 0.5
    __tempVec.y = -((~~(i / 2) % 2) - 0.5)
    __tempVec.z = (~~(i / 4) % 2) - 0.5
    __tempVec.multiply(__defaultShadowBoxSize)
    __tempVec.sub(__defaultShadowBoxCenter)
    sunLight.worldToLocal(__tempVec)
    shadowCam.top = Math.max(shadowCam.top, -__tempVec.y)
    shadowCam.bottom = Math.min(shadowCam.bottom, -__tempVec.y)
    shadowCam.left = Math.min(shadowCam.left, __tempVec.x)
    shadowCam.right = Math.max(shadowCam.right, __tempVec.x)
    shadowCam.near = Math.min(shadowCam.near, __tempVec.z + 2 * distance)
    shadowCam.far = Math.max(shadowCam.far, __tempVec.z + 2 * distance)
  }

  // shadowCam.updateMatrix()
  // shadowCam.updateMatrixWorld(true)

  // sunLight.updateMatrix()
  // sunLight.updateMatrixWorld(true)
  // sunLight.shadow.matrix.copy(sunLight.matrixWorld)
  // shadowCam.matrix.copy(sunLight.matrixWorld)
  shadowCam.updateProjectionMatrix()

  if (getUrlFlag('debugLights')) {
    const sunLightHelper = new DirectionalLightHelper(sunLight)
    const sunLightCamHelper = new CameraHelper(sunLight.shadow.camera)
    scene.add(sunLightHelper)
    scene.add(sunLightCamHelper)
    // sunLightHelper.updateMatrix()
    // sunLightCamHelper.updateMatrix()
    // sunLightHelper.updateMatrixWorld(true)
    // sunLightCamHelper.updateMatrixWorld(true)

    sunLightHelper.update()
    sunLightCamHelper.update()
    // sunLightCamHelper.updateMatrix()
    // sunLightCamHelper.updateMatrixWorld(true)
  }

  scene.fog = new Fog(bgColor.getHex(), 0, 3)
}
