import { PCFShadowMap, WebGLRenderer } from 'three'
import { RESET_USER_SETTINGS_TO_DEFAULTS } from '~/constants'

import device from './device'
import {
  devicePixelRatioUniform,
  pixelSizeInClipSpaceUniform
} from './uniforms'
import { NiceParameter } from './utils/NiceParameter'

const canvas = document.createElement('canvas')
const context = canvas.getContext('webgl') as WebGLRenderingContext
const renderer = new WebGLRenderer({
  canvas,
  context,
  antialias: true,
  premultipliedAlpha: false
  // powerPreference: "high-performance"
  // powerPreference: "low-power"
})
document.body.append(canvas)
const attributeValues: string[] = [
  '-moz-crisp-edges',
  '-webkit-crisp-edges',
  'pixelated',
  'crisp-edges'
]

const canvas3D = renderer.getContext().canvas
if (canvas3D instanceof HTMLCanvasElement) {
  for (const v of attributeValues) {
    canvas3D.style.setProperty('image-rendering', v)
  }
}
renderer.shadowMap.enabled = true
renderer.shadowMap.type = PCFShadowMap
renderer.gammaOutput = true
renderer.gammaFactor = 2.2
renderer.autoClear = false

const downsample = new NiceParameter(
  'pixel-down-sample',
  'Graphics Quality',
  3,
  1,
  3,
  v => v,
  v => {
    switch (v) {
      case 1:
        return 'Low'
      case 2:
        return 'Medium'
      case 3:
        return 'High'
      default:
        return 'High'
    }
  },
  true,
  RESET_USER_SETTINGS_TO_DEFAULTS,
  1
)

let __downsample = 1
function updatePixelRatio() {
  const pixelRatio = device.pixelRatio / (5 - __downsample)
  devicePixelRatioUniform.value = pixelRatio
  renderer.setPixelRatio(pixelRatio)
}

downsample.listen(downsample => {
  __downsample = Math.round(downsample + 1)
  updatePixelRatio()
})

device.onChange(() => {
  updatePixelRatio()
  const { width, height } = device
  renderer.setSize(width, height)
  devicePixelRatioUniform.value = device.pixelRatio
  pixelSizeInClipSpaceUniform.value.set(2 / width, 2 / height)
}, true)
export const maxTextureSize = Math.min(
  8192,
  renderer.capabilities.maxTextureSize
)

export default renderer
