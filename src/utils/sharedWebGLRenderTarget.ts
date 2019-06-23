import {
  Box2,
  BufferAttribute,
  Camera,
  GammaEncoding,
  LinearFilter,
  Mesh,
  MeshBasicMaterial,
  PlaneBufferGeometry,
  RepeatWrapping,
  RGBAFormat,
  Scene,
  Texture,
  TextureEncoding,
  UnsignedByteType,
  Vector2,
  Vector4,
  WebGLRenderer,
  WebGLRenderTarget,
  WebGLRenderTargetOptions
} from 'three'
import { maxTextureSize } from '~/renderer'

import { removeFromArray, replaceManyInArray } from './arrayUtils'

interface FixedWebGLRenderTargetOptions extends WebGLRenderTargetOptions {
  encoding: TextureEncoding //this is correct, but missing from types
  downsampleRatio?: number
}

interface UvOptions extends WebGLRenderTargetOptions {
  geometryUvAttribute?: BufferAttribute
  uvScaleTranslateUniform?: { value: Vector4 }
}

export const prototypePlaneGeometryUvAttribute = new PlaneBufferGeometry(
  1,
  1,
  1,
  1
).attributes.uv as BufferAttribute

const defaultRenderTargetOptions: Partial<FixedWebGLRenderTargetOptions> = {
  wrapS: RepeatWrapping,
  wrapT: RepeatWrapping,
  magFilter: LinearFilter,
  minFilter: LinearFilter,
  format: RGBAFormat,
  type: UnsignedByteType,
  anisotropy: 1,
  depthBuffer: false,
  stencilBuffer: false,
  generateMipmaps: false,
  encoding: GammaEncoding, //this is correct, but missing from types
  downsampleRatio: 1
}

const hashRenderTarget = (obj: any) => {
  const list = Object.keys(obj).map(key => key + ':' + obj[key])
  list.sort()
  return list.join('\n')
}

const __sharedRenderTargets = new Map<string, SharedRenderTarget>()

export function getSharedTexture(index: number): Texture | undefined {
  const len = __sharedRenderTargets.size
  if (len > 0) {
    return Array.from(__sharedRenderTargets.values())[index % len].texture
  } else {
    return undefined
  }
}

let previewMesh: Mesh
let previewMaterial: MeshBasicMaterial
let previewIndex = 0
export function getSharedTexturePreview(): Mesh {
  if (!previewMesh) {
    previewMaterial = new MeshBasicMaterial({
      map: getSharedTexture(previewIndex),
      transparent: true
    })
    previewMesh = new Mesh(new PlaneBufferGeometry(0.2, 0.2), previewMaterial)
    previewMesh.frustumCulled = false
    previewMesh.position.y += 0.1
    previewMesh.renderOrder = -100
  }
  return previewMesh
}

export function shiftPreviewTextureIndex(delta: number) {
  previewIndex += delta
  previewMaterial.map = getSharedTexture(previewIndex) || null
}
class SharedRenderTarget {
  renderTarget: WebGLRenderTarget
  downsampleRatio: number
  subRegions: RenderTargetSubRegion[]
  gridIndex: RenderTargetSubRegion[]
  maxSubRegionSize: number
  tileSize: number
  cols: number
  rows: number
  sharedWebGLTexturePadding: number
  constructor(options: FixedWebGLRenderTargetOptions) {
    this.maxSubRegionSize = maxTextureSize * 0.25
    this.tileSize = 128
    this.cols = maxTextureSize / this.tileSize
    this.rows = maxTextureSize / this.tileSize
    this.sharedWebGLTexturePadding = 2

    this.downsampleRatio = options.downsampleRatio!
    const res = maxTextureSize * this.downsampleRatio
    this.renderTarget = new WebGLRenderTarget(res, res, options)
    this.renderTarget.texture.encoding = options.encoding || GammaEncoding
    this.subRegions = []
    this.gridIndex = new Array(Math.pow(maxTextureSize / this.tileSize, 2))
  }
  register(subRegion: RenderTargetSubRegion) {
    const tilesX = subRegion.tilesX
    const tilesY = subRegion.tilesY
    const tiles = tilesX * tilesY
    let openingIndex = -1
    if (tiles > 0) {
      lookForOpening: for (
        let index = 0;
        index < this.gridIndex.length;
        index++
      ) {
        const tooWide = (index % this.cols) + tilesX > this.cols
        if (this.gridIndex[index] || tooWide) {
          continue lookForOpening
        }
        if (tiles > 1) {
          for (let subIndex = 1; subIndex < tiles; subIndex++) {
            const col = (index % this.cols) + (subIndex % tilesX)
            const row =
              (~~(index / this.cols) + ~~(subIndex / tilesX)) % this.rows
            const finalIndex = row * this.cols + col
            if (this.gridIndex[finalIndex]) {
              continue lookForOpening
            }
          }
        }
        openingIndex = index
        break
      }
    } else {
      openingIndex = 0
    }

    if (openingIndex !== -1) {
      subRegion.updateUvRegion(openingIndex, tilesX, tilesY)
      if (tiles === 1) {
        this.gridIndex[openingIndex] = subRegion
      } else if (tiles > 1) {
        for (let subIndex = 0; subIndex < tiles; subIndex++) {
          const col =
            ((openingIndex % this.cols) + (subIndex % tilesX)) % this.cols
          const row =
            (~~(openingIndex / this.cols) + ~~(subIndex / tilesX)) % this.rows
          const finalIndex = row * this.cols + col
          this.gridIndex[finalIndex] = subRegion
        }
        // } else {
        // throw new Error("How can a subregion be less than 1 tile?")
      }
      this.subRegions.push(subRegion)
    } else {
      throw new Error('Failed to register subregion')
    }
  }
  unregister(subRegion: RenderTargetSubRegion) {
    removeFromArray(this.subRegions, subRegion)
    replaceManyInArray(this.gridIndex, subRegion, undefined)
  }
  updateRegistration(subRegion: RenderTargetSubRegion) {
    this.unregister(subRegion)
    this.register(subRegion)
  }
  getPixelSize() {
    return new Vector2(
      1 / this.renderTarget.width,
      1 / this.renderTarget.height
    )
  }
  get texture() {
    return this.renderTarget.texture
  }
}

const vec2 = new Vector2()
let __ID_SOURCE = 0
export class RenderTargetSubRegion {
  id: number
  sharedRenderTarget: SharedRenderTarget
  geometryUvAttribute?: BufferAttribute
  uvScaleTranslateUniform?: { value: Vector4 }
  tilesX: number
  tilesY: number
  uvRegion: Box2
  pixelSize: Vector2
  constructor(
    options: FixedWebGLRenderTargetOptions,
    uvOptions: UvOptions = {}
  ) {
    options = {
      ...defaultRenderTargetOptions,
      ...options
    }
    this.id = __ID_SOURCE++
    this.geometryUvAttribute = uvOptions.geometryUvAttribute
    this.uvScaleTranslateUniform = uvOptions.uvScaleTranslateUniform
    this.tilesX = 0
    this.tilesY = 0
    const key = hashRenderTarget(options)
    if (!__sharedRenderTargets.has(key)) {
      __sharedRenderTargets.set(key, new SharedRenderTarget(options))
    }
    this.sharedRenderTarget = __sharedRenderTargets.get(key)!
    this.pixelSize = this.sharedRenderTarget.getPixelSize()
    this.uvRegion = new Box2()
    this.sharedRenderTarget.register(this)
  }
  render(
    renderer: WebGLRenderer,
    scene: Scene,
    camera: Camera,
    width: number,
    height: number,
    forceClear?: boolean
  ) {
    if (width > this.sharedRenderTarget.maxSubRegionSize) {
      height *= this.sharedRenderTarget.maxSubRegionSize / width
      width = this.sharedRenderTarget.maxSubRegionSize
    }
    if (height > this.sharedRenderTarget.maxSubRegionSize) {
      width *= this.sharedRenderTarget.maxSubRegionSize / height
      height = this.sharedRenderTarget.maxSubRegionSize
    }
    const newTilesX = Math.ceil(width / this.sharedRenderTarget.tileSize)
    const newTilesY = Math.ceil(height / this.sharedRenderTarget.tileSize)
    if (this.tilesX !== newTilesX || this.tilesY !== newTilesY) {
      this.tilesX = newTilesX
      this.tilesY = newTilesY
      this.sharedRenderTarget.updateRegistration(this)
      this.updateUvs()
    }
    const min = this.uvRegion.min
    const max = this.uvRegion.max
    const res = maxTextureSize * this.sharedRenderTarget.downsampleRatio
    const x = res * min.x
    const y = res * min.y
    const w = res * (max.x - min.x)
    const h = res * (max.y - min.y)

    this.renderTarget.viewport.set(x, y, w, h)
    this.renderTarget.scissor.set(x, y, w, h)
    this.renderTarget.scissorTest = true
    renderer.setRenderTarget(this.renderTarget)
    renderer.render(scene, camera)
  }
  updateUvRegion(index: number, tilesX: number, tilesY: number) {
    const min = this.uvRegion.min
    const max = this.uvRegion.max
    min.x =
      (index % this.sharedRenderTarget.cols) / this.sharedRenderTarget.cols
    min.y =
      ~~(index / this.sharedRenderTarget.cols) / this.sharedRenderTarget.rows
    max.copy(min)
    max.x += tilesX / this.sharedRenderTarget.cols
    max.y += tilesY / this.sharedRenderTarget.rows
  }
  updateUvs() {
    const min = this.uvRegion.min
    const range = vec2.copy(this.uvRegion.max).sub(min)
    const pw =
      this.pixelSize.x * this.sharedRenderTarget.sharedWebGLTexturePadding
    const ph =
      this.pixelSize.y * this.sharedRenderTarget.sharedWebGLTexturePadding
    if (this.geometryUvAttribute) {
      const uv = this.geometryUvAttribute
      const arr = uv.array as Float32Array
      const prototypeArr = prototypePlaneGeometryUvAttribute.array
      for (let i2 = 0; i2 < arr.length; i2 += 2) {
        arr[i2] = min.x + pw + prototypeArr[i2] * (range.x - pw * 2)
        arr[i2 + 1] = min.y + ph + prototypeArr[i2 + 1] * (range.y - ph * 2)
      }
      uv.needsUpdate = true
    }
    if (this.uvScaleTranslateUniform) {
      this.uvScaleTranslateUniform.value.set(
        range.x - pw * 2,
        range.y - ph * 2,
        min.x + pw,
        min.y + ph
      )
      // this.uvScaleTranslateUniform.value.set(range.x, range.y, min.x, min.y)
    }
  }
  dispose() {
    this.geometryUvAttribute = undefined
    this.uvScaleTranslateUniform = undefined
    this.sharedRenderTarget.unregister(this)
  }
  get texture() {
    return this.sharedRenderTarget.texture
  }
  get renderTarget() {
    return this.sharedRenderTarget.renderTarget
  }
}
