import { Color, MeshBasicMaterial } from 'three'
import { rand } from '~/utils/math'

const __whiteColor = new Color(1, 1, 1)
const __tempColor = new Color()

export function addColor(dst: Color, src: Color, amt: number) {
  if (amt === 0) {
    return
  }
  __tempColor.copy(src).multiplyScalar(amt)
  dst.add(__tempColor)
}

export function screenColor(dst: Color, src: Color) {
  __tempColor.copy(__whiteColor).sub(dst)
  dst.add(__tempColor.multiply(src))
}

export function createMaterial(col: Color | string | number) {
  const color = new Color(col)
  const hsl = { h: 0, s: 0, l: 0 }
  color.getHSL(hsl)
  hsl.h += rand(-0.025, 0.025)
  hsl.s += rand(-0.05, 0.05)
  hsl.l += rand(-0.05, 0.05)
  color.setHSL(hsl.h, hsl.s, hsl.l)
  const mat = new MeshBasicMaterial({
    color,
    transparent: true,
    opacity: 0.2,
    depthWrite: false
  })
  mat.color = color
  return mat
}

export function makeHSL(h: number, s: number = 0.75, l: number = 0.5) {
  return new Color().setHSL(h, s, l)
}

//color style algorithm extracted from three.js
export function hexColor(style: string) {
  const m = /^\#([A-Fa-f0-9]+)$/.exec(style)
  if (m) {
    // hex color

    const hex = m[1]
    const size = hex.length

    if (size === 3) {
      const color = new Color()
      // #ff0
      color.r = parseInt(hex.charAt(0) + hex.charAt(0), 16) / 255
      color.g = parseInt(hex.charAt(1) + hex.charAt(1), 16) / 255
      color.b = parseInt(hex.charAt(2) + hex.charAt(2), 16) / 255

      return color
    } else if (size === 6) {
      const color = new Color()
      // #ff0000
      color.r = parseInt(hex.charAt(0) + hex.charAt(1), 16) / 255
      color.g = parseInt(hex.charAt(2) + hex.charAt(3), 16) / 255
      color.b = parseInt(hex.charAt(4) + hex.charAt(5), 16) / 255

      return color
    }
  }
  return new Color(1, 0, 1)
}
