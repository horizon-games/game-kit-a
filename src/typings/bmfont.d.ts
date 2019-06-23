declare interface Char {
  id: number
  x: number
  y: number
  width: number
  height: number
  xoffset: number
  yoffset: number
  xadvance: number
  page: number
  chnl: number
}

declare interface Kerning {
  first: number
  second: number
  amount: number
}

declare interface BMFont {
  pages: string[]
  chars: Char[]
  kernings: Kerning[]
  info: {
    face: string
    size: number
    bold: number
    italic: number
    charset: string
    unicode: number
    stretchH: number
    smooth: number
    aa: number
    padding: [number, number, number, number]
    spacing: [number, number]
  }
  common: {
    lineHeight: number
    base: number
    scaleW: number
    scaleH: number
    pages: number
    packed: number
    alphaChnl: number
    redChnl: number
    greenChnl: number
    blueChnl: number
  }
}
