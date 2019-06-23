interface LayoutOptions {
  font: BMFont
  text?: string
  width?: number
  mode?: 'pre' | 'nowrap'
  align?: 'left' | 'center' | 'right'
  letterSpacing?: number
  lineHeight?: number
  tabSize?: number
  start?: number
  end?: number
}
interface Layout {
  update: (opt: LayoutOptions) => void
  glyphs: Glyph[]
  width: number
  height: number
  baseline: number
  xHeight: number
  descender: number
  ascender: number
  capHeight: number
  lineHeight: number
  _linesTotal: number
}
interface Glyph {
  index: number
  data: Char
  position: [number, number]
  line: number
}
declare module 'layout-bmfont-text' {
  export default function createLayout(options: LayoutOptions): Layout
}
