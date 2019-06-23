/// <reference types="three" />
declare module 'three-bmfont-text' {
  interface TextGeometryOptions extends LayoutOptions {
    flipY?: boolean
    multipage?: boolean
  }
  class TextGeometry extends THREE.BufferGeometry {
    update: (opt: Partial<TextGeometryOptions> | string) => void
    layout: Layout
    visibleGlyphs: Layout['glyphs']
  }
  function createText(opt: TextGeometryOptions): TextGeometry
  export default createText
}
