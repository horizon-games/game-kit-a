import { Box2, BufferAttribute, Camera, Mesh, Scene, Texture, TextureEncoding, Vector2, Vector4, WebGLRenderer, WebGLRenderTarget, WebGLRenderTargetOptions } from 'three';
interface FixedWebGLRenderTargetOptions extends WebGLRenderTargetOptions {
    encoding: TextureEncoding;
    downsampleRatio?: number;
}
interface UvOptions extends WebGLRenderTargetOptions {
    geometryUvAttribute?: BufferAttribute;
    uvScaleTranslateUniform?: {
        value: Vector4;
    };
}
export declare const prototypePlaneGeometryUvAttribute: BufferAttribute;
export declare function getSharedTexture(index: number): Texture | undefined;
export declare function getSharedTexturePreview(): Mesh;
export declare function shiftPreviewTextureIndex(delta: number): void;
declare class SharedRenderTarget {
    renderTarget: WebGLRenderTarget;
    downsampleRatio: number;
    subRegions: RenderTargetSubRegion[];
    gridIndex: RenderTargetSubRegion[];
    maxSubRegionSize: number;
    tileSize: number;
    cols: number;
    rows: number;
    sharedWebGLTexturePadding: number;
    constructor(options: FixedWebGLRenderTargetOptions);
    register(subRegion: RenderTargetSubRegion): void;
    unregister(subRegion: RenderTargetSubRegion): void;
    updateRegistration(subRegion: RenderTargetSubRegion): void;
    getPixelSize(): Vector2;
    readonly texture: Texture;
}
export declare class RenderTargetSubRegion {
    id: number;
    sharedRenderTarget: SharedRenderTarget;
    geometryUvAttribute?: BufferAttribute;
    uvScaleTranslateUniform?: {
        value: Vector4;
    };
    tilesX: number;
    tilesY: number;
    uvRegion: Box2;
    pixelSize: Vector2;
    constructor(options: FixedWebGLRenderTargetOptions, uvOptions?: UvOptions);
    render(renderer: WebGLRenderer, scene: Scene, camera: Camera, width: number, height: number, forceClear?: boolean): void;
    updateUvRegion(index: number, tilesX: number, tilesY: number): void;
    updateUvs(): void;
    dispose(): void;
    readonly texture: Texture;
    readonly renderTarget: WebGLRenderTarget;
}
export {};
