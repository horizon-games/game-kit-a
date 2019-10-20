import { Camera, Color, Scene, WebGLRenderer } from 'three';
export default class BaseTestScene {
    protected scene: Scene;
    protected camera: Camera;
    protected bgColor: Color;
    constructor();
    update(dt: number): void;
    render(renderer: WebGLRenderer, dt: number): void;
}
