import { WebGLRenderer } from 'three';
import BaseTestScene from './BaseTestScene';
export default class TestLightingScene extends BaseTestScene {
    constructor();
    update(dt: number): void;
    render(renderer: WebGLRenderer, dt: number): void;
}
