import { Color, MeshBasicMaterial } from 'three';
export declare function addColor(dst: Color, src: Color, amt: number): void;
export declare function screenColor(dst: Color, src: Color): void;
export declare function createMaterial(col: Color | string | number): MeshBasicMaterial;
export declare function makeHSL(h: number, s?: number, l?: number): Color;
export declare function hexColor(style: string): Color;
