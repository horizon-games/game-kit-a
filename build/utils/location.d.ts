import { Color } from 'three';
export declare function getUrlParam(param: string): string | null;
export declare function getUrlFlag(param: string): boolean;
export declare function getUrlFloat(param: string, defaultVal: number, min?: number, max?: number): number;
export declare function getUrlInt(param: string, defaultVal: number, min?: number, max?: number): number;
export declare function getUrlColor(param: string, defaultColor: string | Color): Color;
