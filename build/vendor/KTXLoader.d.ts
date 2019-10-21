import { CompressedTextureLoader, LoadingManager } from 'three';
/**
 * @author amakaseev / https://github.com/amakaseev
 *
 * for description see https://www.khronos.org/opengles/sdk/tools/KTX/
 * for file layout see https://www.khronos.org/opengles/sdk/tools/KTX/file_format_spec/
 *
 * ported from https://github.com/BabylonJS/Babylon.js/blob/master/src/Tools/babylon.khronosTextureContainer.ts
 */
export declare class KTXLoader extends CompressedTextureLoader {
    _parser: (buffer: ArrayBuffer | SharedArrayBuffer, loadMipmaps: boolean) => {
        mipmaps: {
            data: Uint8Array;
            width: number;
            height: number;
        }[];
        width: number;
        height: number;
        format: number;
        isCubemap: boolean;
        mipmapCount: number;
    };
    constructor(manager: LoadingManager);
    parse(buffer: ArrayBuffer | SharedArrayBuffer, loadMipmaps: boolean): {
        mipmaps: {
            data: Uint8Array;
            width: number;
            height: number;
        }[];
        width: number;
        height: number;
        format: number;
        isCubemap: boolean;
        mipmapCount: number;
    };
}
