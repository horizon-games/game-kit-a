export declare const scaleValuesInArray: (arr: number[], scale: number) => void;
export declare const addToArrayUnique: <T>(arr: T[], value: T) => void;
export declare const removeFromArray: <T>(arr: T[], value: T, strict?: boolean) => T;
export declare const moveBetweenArrays: <T>(src: T[], dst: T[], value: T) => T;
export declare const replaceManyInArray: <T>(arr: (T | undefined)[], value: T, replacement?: T | undefined) => void;
export declare function getArrayDiffs<T>(oldArr: T[], newArr: T[]): {
    added: T[];
    removed: T[];
    equal: T[];
};
export declare const pushToArrayMap: <T, T2>(map: Map<T, T2[]>, key: T, value: T2, oneCopyMax?: boolean) => void;
export declare const cleanRemoveFromArrayMap: <T, T2>(map: Map<T, T2[]>, key: T, value: T2) => void;
export declare function findClosestNumberIndex(arr: number[], value: number): number;
