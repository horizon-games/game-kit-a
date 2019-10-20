import { absFloor, clamp } from './math';
export const scaleValuesInArray = (arr, scale) => {
    for (let i = 0; i < arr.length; i++) {
        arr[i] *= scale;
    }
};
export const addToArrayUnique = (arr, value) => {
    const index = arr.indexOf(value);
    if (index === -1) {
        arr.push(value);
    }
};
export const removeFromArray = (arr, value, strict = false) => {
    const index = arr.indexOf(value);
    if (index !== -1) {
        arr.splice(index, 1);
    }
    else if (strict) {
        throw new Error('could not find value in array');
    }
    return value;
};
export const moveBetweenArrays = (src, dst, value) => {
    dst.push(removeFromArray(src, value));
    return value;
};
export const replaceManyInArray = (arr, value, replacement) => {
    if (value === replacement) {
        throw new Error('Nope. This would cause an infinite loop');
    }
    let index = arr.indexOf(value);
    while (index !== -1) {
        arr[index] = replacement;
        index = arr.indexOf(value);
    }
};
export function getArrayDiffs(oldArr, newArr) {
    const added = newArr.filter(item => !oldArr.includes(item));
    const removed = oldArr.filter(item => !newArr.includes(item));
    const equal = newArr.filter(item => oldArr.includes(item));
    return {
        added,
        removed,
        equal
    };
}
export const pushToArrayMap = (map, key, value, oneCopyMax = false) => {
    if (!map.has(key)) {
        map.set(key, []);
    }
    const arr = map.get(key);
    if (arr) {
        if (oneCopyMax) {
            if (arr.indexOf(value) === -1) {
                arr.push(value);
            }
        }
        else {
            arr.push(value);
        }
    }
};
export const cleanRemoveFromArrayMap = (map, key, value) => {
    if (!map.has(key)) {
        return;
    }
    const arr = map.get(key);
    if (arr) {
        removeFromArray(arr, value);
        if (arr.length === 0) {
            map.delete(key);
        }
    }
};
//binary search only works assuming numbers have been sorted from lowest to highest
export function findClosestNumberIndex(arr, value) {
    const middleIndex = ~~(arr.length * 0.5);
    let step = value > arr[middleIndex] ? 1 : -1;
    let index = middleIndex;
    let oldSample = arr[index];
    let everTurned = false;
    function directionMatches(val, val2) {
        return (val > 0 && val2 > 0) || (val < 0 && val2 < 0);
    }
    let limit = 100;
    while (step !== 0 && limit > 0) {
        index = clamp(index + step, 0, arr.length - 1);
        const newSample = arr[index];
        if (!directionMatches(value - newSample, value - oldSample)) {
            step *= -1;
            everTurned = true;
        }
        step = absFloor(step * (everTurned ? 0.5 : 2));
        oldSample = newSample;
        limit--;
    }
    return index;
}
//# sourceMappingURL=arrayUtils.js.map