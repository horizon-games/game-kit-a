import { Plane, Ray, Raycaster, Vector2, Vector3 } from 'three';
import device from '../device';
export function clamp(val, min, max) {
    return Math.min(max, Math.max(min, val));
}
export function absFloor(val) {
    return Math.floor(Math.abs(val)) * (val < 0 ? -1 : 1);
}
const tiny = 0.00001;
export function closeEnough(val, val2) {
    return Math.abs(val - val2) < tiny;
}
export const TWO_PI = 2 * Math.PI;
export const RADIANS_TO_DEGREES = 180 / Math.PI;
export const DEGREES_TO_RADIANS = Math.PI / 180;
export function radiansToDegrees(radians) {
    return radians * RADIANS_TO_DEGREES;
}
export function degreesToRadians(degrees) {
    return degrees * DEGREES_TO_RADIANS;
}
const ray = new Ray();
const flatPlane = new Plane(new Vector3(0, -1, 0), 1);
const anyPlane = new Plane(new Vector3(0, -1, 0), 1);
const intersection = new Vector3();
// const __cameraPosition = new Vector3()
export function get2DPositionOnPlane(camera, cameraWorldPos, x, y, plane) {
    // __cameraPosition.set(0, 0, 0)
    // camera.localToWorld(__cameraPosition)
    ray.origin.copy(cameraWorldPos);
    ray.direction
        .set(x, y, 0.5)
        .unproject(camera)
        .sub(cameraWorldPos)
        .normalize();
    ray.intersectPlane(plane, intersection);
    return intersection;
}
export function get2DPositionAtDepth(camera, cameraWorldPos, x, y, atDepth = 0) {
    flatPlane.constant = atDepth;
    return get2DPositionOnPlane(camera, cameraWorldPos, x, y, flatPlane);
}
export function get2DPositionOnPlaneHelper(camera, cameraWorldPos, x, y, coPlanarPoint, normal) {
    anyPlane.setFromNormalAndCoplanarPoint(normal, coPlanarPoint);
    return get2DPositionOnPlane(camera, cameraWorldPos, x, y, anyPlane);
}
export function getPixelOnGroundPlane(camera, cameraWorldPos, x, y, depth = 0) {
    return get2DPositionAtDepth(camera, cameraWorldPos, (x / device.width) * 2 - 1, -(y / device.height) * 2 + 1, depth);
}
const __v2 = new Vector2();
const __intersections = [];
const __raycaster = new Raycaster();
let __hitTesting = false;
export function hitTestAtPixel(x, y, items, reaction, camera) {
    if (__hitTesting) {
        throw new Error('recursive hit testing not allowed');
    }
    __hitTesting = true;
    //work in clipspace coordinates (-1 to 1)
    __v2.set((x / device.width) * 2 - 1, -(y / device.height) * 2 + 1);
    __raycaster.setFromCamera(__v2, camera);
    __raycaster.intersectObjects(items, false, __intersections);
    for (const intersection of __intersections) {
        if (reaction(intersection.object, intersection.point)) {
            break;
        }
    }
    __intersections.length = 0;
    __hitTesting = false;
}
export function lerp(a, b, dt) {
    const out = a + dt * (b - a);
    return Math.abs(b - out) > 0.00001 ? out : b;
}
export function unlerp(min, max, value) {
    return (value - min) / (max - min);
}
export function unlerpClamped(min, max, value) {
    return clamp(unlerp(min, max, value), 0, 1);
}
export function degreesDifference(A, B) {
    return ((((A - B) % 360) + 540) % 360) - 180;
}
const tau = Math.PI * 2;
const tauAndHalf = Math.PI * 3;
export function radiansDifference(a, b) {
    return ((((a - b) % tau) + tauAndHalf) % tau) - Math.PI;
}
export function rand(min = 0, max = 1) {
    return Math.random() * (max - min) + min;
}
export function rand2(scale = 1, offset = 0) {
    return (Math.random() * 2 - 1) * scale + offset;
}
export function nextHighestPowerOfTwo(val) {
    return Math.pow(Math.ceil(Math.sqrt(val)), 2);
}
export function inferDirection(val, tolerance = 0.00001) {
    if (val < -tolerance) {
        return -1;
    }
    else if (val > tolerance) {
        return 1;
    }
    else {
        return 0;
    }
}
export function sqr(v) {
    return v * v;
}
export function pixelLengthOnScreen(a, b, camera) {
    a.project(camera);
    b.project(camera);
    return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
}
//# sourceMappingURL=math.js.map