import { Color, Fog, PerspectiveCamera, Scene } from 'three';
import device from '../../device';
import { getUrlColor } from '../../utils/location';
const FOV = 35;
const MOBILE_FOV = 28;
export default class BaseTestScene {
    constructor() {
        const scene = new Scene();
        const bgColor = getUrlColor('bgColor', new Color(0x6f84bc));
        scene.fog = new Fog(bgColor.getHex(), 0, 6);
        scene.autoUpdate = false;
        scene.matrixAutoUpdate = false;
        const camera = new PerspectiveCamera(device.isMobile ? MOBILE_FOV : FOV, device.aspect, 0.01, 10);
        device.onChange(() => {
            camera.fov = device.isMobile ? MOBILE_FOV : FOV;
            camera.aspect = device.aspect;
            camera.updateProjectionMatrix();
        }, true);
        camera.position.set(0, 0.2, 0.4);
        camera.lookAt(0, 0, 0);
        camera.updateProjectionMatrix();
        scene.add(camera);
        this.scene = scene;
        this.camera = camera;
        this.bgColor = bgColor;
    }
    update(dt) {
        this.scene.updateMatrixWorld(false);
    }
    render(renderer, dt) {
        renderer.setClearColor(this.bgColor, 1);
        renderer.clear(true);
        renderer.render(this.scene, this.camera);
    }
}
//# sourceMappingURL=BaseTestScene.js.map