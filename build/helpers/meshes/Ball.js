import { Mesh, MeshBasicMaterial, SphereBufferGeometry } from 'three';
import { Easing } from '../../animation/Easing';
import { AnimatedBool } from '../../utils/AnimatedBool';
import { createMaterial } from '../../utils/colors';
import { NoduleSphere } from './NoduleSphere';
export class Ball extends NoduleSphere {
    constructor(_radius, useCustomCollider = false, nodules = 30, depth = 4) {
        super(createMaterial(0x0c152d), nodules, depth);
        this._radius = _radius;
        this.explodingState = new AnimatedBool(v => this.updateExplosion(v), false, 0, Easing.Exponential.Out, 1000);
        this.validTargets = [];
        const opaque = this.material.clone();
        opaque.transparent = false;
        opaque.depthWrite = true;
        this.color = this.material.color;
        opaque.color = this.color;
        this.material = opaque;
        this.scale.set(_radius, _radius, _radius);
        if (useCustomCollider) {
            this.collider = new Mesh(new SphereBufferGeometry(0.8, 6, 4), new MeshBasicMaterial({
                color: 0xff0000,
                wireframe: true,
                depthTest: false,
                depthWrite: false,
                transparent: true
            }));
            this.add(this.collider);
        }
        else {
            this.collider = this;
        }
    }
    explode(size) {
        this._explosionSize = size;
        this.explodingState.pulse();
    }
    updateExplosion(v) {
        const s = this._radius * (1 + v * this._explosionSize);
        this.scale.set(s, s, s);
    }
}
//# sourceMappingURL=Ball.js.map