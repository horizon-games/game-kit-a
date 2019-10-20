import { AdditiveBlending } from 'three';
import { Easing } from '~/animation/Easing';
import { AnimatedBool } from '~/utils/AnimatedBool';
import { createMaterial } from '~/utils/colors';
import { NoduleSphere } from './NoduleSphere';
export class Explosion extends NoduleSphere {
    constructor(_radius, nodules = 30, depth = 4, size = 0) {
        super(createMaterial(0xe7c02a), nodules, depth);
        this._radius = _radius;
        this.explodingState = new AnimatedBool(v => this.updateExplosion(v), false, 15000, Easing.Exponential.Out);
        this.mat = this.material;
        this.mat.blending = AdditiveBlending;
        this.scale.set(_radius, _radius, _radius);
        if (size > 0) {
            this.explode(size);
        }
    }
    explode(size) {
        this._explosionSize = size;
        this.explodingState.value = true;
    }
    updateExplosion(v) {
        const s = this._radius * (1 + v * this._explosionSize);
        this.scale.set(s, s, s);
        this.mat.opacity = 1 - v;
        if (v === 1 && this.parent) {
            this.parent.remove(this);
        }
    }
}
//# sourceMappingURL=Explosion.js.map