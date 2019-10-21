/*
 * Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
// DEBUG: import { Assert } from "../Common/Settings";
import { Abs, Max, Rot, Sweep, Transform, Vec2 } from '../Common/Math';
import { linearSlop, maxPolygonVertices } from '../Common/Settings';
import { Timer } from '../Common/Timer';
import { Distance, DistanceInput, DistanceOutput, DistanceProxy, SimplexCache } from './Distance';
export let toiTime = 0;
export let toiMaxTime = 0;
export let toiCalls = 0;
export let toiIters = 0;
export let toiMaxIters = 0;
export let toiRootIters = 0;
export let toiMaxRootIters = 0;
export function _toi_reset() {
    toiTime = 0;
    toiMaxTime = 0;
    toiCalls = 0;
    toiIters = 0;
    toiMaxIters = 0;
    toiRootIters = 0;
    toiMaxRootIters = 0;
}
const TimeOfImpact_s_xfA = new Transform();
const TimeOfImpact_s_xfB = new Transform();
const TimeOfImpact_s_pointA = new Vec2();
const TimeOfImpact_s_pointB = new Vec2();
const TimeOfImpact_s_normal = new Vec2();
const TimeOfImpact_s_axisA = new Vec2();
const TimeOfImpact_s_axisB = new Vec2();
/// Input parameters for TimeOfImpact
export class TOIInput {
    constructor() {
        this.proxyA = new DistanceProxy();
        this.proxyB = new DistanceProxy();
        this.sweepA = new Sweep();
        this.sweepB = new Sweep();
        this.tMax = 0; // defines sweep interval [0, tMax]
    }
}
/// Output parameters for TimeOfImpact.
export var TOIOutputState;
(function (TOIOutputState) {
    TOIOutputState[TOIOutputState["e_unknown"] = 0] = "e_unknown";
    TOIOutputState[TOIOutputState["e_failed"] = 1] = "e_failed";
    TOIOutputState[TOIOutputState["e_overlapped"] = 2] = "e_overlapped";
    TOIOutputState[TOIOutputState["e_touching"] = 3] = "e_touching";
    TOIOutputState[TOIOutputState["e_separated"] = 4] = "e_separated";
})(TOIOutputState || (TOIOutputState = {}));
export class TOIOutput {
    constructor() {
        this.state = TOIOutputState.e_unknown;
        this.t = 0;
    }
}
export var SeparationFunctionType;
(function (SeparationFunctionType) {
    SeparationFunctionType[SeparationFunctionType["e_unknown"] = -1] = "e_unknown";
    SeparationFunctionType[SeparationFunctionType["e_points"] = 0] = "e_points";
    SeparationFunctionType[SeparationFunctionType["e_faceA"] = 1] = "e_faceA";
    SeparationFunctionType[SeparationFunctionType["e_faceB"] = 2] = "e_faceB";
})(SeparationFunctionType || (SeparationFunctionType = {}));
export class SeparationFunction {
    constructor() {
        this.m_sweepA = new Sweep();
        this.m_sweepB = new Sweep();
        this.m_type = SeparationFunctionType.e_unknown;
        this.m_localPoint = new Vec2();
        this.m_axis = new Vec2();
    }
    Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1) {
        this.m_proxyA = proxyA;
        this.m_proxyB = proxyB;
        const count = cache.count;
        // DEBUG: Assert(0 < count && count < 3);
        this.m_sweepA.Copy(sweepA);
        this.m_sweepB.Copy(sweepB);
        const xfA = TimeOfImpact_s_xfA;
        const xfB = TimeOfImpact_s_xfB;
        this.m_sweepA.GetTransform(xfA, t1);
        this.m_sweepB.GetTransform(xfB, t1);
        if (count === 1) {
            this.m_type = SeparationFunctionType.e_points;
            const localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            const localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
            const pointA = Transform.MulXV(xfA, localPointA, TimeOfImpact_s_pointA);
            const pointB = Transform.MulXV(xfB, localPointB, TimeOfImpact_s_pointB);
            Vec2.SubVV(pointB, pointA, this.m_axis);
            const s = this.m_axis.Normalize();
            // #if ENABLE_PARTICLE
            this.m_localPoint.SetZero();
            // #endif
            return s;
        }
        else if (cache.indexA[0] === cache.indexA[1]) {
            // Two points on B and one on A.
            this.m_type = SeparationFunctionType.e_faceB;
            const localPointB1 = this.m_proxyB.GetVertex(cache.indexB[0]);
            const localPointB2 = this.m_proxyB.GetVertex(cache.indexB[1]);
            Vec2.CrossVOne(Vec2.SubVV(localPointB2, localPointB1, Vec2.s_t0), this.m_axis).SelfNormalize();
            const normal = Rot.MulRV(xfB.q, this.m_axis, TimeOfImpact_s_normal);
            Vec2.MidVV(localPointB1, localPointB2, this.m_localPoint);
            const pointB = Transform.MulXV(xfB, this.m_localPoint, TimeOfImpact_s_pointB);
            const localPointA = this.m_proxyA.GetVertex(cache.indexA[0]);
            const pointA = Transform.MulXV(xfA, localPointA, TimeOfImpact_s_pointA);
            let s = Vec2.DotVV(Vec2.SubVV(pointA, pointB, Vec2.s_t0), normal);
            if (s < 0) {
                this.m_axis.SelfNeg();
                s = -s;
            }
            return s;
        }
        else {
            // Two points on A and one or two points on B.
            this.m_type = SeparationFunctionType.e_faceA;
            const localPointA1 = this.m_proxyA.GetVertex(cache.indexA[0]);
            const localPointA2 = this.m_proxyA.GetVertex(cache.indexA[1]);
            Vec2.CrossVOne(Vec2.SubVV(localPointA2, localPointA1, Vec2.s_t0), this.m_axis).SelfNormalize();
            const normal = Rot.MulRV(xfA.q, this.m_axis, TimeOfImpact_s_normal);
            Vec2.MidVV(localPointA1, localPointA2, this.m_localPoint);
            const pointA = Transform.MulXV(xfA, this.m_localPoint, TimeOfImpact_s_pointA);
            const localPointB = this.m_proxyB.GetVertex(cache.indexB[0]);
            const pointB = Transform.MulXV(xfB, localPointB, TimeOfImpact_s_pointB);
            let s = Vec2.DotVV(Vec2.SubVV(pointB, pointA, Vec2.s_t0), normal);
            if (s < 0) {
                this.m_axis.SelfNeg();
                s = -s;
            }
            return s;
        }
    }
    FindMinSeparation(indexA, indexB, t) {
        const xfA = TimeOfImpact_s_xfA;
        const xfB = TimeOfImpact_s_xfB;
        this.m_sweepA.GetTransform(xfA, t);
        this.m_sweepB.GetTransform(xfB, t);
        switch (this.m_type) {
            case SeparationFunctionType.e_points: {
                const axisA = Rot.MulTRV(xfA.q, this.m_axis, TimeOfImpact_s_axisA);
                const axisB = Rot.MulTRV(xfB.q, Vec2.NegV(this.m_axis, Vec2.s_t0), TimeOfImpact_s_axisB);
                indexA[0] = this.m_proxyA.GetSupport(axisA);
                indexB[0] = this.m_proxyB.GetSupport(axisB);
                const localPointA = this.m_proxyA.GetVertex(indexA[0]);
                const localPointB = this.m_proxyB.GetVertex(indexB[0]);
                const pointA = Transform.MulXV(xfA, localPointA, TimeOfImpact_s_pointA);
                const pointB = Transform.MulXV(xfB, localPointB, TimeOfImpact_s_pointB);
                const separation = Vec2.DotVV(Vec2.SubVV(pointB, pointA, Vec2.s_t0), this.m_axis);
                return separation;
            }
            case SeparationFunctionType.e_faceA: {
                const normal = Rot.MulRV(xfA.q, this.m_axis, TimeOfImpact_s_normal);
                const pointA = Transform.MulXV(xfA, this.m_localPoint, TimeOfImpact_s_pointA);
                const axisB = Rot.MulTRV(xfB.q, Vec2.NegV(normal, Vec2.s_t0), TimeOfImpact_s_axisB);
                indexA[0] = -1;
                indexB[0] = this.m_proxyB.GetSupport(axisB);
                const localPointB = this.m_proxyB.GetVertex(indexB[0]);
                const pointB = Transform.MulXV(xfB, localPointB, TimeOfImpact_s_pointB);
                const separation = Vec2.DotVV(Vec2.SubVV(pointB, pointA, Vec2.s_t0), normal);
                return separation;
            }
            case SeparationFunctionType.e_faceB: {
                const normal = Rot.MulRV(xfB.q, this.m_axis, TimeOfImpact_s_normal);
                const pointB = Transform.MulXV(xfB, this.m_localPoint, TimeOfImpact_s_pointB);
                const axisA = Rot.MulTRV(xfA.q, Vec2.NegV(normal, Vec2.s_t0), TimeOfImpact_s_axisA);
                indexB[0] = -1;
                indexA[0] = this.m_proxyA.GetSupport(axisA);
                const localPointA = this.m_proxyA.GetVertex(indexA[0]);
                const pointA = Transform.MulXV(xfA, localPointA, TimeOfImpact_s_pointA);
                const separation = Vec2.DotVV(Vec2.SubVV(pointA, pointB, Vec2.s_t0), normal);
                return separation;
            }
            default:
                // DEBUG: Assert(false);
                indexA[0] = -1;
                indexB[0] = -1;
                return 0;
        }
    }
    Evaluate(indexA, indexB, t) {
        const xfA = TimeOfImpact_s_xfA;
        const xfB = TimeOfImpact_s_xfB;
        this.m_sweepA.GetTransform(xfA, t);
        this.m_sweepB.GetTransform(xfB, t);
        switch (this.m_type) {
            case SeparationFunctionType.e_points: {
                const localPointA = this.m_proxyA.GetVertex(indexA);
                const localPointB = this.m_proxyB.GetVertex(indexB);
                const pointA = Transform.MulXV(xfA, localPointA, TimeOfImpact_s_pointA);
                const pointB = Transform.MulXV(xfB, localPointB, TimeOfImpact_s_pointB);
                const separation = Vec2.DotVV(Vec2.SubVV(pointB, pointA, Vec2.s_t0), this.m_axis);
                return separation;
            }
            case SeparationFunctionType.e_faceA: {
                const normal = Rot.MulRV(xfA.q, this.m_axis, TimeOfImpact_s_normal);
                const pointA = Transform.MulXV(xfA, this.m_localPoint, TimeOfImpact_s_pointA);
                const localPointB = this.m_proxyB.GetVertex(indexB);
                const pointB = Transform.MulXV(xfB, localPointB, TimeOfImpact_s_pointB);
                const separation = Vec2.DotVV(Vec2.SubVV(pointB, pointA, Vec2.s_t0), normal);
                return separation;
            }
            case SeparationFunctionType.e_faceB: {
                const normal = Rot.MulRV(xfB.q, this.m_axis, TimeOfImpact_s_normal);
                const pointB = Transform.MulXV(xfB, this.m_localPoint, TimeOfImpact_s_pointB);
                const localPointA = this.m_proxyA.GetVertex(indexA);
                const pointA = Transform.MulXV(xfA, localPointA, TimeOfImpact_s_pointA);
                const separation = Vec2.DotVV(Vec2.SubVV(pointA, pointB, Vec2.s_t0), normal);
                return separation;
            }
            default:
                // DEBUG: Assert(false);
                return 0;
        }
    }
}
const TimeOfImpact_s_timer = new Timer();
const TimeOfImpact_s_cache = new SimplexCache();
const TimeOfImpact_s_distanceInput = new DistanceInput();
const TimeOfImpact_s_distanceOutput = new DistanceOutput();
const TimeOfImpact_s_fcn = new SeparationFunction();
const TimeOfImpact_s_indexA = [0];
const TimeOfImpact_s_indexB = [0];
const TimeOfImpact_s_sweepA = new Sweep();
const TimeOfImpact_s_sweepB = new Sweep();
export function TimeOfImpact(output, input) {
    const timer = TimeOfImpact_s_timer.Reset();
    ++toiCalls;
    output.state = TOIOutputState.e_unknown;
    output.t = input.tMax;
    const proxyA = input.proxyA;
    const proxyB = input.proxyB;
    const maxVertices = Max(maxPolygonVertices, proxyA.m_count, proxyB.m_count);
    const sweepA = TimeOfImpact_s_sweepA.Copy(input.sweepA);
    const sweepB = TimeOfImpact_s_sweepB.Copy(input.sweepB);
    // Large rotations can make the root finder fail, so we normalize the
    // sweep angles.
    sweepA.Normalize();
    sweepB.Normalize();
    const tMax = input.tMax;
    const totalRadius = proxyA.m_radius + proxyB.m_radius;
    const target = Max(linearSlop, totalRadius - 3 * linearSlop);
    const tolerance = 0.25 * linearSlop;
    // DEBUG: Assert(target > tolerance);
    let t1 = 0;
    const k_maxIterations = 20; // TODO_ERIN Settings
    let iter = 0;
    // Prepare input for distance query.
    const cache = TimeOfImpact_s_cache;
    cache.count = 0;
    const distanceInput = TimeOfImpact_s_distanceInput;
    distanceInput.proxyA.Copy(input.proxyA);
    distanceInput.proxyB.Copy(input.proxyB);
    distanceInput.useRadii = false;
    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for (;;) {
        const xfA = TimeOfImpact_s_xfA;
        const xfB = TimeOfImpact_s_xfB;
        sweepA.GetTransform(xfA, t1);
        sweepB.GetTransform(xfB, t1);
        // Get the distance between shapes. We can also use the results
        // to get a separating axis.
        distanceInput.transformA.Copy(xfA);
        distanceInput.transformB.Copy(xfB);
        const distanceOutput = TimeOfImpact_s_distanceOutput;
        Distance(distanceOutput, cache, distanceInput);
        // If the shapes are overlapped, we give up on continuous collision.
        if (distanceOutput.distance <= 0) {
            // Failure!
            output.state = TOIOutputState.e_overlapped;
            output.t = 0;
            break;
        }
        if (distanceOutput.distance < target + tolerance) {
            // Victory!
            output.state = TOIOutputState.e_touching;
            output.t = t1;
            break;
        }
        // Initialize the separating axis.
        const fcn = TimeOfImpact_s_fcn;
        fcn.Initialize(cache, proxyA, sweepA, proxyB, sweepB, t1);
        /*
    #if 0
        // Dump the curve seen by the root finder {
          const int32 N = 100;
          float32 dx = 1.0f / N;
          float32 xs[N+1];
          float32 fs[N+1];
    
          float32 x = 0.0f;
    
          for (int32 i = 0; i <= N; ++i) {
            sweepA.GetTransform(&xfA, x);
            sweepB.GetTransform(&xfB, x);
            float32 f = fcn.Evaluate(xfA, xfB) - target;
    
            printf("%g %g\n", x, f);
    
            xs[i] = x;
            fs[i] = f;
    
            x += dx;
          }
        }
    #endif
    */
        // Compute the TOI on the separating axis. We do this by successively
        // resolving the deepest point. This loop is bounded by the number of vertices.
        let done = false;
        let t2 = tMax;
        let pushBackIter = 0;
        for (;;) {
            // Find the deepest point at t2. Store the witness point indices.
            const indexA = TimeOfImpact_s_indexA;
            const indexB = TimeOfImpact_s_indexB;
            let s2 = fcn.FindMinSeparation(indexA, indexB, t2);
            // Is the final configuration separated?
            if (s2 > target + tolerance) {
                // Victory!
                output.state = TOIOutputState.e_separated;
                output.t = tMax;
                done = true;
                break;
            }
            // Has the separation reached tolerance?
            if (s2 > target - tolerance) {
                // Advance the sweeps
                t1 = t2;
                break;
            }
            // Compute the initial separation of the witness points.
            let s1 = fcn.Evaluate(indexA[0], indexB[0], t1);
            // Check for initial overlap. This might happen if the root finder
            // runs out of iterations.
            if (s1 < target - tolerance) {
                output.state = TOIOutputState.e_failed;
                output.t = t1;
                done = true;
                break;
            }
            // Check for touching
            if (s1 <= target + tolerance) {
                // Victory! t1 should hold the TOI (could be 0.0).
                output.state = TOIOutputState.e_touching;
                output.t = t1;
                done = true;
                break;
            }
            // Compute 1D root of: f(x) - target = 0
            let rootIterCount = 0;
            let a1 = t1;
            let a2 = t2;
            for (;;) {
                // Use a mix of the secant rule and bisection.
                const t = rootIterCount & 1
                    ? // Secant rule to improve convergence.
                        a1 + ((target - s1) * (a2 - a1)) / (s2 - s1)
                    : // Bisection to guarantee progress.
                        0.5 * (a1 + a2);
                ++rootIterCount;
                ++toiRootIters;
                const s = fcn.Evaluate(indexA[0], indexB[0], t);
                if (Abs(s - target) < tolerance) {
                    // t2 holds a tentative value for t1
                    t2 = t;
                    break;
                }
                // Ensure we continue to bracket the root.
                if (s > target) {
                    a1 = t;
                    s1 = s;
                }
                else {
                    a2 = t;
                    s2 = s;
                }
                if (rootIterCount === 50) {
                    break;
                }
            }
            toiMaxRootIters = Max(toiMaxRootIters, rootIterCount);
            ++pushBackIter;
            if (pushBackIter === maxVertices) {
                break;
            }
        }
        ++iter;
        ++toiIters;
        if (done) {
            break;
        }
        if (iter === k_maxIterations) {
            // Root finder got stuck. Semi-victory.
            output.state = TOIOutputState.e_failed;
            output.t = t1;
            break;
        }
    }
    toiMaxIters = Max(toiMaxIters, iter);
    const time = timer.GetMilliseconds();
    toiMaxTime = Max(toiMaxTime, time);
    toiTime += time;
}
//# sourceMappingURL=TimeOfImpact.js.map