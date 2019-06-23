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

// #if ENABLE_CONTROLLER

import { Draw } from '../Common/Draw'
import { Vec2 } from '../Common/Math'
import { TimeStep } from '../Dynamics/TimeStep'

import { Controller } from './Controller'

/**
 * Applies a force every frame
 */
export class ConstantAccelController extends Controller {
  private static Step_s_dtA = new Vec2()
  /**
   * The acceleration to apply
   */
  readonly A = new Vec2(0, 0)

  Step(step: TimeStep) {
    const dtA = Vec2.MulSV(step.dt, this.A, ConstantAccelController.Step_s_dtA)
    for (let i = this.m_bodyList; i; i = i.nextBody) {
      const body = i.body
      if (!body.IsAwake()) {
        continue
      }
      body.SetLinearVelocity(
        Vec2.AddVV(body.GetLinearVelocity(), dtA, Vec2.s_t0)
      )
    }
  }
  // tslint:disable-next-line
  Draw(draw: Draw) {}
}

// #endif
