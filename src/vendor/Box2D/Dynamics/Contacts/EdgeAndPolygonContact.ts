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

// DEBUG: import { Assert } from "../../Common/Settings";
// DEBUG: import { ShapeType } from "../../Collision/Shapes/Shape";
import { CollideEdgeAndPolygon } from '../../Collision/CollideEdge'
import { Manifold } from '../../Collision/Collision'
import { EdgeShape } from '../../Collision/Shapes/EdgeShape'
import { PolygonShape } from '../../Collision/Shapes/PolygonShape'
import { Shape } from '../../Collision/Shapes/Shape'
import { Transform } from '../../Common/Math'
import { Fixture } from '../Fixture'

import { Contact } from './Contact'

export class EdgeAndPolygonContact extends Contact {
  static Create(allocator: any): Contact {
    return new EdgeAndPolygonContact()
  }

  // tslint:disable-next-line
  static Destroy(contact: Contact, allocator: any): void {}
  constructor() {
    super()
  }

  Reset(
    fixtureA: Fixture,
    indexA: number,
    fixtureB: Fixture,
    indexB: number
  ): void {
    super.Reset(fixtureA, indexA, fixtureB, indexB)
    // DEBUG: Assert(fixtureA.GetType() === ShapeType.e_edgeShape);
    // DEBUG: Assert(fixtureB.GetType() === ShapeType.e_polygonShape);
  }

  Evaluate(manifold: Manifold, xfA: Transform, xfB: Transform): void {
    const shapeA: Shape = this.m_fixtureA.GetShape()
    const shapeB: Shape = this.m_fixtureB.GetShape()
    // DEBUG: Assert(shapeA instanceof EdgeShape);
    // DEBUG: Assert(shapeB instanceof PolygonShape);
    CollideEdgeAndPolygon(
      manifold,
      shapeA as EdgeShape,
      xfA,
      shapeB as PolygonShape,
      xfB
    )
  }
}
