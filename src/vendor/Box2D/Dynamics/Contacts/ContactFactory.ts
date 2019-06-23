// DEBUG: import { Assert } from "../../Common/Settings";
import { ShapeType } from '../../Collision/Shapes/Shape'
import { MakeArray } from '../../Common/Settings'
import { Fixture } from '../Fixture'

import { ChainAndCircleContact } from './ChainAndCircleContact'
import { ChainAndPolygonContact } from './ChainAndPolygonContact'
import { CircleContact } from './CircleContact'
import { Contact } from './Contact'
import { EdgeAndCircleContact } from './EdgeAndCircleContact'
import { EdgeAndPolygonContact } from './EdgeAndPolygonContact'
import { PolygonAndCircleContact } from './PolygonAndCircleContact'
import { PolygonContact } from './PolygonContact'

export class ContactRegister {
  // public pool: Contact[];
  createFcn: ((allocator: any) => Contact) | null = null
  destroyFcn: ((contact: Contact, allocator: any) => void) | null = null
  primary: boolean = false
}

export class ContactFactory {
  m_allocator: any = null
  m_registers!: ContactRegister[][]

  constructor(allocator: any) {
    this.m_allocator = allocator
    this.InitializeRegisters()
  }

  Create(
    fixtureA: Fixture,
    indexA: number,
    fixtureB: Fixture,
    indexB: number
  ): Contact | null {
    const type1: ShapeType = fixtureA.GetType()
    const type2: ShapeType = fixtureB.GetType()

    // DEBUG: Assert(0 <= type1 && type1 < ShapeType.e_shapeTypeCount);
    // DEBUG: Assert(0 <= type2 && type2 < ShapeType.e_shapeTypeCount);

    const reg: ContactRegister = this.m_registers[type1][type2]
    if (reg.createFcn) {
      const c: Contact = reg.createFcn(this.m_allocator)
      if (reg.primary) {
        c.Reset(fixtureA, indexA, fixtureB, indexB)
      } else {
        c.Reset(fixtureB, indexB, fixtureA, indexA)
      }
      return c
    } else {
      return null
    }
  }

  Destroy(contact: Contact): void {
    const fixtureA: Fixture = contact.m_fixtureA
    const fixtureB: Fixture = contact.m_fixtureB

    if (
      contact.m_manifold.pointCount > 0 &&
      !fixtureA.IsSensor() &&
      !fixtureB.IsSensor()
    ) {
      fixtureA.GetBody().SetAwake(true)
      fixtureB.GetBody().SetAwake(true)
    }

    const typeA: ShapeType = fixtureA.GetType()
    const typeB: ShapeType = fixtureB.GetType()

    // DEBUG: Assert(0 <= typeA && typeB < ShapeType.e_shapeTypeCount);
    // DEBUG: Assert(0 <= typeA && typeB < ShapeType.e_shapeTypeCount);

    const reg: ContactRegister = this.m_registers[typeA][typeB]
    if (reg.destroyFcn) {
      reg.destroyFcn(contact, this.m_allocator)
    }
  }

  private AddType(
    createFcn: (allocator: any) => Contact,
    destroyFcn: (contact: Contact, allocator: any) => void,
    type1: ShapeType,
    type2: ShapeType
  ): void {
    const pool: Contact[] = MakeArray(256, (i: number) =>
      createFcn(this.m_allocator)
    ) // TODO: Settings

    function poolCreateFcn(allocator: any): Contact {
      // if (pool.length > 0) {
      //   return pool.pop();
      // }

      // return createFcn(allocator);
      return pool.pop() || createFcn(allocator)
    }

    function poolDestroyFcn(contact: Contact, allocator: any): void {
      pool.push(contact)
    }

    // this.m_registers[type1][type2].pool = pool;
    this.m_registers[type1][type2].createFcn = poolCreateFcn
    this.m_registers[type1][type2].destroyFcn = poolDestroyFcn
    this.m_registers[type1][type2].primary = true

    if (type1 !== type2) {
      // this.m_registers[type2][type1].pool = pool;
      this.m_registers[type2][type1].createFcn = poolCreateFcn
      this.m_registers[type2][type1].destroyFcn = poolDestroyFcn
      this.m_registers[type2][type1].primary = false
    }

    /*
    this.m_registers[type1][type2].createFcn = createFcn;
    this.m_registers[type1][type2].destroyFcn = destroyFcn;
    this.m_registers[type1][type2].primary = true;

    if (type1 !== type2) {
      this.m_registers[type2][type1].createFcn = createFcn;
      this.m_registers[type2][type1].destroyFcn = destroyFcn;
      this.m_registers[type2][type1].primary = false;
    }
    */
  }

  private InitializeRegisters(): void {
    this.m_registers = [
      /*ShapeType.e_shapeTypeCount*/
    ]

    for (let i: number = 0; i < ShapeType.e_shapeTypeCount; i++) {
      this.m_registers[i] = [
        /*ShapeType.e_shapeTypeCount*/
      ]

      for (let j: number = 0; j < ShapeType.e_shapeTypeCount; j++) {
        this.m_registers[i][j] = new ContactRegister()
      }
    }

    this.AddType(
      CircleContact.Create,
      CircleContact.Destroy,
      ShapeType.e_circleShape,
      ShapeType.e_circleShape
    )
    this.AddType(
      PolygonAndCircleContact.Create,
      PolygonAndCircleContact.Destroy,
      ShapeType.e_polygonShape,
      ShapeType.e_circleShape
    )
    this.AddType(
      PolygonContact.Create,
      PolygonContact.Destroy,
      ShapeType.e_polygonShape,
      ShapeType.e_polygonShape
    )
    this.AddType(
      EdgeAndCircleContact.Create,
      EdgeAndCircleContact.Destroy,
      ShapeType.e_edgeShape,
      ShapeType.e_circleShape
    )
    this.AddType(
      EdgeAndPolygonContact.Create,
      EdgeAndPolygonContact.Destroy,
      ShapeType.e_edgeShape,
      ShapeType.e_polygonShape
    )
    this.AddType(
      ChainAndCircleContact.Create,
      ChainAndCircleContact.Destroy,
      ShapeType.e_chainShape,
      ShapeType.e_circleShape
    )
    this.AddType(
      ChainAndPolygonContact.Create,
      ChainAndPolygonContact.Destroy,
      ShapeType.e_chainShape,
      ShapeType.e_polygonShape
    )
  }
}
