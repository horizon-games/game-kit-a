import { Manifold } from '../../Collision/Collision';
import { Transform } from '../../Common/Math';
import { Fixture } from '../Fixture';
import { Contact } from './Contact';
export declare class CircleContact extends Contact {
    static Create(allocator: any): Contact;
    static Destroy(contact: Contact, allocator: any): void;
    constructor();
    Reset(fixtureA: Fixture, indexA: number, fixtureB: Fixture, indexB: number): void;
    Evaluate(manifold: Manifold, xfA: Transform, xfB: Transform): void;
}
