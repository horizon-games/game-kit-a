import { Manifold } from '../../Collision/Collision';
import { Transform } from '../../Common/Math';
import { Fixture } from '../Fixture';
import { Contact } from './Contact';
export declare class ChainAndPolygonContact extends Contact {
    static Create(allocator: any): Contact;
    static Destroy(contact: Contact, allocator: any): void;
    private static Evaluate_s_edge;
    constructor();
    Reset(fixtureA: Fixture, indexA: number, fixtureB: Fixture, indexB: number): void;
    Evaluate(manifold: Manifold, xfA: Transform, xfB: Transform): void;
}
