import { BroadPhase } from '../Collision/BroadPhase';
import { Contact } from './Contacts/Contact';
import { ContactFactory } from './Contacts/ContactFactory';
import { FixtureProxy } from './Fixture';
import { ContactFilter, ContactListener } from './WorldCallbacks';
export declare class ContactManager {
    readonly m_broadPhase: BroadPhase<FixtureProxy>;
    m_contactList: Contact | null;
    m_contactCount: number;
    m_contactFilter: ContactFilter;
    m_contactListener: ContactListener;
    m_allocator: any;
    m_contactFactory: ContactFactory;
    constructor();
    AddPair(proxyA: FixtureProxy, proxyB: FixtureProxy): void;
    FindNewContacts(): void;
    Destroy(c: Contact): void;
    Collide(): void;
}
