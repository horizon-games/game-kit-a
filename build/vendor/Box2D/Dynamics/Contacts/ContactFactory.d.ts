import { Fixture } from '../Fixture';
import { Contact } from './Contact';
export declare class ContactRegister {
    createFcn: ((allocator: any) => Contact) | null;
    destroyFcn: ((contact: Contact, allocator: any) => void) | null;
    primary: boolean;
}
export declare class ContactFactory {
    m_allocator: any;
    m_registers: ContactRegister[][];
    constructor(allocator: any);
    Create(fixtureA: Fixture, indexA: number, fixtureB: Fixture, indexB: number): Contact | null;
    Destroy(contact: Contact): void;
    private AddType;
    private InitializeRegisters;
}
