declare type ChangeCallback = (newVal: any, oldVal: any) => void;
export declare function listenToProperty(obj: any, propName: string, onChange: ChangeCallback, firstOneForFree?: boolean): void;
export declare function stopListeningToProperty(obj: any, propName: string, onChange: ChangeCallback): void;
export declare function migrateLiveProperty(oldObj: any, newObj: any, propName: string): void;
export {};
