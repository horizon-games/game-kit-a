declare type UpdateCallback = (dt: number) => void;
interface Updater {
    update: UpdateCallback;
}
declare const UpdateManager: {
    update: (dt: number) => void;
    register: (sib: Updater) => void;
    unregister: (sib: Updater) => void;
};
export default UpdateManager;
