import { removeFromArray } from '~/utils/arrayUtils';
const updaters = [];
const update = (dt) => {
    for (const updater of updaters) {
        updater.update(dt);
    }
};
const register = (sib) => {
    updaters.push(sib);
};
const unregister = (sib) => {
    removeFromArray(updaters, sib);
};
const UpdateManager = {
    update,
    register,
    unregister
};
export default UpdateManager;
//# sourceMappingURL=UpdateManager.js.map