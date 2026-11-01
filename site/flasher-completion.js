'use strict';
// ESP Web Tools 10.1.1 reports FINISHED only after writeFlash, reset and
// transport disconnect succeed. Improv is disabled for this firmware, so
// do not keep its completion screen waiting for a serial port to reopen.
function patchFlasherCompletion(ctor) {
  const p = ctor && ctor.prototype;
  if (!p || typeof p._renderInstall !== 'function') return false;
  const original = p._renderInstall;
  p._renderInstall = function (...args) {
    if (this._installState && this._installState.state === 'finished' &&
        this._client === undefined && this._manifest &&
        this._manifest.new_install_improv_wait_time === 0) {
      this._client = null;
    }
    return original.apply(this, args);
  };
  return true;
}
if (typeof module !== 'undefined') module.exports = {patchFlasherCompletion};
if (typeof customElements !== 'undefined') {
  customElements.whenDefined('ewt-install-dialog').then(() => {
    if (!patchFlasherCompletion(customElements.get('ewt-install-dialog'))) {
      console.warn('ESP Web Tools completion compatibility patch unavailable');
    }
  });
}
