'use strict';
const assert = require('assert');
const {patchFlasherCompletion} = require('../site/flasher-completion.js');
class Dialog { _renderInstall() { return this._client; } }
assert(patchFlasherCompletion(Dialog));
const d = new Dialog();
d._manifest = {new_install_improv_wait_time: 0};
for (const state of ['initializing', 'preparing', 'erasing', 'writing', 'error']) {
  d._client = undefined; d._installState = {state};
  assert.strictEqual(d._renderInstall(), undefined);
}
d._installState = {state:'finished'};
assert.strictEqual(d._renderInstall(), null);
d._client = undefined; d._manifest.new_install_improv_wait_time = 10;
assert.strictEqual(d._renderInstall(), undefined);
assert.strictEqual(patchFlasherCompletion(class {}), false);
console.log('Flasher completion waits for FINISHED and only skips disabled Improv PASS');
