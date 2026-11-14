// node web/test/bond_selection_test.cjs
const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const html = fs.readFileSync(`${__dirname}/../dongle_console.html`, 'utf8');
const context = { module: { exports: {} }, TextEncoder, TextDecoder, Uint8Array };
vm.runInNewContext(html.match(/<script id="proto">([\s\S]*?)<\/script>/)[1], context);
const p = context.module.exports;
const legacy = Uint8Array.from([1, 2,
  1, 2, 3, 4, 5, 6, 0, 1, 1, 65,
  6, 5, 4, 3, 2, 1, 1, 0, 1, 66]);
const old = p.parseBonds(legacy);
assert.equal(old.supportsSelection, false);
assert.equal(old.bonds[0].connected, true);
assert.equal(old.bonds[1].selected, false);
const current = p.parseBonds(Uint8Array.from([...legacy, 2]));
assert.equal(current.supportsSelection, true);
assert.equal(current.bonds[0].connected, true);
assert.equal(current.bonds[0].selected, false);
assert.equal(current.bonds[1].connected, false);
assert.equal(current.bonds[1].selected, true);
assert.deepEqual(Array.from(p.encodeSelectBond(current.bonds[1])), [6, 5, 4, 3, 2, 1, 1]);
assert.equal(p.DEV.SELECT_BOND, 8);
assert.equal(p.isDongle({ vendorId: 0x057e, productId: 0x2009 }), true);
assert.equal(p.isDongle({ vendorId: 0x1209, productId: 0x0001 }), true);
assert.equal(p.isDongle({ vendorId: 0x1209, productId: 0x9999 }), false);
assert.equal(p.isDongle(null), false);
assert.ok(p.DONGLE_USB_FILTERS.some(f => f.vendorId === 0x1209 && f.productId === 0x0001));
assert.equal(p.parseBonds(Uint8Array.from([1, 0, 0])).supportsSelection, true);
assert.equal(p.parseBonds(Uint8Array.from([1, 0])).supportsSelection, false);
assert.throws(() => p.parseBonds(Uint8Array.from([...legacy, 3])), /selected bond/);
assert.throws(() => p.parseBonds(legacy.subarray(0, legacy.length - 1)), /truncated/);
for (const [, script] of html.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/g)) new vm.Script(script);
console.log('Web bond selection compatibility and script syntax checks passed');
