// Hardware test: runs app.js under a stub DOM, with Web Serial replaced by serial_bridge.py, and updates a
// real DroneCAN node through an ArduPilot FC, as a user clicking through the page would.
//   node test/hw-update.mjs <FC serial port> <firmware.bin> <node id>
import { spawn } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { basename } from 'node:path';
import { fileURLToPath } from 'node:url';
import { FD_LENGTHS } from '../js/dronecan.js';

const [portPath, binPath, nodeArg] = process.argv.slice(2);
if (!portPath || !binPath || !nodeArg) {
  console.error('usage: node test/hw-update.mjs <FC serial port> <firmware.bin> <node id>');
  process.exit(2);
}
const targetNode = Number(nodeArg);
const bridge = fileURLToPath(new URL('serial_bridge.py', import.meta.url));
const wait = ms => new Promise(r => setTimeout(r, ms));

// With HW_TRACE set, logs each file.Read request (offset, classic or FD) and SLCAN error replies (BEL)
const TRACE = !!process.env.HW_TRACE;
const traceStart = Date.now();
let traceBuf = '';
function trace(d) {
  if (!TRACE) return;
  traceBuf += d.toString('latin1');
  let i;
  while ((i = traceBuf.search(/[\r\x07]/)) >= 0) {
    const line = traceBuf.slice(0, i), sep = traceBuf[i];
    traceBuf = traceBuf.slice(i + 1);
    if (sep === '\x07' && /SLCAN is routed/.test(logText)) console.log(`TRACE ${Date.now() - traceStart} ms SLCAN error reply (BEL)`);
    const m = /^([TD])([0-9A-F]{8})([0-9A-F])([0-9A-F]+)$/.exec(line);
    if (!m) continue;
    const id = parseInt(m[2], 16);
    if (!(id & 0x80) || ((id >> 16) & 0xff) !== 48 || !((id >> 15) & 1)) continue;
    const b = Buffer.from(m[4], 'hex');  // data, then ArduPilot's 2-byte timestamp
    const len = m[1] === 'D' ? FD_LENGTHS[parseInt(m[3], 16)] : parseInt(m[3], 16);
    if (!(b[len - 1] & 0x80)) continue;  // offset is only in a transfer's first frame
    // A classic multi-frame request starts with its 2-byte CRC
    const o = m[1] === 'T' && !(b[len - 1] & 0x40) ? 2 : 0;
    const off = b[o] + b[o + 1] * 256 + b[o + 2] * 65536 + b[o + 3] * 16777216;
    console.log(`TRACE ${Date.now() - traceStart} ms read ${m[1] === 'D' ? 'FD' : 'classic'} offset ${off} tid ${b[len - 1] & 0x1f}`);
  }
}

// ---------- Web Serial stand-in ----------
class BridgePort {
  async open() {
    this.proc = spawn('python3', [bridge, portPath], { stdio: ['pipe', 'pipe', 'inherit'] });
    const proc = this.proc;
    process.on('exit', () => proc.kill());
    const out = this.proc.stdout;
    this.readable = new ReadableStream({
      // The app cancels the reader on disconnect while the bridge may still deliver data
      start(c) {
        out.on('data', d => { trace(d); try { c.enqueue(new Uint8Array(d)); } catch {} });
        out.on('end', () => { try { c.close(); } catch {} });
      },
    });
    const stdin = this.proc.stdin;
    this.writable = new WritableStream({ write: chunk => new Promise(r => stdin.write(Buffer.from(chunk), r)) });
  }
  async setSignals() {}
  async close() { this.proc.kill(); await wait(300); }
}
Object.defineProperty(globalThis, 'navigator', {
  value: { serial: { requestPort: async () => new BridgePort() } }, configurable: true });

// ---------- DOM stand-in ----------
const els = new Map();
let rows = [];
let logText = '';
const el = id => {
  if (!els.has(id)) {
    const e = { id, style: {}, hidden: id === 'reboot', disabled: false, scrollTop: 0, scrollHeight: 0, _text: '' };
    Object.defineProperty(e, 'textContent', {
      get: () => (id === 'log' ? logText : e._text),
      set: v => {
        if (id === 'log') { process.stdout.write(v.slice(logText.length)); logText = v; } else e._text = v;
      },
    });
    if (id === 'nodes') {
      Object.defineProperty(e, 'innerHTML', { set: () => { rows = []; }, get: () => '' });
      e.appendChild = tr => rows.push(tr);
    }
    els.set(id, e);
  }
  return els.get(id);
};
globalThis.document = { getElementById: el, createElement: () => ({ className: '', innerHTML: '', onclick: null }) };
const loaded = [];
globalThis.window = { addEventListener: (ev, fn) => { if (ev === 'DOMContentLoaded') loaded.push(fn); } };

await import(new URL('../js/app.js', import.meta.url));
loaded.forEach(fn => fn());

const until = async (cond, ms, what) => {
  const end = Date.now() + ms;
  while (Date.now() < end) { if (cond()) return true; await wait(100); }
  throw new Error('timed out waiting for ' + what);
};
const row = () => rows.find(r => r.innerHTML.startsWith(`<td>${targetNode}</td>`));

// ---------- the user's clicks ----------
const bytes = readFileSync(binPath);
el('file').onchange({ target: { files: [{ name: basename(binPath), arrayBuffer: async () => bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.length) }] } });

await el('connect').onclick();
if (!el('reboot').hidden) {
  await el('reboot').onclick();
  await el('connect').onclick();  // disconnects
  await wait(8000);               // FC reboot and USB re-enumeration
  await el('connect').onclick();
}
await until(() => row()?.innerHTML.includes('</td><td>org.'), 20000, `node ${targetNode} and its GetNodeInfo`);
console.log('>> node row: ' + row().innerHTML.replace(/<\/?td>/g, ' ').trim());
row().onclick();
await until(() => !el('flash').disabled, 2000, 'the Update button');
await el('flash').onclick();
await until(() => /Firmware transfer complete|refused|Timed out/.test(logText), 180000, 'the transfer to finish');
if (!/Firmware transfer complete/.test(logText)) { await el('connect').onclick(); process.exit(1); }
const doneAt = Date.now();
await until(() => /OPERATIONAL/.test(row()?.innerHTML || '') && Date.now() - doneAt > 3000, 20000, 'the node back in OPERATIONAL');
console.log('>> node row after update: ' + row().innerHTML.replace(/<\/?td>/g, ' ').trim());
await el('connect').onclick();
process.exit(0);
