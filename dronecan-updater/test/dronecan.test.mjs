// file.Read responses, classic and CAN FD, checked against pydronecan (vectors.json, from gen_vectors.py),
// plus SLCAN 'D' line framing and FD reassembly.
import { readFileSync } from 'node:fs';
import { SIG, SVC, FD_LENGTHS, serviceId, packTransfer, encodeReadResponse, transferCrc, Reassembler,
         encodeBeginFirmwareUpdate, decodeBeginFirmwareUpdateResponse } from '../js/dronecan.js';
import { SLCAN } from '../js/slcan.js';

let fails = 0;
const ok = (c, m) => { console.log((c ? 'PASS' : 'FAIL') + ' ' + m); if (!c) fails++; };
const hex = b => Buffer.from(b).toString('hex');
const unhex = s => new Uint8Array(Buffer.from(s, 'hex'));

const vectors = JSON.parse(readFileSync(new URL('vectors.json', import.meta.url), 'utf8'));
for (const v of vectors) {
  const id = serviceId(20, SVC.FILE_READ, false, 68, 127);
  const chunk = unhex(v.chunk);
  const frames = packTransfer(id, encodeReadResponse(0, chunk, v.fd), v.tid, SIG.FILE_READ, v.fd);
  const same = id === v.id && frames.length === v.frames.length && frames.every((f, i) => hex(f.data) === v.frames[i]);
  ok(same, `${v.fd ? 'FD     ' : 'classic'} ${String(chunk.length).padStart(3)}-byte read matches pydronecan` +
     (same ? '' : `\n  got  ${frames.map(f => hex(f.data)).join(' ')}\n  want ${v.frames.join(' ')}`));
}

// BeginFirmwareUpdate request framing and FD response decoding, against pydronecan
for (const v of JSON.parse(readFileSync(new URL('bfu_vectors.json', import.meta.url), 'utf8'))) {
  if (v.kind === 'request') {
    const id = serviceId(20, SVC.BEGIN_FIRMWARE_UPDATE, true, 68, 127);
    const p = encodeBeginFirmwareUpdate(127, new TextEncoder().encode(v.path), v.fd);
    const frames = packTransfer(id, p, 5, SIG.BEGIN_FIRMWARE_UPDATE, v.fd);
    ok(id === v.id && frames.map(f => hex(f.data)).join() === v.frames.join(),
       `${v.fd ? 'FD' : 'classic'} BeginFirmwareUpdate request matches pydronecan`);
  } else {
    const r = new Reassembler();
    let res = null;
    for (const f of v.frames) res = r.push(0, unhex(f), true) || res;
    const d = decodeBeginFirmwareUpdateResponse(res.payload, true);
    ok(d.error === v.error && d.message === v.message, `FD BeginFirmwareUpdate response decodes to ${JSON.stringify(d)}`);
  }
}

// Lengths where pydronecan pads wrongly: every frame must still be a valid FD length, and the
// transfer must reassemble to the padded payload with a matching CRC.
for (const n of [119, 182, 245]) {
  const chunk = new Uint8Array(n).map((_, i) => i * 7);
  const payload = encodeReadResponse(0, chunk, true);
  const id = serviceId(20, SVC.FILE_READ, false, 68, 127);
  const frames = packTransfer(id, payload, 3, SIG.FILE_READ, true);
  const r = new Reassembler();
  let res = null;
  for (const f of frames) res = r.push(id, f.data, true) || res;
  ok(frames.every(f => FD_LENGTHS.includes(f.data.length)) && res && res.fd &&
     hex(res.payload.subarray(0, payload.length)) === hex(payload) &&
     res.payload.subarray(payload.length).every(b => b === 0) &&
     res.crc === transferCrc(SIG.FILE_READ, res.payload),
     `FD ${n}-byte read: valid frame lengths, reassembles with CRC`);
}

// SLCAN: an FD frame goes out as a 'D' line with a length code, and parses back.
{
  const s = new SLCAN({ onFrame: (...a) => got.push(a) });
  const got = [];
  const data = new Uint8Array(12).map((_, i) => i);
  const line = s.frameLine(0x143044ff, data, true);
  ok(line === 'D143044FF9000102030405060708090A0B\r', 'FD frame line: ' + JSON.stringify(line));
  s.handleLine(line.slice(0, -1) + '1A2B');  // ArduPilot appends a timestamp
  ok(got.length === 1 && got[0][0] === 0x143044ff && hex(got[0][1]) === hex(data) && got[0][2] === true && got[0][3] === true,
     "'D' line parsed as an extended FD frame");
  s.handleLine('T143044FF80102030405060708');
  ok(got.length === 2 && got[1][1].length === 8 && got[1][3] === false, "'T' line still parsed as classic");
  s.handleLine('T143044FF9000102030405060708090A0B');
  ok(got.length === 2, "classic line with DLC 9 rejected");
}

// A write issued while a multi-chunk transfer is going out (the 1 Hz NodeStatus) must queue
// behind it, and chunks must end on line boundaries, so no SLCAN line is ever spliced.
{
  const s = new SLCAN({ onFrame: () => {} });
  const writes = [];
  s.writer = { write: async b => { writes.push(Buffer.from(b).toString('latin1')); } };
  const id = serviceId(20, SVC.FILE_READ, false, 68, 127);
  const reply = packTransfer(id, encodeReadResponse(0, new Uint8Array(256).fill(0x5a), true), 1, SIG.FILE_READ, true);
  const big = s.sendFrames(reply);
  await new Promise(r => setImmediate(r));
  const status = s.writeRaw('T1401557F0100000000000000C0\r');
  await Promise.all([big, status]);
  const lines = writes.join('').split('\r').filter(Boolean);
  ok(writes.length > 2 && writes.every(w => w.endsWith('\r')), `${writes.length} writes, each ending on a line`);
  ok(lines.length === 6 && lines.slice(0, 5).every(l => l[0] === 'D') && lines[5][0] === 'T',
     'status line queued after the whole FD reply');
}

console.log(fails ? `\n${fails} FAILED` : '\nall passed');
process.exit(fails ? 1 : 0);
