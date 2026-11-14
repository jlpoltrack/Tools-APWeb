#!/usr/bin/env python3
"""Writes vectors.json: file.Read responses framed by pydronecan, classic and CAN FD, for dronecan.test.mjs."""
import json
import os
import random

import dronecan
from dronecan import transport, uavcan

random.seed(1)
vectors = []
for fd in (False, True):
    # pydronecan pads the last FD frame wrongly when it would hold exactly 63 bytes (n = 119, 182, 245)
    for n in (0, 1, 3, 7, 8, 56, 59, 60, 100, 200, 254, 255, 256):
        chunk = bytes(random.randrange(256) for _ in range(n))
        resp = uavcan.protocol.file.Read.Response()
        resp.error.value = 0
        resp.data = bytearray(chunk)
        t = transport.Transfer(payload=resp, source_node_id=127, dest_node_id=68, transfer_id=n & 0x1F,
                               transfer_priority=20, service_not_message=True, request_not_response=False,
                               canfd=fd)
        vectors.append({'fd': fd, 'chunk': chunk.hex(), 'tid': n & 0x1F, 'id': t.message_id,
                        'frames': [bytes(f.bytes).hex() for f in t.to_frames()]})
# BeginFirmwareUpdate: request (classic and FD) and FD responses with and without a message
bfu = []
for fd in (False, True):
    r = uavcan.protocol.file.BeginFirmwareUpdate.Request()
    r.source_node_id = 127
    r.image_file_remote_path.path = 'firmware.bin'
    t = transport.Transfer(payload=r, source_node_id=127, dest_node_id=68, transfer_id=5, transfer_priority=20,
                           service_not_message=True, request_not_response=True, canfd=fd)
    bfu.append({'kind': 'request', 'fd': fd, 'path': 'firmware.bin', 'id': t.message_id,
                'frames': [bytes(f.bytes).hex() for f in t.to_frames()]})
for msg in ('', 'Refused: busy'):
    s = uavcan.protocol.file.BeginFirmwareUpdate.Response()
    s.error = 1
    s.optional_error_message = msg
    t = transport.Transfer(payload=s, source_node_id=68, dest_node_id=127, transfer_id=5, transfer_priority=20,
                           service_not_message=True, request_not_response=False, canfd=True)
    bfu.append({'kind': 'response', 'fd': True, 'error': 1, 'message': msg,
                'frames': [bytes(f.bytes).hex() for f in t.to_frames()]})
json.dump(bfu, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bfu_vectors.json'), 'w'), indent=1)

out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vectors.json')
json.dump(vectors, open(out, 'w'), indent=1)
print(f'{len(vectors)} vectors -> {out}')
