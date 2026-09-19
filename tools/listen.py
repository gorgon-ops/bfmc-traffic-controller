#!/usr/bin/env python3
"""Listen for semaphore broadcasts and print them as a car would see them.

This is the other end of `sem_sim --broadcast` and of the ESP module on the
node. It is about forty lines because that is genuinely all the car side of
this protocol is: bind a UDP socket, parse a small JSON object, index it by id.

    python tools/listen.py                 # default port
    python tools/listen.py --port 5007
    python tools/listen.py --raw           # one line per datagram

Run it beside `build/host/sem_sim --broadcast` to watch a full cycle arrive.
"""

import argparse
import json
import socket
import time

STATE = {0: "RED", 1: "YELLOW", 2: "GREEN"}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--port", type=int, default=5007)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--raw", action="store_true",
                    help="print datagrams instead of a table")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.settimeout(1.0)

    print(f"listening on {args.bind}:{args.port}, ctrl-c to stop\n")

    # Last frame seen per semaphore id, so the table shows the whole
    # intersection rather than whichever head spoke most recently.
    heads: dict[int, dict] = {}
    last_draw = 0.0

    while True:
        try:
            data, _ = sock.recvfrom(512)
        except socket.timeout:
            continue
        except KeyboardInterrupt:
            print()
            return 0

        for line in data.decode("utf-8", "replace").splitlines():
            line = line.strip()
            if not line:
                continue
            if args.raw:
                print(line)
                continue
            try:
                frame = json.loads(line)
            except json.JSONDecodeError:
                print(f"  unparseable: {line!r}")
                continue
            if "id" not in frame or "state" not in frame:
                # The two required keys. Anything else is optional by design.
                print(f"  not a semaphore frame: {line!r}")
                continue
            heads[int(frame["id"])] = frame

        now = time.monotonic()
        if not args.raw and now - last_draw > 0.25:
            last_draw = now
            cells = []
            for sid in sorted(heads):
                f = heads[sid]
                state = STATE.get(int(f["state"]), "?")
                ttc = f.get("ttc_ms")
                if f.get("fault"):
                    note = "FAULT"
                elif ttc is None:
                    note = ""
                elif f.get("exact"):
                    note = f"{ttc / 1000:.1f}s"
                else:
                    note = f">{ttc / 1000:.1f}s"
                cells.append(f"{sid}:{state:<6}{note:>7}")
            print("  " + " | ".join(cells), end="\r", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
