# Basic arm viewer (co-located with the station)

A single self-contained `index.html` that connects to a station's WebSocket
endpoint, follows `st3215/inference`, and renders one rotating cube per
motor plus a live raw-position readout. No build step — `protobuf.js` parses
`normfs.proto` / `st3215.proto` at runtime, and Three.js is loaded as an ES
module, both from CDN.

Meant to run on the same host as the station (see parent README for why
that avoids CORS/mixed-content questions), on a different port than the
station's own `--web` port.

## Run

On the station host:

```bash
# Station must be started with --web so 8889 is listening.
station --tcp --web ...

# Serve this directory on a different port.
cd software/station/examples/local/webviewer
python3 -m http.server 8080
```

Then open `http://<station-host>:8080/` from any browser on the LAN. The
"Station" field defaults to `<page-host>:8889` (i.e. assumes co-location);
override it if you're pointing at a different host.

## Notes

- `normfs.proto` and `st3215.proto` are copies of the ones in
  `protobufs/normfs/` and `protobufs/drivers/st3215/` — keep them in sync if
  those change.
- Motor position is read from the raw ST3215 state register at `0x38`
  (present position), same as the Python examples in `examples/local/` and
  `examples/st3215-remote-teleop-py/`.
- If the panel shows "Queue not found", the `st3215` driver isn't enabled in
  the station's `station.yaml`.
