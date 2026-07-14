# Robot Dog (Yahboom Dogzilla Lite) — deployment notes

Lessons learned getting a working `yahboom-dogzilla-lite` (+ `ov5647` camera)
`station` build running on real hardware: a Bullseye CM4 board at
`<robot-host>`. Kept as a reference for the next release, the next target OS,
or a rollback.

## Current state of the Pi (<robot-host>)

- Binary: `/home/pi/station`, deployed from the `v0.1.0-beta.9-dogzilla-bullseye`
  release (see below). Confirmed working: camera connects, dogzilla-lite
  motors detected.
- Config: `/home/pi/config.yaml` (not the default `station.yaml` — pass
  `-c config.yaml` explicitly).
- Launch: `./station --tcp --web -c config.yaml` (run manually; no systemd
  service, cron job, or autostart is configured).
- Old binary backup: `/home/pi/station.bak.<timestamp>` — this is the
  binary that was on the device *before* this work. **It does not run**
  (`GLIBC_2.32' not found`) — it was already broken, built against the
  original mis-targeted libcamera artifacts (see Lesson 5). Keeping it only
  for diffing/reference, not as a rollback target.
- `/boot/config.txt` backup: `/boot/config.txt.bak.<timestamp>` (see next
  section before touching this).

## Boot config change (`/boot/config.txt`)

The camera didn't work even after a correctly-built binary was deployed. Root
cause turned out to be nothing to do with the software: the board was
configured for the **legacy MMAL camera stack**, not `libcamera`. Without
this change the kernel never loads the `unicam` V4L2 driver, so `libcamera`
enumerates zero cameras no matter how correct the build is.

```diff
- start_x=1
+ camera_auto_detect=1

 [pi4]
- dtoverlay=vc4-fkms-v3d
+ dtoverlay=vc4-kms-v3d
```

Applied via `sudo sed -i` on the live file, with a timestamped backup taken
first (`/boot/config.txt.bak.<timestamp>`), then `sudo reboot`. Confirmed
fixed post-reboot: `media-ctl -d /dev/media4 -p` now shows a `unicam` entity
with `ov5647 10-0036` wired to `unicam-image` (previously the `unicam` media
device didn't exist at all — only `bcm2835-isp`, `bcm2835-codec`, `rpivid`).

**To roll back this specific change** (if you need the legacy camera stack
back for something else, e.g. `RaspberryPi-CM4-main/main.py` or other MMAL/
`picamera`-v1 code — never checked whether that depends on it):

```bash
ssh pi@<robot-host>
sudo cp /boot/config.txt.bak.<timestamp> /boot/config.txt
sudo reboot
```

**Important:** reverting this will break the `ov5647` camera for the new
`station` binary again (`unicam` disappears, back to the `libcamera v0.7.0`
retry-loop-with-no-camera symptom). Only revert it if you're also rolling
back to software that uses the legacy stack instead.

## Release artifacts

Two working releases, both from branch `SIM` in this repo
(`github.com/robomotic/norma-core`):

| Release | Target OS | glibc ceiling | Asset for the Pi |
|---|---|---|---|
| [`v0.1.0-beta.9-dogzilla-2`](https://github.com/robomotic/norma-core/releases/tag/v0.1.0-beta.9-dogzilla-2) | Raspberry Pi OS / Debian **Bookworm** | 2.36 (built binary needs ≤2.34) | `station-linux-arm64.tar.gz` |
| [`v0.1.0-beta.9-dogzilla-bullseye`](https://github.com/robomotic/norma-core/releases/tag/v0.1.0-beta.9-dogzilla-bullseye) | Raspberry Pi OS / Debian **Bullseye** | 2.31 (built binary needs ≤2.30) | `station-linux-arm64.tar.gz` |

**This CM4 board is Bullseye** (`cat /etc/os-release` → `VERSION_CODENAME=bullseye`,
`ldd --version` → `2.31`) — always deploy from the `-bullseye` release here,
not the plain one.

Each release also has `station-linux-amd64.tar.gz` and `station-macos-arm64.zip`
(generic builds, no camera feature) — irrelevant for this board.

**Stale/broken tag:** `v0.1.0-beta.9-dogzilla` (no suffix) predates the glibc
fix below and links against libcamera artifacts that need glibc 2.38 — it
will not run on either Bookworm or Bullseye hardware. Not deleted, but don't
use it.

To download the right asset directly:
```bash
gh release download v0.1.0-beta.9-dogzilla-bullseye -R robomotic/norma-core -p station-linux-arm64.tar.gz
```

## How to cut a release for a new/other target

```bash
gh workflow run station-release.yml --ref SIM \
  -f version=<tag> \
  -f prerelease=true \
  -f features=yahboom-dogzilla-lite \
  -f glibc=2.36   # or 2.31 for Bullseye
```
(`version` can be left blank since `572fce6` — it auto-generates a tag from
the workspace version + run number.) Watch it with
`gh run watch <run-id> --exit-status`.

## Lessons learned, in the order we hit them

### 1. No Rust toolchain in the working sandbox
The exact local `make protobuf` / `make -C .../station FEATURES=... build-arm64`
commands the task started with can't run here — no `cargo`/`rustup` installed.
Used the GitHub Actions release workflow (`.github/workflows/station-release.yml`)
instead of a local build for everything that follows.

### 2. CI workflow had no way to pass Cargo features
`station-release.yml` always did a plain `cargo build --package=station`, no
feature flags, no glibc targeting. Added a `features` input (applied to the
ARM64 leg only) and later a `glibc` input.
— `82ef432`

### 3. `libcamera-sys-static`'s cross-compile detection was wrong
```rust
let is_cross = target != host && target.contains("linux") && !host.contains("linux");
```
This only recognized cross-compiling from a **non-Linux** host (e.g. someone's
Mac). On the Linux-amd64 GitHub Actions runner cross-compiling to arm64,
`is_cross` came out `false`, so the build fell through to compiling
`wrapper.cpp` from source using pkg-config's `Cflags` — which bake in an
absolute path from whoever originally built the checked-in libcamera
artifacts (`/Users/<name>/...`), breaking on any other machine
(`libcamera/libcamera.h: file not found`). Fix: any target/host mismatch is a
cross-compile.
— `ee48743`, `shared/libcamera-sys-static/build.rs`

### 4. Same file, two more latent bugs found while fixing #3
- `bindgen`'s include path used the same broken pkg-config `Cflags`
  unconditionally (bindgen runs regardless of the cross/native branch). Fixed
  by deriving the include dir from `LIBCAMERA_LIB_DIR` when set.
- That derivation was initially missing a path segment: libcamera's own
  `meson.build` installs public headers to `<includedir>/libcamera/libcamera/*.h`
  (yes, twice) — not `<includedir>/libcamera/*.h`. Didn't break the Bookworm
  CI build (bindgen only parses `wrapper.hpp`, which doesn't include libcamera
  headers, and the cross branch never compiles `wrapper.cpp` at all) but would
  have broken a genuine native build.
— `ee48743` (include-path derivation), fixed properly in `3a1194d`

### 5. The vendored libcamera `.so` files need glibc 2.38 — newer than either target OS
The real reason nothing would have run on real hardware, independent of every
CI/link fix above. `shared/libcamera/library/0.7.0/` was built inside
`debian:trixie-slim` (Debian 13) with no lower-glibc constraint:
```
$ readelf -V libcamera.so.0.7.0 | grep -oE 'GLIBC_[0-9]+\.[0-9]+' | sort -V -u | tail -1
GLIBC_2.38
```
Bookworm ships 2.36, Bullseye ships 2.31 — both older. The `--target=...2.36`
zigbuild flag only constrains the **Rust code's own** linking; it can't
retroactively lower what an already-compiled `.so` needs. Confirmed later,
concretely: the binary that was already on the Pi before this work
(`station-linux-arm64.tar.gz` dated Jul 3) failed with
`GLIBC_2.32' not found` the moment it was invoked.

Fix: rebuild `libcamera`/`libpisp` from source per target OS, inside a
matching base image, so the resulting `.so` only requires glibc symbols that
OS actually has.
— `3a1194d`

### 6. Parameterized the libcamera build pipeline by target OS
`shared/libcamera/{Dockerfile,Makefile,build.sh}` gained a `LIBCAMERA_VARIANT`
(`bookworm` / `bullseye` / `trixie`) that picks the Docker base image and
produces artifacts in a variant-suffixed directory:
`shared/libcamera/library/0.7.0-bookworm/`, `.../0.7.0-bullseye/`. Both
`software/station/bin/station/Makefile` and the CI workflow gained a matching
`GLIBC`/`glibc` parameter (default `2.36` when `yahboom-dogzilla-lite` is in
`FEATURES`) that selects the right directory and zigbuild target.
— `3a1194d`

### 7. Sandbox had no QEMU emulation registered for the Bookworm rebuild
`docker buildx build --platform linux/arm64 ...` failed with `exec format
error` — `binfmt_misc` had no `qemu-aarch64` handler. Fixed once, for the
session:
```bash
docker run --rm --privileged tonistiigi/binfmt --install arm64
```

### 8. `meson`/`ninja` from a venv without `--system-site-packages`
Pinned `meson`/`ninja` via pip in a venv for version consistency across base
images (Bullseye's apt `meson` is too old for libcamera's `meson.build`
minimum). But `python3 -m venv /opt/venv` (no system-site-packages) hid the
apt-installed `python3-jinja2`/`python3-yaml`/`python3-ply` from anything run
inside that venv → `Python module 'jinja2' not found` deep into the meson
configure step. Fix: `python3 -m venv --system-site-packages /opt/venv`.
— `3a1194d`, `shared/libcamera/Dockerfile`

### 9. Root-owned files from a failed Docker run blocked the next retry
Docker containers run as root by default; a mid-build failure left root-owned
files in the bind-mounted `target/tmp/libcamera-bookworm/`, and the
host-side `git clean -fdx` (run before the next `docker run`) couldn't remove
them (`Permission denied`). Fixed by running the container as the host
uid/gid from the start: `docker run --user "$(id -u):$(id -g)" ...`.
— `3a1194d`, `shared/libcamera/build.sh`

### 10. Bookworm rebuild succeeded via Docker/QEMU; Bullseye done natively instead
Rather than fight QEMU emulation twice, fetched the Bullseye artifacts
straight from this CM4 board over SSH — it already had a native `libcamera`
build at `/usr/local` (glibc 2.31 native, so trivially correct) from earlier
manual work. Also built `libcamera_wrapper.a` (the small static shim this
repo's Rust bindings link against, from `shared/libcamera-sys-static/wrapper.cpp`)
**natively on the Pi**, sidestepping cross-compilation/emulation for that
target entirely. Much faster than the Docker path, and inherently
ABI-correct since it's compiled by the same toolchain that ships on the
target OS.

### 11. GNU `ld --gc-sections` + LLVM `lld` disagreed on Bullseye
The Bullseye release build got much further than the earlier failures — it
compiled and got to the final Rust link — then failed:
```
ld.lld: error: relocation refers to a discarded section: .gcc_except_table...
>>> defined in ...liblibcamera_sys_static....rlib(camera_wrapper.o)
```
`camera_wrapper.o` is pre-linked with `ld -r --gc-sections` (using whatever
GNU `ld`/binutils the build runs on — Bullseye's is 2.35, notably older than
Bookworm's), then the *final* link is done by `lld` (via the Rust toolchain,
`-fuse-ld=lld`). The two linkers disagreed about which sections were safe to
discard, specifically around libstdc++'s locale facet shim templates
(`money_get_shim<...>`) pulled in from the statically-linked `libstdc++.a`.
Same wrapper-build recipe worked fine for Bookworm (presumably because that
container's newer binutils produces a layout `lld` agrees with).

Fix: drop `--gc-sections` from the wrapper's *intermediate* relocatable link.
The final Rust link already does its own `-Wl,--gc-sections` pass, so this
step doesn't need to do dead-code elimination itself — it only needs to
produce a linkable object. Rebuilt the Bullseye `libcamera_wrapper.a` (again,
natively on the Pi) with the fix; applied the same change to `build.sh` for
the Docker/QEMU pipeline so it doesn't hit this on some future binutils
combination either.
— `58441c6`

### 12. Working binary deployed, camera still didn't work — hardware config, not software
See "Boot config change" above. `media-ctl` topology was the diagnostic: no
`unicam` entity anywhere across `/dev/media0`–`3`, only `bcm2835-isp`/
`bcm2835-codec`/`rpivid` — meaning the CSI sensor was never even probed by
the kernel, regardless of what `libcamera` build was linked in. `start_x=1`
(legacy MMAL stack) was the giveaway.

## Key files changed in this repo

- `.github/workflows/station-release.yml` — release CI: `features`/`glibc`
  inputs, per-variant libcamera linking for the ARM64 leg.
- `software/station/bin/station/Makefile` — same `FEATURES`/`GLIBC` handling
  for local builds (blocked on a working local Rust toolchain to actually
  exercise, but should mirror the CI logic).
- `shared/libcamera/{Dockerfile,Makefile,build.sh}` — the libcamera
  from-source vendoring pipeline, now parameterized by target OS.
- `shared/libcamera-sys-static/build.rs` — the Rust FFI crate linking
  against `libcamera`/the wrapper; cross-detection and include-path fixes.
- `shared/libcamera/library/0.7.0-bookworm/` — vendored Bookworm artifacts
  (built via Docker/QEMU).
- `shared/libcamera/library/0.7.0-bullseye/` — vendored Bullseye artifacts
  (fetched from a real board; `libcamera_wrapper.a` rebuilt twice, see
  Lesson 11 for the final version).
