# libfdt read-only subset

This directory is pinned to the public API and read-only wire semantics of
`dtc`/`libfdt` v1.7.2 (`b6910bec11614980a21e46fbccc35934b671bd81`).
Only the read-only calls used by athrill-device are included.  It is used
under libfdt's BSD-2-Clause licensing option.

Upstream: <https://github.com/dgibson/dtc/tree/v1.7.2/libfdt>

The implementation retains the upstream copyright and performs the same
header, bounds, structure, string, property, and phandle checks needed by the
loader.  It deliberately omits libfdt's tree-writing and overlay APIs.
