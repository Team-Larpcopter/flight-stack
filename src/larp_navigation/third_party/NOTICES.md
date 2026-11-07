# Vendored source

`upstream-lock.json` records upstream commits and the original bytes before our
patches. Builds use these copied sources, not Git submodules or `references/`.

- sqrtVINS: root license is LGPL-3.0; some inherited source/package declarations
  say GPL-3.0. All original notices are retained; do not assume a uniform license.
- SUPER and bundled ROG-Map: source headers declare LGPL-3.0-or-later. The full
  GPL and LGPL license texts are included alongside the preserved notices.
- Air-FAR: package metadata declares BSD; source headers retain their copyright
  notices. Upstream did not supply a standalone license text in this revision.
- px4_msgs: its upstream BSD license is retained in `src/px4_msgs`.

Only the estimator/message packages and navigation components are vendored in
this integration. No simulator or proprietary FlyX reference bundle is included.
The lock lists original upstream hashes; locally adapted files intentionally
have different hashes. sqrtVINS packages live in `src/vendor`.
