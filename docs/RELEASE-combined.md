# Combined release: product-acc + pipeline4 + spill-retry

Branch `perf/combined` (commit `4015f7d`) combines the accepted compiler work on
top of `perf/gather-reads` (15bb418). It is the compiler of record for the
release artefact `vc4-combined-20261119` and is also installed on the Pi.

## Branch composition

Order used: `perf/pipeline4` first, then `perf/product-acc`, then
`perf/spill-retry`.

| step | result |
|---|---|
| branch point | `perf/pipeline4` (5e5ace5), which already contains `perf/pipeline2` (2720dae), `perf/pipeline3` (dfb5b45), the accumulator pass `perf/product-acc` (c32d3d8) and the `perf/gather-reads` base |
| `perf/product-acc` | already an ancestor -> nothing to merge |
| `perf/spill-retry` | merge commit `4015f7d`, 5 files, +295/-113; **no textual conflicts** (git ort auto-merge) |

No hunk overlapped, but the two features touch the same files, so the merge was
reviewed semantically:

- `GraphColoring::colorGraph()` keeps the product-acc overload: the no-argument
  entry point first tries `colorGraph(true)` (prefer accumulators), catches a
  failure, resets the graph and recolours with `colorGraph(false)`.
- The spill-retry additions (`getUnassignedLocals()`, `hasValueCopyConflicts()`,
  `spillFailingLocals()`) are intact, and the spill path calls the no-argument
  `colorGraph()`, so a spill retry still benefits from the accumulator
  preference and still falls back safely.
- `CodeGenerator::generateInstructions()` keeps the spill block *before* the
  fix-up steps and keeps the fail-closed logic: `spillAllowed` is decided once
  from the pristine colouring via `hasValueCopyConflicts()`, and if spilling
  does not make the whole method colourable the loop `break`s instead of handing
  the partially spilled program to the scalar repair steps.
- `RegisterFixes.cpp`/`.h` are the `perf/spill-retry` versions; they build on
  the `perf/gather-reads` refusal of vector/pointer value copies
  (`needsValueCopy()`/`refuseValueCopyRepair()`), which is still present.

Nothing was dropped. The corpus sweep below is the functional check that both
features are active: the 8 kernels that only `perf/spill-retry` could compile
still compile, and the 50 genuinely refused kernels are still refused.

## Build

```
cmake -DCMAKE_BUILD_TYPE=Release \
      -DVC4CL_STDLIB_DIR=/home/raspi/vc4cl/VC4CLStdLib/include \
      -DLLVM_DIR=/usr/lib/llvm-14/lib/cmake/llvm \
      -DLLVM_SHARED_LIBRARY=/usr/lib/llvm-14/lib/libLLVM.so
make -j2 VC4CC VC4C
```

`vc4c --version`: `0.4.9999 (4015f7d)`.

## Gate (stock clocks: V3D 300 MHz, core 400 MHz; one GPU job at a time)

Measurements through the combined build (before installing). `core` is
`vcgencmd measure_clock core`; every run started and ended at 400 MHz. A 100 ms
sampler during the installed N=256 run recorded 40/40 samples at 400 MHz.

| case | settings | best ms | GFLOP/s | mismatches | accepted best |
|---|---|---|---|---|---|
| `gemm_v16` N=128, unpadded | no local size, `VC4C_PIPELINE_LOADS=2` | 2.0018 | 2.0953 | 0 | 1.687 (`perf/pipeline4`); target >= 1.44 |
| `r16g_128` N=128, padded | `GEMM19_PADY=9`, local 64x3, `VC4C_PIPELINE_LOADS=2` | 1.5740 | 2.6647 | 0 | 2.668-2.677 |
| `r16g_256` N=256, padded | `GEMM19_PADY=18`, local 64x3, `VC4C_PIPELINE_LOADS=2` | 10.8766 | 3.0850 | 0 | 3.085 |

`bench_vc4cl all`: 5/5 PASS (vector 1024/65536/1048576, matmul 32/64), 0
mismatches, `STATE: LIVE` after every run.

Corpus sweep (`~/work/sweep/sweep_spill.sh`, same 180 sources as the accepted
baseline): **130 OK, 50 FAIL, 0 compilation regressions** against
`sweep_after_final.tsv` (the `perf/spill-retry` baseline). All 8 kernels that
`perf/spill-retry` newly compiled are still OK; the same 50 kernels still fail
closed.

## VC4CL runtime

Built `VC4CL/build-icd` against the installed combined compiler:

```
cmake -DBUILD_ICD=ON -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release \
      -DVC4CC_LIBRARY=/usr/local/lib/libVC4CC.so \
      -DVC4C_HEADER_PATH=/usr/local/include/vc4cc/VC4C.h \
      -DVC4C_TOOLS_HEADER_PATH=/usr/local/include/vc4cc/tools.h
```

Installed into `/usr/local` after backing up the previous files:

| path | file |
|---|---|
| `/usr/local/bin/vc4c` | 0.4.9999 (4015f7d), RUNPATH `/usr/local/lib` |
| `/usr/local/lib/libVC4CC.so.0.4.9999` (+ `.so.1.2`, `.so`) | build ID `9d72aa52b822cae2d2197a9ae9e57d1a92473a90` |
| `/usr/local/lib/libVC4CL.so.0.4.9999` (+ `.so.1.2`, `.so`) | build ID `d62a2156f2d26b6bb73cd6019dec59be82aec1c5` |
| `/etc/OpenCL/vendors/VC4CL.icd` | `/usr/local/lib/libVC4CL.so` |
| `/usr/local/include/vc4cc/*`, `/usr/local/share/vc4cc/*`, `/usr/local/bin/v3d_*`, `vc4cl_dump_analyzer` | headers/tools from the same build |

`ldd /usr/local/lib/libVC4CL.so.0.4.9999` resolves `libVC4CC.so.1.2` to
`/usr/local/lib/libVC4CC.so.1.2`, and the installed `libVC4CC` has the same build
ID as the combined build output, so the runtime uses the new compiler.

Installed-path verification (ICD loader, no `LD_LIBRARY_PATH`):

| case | best ms | GFLOP/s | mismatches |
|---|---|---|---|
| `gemm_v16` N=128 unpadded | 2.0031 | 2.0939 | 0 |
| `r16g_128` N=128 padded | 1.5737 | 2.6652 | 0 |
| `r16g_256` N=256 padded | 10.8777 | 3.0847 | 0 |

`clinfo -l` sees `Platform #0: OpenCL for the Raspberry Pi VideoCore IV GPU`;
`bench_vc4cl` rebuilt against `libOpenCL` passes 5/5.

## Rollback

Previous state (VC4C `0.4.9999` / `6fb14c4`, matching VC4CL) is backed up in:

- `/home/raspi/work/release/backup-20261119/` (with `SHA256SUMS`)
- `/usr/local/lib/vc4cl-backup-20261119/`

```bash
B=/home/raspi/work/release/backup-20261119
sudo cp -a $B/vc4c /usr/local/bin/
sudo cp -a $B/libVC4CC.so* $B/libVC4CL.so* /usr/local/lib/
sudo cp -a $B/VC4CL.icd /etc/OpenCL/vendors/VC4CL.icd
sudo ldconfig
/usr/local/bin/vc4c --version     # 0.4.9999 (6fb14c4)
```

## Release artefact

`/home/raspi/work/release/vc4-combined-20261119.tar.gz`
(sha256 `840c4254ac4912f77d05b46bf0e7cf5f0aec49449d2cbd5415ffe64c5cad9e8f`),
containing `bin/vc4c`, `lib/libVC4CC.so*`, `lib/libVC4CL.so*`,
`etc/OpenCL/vendors/VC4CL.icd`, `README.md` (install/rollback) and
`SHA256SUMS`. Unpacked-tree checksum verification passes. The tarball is not
self-installing; installation is the `cp` sequence in its README.

## Notes / anomalies during the session

- One `make -j4` build was interrupted by an SD/MMC stall (`mmc_rescan` hung
  task) and the board rebooted; the build was resumed with `-j2` and completed.
  No GPU job was running at the time.
- One first run of the installed bench aborted in the DMA-heap constructor
  (`/dev/dma_heap/linux,cma`: ENOMEM) while CMA reported >200 MiB free; the
  immediate retry and all later runs succeeded, so this was transient CMA
  fragmentation/busy state, not a regression.
- Core clock droops under load (observed 400 -> 250/275 -> 300 -> 400 in an
  unrelated sample window); all accepted numbers above were taken with the
  clock at 400 MHz at start and end, and the N=256 gate run sampled 400 MHz
  throughout.
