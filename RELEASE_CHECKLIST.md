# Release Checklist

Use this checklist before publishing a new version of the module.

## Code and configuration

- [ ] The driver builds against current `zmk/main`.
- [ ] The smoke consumer CI build passes.
- [ ] The real consumer build used for validation still passes.
- [ ] Public headers document every exported runtime function.
- [ ] New devicetree properties are documented in the binding and in `README.md`.
- [ ] New Kconfig options are documented in `README.md`.

## Runtime behavior features

- [ ] CPI runtime tuning still works on real hardware.
- [ ] Burst accumulation runtime tuning still works on real hardware.
- [ ] Default burst accumulation value was re-evaluated on BLE hardware.
- [ ] Any behavior limitations are called out explicitly in `README.md`.

## Documentation

- [ ] `README.md` still reflects the current API and property names.
- [ ] The burst accumulation section still matches actual driver behavior.
- [ ] Example files under `examples/` still build or remain structurally correct.
- [ ] Compatibility notes mention the tested ZMK baseline.

## Legal and repository hygiene

- [ ] `LICENSE` matches the actual provenance of the code.
- [ ] Copyright headers on changed files are current.
- [ ] `.gitignore` covers local build outputs.

## Suggested release flow

1. Create a clean commit set.
2. Bump or confirm the version note in release text.
3. Tag the release using a simple semantic tag such as `v0.1.0`.
4. Publish release notes that mention:
   - tested ZMK baseline
   - notable driver changes
   - any runtime behavior or tuning changes
   - known limitations
