# Spec structure & format

Type: grilling (HITL)

## Status

resolved 2026-07-29

## Question

How is the destination spec itself organized? Decide: the document layout (one big spec doc vs a directory with an overview + per-part files), the template every part definition follows (inputs, outputs, protocol, parameters, failure behavior, interfaces to other parts), the system-level block diagram (inputs/outputs), and where the spec lives in the repo. Every other resolved part feeds its decision into this structure, so the format must exist before part decisions can be written up as the spec.

## Assumptions

- The spec's audience is the owner building each part independently, then assembling — per the destination.
- Spec lives in this repo as Markdown (mermaid diagrams allowed), alongside this map.
- A full mermaid system-diagram hierarchy already exists: [System diagrams](./system-diagrams.md) (L0 context → L1 firmware blocks → L2 per-part subcomponents, transpiled vs bespoke annotated). Adopt it as the spec's system-level block diagram rather than starting from blank; decide here where it lives in the final spec layout.

## Decision

- **Layout:** one comprehensive spec document, not a directory of per-part files.
- **Location:** `docs/spec/pico-wing-fc.md` — a new `docs/spec/` directory, parallel to `docs/wayfinding/pico-wing-fc/`. Wayfinding stays the "how we decided" trail (questions, assumptions, rationale, still useful once nav/telemetry resume); `docs/spec/` is the clean "what to build" deliverable a builder reads without the back-and-forth.
- **Template per part:** inputs, outputs, protocol/interfaces to other parts, parameters (name/range/default), failure behavior — drawn from the Decision section already recorded in each resolved wayfinding doc.
- **System diagram:** link to [System diagrams](./system-diagrams.md) (L0 context → L1 firmware → L2 per-part, transpiled-vs-bespoke annotated) rather than duplicating the mermaid source into the spec.
- **Scope of this version:** covers the MVP as decided — manual + stabilized modes, no CRSF telemetry TX, no autonomous/GPS nav. A "Deferred" section in the spec calls out telemetry and autonomous nav as known future additions, pointing at their (still-open/deferred) wayfinding docs.
