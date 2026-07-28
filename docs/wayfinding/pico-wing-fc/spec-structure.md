# Spec structure & format

Type: grilling (HITL)

## Status

open

## Question

How is the destination spec itself organized? Decide: the document layout (one big spec doc vs a directory with an overview + per-part files), the template every part definition follows (inputs, outputs, protocol, parameters, failure behavior, interfaces to other parts), the system-level block diagram (inputs/outputs), and where the spec lives in the repo. Every other resolved part feeds its decision into this structure, so the format must exist before part decisions can be written up as the spec.

## Assumptions

- The spec's audience is the owner building each part independently, then assembling — per the destination.
- Spec lives in this repo as Markdown (mermaid diagrams allowed), alongside this map.
- A candidate system-level block diagram (transpiled vs bespoke modules, in mermaid) already exists in the Decision of "Research: INAV transpile survey" — adopt or refine it here rather than starting from blank.

## Decision
