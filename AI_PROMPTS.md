# AI Prompts Used in SenseEdge Development

The following prompts were used with Claude (Anthropic) during the development of this project.

---

## Prompt 1 — Codebase Understanding

```
understand this codebase well and thoroughly first
```

## Prompt 2 — README Bio Addition

```
add my qualifications. I am a PhD student in Analog and mixed Signals IC design. also here is my bio. Make it very professional.
```

## Prompt 3 — Proposal Submission Research

```
make the submission
```

## Prompt 4 — Git Push

```
push to git
```

## Prompt 5 — Verilog Debugging

```
debug my verilog RTL code. run the unit testbenches and show any errors
```

## Prompt 6 — Timing Closure Debugging

```
my OpenLane hardening run is showing hold violations at the slow corner. the worst slack is -0.324ns across 170 endpoints. analyze the STA reports and tell me if these are structural violations or fixable with buffer insertion. suggest a strategy to close timing.
```

## Prompt 7 — Timing Closure Configuration

```
the hold violations are not improving with buffer insertion. I think it's clock skew related. update the OpenLane config to increase the hold buffer budget and rerun. explain what each parameter change does.
```

## Prompt 8 — Presentation Generation from README

```
generate an 8-slide PowerPoint presentation for a 3-minute demo video based on my README. the audience is hardware engineers and contest judges. cover: problem, solution architecture, silicon design, verification results, PCBA, and deployment scope. keep each slide concise with key numbers and diagrams.
```

## Prompt 9 — Code Commenting

```
add clear inline comments to my Verilog modules explaining the signal flow, state machine transitions, and key arithmetic operations. focus on the FFT engine and neural network inference engine — these are the most complex. comments should help a hardware engineer understand the design without reading the full spec.
```
