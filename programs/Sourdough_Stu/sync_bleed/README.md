# SYNC BLEED

```
      ╔═══════════════════════════════════════════════════════════════╗
      ║                                                               ║
      ║   ███████╗██╗   ██╗███╗   ██╗ ██████╗                         ║
      ║   ██╔════╝╚██╗ ██╔╝████╗  ██║██╔════╝                         ║
      ║   ███████╗ ╚████╔╝ ██╔██╗ ██║██║                              ║
      ║   ╚════██║  ╚██╔╝  ██║╚██╗██║██║                              ║
      ║   ███████║   ██║   ██║ ╚████║╚██████╗                         ║
      ║   ╚══════╝   ╚═╝   ╚═╝  ╚═══╝ ╚═════╝                         ║
      ║                                                               ║
      ║          ██████╗ ██╗     ███████╗███████╗██████╗              ║
      ║          ██╔══██╗██║     ██╔════╝██╔════╝██╔══██╗             ║
      ║          ██████╔╝██║     █████╗  █████╗  ██║  ██║             ║
      ║          ██╔══██╗██║     ██╔══╝  ██╔══╝  ██║  ██║             ║
      ║          ██████╔╝███████╗███████╗███████╗██████╔╝             ║
      ║          ╚═════╝ ╚══════╝╚══════╝╚══════╝╚═════╝              ║
      ║                                                               ║
      ║         analog dirty mixer · broken TBC simulator             ║
      ║                                                               ║
      ╚═══════════════════════════════════════════════════════════════╝
```

A chain of degraded analog video processors for the Videomancer
platform — capacitor-bleed smear, diode wavefolder, NTSC chroma comb,
sin/cos hue rotation, TBC line jitter, and tape noise.  All of it
content-modulated: an envelope follower watches the live video and
pushes each stage's character along with the picture.  At rest, it's
a static effect chain; with Envelope Reactivity cranked, the dirt
breathes with the input.


─────────────────────────────────────────────────────────────────────

## QUICK START

1. Master Mix fader up to taste — start ~50% to hear dry/wet blend.
2. Knob 1 (Cap Bleed) up half — gives you the smear floor.
3. Knob 4 (NTSC Comb) up a touch — adds chroma fringing.
4. Knob 6 (Env React) up — now everything reacts to the picture.
5. Toggle 7 (AC/DC) flips the smear character — try both.
6. Toggle 11 (Bleed Freeze) for "tape head sticking" persistent smear.

If output looks dead: check Master Mix is not at 0, and at least one
knob other than Master is non-zero.


─────────────────────────────────────────────────────────────────────

## SIGNAL FLOW

```
                      ╭─── envelope follower ───╮
                      │  luma + edge per half    │
                      │  → cv_global → cv_eff    │
                      ╰────────────┬─────────────╯
                                   │  (modulation
                                   │   bus — see
                                   │   "HOW IT
                                   │   MODULATES")
                                   │
                                   ▼
   IN ─┬─► [TBC LINE BUFFER] ──► [DIODE WAVEFOLDER] ──► [NTSC COMB]
       │      + per-line              4-bank cross-       opposite-
       │      jitter offset           fade (Y, U, V)      signed UV
       │                                                      │
       │                                                      ▼
       │                                               [HUE ROTATION]
       │                                                  16-bin
       │                                                  cross-fade
       │                                                      │
       │                                                      ▼
       │                                              [CAP BLEED IIR]
       │                                                + AC/DC mirror
       │                                                + edge ringing
       │                                                + Bleed Freeze
       │                                                      │
       │                                                      ▼
       │                                                [TAPE NOISE]
       │                                                 (Y-domain,
       │                                                  midtone-loud)
       │                                                      │
       │                                                      │
       │             ┌────────── wet path ────────────────────┘
       │             │
       │             ▼
       └─► dry  ──► [MASTER MIX]
                         │
                         ▼
                        OUT
```

19 clock cycles, input to output.  Dry path runs through a sync-delay
shift register so the master mix interpolator sees positionally-
identical pixels at all-knobs-zero.


─────────────────────────────────────────────────────────────────────

## HOW IT MODULATES — the cv map

Two internal "control signals" emerge from the input:

```
  cv_global   content-derived envelope (luma + edge accumulated per
              spatial half across the frame, max'd into one signal,
              slow-attacked across frames so it doesn't flicker).
              Frame-stable.

  cv_eff      cv_global + (Knob 6 >> 2).  Knob 6 nudges cv_eff up so
              reactive stages reach further along the modulation
              ladder at lower content brightness than they otherwise
              would.
```

```
  ┌───────────────────────────────────────────────────────────────┐
  │                                                                │
  │     INPUT ─► [LUMA + EDGE / per-half] ──► cv_global            │
  │                                              │                 │
  │     KNOB 6 (Env React) ──────► env_react ────┤                 │
  │                                  │           │                 │
  │                                  └─►(>> 2)──►│                 │
  │                                              ▼                 │
  │                                          cv_eff                │
  │                                              │                 │
  │                                              ▼                 │
  │                                   ▶ DIODE BANK select          │
  │                                   ▶ BANK cross-fade weight     │
  │                                   ▶ EDGE RING env-boost gate   │
  │                                   ▶ NTSC COMB env-boost gate   │
  │                                                                │
  │     cv_global (without Knob 6 amplification) drives:           │
  │                                   ▶ HUE ROTATION angle base    │
  │                                   ▶ TAPE NOISE amplitude       │
  │                                   ▶ TBC JITTER env contribution│
  │                                                                │
  └───────────────────────────────────────────────────────────────┘
```

Knob 6 is the master modulator.  At zero, the program runs as a
static effect chain.  Cranked, every reactive stage gets pushed up
the modulation ladder.


─────────────────────────────────────────────────────────────────────

## WHO TOUCHES WHAT

```
                         K1   K2   K3   K4   K5   K6   T7   T8   T10  T11
                        Cap  Edg  Hue  NTS  TBC  Env  AC/  Edg  Dio  Frz
                        Bld  Res  Twst Cmb  Jit  Rct  DC   Phs  Bse
   Cap Bleed IIR  ────  ●                        ●    ●              ●
   Edge Ringing   ──────── ●                     ●         ●
   Hue Rotation   ──────────── ●                 ●
   NTSC Comb      ──────────────── ●             ●
   TBC Jitter     ──────────────────── ●         ●
   Diode Bank     ────────────────────────────   ●              ●
   Tape Noise     ────────────────────────────   ●
   Bleed Freeze   ── ●                                                ●
```

Read each row: *"this stage is modulated by these knobs/toggles."*
The Knob 6 column has the most dots — it's the meta-control.


─────────────────────────────────────────────────────────────────────

## DIODE WAVEFOLDER — 4 banks, cv-driven cross-fade

The Y/UV diode stage has four LUT banks with progressively wilder
curves: soft tanh saturation, hard clipping, single fold, double fold
(inversion).  As cv_eff rises, you cross-fade up the ladder through
adjacent bank pairs.

```
   cv_eff  0 ──── 256 ──── 512 ──── 768 ──── 1023
              │          │          │          │
              ├─ bank 0 ─┤          │          │
              │          │          │          │
              │     bank 0 + bank 1 blend     │
              │          │          │          │
              │          │     bank 1 + bank 2 blend
              │          │          │          │
              │          │          │     bank 2 + bank 3 blend
              │          │          │          │
              │          │          │          ├─ bank 3 ─

           soft       gentle      stronger    full
           tanh       blend       blend       wavefolder
           (clean)    (warm       (clipped    (inversion:
                       grit)      shapes)      black ↔ white
                                                via wraps)


   Toggle 10 (Diode Base) shifts which banks are on the table:

         T10 OFF (Soft)  ─►  bank pairs 0..1, 1..2, 2..2
         T10 ON  (Hard)  ─►  bank pairs 1..2, 2..3, 3..3
                              (skip the gentle tanh entry —
                               jump straight into clipping
                               and wavefolder territory)
```

Bank transitions are hysteresis-stabilised (cv must cross the
threshold by ±128 to flip) so cv hovering near a boundary won't
strobe between banks per frame.


─────────────────────────────────────────────────────────────────────

## CONTROLS

```
   KNOB 1   CAP BLEED         capacitor-bleed smear decay
                              0   ▏░░░░░░░░░░░▕   no smear, fast tracking
                              max ▏▓▓▓▓▓▓▓▓▓▓▓▕   long persistent trails

   KNOB 2   EDGE RESONANCE    edge-ringing magnitude
                              < 64 → strict null (true off)
                              max  → visible halos / outlines on edges

   KNOB 3   CHROMA TWIST      hue rotation angle
                              512 = no offset
                              away from centre = static hue shift

   KNOB 4   NTSC COMB         comb delta magnitude (UV chroma fringe)
                              high values + Knob 6 high → UV saturation;
                              dial Knob 6 back if comb feels "off"

   KNOB 5   TBC JITTER        per-line offset, 0..16 pixels
                              wobble / horizontal-tearing character

   KNOB 6   ENVELOPE REACT    content-coupling strength (master modulator)
                              0   = static effect chain
                              max = each stage modulated by content cv

   TGL 7    AC / DC           IIR mirror polarity
                              DC  = capacitor-bleed trail (bright→dark)
                              AC  = ghost-image / negative-shadow smear
                                    (output = 2·input − state)

   TGL 8    EDGE PHASE        edge ringing sign
                              off = bright outlines on edges
                              on  = dark outlines

   TGL 9    UV SWAP           swap U and V channels
                              instant complementary-hue shift

   TGL 10   DIODE BASE        bank range
                              SOFT = banks 0..2 (gentle to clipped)
                              HARD = banks 1..3 (clipped to wavefolder)

   TGL 11   BLEED FREEZE      IIR line-reset gate + 8× slower decay
                              tape-head-sticking persistent smear that
                              builds across the frame instead of resetting
                              every line

   FADER    MASTER MIX        dry / wet
```


─────────────────────────────────────────────────────────────────────

## INTERACTIONS — what plays well, what fights

```
   K4 (NTSC Comb) × K6 (Env React)
       High + high → UV channels saturate past about 3/4 on the
       NTSC knob.  The comb pushes UV out of the linear range and
       per-pixel variation collapses (looks "flat").  Back off K6
       when running NTSC high.

   K1 (Cap Bleed) × T7 (AC/DC)
       AC mode mirrors the smear contribution.
       High K1 + AC = strong negative-ghost overlay (image leaves
                      reverse-trail when motion happens).
       High K1 + DC = classic capacitor smear (bright trails into
                      dark areas, lingering).

   T11 (Bleed Freeze) × K1 (Cap Bleed)
       Freeze gates the per-line state reset AND adds 8× slower decay.
       K1 low  + Freeze = short halos.
       K1 high + Freeze = smear builds and holds across the frame;
                          won't release until Freeze toggles back off.

   T10 (Diode Base = Hard) × content brightness × K6
       Hard mode shifts the bank range to 1..3.  Bright content + K6
       cranked pushes into Bank 3 — black input wraps to mid-grey,
       bright wraps to dark, gradients break into wavefolder bands.

   K5 (TBC Jitter) × K2 (Edge Resonance)
       Edge ringing tap-position shifts WITH the jitter offset, so
       the rings track the displaced wet image rather than lock at
       the input edge.  Low K5 → rings stay near source edges; high
       K5 → rings wander with the line shift.
```


─────────────────────────────────────────────────────────────────────

## SOUNDS TO CHASE

A few starting points.  All assume Master Mix at full / near-full and
the rest of the controls at TOML defaults unless noted.

```
   ┌─ "VHS dropout" ──────────────────────────────────────────┐
   │  K1 Cap Bleed       70%                                  │
   │  K5 TBC Jitter      30%                                  │
   │  K6 Env React       80%                                  │
   │  T7 AC/DC           AC                                   │
   │  Hits hardest on motion-rich content.  Keeps bright      │
   │  highlights ghosting / leaving negative trails behind.   │
   └──────────────────────────────────────────────────────────┘

   ┌─ "Wavefolder lab" ───────────────────────────────────────┐
   │  K3 Hue Twist       300                                  │
   │  K6 Env React       100%                                 │
   │  T10 Diode Base     HARD                                 │
   │  Sweep brightness in your source.  Watch banks 1→2→3     │
   │  ladder up and the diode stage chew through the picture. │
   │  Add K4 (NTSC) sparingly for chroma drift.               │
   └──────────────────────────────────────────────────────────┘

   ┌─ "Frozen ghost" ─────────────────────────────────────────┐
   │  K1 Cap Bleed       60%                                  │
   │  T7 AC/DC           AC                                   │
   │  T11 Bleed Freeze   ON                                   │
   │  Master Mix         ~70%                                 │
   │  Freeze the smear, let new content layer over the held   │
   │  state.  Toggle Freeze off briefly to refresh the       │
   │  "ghost".                                                │
   └──────────────────────────────────────────────────────────┘

   ┌─ "Crisp NTSC" ───────────────────────────────────────────┐
   │  K4 NTSC Comb       45%                                  │
   │  K6 Env React       40%                                  │
   │  T7 AC/DC           DC                                   │
   │  T10 Diode Base     SOFT                                 │
   │  Subtle chroma fringe + soft saturation.  Closest to     │
   │  "convincingly bad analog deck" — not a glitch effect,   │
   │  more like a worn signal path.                           │
   └──────────────────────────────────────────────────────────┘

   ┌─ "Hue drift, soft" ──────────────────────────────────────┐
   │  K3 Hue Twist        sweep 256 ↔ 768                     │
   │  K1 Cap Bleed        40%                                 │
   │  K6 Env React        20%                                 │
   │  T7 AC/DC            DC                                  │
   │  Move K3 slowly while content plays.  Smear holds the    │
   │  hue shift just past the move so it feels like the       │
   │  frame is "settling into" each new colour position.      │
   └──────────────────────────────────────────────────────────┘
```


─────────────────────────────────────────────────────────────────────

## NOTES

- **Hardware**: iCE40HX4K (Lattice).  Resource usage 7,657 / 7,680 LCs
  (99%), 29 / 32 EBR (90%).  Tight fit by design — adding new stages
  would require trimming an existing one.
- **Resolution**: portable across the Videomancer SDK's supported
  modes (1080p / 720p / 480p) via runtime-counted envelope divisor,
  spatial-half boundary, and prev-line BRAM offset.
- **Pipeline depth**: 19 clocks input-to-output.
- **Default-load safe**: at all-knobs-zero with Master Mix at 0, the
  program is a clean dry pass-through.
- **License**: GPL-3.0.  See file headers.

```
                 ─ ─ ─   broken TBC rides again   ─ ─ ─
```
