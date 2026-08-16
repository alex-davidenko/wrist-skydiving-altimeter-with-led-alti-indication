# Hardware

Enclosure, wiring and the parts that are not firmware.

| what | put it here as |
|---|---|
| enclosure, editable | `.step` or `.f3d` — the source, so others can change it |
| enclosure, printable | `.stl` or `.3mf` |
| wiring notes | see the pinout table in the root README |

**STEP beats STL.** An STL is a mesh: printable, effectively not editable. A STEP
file can be opened and modified in almost any CAD package. Publishing only the
STL is publishing the output and keeping the source.

Print settings that matter for this part are in the root README — the short
version is PETG, and 100% infill is **not** airtight (you can blow through it),
so a sealed sensor chamber needs a coating or a resin insert.

Keep files under ~50 MB each; GitHub warns above that. A typical enclosure STL
is 1-10 MB, which is fine committed directly — Git LFS is only worth the
complication for something much larger.
