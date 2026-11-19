# Legacy (2020) files

These are the original artefacts from the 2020 BEng individual project
*"Design of a tool for kinematics development of the double wishbone and
McPherson suspension systems"* (Coventry University). They are kept for
reference only; the maintained implementation is the `suspkin` Python
package in `src/`.

| File | What it is |
|------|------------|
| `suspension_tool_2020.xlsx` | The original Excel tool: *Double Wishbone* and *McPherson* sheets, scroll-bar driven bump / roll / camber / caster / toe / steer inputs, front/side/top view charts and the -150...+150 mm result tables. |
| `adams/sus_2d.cmd` | MSC ADAMS/View command file for the planar double wishbone used to validate the spreadsheet (upper arm driven by `30d*sin(360d*time)`). The hardpoints in the file are the tutorial template values; the validation model itself used the dimensions in `examples/adams_validation_2020.json`. |

## How the spreadsheet maps to the new code

* Spreadsheet inputs (chassis widths/heights, arm lengths and widths, knuckle
  length, tyre size, caster) -> `suspkin.ExcelDoubleWishbone`, which builds a
  3-D hardpoint model. Example: `examples/excel_default_2020.json`.
* The spreadsheet's camber equation (which assumed the knuckle stays vertical
  while locating the upper ball joint) is reproduced in
  `ExcelDoubleWishbone.excel_camber()` for comparison. It matches the values
  the 2020 report tabulated, and its error against ADAMS grows to ~1 deg at
  160 mm bump - the exact solver stays within 0.015 deg
  (see `validation/`).
* Known spreadsheet quirks that are documented rather than silently fixed:
  tyre "height" was `rim + one sidewall` (used as the overall diameter), the
  McPherson sheet was still work-in-progress (left/right sides disagree at
  static ride height), and toe/steer were applied as a direct wheel rotation
  rather than through a tie rod.
