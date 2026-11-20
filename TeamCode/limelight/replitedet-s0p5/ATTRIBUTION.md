# Third-party datasets: attribution and licences

RepLiteDet-S0.5 was evaluated (and developed) with BIOBUZZ (FIRST Tech Challenge 2026-27) datasets
shared on Roboflow Universe by other teams. No image from these datasets is in the released model's
training data, and no dataset image ships in this repository.

| dataset (Roboflow Universe) | owner | version | licence | use |
|---|---|---|---|---|
| [FTC-BioBuzz](https://universe.roboflow.com/bluedarkup/ftc-biobuzz-6vjbl) | BlueDarkUP | v1 | MIT | evaluation (second-field validation and test set) |
| [FTC-BIOBUZZ](https://universe.roboflow.com/jonathan-qi/ftc-biobuzz) | jonathan-qi (FTC 11347) | v3 | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) | evaluation (held-out new-venue test set) |
| [FTC BioBuzz Game Elements](https://universe.roboflow.com/krishna-seshan/ftc-biobuzz-game-elements) | krishna-seshan (Brickbots) | v2 | MIT | evaluation (held-out new-venue test set) |
| [BioBuzz Pollen Detection](https://universe.roboflow.com/biobuzz/biobuzz-pollen-detection) | BioBuzz Projects (FTC 11093) | v2 | [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/) | development experiments only; not in the released weights |

## Changes made

- All four: Roboflow augmented copies removed (one image kept per original); class names mapped to
  Pollen / RedNectar / BlueNectar; images cropped to the 4:3 camera aspect (re-encoded as JPEG where
  cropped).
- FTC-BioBuzz (BlueDarkUP): restored from 320 x 320 to its recorded 16:9 aspect; near-duplicate
  consecutive frames removed; re-split into time chunks; boxes covering only the visible part of a
  hidden ball were down-weighted (not deleted) in development runs.
- FTC-BIOBUZZ (jonathan-qi): black letterbox padding cropped.
- BioBuzz Pollen Detection: restored from 300 x 300 to 4:3; 16 near-duplicate images dropped.
- No labels in these datasets were edited.

These teams do not endorse this model. Each dataset keeps its own licence.
