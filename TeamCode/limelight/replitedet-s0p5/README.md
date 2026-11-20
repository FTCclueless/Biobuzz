# Ball detector for the Limelight 3A

`replitedet_s0p5_int8.tflite` is RepLiteDet-S0.5 by FTC 8565, from
https://huggingface.co/ftc8565/replitedet-s0p5, under Apache 2.0 (see `LICENSE`). It was evaluated
on BIOBUZZ datasets from Roboflow Universe; `ATTRIBUTION.md` lists them. It is a 3-class detector,
`Pollen` / `RedNectar` / `BlueNectar`, with a 320x320 int8 input. At 0.45 confidence its precision
and recall are about 0.85-0.90 per class.

## Setup (one time, per Limelight)

1. Power the robot, join its Wi-Fi, open http://172.29.0.1:5801.
2. Pipeline slot **0** (`BallDetection.PIPELINE`): type **Neural Detector**.
3. Upload `replitedet_s0p5_int8.tflite` as the model and `labels.txt` as the labels.
   Runtime: **CPU**.
4. Confidence threshold: **0.45**. The code filters again at `BallDetection.MIN_CONFIDENCE`.
5. Camera: lowest resolution and highest FPS mode. The net only sees 320x320, so extra
   pixels just cost resize time. Exposure low enough that balls aren't blurred while driving,
   then raise gain until they're bright.
6. Save. Open "Cluster Chase Auto" in init and check that "Balls tracked" counts balls on the floor.

## Speed

On the Limelight the net takes about 50-60 ms per frame (~18 FPS); the 1-class S0 version
measures 52.8 ms on an LL3A. That is the floor for how fast a new ball shows up. Everything after
it runs on the Control Hub in well under 1 ms per loop: projection, tracking and clustering a few
dozen balls. `Robot.update()` runs the tracker every loop of the whole auto, so balls the camera
sees on the way are already known when the chase starts.

## Known limits

- Its NMS (IoU 0.35) can merge touching balls into one box, so a tight pile may count as fewer
  balls. The cluster is still found and driven through.
- It is weakest on very large, close-up balls (over ~180 px), so detections near the intake are
  less reliable. The chase commits at `ClusterChaser.COMMIT_DIST` and stops re-planning there for
  that reason.
