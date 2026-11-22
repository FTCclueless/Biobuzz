package org.firstinspires.ftc.teamcode.opmodes;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.junit.Test;

public class TeleopCornerTest {
    @Test
    public void snapsToNearestCornerWithRobotFootprint() {
        double l = Globals.ROBOT_LENGTH / 2, w = Globals.ROBOT_WIDTH / 2;
        Pose2d p = Teleop.nearestCorner(new Pose2d(-60, 55, 0.1)); // facing +x: length along x
        assertEquals(-72 + l, p.x, 1e-9);
        assertEquals(72 - w, p.y, 1e-9);
        assertEquals(0.1, p.heading, 1e-9);

        Pose2d q = Teleop.nearestCorner(new Pose2d(10, -3, Math.PI / 2)); // facing +y: width along x
        assertEquals(72 - w, q.x, 1e-9);
        assertEquals(-72 + l, q.y, 1e-9);
    }
}
