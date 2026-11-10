package org.firstinspires.ftc.teamcode.field;

import static org.junit.Assert.assertEquals;

import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector3;
import org.junit.Test;

public class BiobuzzFieldTest {
    @Test
    public void nearestCellIsOnTheRobotsSideOfTheAllianceHive() {
        Vector3 red = BiobuzzField.nearestHiveCell(true, new Pose2d(40, -30, 0));
        assertEquals(BiobuzzField.HIVE_CELL_X, red.x, 1e-9);
        assertEquals(-12.75, red.y, 1e-9);

        Vector3 blue = BiobuzzField.nearestHiveCell(false, new Pose2d(-40, -30, 0));
        assertEquals(-BiobuzzField.HIVE_CELL_X, blue.x, 1e-9);
        assertEquals(12.75, blue.y, 1e-9);
    }

    @Test
    public void cellGeometryMatchesTheManual() {
        assertEquals(13.37, BiobuzzField.HIVE_CELL_X, 0.01);
        assertEquals(59.55, BiobuzzField.HIVE_CELL_Z, 0.01);
    }
}
