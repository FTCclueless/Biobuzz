package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.utils.Globals;
import org.firstinspires.ftc.teamcode.utils.Pose2d;
import org.firstinspires.ftc.teamcode.utils.Vector3;

public abstract class BiobuzzField
{
    public enum Side
    {
        RED,
        BLUE,
    };

    private static Side side = Side.RED;

    // Competition Manual TU06 §9.6, Figures 9-9 and 9-10.
    public static double HIVE_PIVOT_Y = 25.5 / 2; // hives are 25.5 in center to center
    public static double HIVE_CELL_X = (18.84 + 12.04) / 2 * Math.cos(Math.toRadians(30)); // up cell, hive tipped 30 deg
    public static double HIVE_CELL_Z = (53.5 + 65.6) / 2; // middle of the up cell's opening above the tiles

    public static Pose2d RED_PIVOT = new Pose2d(0, -HIVE_PIVOT_Y); // in
    public static Pose2d BLUE_PIVOT = new Pose2d(0, HIVE_PIVOT_Y); // in
    public static Pose2d RED_FLOWER_ALLY_STATION = new Pose2d(4*24-72, 0-72); // in
    public static Pose2d RED_FLOWER_SIDE = new Pose2d(0-72, 2*24-72); // in
    public static Pose2d BLUE_FLOWER_ALLY_STATION = new Pose2d(2*24-72, 6*24-72); // in
    public static Pose2d BLUE_FLOWER_SIDE = new Pose2d(6*24-72, 4*24-72); // in
    public static Pose2d RED_GARDEN = new Pose2d(6*24-72, 12-72);
    public static Pose2d BLUE_GARDEN = new Pose2d(-72, 5*24+12-72);
    public static Pose2d RED_START = new Pose2d(-2*24+Globals.ROBOT_WIDTH/2, -72+Globals.ROBOT_FORWARD_LENGTH/2);
    public static Pose2d BLUE_START = new Pose2d(24+Globals.ROBOT_WIDTH/2, 72-Globals.ROBOT_FORWARD_LENGTH/2);

    public static double RIM_HEIGHT = 21.5; // in
    public static double APERTURE = 4; // in

    // April Tags
    public static int[] BLUE_AUDIENCE_APRIL_TAGS = new int[] {38, 39, 40, 41};
    public static int[] BLUE_SCORING_APRIL_TAGS = new int[] {42, 43, 44, 45};

    public static int[] RED_AUDIENCE_APRIL_TAGS = new int[] {34, 35, 36, 37};
    public static int[] RED_SCORING_APRIL_TAGS = new int[] {30, 31, 32, 33};

    private static BiobuzzField currentField = null;

    public static Vector3 hiveCell(boolean red, int xSide) {
        return new Vector3(xSide >= 0 ? HIVE_CELL_X : -HIVE_CELL_X, red ? -HIVE_PIVOT_Y : HIVE_PIVOT_Y, HIVE_CELL_Z);
    }

    public static Vector3 nearestHiveCell(boolean red, Pose2d robot) {
        return hiveCell(red, robot.x >= 0 ? 1 : -1);
    }

    public abstract Pose2d pivot();
    public abstract Pose2d flowerAllyStation();
    public abstract Pose2d flowerSide();
    public abstract int[] audienceAprilTags();
    public abstract int[] scoringAprilTags();
    public abstract Pose2d garden();
    public abstract Pose2d start();

    public static void setSide(Side sideOfTheField)
    {
        side = sideOfTheField;
        currentField = sideOfTheField == Side.RED ? new RedField() : new BlueField();
    }

    public static BiobuzzField field()
    {
        if(currentField == null)
        {
            setSide(side);
        }
        return currentField;
    }
}

