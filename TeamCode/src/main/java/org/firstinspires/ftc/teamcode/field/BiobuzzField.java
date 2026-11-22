package org.firstinspires.ftc.teamcode.field;

import com.pedropathing.math.Pose;

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
    // Layout from Competition Manual §9.3 Figure 9-2 / §10.5.4 (+/- 1 in). +x is the audience wall, which is on
    // the red drivers' right; the field is 180-deg symmetric, so blue's right is -x (the rear wall).
    // 4 FLOWERS on the perimeter, 2 per alliance side: one on the alliance wall 2 tiles to the drivers' right
    // (ALLY_STATION, next to the GARDEN), one on the side wall 2 tiles out on the drivers' left (FAR).
    public static Pose2d RED_FLOWER_ALLY_STATION = new Pose2d(4*24-72, 0-72); // in
    public static Pose2d RED_FLOWER_SIDE = new Pose2d(12, -72+Globals.ROBOT_FORWARD_LENGTH/2); // in
    public static Pose2d BLUE_FLOWER_ALLY_STATION = new Pose2d(2*24-72, 6*24-72); // in
    public static Pose2d BLUE_FLOWER_SIDE = new Pose2d(-12, 72-Globals.ROBOT_FORWARD_LENGTH/2); // in
    public static Pose2d RED_FAR_FLOWER = new Pose2d(-72, -24); // in, on the rear wall
    public static Pose2d BLUE_FAR_FLOWER = new Pose2d(72, 24); // in, on the audience wall
    // GARDEN zone centre (not a robot pose): a 23 x 2 in strip along the audience (red) / rear (blue) wall,
    // starting in the corner at the drivers' right. 4 POLLEN are staged in it, in a line from that corner.
    public static Pose2d RED_GARDEN = new Pose2d(72 - 1, -72 + 23.0/2);
    public static Pose2d BLUE_GARDEN = new Pose2d(-72 + 1, 72 - 23.0/2);
    public static Pose RED_SHOOT = new Pose(-24, 48);
    public static Pose BLUE_SHOOT = new Pose(24, -48);
    // PARK scores when the robot is partly in its LOADING ZONE: 23 x 11 in on its own alliance wall, between the
    // tile seams 1 and 2 tiles from the drivers'-left corner. Robot parallel to the wall, 4 in off it so it
    // overlaps the zone by ~7 in and still isn't touching the wall (LEAVE).
    public static Pose RED_PARK = new Pose(-36, -72 + Globals.ROBOT_WIDTH/2 + 4);
    public static Pose BLUE_PARK = new Pose(36, 72 - Globals.ROBOT_WIDTH/2 - 4);


    public static Pose2d RED_START = new Pose2d(0, -72+Globals.ROBOT_FORWARD_LENGTH/2);
    public static Pose2d BLUE_START = new Pose2d(0, 72-Globals.ROBOT_FORWARD_LENGTH/2);

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
    public abstract Pose2d farFlower();
    public abstract int[] audienceAprilTags();
    public abstract int[] scoringAprilTags();
    public abstract Pose2d garden();
    public abstract Pose2d start();
    public abstract Pose shoot();
    public abstract Pose park();

    public static void setSide(Side sideOfTheField)
    {
        side = sideOfTheField;
        Globals.isRed = sideOfTheField == Side.RED; // vision and aiming read this, keep them in sync
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

