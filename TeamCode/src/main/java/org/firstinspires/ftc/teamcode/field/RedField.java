package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class RedField extends BiobuzzField
{
    @Override
    public Pose2d pivot() {
        return RED_PIVOT;
    }

    @Override
    public Pose2d flowerAllyStation() {
        return RED_FLOWER_ALLY_STATION;
    }

    @Override
    public Pose2d flowerSide() {
        return RED_FLOWER_SIDE;
    }

    @Override
    public int[] audienceAprilTags() {
        return RED_AUDIENCE_APRIL_TAGS;
    }

    @Override
    public int[] scoringAprilTags() {
        return RED_SCORING_APRIL_TAGS;
    }

    @Override
    public Pose2d garden() {
        return RED_GARDEN;
    }

    @Override
    public Pose2d start() {
        return RED_START;
    }
}
