package org.firstinspires.ftc.teamcode.field;

import org.firstinspires.ftc.teamcode.utils.Pose2d;

public class BlueField extends BiobuzzField
{
    @Override
    public Pose2d pivot() {
        return BLUE_PIVOT;
    }

    @Override
    public Pose2d flowerAllyStation() {
        return BLUE_FLOWER_ALLY_STATION;
    }

    @Override
    public Pose2d flowerSide() {
        return BLUE_FLOWER_SIDE;
    }

    @Override
    public int[] audienceAprilTags() {
        return BLUE_AUDIENCE_APRIL_TAGS;
    }

    @Override
    public int[] scoringAprilTags() {
        return BLUE_SCORING_APRIL_TAGS;
    }

    @Override
    public Pose2d garden() {
        return BLUE_GARDEN;
    }

    @Override
    public Pose2d start() {
        return BLUE_START;
    }
}

