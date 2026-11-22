package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.BiobuzzField;

@Autonomous(name = "Blue Corner Park", group = "Simple")
public class BlueCornerPark extends SimpleAuto {
    @Override protected BiobuzzField.Side side() { return BiobuzzField.Side.BLUE; }
    @Override protected Mode mode() { return Mode.CORNER_PARK; }
}
