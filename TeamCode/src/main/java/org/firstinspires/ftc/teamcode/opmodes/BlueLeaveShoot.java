package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.BiobuzzField;

@Autonomous(name = "Blue Leave + Shoot", group = "Simple")
public class BlueLeaveShoot extends SimpleAuto {
    @Override protected BiobuzzField.Side side() { return BiobuzzField.Side.BLUE; }
    @Override protected Mode mode() { return Mode.SHOOT; }
}
