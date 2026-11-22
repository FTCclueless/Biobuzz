package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.field.BiobuzzField;

@Autonomous(name = "Red Leave + Shoot", group = "Simple")
public class RedLeaveShoot extends SimpleAuto {
    @Override protected BiobuzzField.Side side() { return BiobuzzField.Side.RED; }
    @Override protected Mode mode() { return Mode.SHOOT; }
}
