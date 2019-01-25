package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

public class turnTest extends AlmondLinear {
    public void runOpMode(){
        hardwareMap();
        initImu();
        waitForStart();
        float origin = getCurrentAngle();
        turn(90);
        float end = getCurrentAngle();
        telemetry.addData("Start",origin);
        telemetry.addData("End",end);
        telemetry.update();
    }
}
