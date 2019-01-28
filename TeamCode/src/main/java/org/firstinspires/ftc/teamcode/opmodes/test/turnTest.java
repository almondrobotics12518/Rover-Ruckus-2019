package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(group = "test",name = "Turn Test")
public class turnTest extends AlmondLinear {
    public void runOpMode(){
        hardwareMap();
        initImu();
        waitForStart();
        float origin = getCurrentAngle();
        PIDDrive(1000,1000,-1000,-1000);
        float end = getCurrentAngle();
        while(opModeIsActive()) {
            telemetry.addData("Start", origin);
            telemetry.addData("End", end);
            telemetry.addData("Current",getCurrentAngle());
            telemetry.update();
        }
    }
}
