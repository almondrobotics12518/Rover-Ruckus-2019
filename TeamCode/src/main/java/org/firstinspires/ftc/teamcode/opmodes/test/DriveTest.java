package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name="Motor Test",group ="test")
public class DriveTest extends AlmondLinear {
    public void runOpMode() throws InterruptedException{
        hardwareMap();
        isAuto = false;
        waitForStart();
        setPowerAll(1);
        while(opModeIsActive()){

        }
    }
}
