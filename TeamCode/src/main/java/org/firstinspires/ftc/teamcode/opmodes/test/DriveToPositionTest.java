package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name="Drive Position Test",group = "test")
public class DriveToPositionTest extends AlmondLinear {
    public void runOpMode() throws InterruptedException{
        hardwareMap();
        setModeRunUsingEncoders();
        waitForStart();
        driveToPosition(5000,5000,5000,5000,1);
        setPowerAll(0);
        requestOpModeStop();
    }
}
