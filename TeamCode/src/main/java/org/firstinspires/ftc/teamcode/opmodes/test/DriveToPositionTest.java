package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name="Drive Position Test",group = "test")
public class DriveToPositionTest extends AlmondLinear {
    public void runOpMode() throws InterruptedException{
        hardwareMap();
        setModeRunUsingEncoders();
        waitForStart();
        PIDDrive(1000,1000,1000,1000);
        setPowerAll(0);
        requestOpModeStop();
    }
}
