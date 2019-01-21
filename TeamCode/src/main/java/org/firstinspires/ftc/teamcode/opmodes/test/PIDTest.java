package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name = "SideWaysTest",group = "test")
public class PIDTest extends AlmondLinear {

    public void runOpMode() throws InterruptedException
    {

        hardwareMap();
        setModeRunUsingEncoders();
        waitForStart();
        while(opModeIsActive()&&isRunning){
            encoderDrive(10000,-10000,-10000,10000,1);
            encoderDrive(-10000,10000,10000,-10000,1);
            isRunning=false;
        }
        setPowerAll(0);
    }
}