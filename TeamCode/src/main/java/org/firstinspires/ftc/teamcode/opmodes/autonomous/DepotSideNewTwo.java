package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name = "Depot Side Two",group = "auto")
public class DepotSideNewTwo extends AlmondLinear {
    public void runOpMode() throws InterruptedException{
        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(0.8);

        waitForStart();
        unlatch();
        PIDDrive(-150,-150,-150,-150);
        PIDDrive(250,-250,-250,250);
        PIDDrive(250,250,250,250);
        initImu();
        turn(90);
        driveToDistance(12);

        slide.setPower(-1);
        sleep(800);
        slide.setPower(0);
        slide.setPower(1);
        sleep(1000);
        slide.setPower(0);

        turn(-80);
        driveToDistance(40);
        turn(-55);
        driveToDistance(-48);
        teamMarker.setPosition(0.4);
        sleep(400);
        driveToDistance(58);

        slide.setPower(-1);
        sleep(800);
        slide.setPower(0);


    }
}
