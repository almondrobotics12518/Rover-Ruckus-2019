package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name = "Crater Side New", group = "auto")
public class CraterSideNew extends AlmondLinear {
    public void runOpMode() throws InterruptedException {

        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(1);
        waitForStart();

        while (opModeIsActive() && isRunning) {
            unlatch();
            detectorEnable();
            PIDDrive(-100, -100, -100, -100);
            PIDDrive(250, -250, -250, 250);
            PIDDrive(150, 150, 150, 150);
            initImu();

            /*
             * Sampling code that scans and returns a position
             * for the mineral.
             *
             */

            if (detector.isFound() && detector.getWidth()>40) {
                detector.disable();
                turn(90);
                driveToDistance(20);
                driveToDistance(-10);
                turn(-80);

            } else {
                turn(30);
                if(detector.isFound() && detector.getWidth()>20){
                    detector.disable();
                    turn(60);
                    driveToDistance(10);
                    turn(45);
                    driveToDistance(20);
                    driveToDistance(-20);
                    turn(-130);
                } else {
                    detector.disable();
                    turn(60);
                    driveToDistance(10);
                    turn(-45);
                    driveToDistance(20);
                    driveToDistance(-20);
                    turn(-35);
                }
            }

            /*
             * Turns based on the position returned by sampling
             * above.
             */

            driveToDistance(45);
            turn(-55);
            driveToDistance(32);
            turn(-90);
            teamMarker.setPosition(0.4);
            sleep(300);
            turn(-95);
            driveToDistance(45);

            slide.setPower(-1);
            sleep(800);
            slide.setPower(0);

            detector.disable();
            isRunning = false;
        }
    }
}
