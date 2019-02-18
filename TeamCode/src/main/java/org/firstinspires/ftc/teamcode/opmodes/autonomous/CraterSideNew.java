package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name = "Crater Side New", group = "auto")
public class CraterSideNew extends AlmondLinear {
    public void runOpMode() throws InterruptedException {

        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(0.4);
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
                turn(90); // turns towards gold
                driveToDistance(22); // pushes gold
                driveToDistance(-12); // goes away from gold
                turn(-80); // turns towards  side

            } else {
                turn(30); // turns towards position 2
                if(detector.isFound() && detector.getWidth()>20){
                    detector.disable();
                    turn(60); // face mid
                    driveToDistance(10); // goes mid way
                    turn(45); // turns to 2nd position
                    driveToDistance(20); // pushes gold
                    driveToDistance(-20); //  goes back
                    turn(-130); // turns towards side
                    // an attempt
                    //turn(120);
                    //driveToDistance(24);
                    //driveToDistance(-10);
                    //turn(-110);
                } else {
                    detector.disable();
                    turn(60); // face mid
                    driveToDistance(10); //goes mid way
                    turn(-45); // turns to 3rd position
                    driveToDistance(20); //pushes gold
                    driveToDistance(-20); // goes back
                    turn(-35); // turns to side
                    // an attempt
                    //turn(-30);
                    //turn(75);
                    //driveToDistance(24);
                    //driveToDistance(-10);
                    //turn(-130);

                }
            }

            /*
             * Turns based on the position returned by sampling
             * above.
             */

            driveToDistance(45);
            turn(-55);
            driveToDistance(32);
            turn(-180);
            teamMarker.setPosition(1);
            sleep(300);
            driveToDistance(45);

            slide.setPower(-1);
            sleep(800);
            slide.setPower(0);

            detector.disable();
            isRunning = false;
        }
    }
}
