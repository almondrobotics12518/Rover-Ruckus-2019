package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name = "Depot Side New",group = "auto")
public class DepotSideNew extends AlmondLinear {
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(0.8);
        mineralPosition position;

        waitForStart();
        unlatch();
        detectorEnable();
        PIDDrive(-100, -100, -100, -100);
        PIDDrive(250, -250, -250, 250);
        PIDDrive(200, 200, 200, 200);
        initImu();

        /*
         * Sampling code that scans and returns a position
         * for the mineral.
         *
         */


        if (detector.isFound()) {
            detector.disable();
            turn(90);
            driveToDistance(20);
            driveToDistance(-10);
            turn(-80);

        } else {
            turn(30);
            if(detector.isFound()){
                detector.disable();
                turn(60);
                driveToDistance(10);
                turn(45);
                driveToDistance(16);
                driveToDistance(-16);
                turn(-130);
            } else {
                detector.disable();
                turn(60);
                driveToDistance(10);
                turn(-45);
                driveToDistance(16);
                driveToDistance(-16);
                turn(-35);

            }
        }

        /*
         * Turns based on the position returned by sampling
         * above.
         */

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

