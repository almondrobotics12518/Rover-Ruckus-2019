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
        globalAngle -= 5;

        if (detector.isFound()&&detector.getWidth()>40) {
            detector.disable();
            turn(90);
            driveToDistance(20);
            driveToDistance(-10);
            turn(-80);

        } else {
            turn(30);
            if(detector.isFound()&&detector.getWidth()>20){
                detector.disable();
                turn(60);
                driveToDistance(9);
                turn(50);
                driveToDistance(18);
                driveToDistance(-18);
                turn(-130);
            } else {
                detector.disable();
                turn(60);
                driveToDistance(9);
                turn(-50);
                driveToDistance(18);
                driveToDistance(-18);
                turn(-30);

            }
        }

        /*
         * Turns based on the position returned by sampling
         * above.
         */

        driveToDistance(43);
        turn(-52);
        driveToDistance(-50);
        teamMarker.setPosition(0.4);
        sleep(400);
        driveToDistance(53);


        
        slide.setPower(-1);
        sleep(800);
        slide.setPower(0);



        }
    }

