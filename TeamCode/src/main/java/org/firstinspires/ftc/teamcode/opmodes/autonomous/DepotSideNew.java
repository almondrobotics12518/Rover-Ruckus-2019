package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name = "Depot Side New",group = "auto")
public class DepotSideNew extends AlmondLinear {
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(0.4);
        mineralPosition position;

        waitForStart();
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
        globalAngle -= 5;

        if (detector.isFound()&&detector.getFoundRect().width>40&&detector.getFoundRect().width<70&&detector.getFoundRect().y<100) {
            detector.disable();
            turn(90);
            driveToDistance(22);
            driveToDistance(-12);
            turn(-80);

        } else {
            turn(30);
            if(detector.isFound()&&detector.getFoundRect().width>20&&detector.getFoundRect().width<70&&detector.getFoundRect().y<140){
                detector.disable();
                turn(60);
                driveToDistance(10);
                turn(50);
                driveToDistance(20);
                driveToDistance(-20);
                turn(-130);
            } else {
                detector.disable();
                turn(60);
                driveToDistance(10);
                turn(-50);
                driveToDistance(20);
                driveToDistance(-20);
                turn(-30);

            }
        }

        /*
         * Turns based on the position returned by sampling
         * above.
         */

        driveToDistance(48);
        turn(-50);
        driveToDistance(-45);

        teamMarker.setPosition(1);
        sleep(300);

        driveToDistance(57);







        }
    }

