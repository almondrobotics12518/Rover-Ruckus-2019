package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name = "Depot Side New",group = "auto")
public class DepotSideNew extends AlmondLinear {
    public void runOpMode() throws InterruptedException{
        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(0.8);
        mineralPosition position;

        waitForStart();
        unlatch();
        PIDDrive(-150,-150,-150,-150);
        PIDDrive(250,-250,-250,250);
        PIDDrive(150,150,150,150);
        initImu();

        /*
         * Sampling code that scans and returns a position
         * for the mineral.
         *
         */
        detectorEnable();
        if(detector.isFound()){
            position = mineralPosition.MIDDLE;
        } else {
            turn(30);
            if(detector.isFound()){
                position = mineralPosition.RIGHT;
                turn(-30);
            } else {
                position = mineralPosition.LEFT;
                turn(-30);
            }
        }
        detector.disable();


        /*
         * Turns based on the position returned by sampling
         * above.
         */


        turn(90);
        switch(position){
            case MIDDLE:
                driveToDistance(10);
                slide.setPower(-1);
                sleep(800);
                slide.setPower(1);
                sleep(1000);
                turn(-80);
                break;
            case LEFT:
                driveToDistance(10);
                turn(-45);
                slide.setPower(-1);
                sleep(800);
                slide.setPower(1);
                sleep(1000);
                turn(-35);
                break;
            case RIGHT:
                driveToDistance(10);
                turn(45);
                slide.setPower(-1);
                sleep(800);
                slide.setPower(1);
                sleep(1000);
                turn(-125);
        }


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
