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

        switch(position){
            case RIGHT:
                turn(120);
                slide.setPower(-1);
                sleep(800);
                slide.setPower(0);
                slide.setPower(1);
                sleep(1000);
                slide.setPower(0);
                turn(-30);
                break;
            case MIDDLE:
                turn(90);
                slide.setPower(-1);
                sleep(800);
                slide.setPower(0);
                slide.setPower(1);
                sleep(1000);
                slide.setPower(0);
                break;
            case LEFT:
                turn(60);
                slide.setPower(-1);
                sleep(800);
                slide.setPower(0);
                slide.setPower(1);
                sleep(1000);
                slide.setPower(0);
                turn(30);

                break;
        }
        
        driveToDistance(10);

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
