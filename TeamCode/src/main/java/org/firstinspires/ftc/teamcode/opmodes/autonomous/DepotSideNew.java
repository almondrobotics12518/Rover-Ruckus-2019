package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.motion.PID;
import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;



@Autonomous(name="Depot Side New",group = "auto")
public class DepotSideNew extends AlmondLinear {

    public void runOpMode() throws InterruptedException {
        mineralPosition position = mineralPosition.UNKNOWN;
        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(0.4);
        waitForStart();
        lScrew.setPower(1);
        sleep(9000);
        lScrew.setPower(0);
        PIDDrive(-300,-300,-300,-300);
        PIDDrive(500,-500,-500,500);
        PIDDrive(300,300,300,300);

        while(opModeIsActive()&&isRunning)
            switch(scan()){
                case MIDDLE:
                    position = mineralPosition.MIDDLE;
                    encoderTurn(90);
                    driveToDistance(32);
                    driveToDistance(-32);
                    break;
                case LEFT:
                    position = mineralPosition.LEFT;
                    encoderTurn(60);
                    driveToDistance(37);
                    driveToDistance(-37);
                    encoderTurn(30);
                    break;
                case RIGHT:
                    position = mineralPosition.RIGHT;
                    encoderTurn(120);
                    driveToDistance(37);
                    driveToDistance(-37);
                    encoderTurn(-30);
                    break;
            }

            driveToDistance(20);

            encoderTurn(90);
            driveToDistance(45.3);

            encoderTurn(45);
            driveToDistance(-54);
            teamMarker.setPosition(0.4);
            sleep(400);
            driveToDistance(48);

            encoderTurn(45);
            switch(position){
                case LEFT:
                    driveToDistance(25.5);
                    encoderTurn(-90);
                    driveToDistance(10);
                    break;
                case MIDDLE:
                    driveToDistance(34);
                    encoderTurn(-90);
                    driveToDistance(10);
                    break;
                case RIGHT:
                    driveToDistance(42.5);
                    encoderTurn(-90);
                    driveToDistance(10);
                    break;
            }
            slide.setPower(-1);
            sleep(800);
            slide.setPower(0);


    }

    public mineralPosition scan() {
        detectorEnable();
        mineralPosition position;
        sleep(100);
        if(detector.isFound()&&detector.getWidth()>40&&opModeIsActive()){
            detector.disable();
            position = mineralPosition.MIDDLE;
        } else {
            encoderTurn(30);
            if(detector.isFound()&&detector.getWidth()>40&&opModeIsActive()){
                detector.disable();
                encoderTurn(-30);
                position = mineralPosition.RIGHT;
            } else {
                detector.disable();
                encoderTurn(-30);
                position = mineralPosition.LEFT;
            }
        }
        return position;
    }
}
