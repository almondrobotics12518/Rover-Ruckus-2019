package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name="Depot Side New",group = "auto")
public class DepotSideNew extends AlmondLinear {

    public void runOpMode() throws InterruptedException {

        hardwareMap();
        setModeRunUsingEncoders();
        waitForStart();
        lScrew.setPower(1);
        sleep(9000);
        lScrew.setPower(0);
        while(opModeIsActive()&&isRunning)
            switch(scan()){
                case MIDDLE:
                    encoderTurn(90);
                    driveToDistance(32);
                    driveToDistance(-32);
                    break;
                case LEFT:
                    encoderTurn(60);
                    driveToDistance(37);
                    driveToDistance(-37);
                    encoderTurn(30);
                    break;
                case RIGHT:
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
            driveToDistance(60);

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
