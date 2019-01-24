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
                case MIDDLE:
                    driveToDistance(34);
                    encoderTurn(-90);
                    driveToDistance(10);
                case RIGHT:
                    driveToDistance(42.5);
                    encoderTurn(-90);
                    driveToDistance(10);
            }
            slide.setPower(-1);
            sleep(800);
            slide.setPower(0);


    }

}
