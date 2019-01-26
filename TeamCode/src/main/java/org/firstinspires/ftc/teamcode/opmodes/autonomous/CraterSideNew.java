package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name = "Crater Side New", group = "auto")
public class CraterSideNew extends AlmondLinear{
    public void runOpMode() throws InterruptedException {
        mineralPosition position = mineralPosition.UNKNOWN;
        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(0.4);
        waitForStart();


        while (opModeIsActive() && isRunning) {
            lScrew.setPower(1);
            sleep(9000);
            lScrew.setPower(0);
            PIDDrive(-300, -300, -300, -300);
            PIDDrive(500, -500, -500, 500);
            PIDDrive(300, 300, 300, 300);
            detectorEnable();
            if (detector.isFound() && detector.getWidth() > 30) {
                driveToDistance(32);


            } else {
                encoderTurn(90); // Moves clockwise
                sleep(500);
            }
                if (detector.isFound() && detector.getWidth() > 30) {


                } else {
                encoderTurn(-180);

                }


            }
            
        detector.disable();
        isRunning = false;

        }
    }
