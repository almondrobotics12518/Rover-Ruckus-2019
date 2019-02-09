package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name="Depot Less Turn",group="auto")
public class DepotLessTurn extends AlmondLinear {

    public void runOpMode() throws InterruptedException {
        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(1);
        waitForStart();
        while(opModeIsActive()&&isRunning){
            unlatch();
            detectorEnable();
            PIDDrive(-100, -100, -100, -100);
            PIDDrive(250, -250, -250, 250);
            PIDDrive(150, 150, 150, 150);
            initImu();
            globalAngle -= 3;
            turn(60);
            driveToDistance(24);
            turn(-15);
            driveToDistance(-14);


            isRunning=false;

        }

    }

}
