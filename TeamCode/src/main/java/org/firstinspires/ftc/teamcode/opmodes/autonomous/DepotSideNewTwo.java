package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name = "Depot Side Two",group = "auto")
public class DepotSideNewTwo extends AlmondLinear {
    public void runOpMode() throws InterruptedException{
        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(0.4);

        waitForStart();
        unlatch();
        PIDDrive(-200,200,200,-200);
        PIDDrive(-300,-300,-300,-300);
        PIDDrive(500,-500,-500,500);
        PIDDrive(300,300,300,300);
        initImu();
        encoderTurn(90);
        driveToDistance(12);

        slide.setPower(-1);
        sleep(800);
        slide.setPower(1);
        sleep(1000);
        slide.setPower(0);

        encoderTurn(-72);
        driveToDistance(40);
        encoderTurn(-63);
        driveToDistance(-48);
        teamMarker.setPosition(0.8);
        driveToDistance(78);

        slide.setPower(-1);
        sleep(800);
        slide.setPower(0);


    }
}
