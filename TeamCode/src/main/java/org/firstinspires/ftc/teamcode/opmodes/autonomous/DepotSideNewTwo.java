package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

public class DepotSideNewTwo extends AlmondLinear {
    public void runOpMode() throws InterruptedException{
        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(0.4);
        waitForStart();
        unlatch();
        PIDDrive(-300,-300,-300,-300);
        PIDDrive(500,-500,-500,500);
        PIDDrive(300,300,300,300);
        turn(90);
        driveToDistance(17);

        slide.setPower(-1);
        sleep(800);
        slide.setPower(1);
        sleep(1000);
        slide.setPower(0);

        turn(-72);
        driveToDistance(40);
        turn(-63);
        driveToDistance(-48);
        teamMarker.setPosition(0.8);
        driveToDistance(78);

        slide.setPower(-1);
        sleep(800);
        slide.setPower(0);


    }
}
