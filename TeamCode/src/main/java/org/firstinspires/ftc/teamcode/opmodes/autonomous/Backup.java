package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(group="opmodes",name="Back-up")
public class Backup extends AlmondLinear {
    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        waitForStart();
        lScrew.setPower(-1);
        sleep(5500);

    }
}