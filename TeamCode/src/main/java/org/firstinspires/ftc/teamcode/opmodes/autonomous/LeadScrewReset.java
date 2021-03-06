package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name="Lead Screw Reset",group="auto")
public class LeadScrewReset extends AlmondLinear {

    @Override
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        waitForStart();
        resetLatch();

    }
}