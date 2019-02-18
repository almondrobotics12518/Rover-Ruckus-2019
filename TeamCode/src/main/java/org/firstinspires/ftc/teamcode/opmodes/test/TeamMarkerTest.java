package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

public class TeamMarkerTest extends AlmondLinear {
    public void runOpMode(){
        hardwareMap();
        waitForStart();
        teamMarker.setPosition(0);
        sleep(500);
        teamMarker.setPosition(0.2);
        sleep(500);
        teamMarker.setPosition(0.4);
        sleep(500);
        teamMarker.setPosition(0.6);
        sleep(500);
        teamMarker.setPosition(0.8);
        sleep(500);
        teamMarker.setPosition(1);
        sleep(500);
    }
}
