package org.firstinspires.ftc.teamcode.opmodes.test;

import org.firstinspires.ftc.teamcode.control.motion.PID;
import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

public class EndAutoTest extends AlmondLinear {
    double armPower;
    int armError;
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        while(opModeIsActive()&&isRunning){
            armError = -1000 - armLeft.getCurrentPosition();
            armPower = PID.calculate(0.001,0,0, armError,0,0,0,0);
            setAngle(180);
            telemetry.addData("Arm Position",armLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}
