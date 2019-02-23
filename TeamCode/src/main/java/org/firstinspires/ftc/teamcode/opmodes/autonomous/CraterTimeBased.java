package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

import static org.firstinspires.ftc.teamcode.control.constants.DriveConstants.VELOCITY;
import static org.firstinspires.ftc.teamcode.opmodes.AlmondLinear.mineralPosition.LEFT;
import static org.firstinspires.ftc.teamcode.opmodes.AlmondLinear.mineralPosition.MIDDLE;
import static org.firstinspires.ftc.teamcode.opmodes.AlmondLinear.mineralPosition.RIGHT;
@Autonomous(name = "Crater Time Based",group = "auto")
public class CraterTimeBased extends AlmondLinear {
    private ElapsedTime timer;
    public void runOpMode() throws InterruptedException {
        hardwareMap();
        setModeRunUsingEncoders();
        teamMarker.setPosition(0.3);
        timer = new ElapsedTime();
        initImu();
        waitForStart();
        while(opModeIsActive()&&isRunning){
            unlatch();
            globalAngle = getCurrentAngle();
            detectorEnable();
            PIDDrive(-100, -100, -100, -100);
            PIDDrive(250, -250, -250, 250);
            PIDDrive(150, 150, 150, 150);
            scan();
            turn(-45);
            turn(0);
            driveToDistance(24);
            double innerArcLength = 2 * 9 * Math.PI/4;
            double outerArcLength = 2 * 27 * Math.PI/4;
            int time = (int) (outerArcLength/VELOCITY) * 1000;
            double rightSpeed = (innerArcLength/outerArcLength) * 0.5;
            double leftSpeed = 0.5;
            double rightPower;
            double leftPower;
            double scale;
            timer.reset();
            while(opModeIsActive() && timer.milliseconds()>500){
                scale = timer.milliseconds()/500;
                rightPower = rightSpeed * scale;
                leftPower = 0.5 * scale;
                setPower(leftPower,leftPower,rightPower,rightPower);
            }
            while(opModeIsActive() && timer.milliseconds() < time){
                rightPower = rightSpeed;
                leftPower = leftSpeed;
                setPower(leftPower,leftPower,rightPower,rightPower);
            }
            driveToDistance(48);
            turn(180);
            teamMarker.setPosition(1);
            timer.reset();

        }
    }

    public mineralPosition scan(){
        mineralPosition position;
        if(detector.isFound()&&detector.getWidth()>35){
            position = MIDDLE;
        } else {
            turn(30);
            if(detector.isFound()&&detector.getWidth()>20){
                position = RIGHT;
                turn(-30);
            } else {
                position = LEFT;
                turn(-30);
            }
        }
        detector.disable();
        return position;
    }
}
