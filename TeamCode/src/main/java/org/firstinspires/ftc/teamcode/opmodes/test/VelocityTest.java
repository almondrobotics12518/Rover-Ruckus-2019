package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmodes.AlmondLinear;

@Autonomous(name="Velocity Test", group="test")
public class VelocityTest extends AlmondLinear {
    ElapsedTime timer;
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    double average = 0;
    boolean isRunning = true;
    public void runOpMode() throws InterruptedException {

        leftFront = (DcMotorEx)hardwareMap.get(DcMotor.class,"LeftFront");
        leftBack = (DcMotorEx)hardwareMap.get(DcMotor.class,"LeftBack");
        rightFront = (DcMotorEx)hardwareMap.get(DcMotor.class,"RightFront");
        rightBack = (DcMotorEx)hardwareMap.get(DcMotor.class,"RightBack");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()&& isRunning){
            timer = new ElapsedTime();
            timer.reset();



            leftFront.setPower(0.5);
            leftBack.setPower(0.5);
            rightFront.setPower(0.5);
            rightBack.setPower(0.5);

            while(timer.milliseconds()<3000 && opModeIsActive()){
                telemetry.addData("Left Front Velocity",leftFront.getVelocity());
                telemetry.addData("Right Front Velocity",rightFront.getVelocity());
                telemetry.addData("Left Back Velocity",leftBack.getVelocity());
                telemetry.addData("Right Back Velocity",rightBack.getVelocity());
                telemetry.addData("Time",timer.milliseconds());
                average = (leftFront.getVelocity() + leftBack.getVelocity() + rightFront.getVelocity() + rightBack.getVelocity())-8;
                telemetry.addData("average",average);
                telemetry.update();
            }

            leftFront.setPower(0);
            leftBack.setPower(0);
            rightFront.setPower(0);
            rightBack.setPower(0);


            isRunning = false;
        }
    }
}