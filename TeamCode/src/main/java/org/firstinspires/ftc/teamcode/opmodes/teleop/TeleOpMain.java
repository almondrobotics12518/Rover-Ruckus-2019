package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static org.firstinspires.ftc.teamcode.control.constants.ArmConstants.OFF_SET_TICKS;
import static org.firstinspires.ftc.teamcode.control.constants.ArmConstants.TICKS_PER_DEGREE;


@TeleOp(name="LinearTeleOp",group="teleop")
public class TeleOpMain extends LinearOpMode {

    private DcMotor lScrew;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor slide;
    private DcMotor armLeft;
    private DcMotor armRight;
    private CRServo intake;
    private double leftX=0;
    private double leftY=0;
    private double rightX=0;
    private double LF=0;
    private double RF=0;
    private double LB=0;
    private double RB=0;
    private double armY=0;
    private double rightMultiplier=0;
    private double armPosTicks;
    private double armPosAngle;
    private double armPowerOffset;
    @Override
    public void runOpMode() throws InterruptedException{
        // Setting dcMotor variables to motors
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        armLeft = hardwareMap.dcMotor.get("ArmLeft");
        armRight = hardwareMap.dcMotor.get("ArmRight");
        intake = hardwareMap.crservo.get("intake");
        slide = hardwareMap.dcMotor.get("Slide");
        lScrew = hardwareMap.dcMotor.get("LScrew");

        // Reversing direction of right side motors
        //leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armRight.setDirection(DcMotorSimple.Direction.REVERSE);
        /*leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        // dont need isrunning
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.right_bumper){
                rightMultiplier = 1;
            } else {
                rightMultiplier = 0.5;
            }
            leftX = gamepad1.left_stick_x; // Reverse left joystick's X coordinate
            leftY = -gamepad1.left_stick_y; // Reverse left joystick's Y coordinate
            rightX = -gamepad1.right_stick_x * rightMultiplier;
            double speed = Math.hypot(leftX, leftY); // Takes hypotenuse of leftX and leftY
            double angle = Math.atan2(leftY, leftX) - Math.PI / 4; // Calculates angle of direction

            LF = speed * Math.cos(angle)+rightX; // Calculates power for moving for the LF wheel
            LB = speed * Math.sin(angle)+rightX; // Calculates power for moving for the LB wheel
            RF = speed * Math.sin(angle)-rightX; // Calculates power for moving for the RF wheel
            RB = speed * Math.cos(angle)-rightX; // Calculates power for moving for the RB wheel

            armPosTicks = (armLeft.getCurrentPosition()*-1)-OFF_SET_TICKS;
            armPosAngle = armPosTicks/TICKS_PER_DEGREE;
            armPowerOffset = Math.cos(Math.toRadians(armPosAngle))*0.25;


            armY = gamepad2.right_stick_y*0.25;
            if (Math.abs(armY)==0){
                armLeft.setPower(0);
                armRight.setPower(0);
            } else {

                armLeft.setPower(armY+armPowerOffset); // Gives power to the arm
                armRight.setPower(armY+armPowerOffset);
            }


            leftFront.setPower(LF * 0.65); // Gives power to LF wheels
            leftBack.setPower(LB * 0.65); // Gives power to LB wheels
            rightFront.setPower(RF * 0.65); // Gives power to RF wheels
            rightBack.setPower(RB * 0.65); // Gives power to RB wheels

            lScrew.setPower(gamepad1.right_trigger-gamepad1.left_trigger); // Gives power to the lScrew

            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger); //Spins the Intake

            slide.setPower(-gamepad2.left_stick_y*0.5); // Gives power to the slide

            // Add telemetry variables and updating them
            telemetry.addData("FrontLeftPower",LF);
            telemetry.addData("FrontRightPower",RF);
            telemetry.addData("BackLeftPower",LB);
            telemetry.addData("BackRightPower",RB);
            telemetry.addData("Encoders --","------");
            telemetry.addData("LeftFront",leftFront.getCurrentPosition());
            telemetry.addData("LeftBack",leftBack.getCurrentPosition());
            telemetry.addData("RightFront",rightFront.getCurrentPosition());
            telemetry.addData("RightBack",rightBack.getCurrentPosition());
            telemetry.addData("Hang",lScrew.getCurrentPosition());
            telemetry.addData("ArmLeft zeroPowerBehavior",armLeft.getZeroPowerBehavior());
            telemetry.addData("ArmRight zeroPowerBehavior", armRight.getZeroPowerBehavior());
            telemetry.addData("ArmRight Power",armRight.getPower());
            telemetry.addData("ArmLeft Power",armLeft.getPower());
            telemetry.addData("Arm",armLeft.getCurrentPosition());
            telemetry.addData("Arm",armRight.getCurrentPosition());
            telemetry.addData("Arm Power Offset",armPowerOffset);
            telemetry.addData("Slide",slide.getCurrentPosition());
            telemetry.addData("Arm y",armY);
            telemetry.addData("Gamepad 2 left stick Y",gamepad2.left_stick_y);
            telemetry.update();

            // dont need this

        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        slide.setPower(0);
        armLeft.setPower(0);
        armRight.setPower(0);
        lScrew.setPower(0);



    }
}