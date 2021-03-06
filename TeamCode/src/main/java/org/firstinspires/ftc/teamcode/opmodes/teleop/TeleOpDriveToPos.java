package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.motion.PID;

import static org.firstinspires.ftc.teamcode.control.constants.ArmConstants.OFF_SET_TICKS;
import static org.firstinspires.ftc.teamcode.control.constants.ArmConstants.TICKS_PER_DEGREE;


@TeleOp(name="TeleOp New Testing",group="teleop")
public class TeleOpDriveToPos extends LinearOpMode {
    private int targetArmPos = 900;
    private int bottomArmPos = 1700;

    double currentTime;
    double lastTime;
    int currentArmPos;
    private DcMotor lScrew;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private DcMotor slide;
    private DcMotorEx armLeft;
    private DcMotorEx armRight;
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

        ElapsedTime timer = new ElapsedTime();
        // Setting dcMotor variables to motors
        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        armLeft = (DcMotorEx)hardwareMap.get(DcMotor.class,"ArmLeft");
        armRight = (DcMotorEx)hardwareMap.get(DcMotor.class,"ArmRight");
        intake = hardwareMap.crservo.get("intake");
        slide = hardwareMap.dcMotor.get("Slide");
        lScrew = hardwareMap.dcMotor.get("LScrew");


        PIDFCoefficients armVelocityPid = new PIDFCoefficients(0,0,0,0);
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
        armLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /*leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lScrew.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        // dont need isrunning
        waitForStart();
        PIDFCoefficients pidOrig = armLeft.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        timer.reset();
        while(opModeIsActive()){
            currentTime = timer.milliseconds();
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

            /*
            armPosTicks = (armLeft.getCurrentPosition()*-1)-OFF_SET_TICKS;
            armPosAngle = armPosTicks/TICKS_PER_DEGREE;
            armPowerOffset = Math.cos(Math.toRadians(armPosAngle))*0.25;
*/
            armPosTicks = armLeft.getCurrentPosition();
            if(gamepad2.left_bumper && opModeIsActive() && gamepad2.right_stick_y == 0){
                armY = PID.calculate(0.0015,0,0,armLeft.getCurrentPosition()-targetArmPos,0,0,0,0);
            } else if(gamepad2.right_bumper && opModeIsActive() && gamepad2.right_stick_y == 0){
                armY = PID.calculate(0.0007,0,0,armLeft.getCurrentPosition()-bottomArmPos,0,0,0,0);
            } else {
                armY = gamepad2.right_stick_y * 0.2;
            }


            armLeft.setPower(armY);
            armRight.setPower(armY);




            leftFront.setPower(LF * 0.65); // Gives power to LF wheels
            leftBack.setPower(LB * 0.65); // Gives power to LB wheels
            rightFront.setPower(RF * 0.65); // Gives power to RF wheels
            rightBack.setPower(RB * 0.65); // Gives power to RB wheels

            lScrew.setPower(gamepad1.right_trigger-gamepad1.left_trigger); // Gives power to the lScrew

            intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger); //Spins the Intake

            slide.setPower(-gamepad2.left_stick_y*0.5); // Gives power to the slide

            // Add telemetry variables and updating them
            /*
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
            */
            telemetry.addData("ArmRight Power",armRight.getPower());
            telemetry.addData("ArmLeft Power",armLeft.getPower());
            telemetry.addData("Left Arm Motor Encoder",armLeft.getCurrentPosition());
            telemetry.addData("Right Arm Motor Encoder",armRight.getCurrentPosition());
            telemetry.addData("Arm y",armY);
            telemetry.addData("P,I,D (orig)", "%.04f, %.04f, %.0f, %.0f",
                    pidOrig.p, pidOrig.i, pidOrig.d,pidOrig.f);
            telemetry.addData("Arm velocity",armLeft.getVelocity());
            telemetry.addData("Gamepad 2 left stick Y",gamepad2.left_stick_y);
            telemetry.addData("Loop Time",currentTime-lastTime);
            telemetry.update();
            lastTime = currentTime;

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

