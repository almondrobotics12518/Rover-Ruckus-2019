package org.firstinspires.ftc.teamcode.opmodes;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.control.constants.DriveConstants;

import org.firstinspires.ftc.teamcode.control.motion.PID;

import java.security.cert.CertStoreParameters;
import java.sql.ResultSet;

import static org.firstinspires.ftc.teamcode.control.constants.DriveConstants.TICKS_PER_DEGREE;
import static org.firstinspires.ftc.teamcode.control.constants.DriveConstants.TICKS_PER_INCH;

public abstract class AlmondLinear extends LinearOpMode
{
    public float imuOffset;
    public float globalAngle;


    public int lfEnc = 0;
    public int lbEnc = 0;
    public int rbEnc = 0;
    public int rfEnc = 0;

    public float currentAngle;
    public float lastAngle;
    // Gyro variable declaration

    public BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    public BNO055IMU imu;

    //dogecv detect declaration


    public GoldAlignDetector detector;

    /*  -------------------------
        Declare drivetrain motors
        ------------------------- */

    public DcMotor leftFront;
    public DcMotor  rightFront;
    public DcMotor leftBack;
    public DcMotor rightBack;

    /*  -------------------------------------------
        Declare scoring mechanism servos and motors.
        ------------------------------------------- */

    public DcMotor armLeft;
    public DcMotor armRight;
    public DcMotor slide;
    public CRServo intake;

    /* -------------------------------------
        Declare hanging lead screw motor.
       ------------------------------------- */

    public DcMotor lScrew;

    /*  -------------------------
        Declare team Marker servo.
        ------------------------- */

    public Servo teamMarker;

    public boolean isRunning = true;
    public boolean isAuto = true;

    /**
     * Sets hardwaremap for every motor.
     */
    final public void hardwareMap()
    {

        leftFront = hardwareMap.dcMotor.get("LeftFront");
        leftBack = hardwareMap.dcMotor.get("LeftBack");
        rightFront = hardwareMap.dcMotor.get("RightFront");
        rightBack = hardwareMap.dcMotor.get("RightBack");
        intake = hardwareMap.crservo.get("intake");
        lScrew = hardwareMap.dcMotor.get("LScrew");
        teamMarker = hardwareMap.servo.get("tm");
        slide = hardwareMap.dcMotor.get("Slide");
        armRight = hardwareMap.dcMotor.get("ArmRight");
        armLeft = hardwareMap.dcMotor.get("ArmLeft");
        armRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (isAuto)
        {
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        lScrew.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets power of each motor
     * @param lf left front power
     * @param lb left back power
     * @param rf right front power
     * @param rb right back power
     */
    public void setPower(double lf, double lb, double rf, double rb)
    {
        leftFront.setPower(lf);
        leftBack.setPower(lb);
        rightFront.setPower(rf);
        rightBack.setPower(rb);
    }

    /**
     * Sets all motors to the same powers specified by
     * @param power power sent to motors
     */
    public void setPowerAll(double power){
        leftFront.setPower(power);
        leftBack.setPower(power);
        rightFront.setPower(power);
        rightBack.setPower(power);
    }

    /**
     * Method that sets power of motors to travel in a
     * specified direction
     * @param power velocity or amount of power to motor
     * @param direction direction of travel (forward,left,right,backward)
     */
    public void setPowerDirection(double power, Direction direction)
    {
        switch(direction)
        {
            case FORWARD:
                setPower(power,power,power,power);
                break;
            case BACK:
                setPower(-power,-power,-power,-power);
                break;
            case LEFT:
                setPower(-power,power,power,-power);
                break;
            case RIGHT:
                setPower(power,-power,-power,power);
                break;
        }
    }

    /**
     * Stores encoder values as variables.
     */
    public void updateEncoders(){
        lfEnc = leftFront.getCurrentPosition();
        lbEnc = leftBack.getCurrentPosition();
        rfEnc = rightFront.getCurrentPosition();
        rbEnc = rightBack.getCurrentPosition();
    }

    /**
     * Enumeration for the 4 cardinal
     * directions.
     */
    public enum Direction{
        FORWARD,
        BACK,
        LEFT,
        RIGHT
    }

    /**
     * Sets the run mode of the motors to
     * RUN_USING_ENCODER
     */
    public void setModeRunUsingEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveToDistance(double inches){
        int target = (int)(inches*TICKS_PER_INCH);
        PIDDrive(target,target,target,target);
    }

    /**
     * Turns to a certain angle using encoder ticks.
     * @param angle target angle
     */
    public void encoderTurn(double angle){
        int target = (int)(angle*TICKS_PER_DEGREE);
        target+=200;
        PIDDrive(target,target,-target,-target);
    }

    /**
     * turns to a target angle based on imu sensor.
     * @param angle target angle
     */
    public void turn(double angle){

        double kp=0.02;
        double ki=0;
        double kd=0;
        double feedForward = 0.1;
        turnDirection direction;

        double target = (globalAngle + angle) % 360;
        double powerTurn = 0;
        double errorR = (target - getCurrentAngle())%360;
        double errorL = (getCurrentAngle()-target)%360;
        double error;
        double errorT = 0;
        double lastError = 0;


        /*
        This code determines whether clockwise or counterclockwise is closer.
         */

        if(errorR>errorL){
            error = errorL;
            direction = turnDirection.COUNTERCLOCKWISE;
        } else {
            error = errorR;
            direction = turnDirection.CLOCKWISE;
        }

        while(opModeIsActive()&&Math.abs(error)>1){
            if(direction == turnDirection.CLOCKWISE){
                error = (((target-getCurrentAngle())%360)+360)%360;
                if(error>270){
                    error-=360;
                }
            } else {
                error = (((getCurrentAngle()-target)%360)+360)%360;
                if(error>270){
                    error-=360;
                }
            }

            powerTurn = PID.calculate(kp,ki,kd,error,errorT,lastError,0,0.5);
            powerTurn += (powerTurn/Math.abs(powerTurn))*feedForward;
            if (Math.abs(powerTurn)>1){powerTurn = powerTurn/(Math.abs(powerTurn));}
            if(direction == turnDirection.CLOCKWISE){
                setPower(powerTurn,powerTurn,-powerTurn,-powerTurn);
            } else {
                setPower(-powerTurn,-powerTurn,powerTurn,powerTurn);
            }

            errorT += error;
            lastError = error;
            telemetry.addData("error",error);
            telemetry.addData("Power Variable",powerTurn);
            telemetry.addData("Left Front Power",leftFront.getPower());
            telemetry.addData("Left Back Power",leftBack.getPower());
            telemetry.addData("Right Front Power",rightFront.getPower());
            telemetry.addData("Right Back Power",rightBack.getPower());
            telemetry.addData("Current Angle",getCurrentAngle());
            telemetry.update();
        }
        globalAngle += angle;

        setPowerAll(0);

    }

    /**
     * Drives to target encoder position using PID loop.
     * Also uses built-in velocity PID.
     * @param lf left front encoder target
     * @param lb left back encoder target
     * @param rf right front encoder target
     * @param rb right back encoder target
     */

    public void PIDDrive(int lf,int lb, int rf, int rb){
        double kp = 0.003;
        double ki = 0;
        double kd = 0.0005;
        double feedForward = 0.05;
        double powerLf;
        double powerLb;
        double powerRf;
        double powerRb;
        double tarLf;
        double tarLb;
        double tarRf;
        double tarRb;
        double errorLf;
        double errorLb;
        double errorRf;
        double errorRb;
        double lastErrorLf=0;
        double lastErrorLb=0;
        double lastErrorRf=0;
        double lastErrorRb=0;
        double errorTLf=0;
        double errorTLb=0;
        double errorTRf=0;
        double errorTRb=0;
        double maxPower;

        tarLf = leftFront.getCurrentPosition()+lf;
        tarLb = leftBack.getCurrentPosition()+lb;
        tarRf = rightFront.getCurrentPosition()+rf;
        tarRb = rightBack.getCurrentPosition()+rb;

        while(opModeIsActive()&&(Math.abs(leftFront.getCurrentPosition()-tarLf)>20||
                Math.abs(rightFront.getCurrentPosition()-tarRf)>20 ||
                Math.abs(leftBack.getCurrentPosition()-tarLb)>20 ||
                Math.abs(rightBack.getCurrentPosition()-tarRb)>20)){
            maxPower = 1;

            errorLf = tarLf - leftFront.getCurrentPosition();
            errorLb = tarLb - leftBack.getCurrentPosition();
            errorRf = tarRf - rightFront.getCurrentPosition();
            errorRb = tarRb - rightBack.getCurrentPosition();

            errorTLf += errorLf;
            errorTLb += errorLb;
            errorTRf += errorRf;
            errorTRb += errorRb;

            powerLf = PID.calculate(kp,ki,kd,errorLf,errorTLf,lastErrorLf,200,20);
            powerLb = PID.calculate(kp,ki,kd,errorLb,errorTLb,lastErrorLb,200,20);
            powerRf = PID.calculate(kp,ki,kd,errorRf,errorTRf,lastErrorRf,200,20);
            powerRb = PID.calculate(kp,ki,kd,errorRb,errorTRb,lastErrorRb,200,20);

            powerLf+=feedForward*(powerLf/Math.abs(powerLf));
            powerLb+=feedForward*(powerLb/Math.abs(powerLb));
            powerRf+= feedForward*(powerRf/Math.abs(powerRf));
            powerRb += feedForward*(powerRb/Math.abs(powerRb));

            if(Math.abs(powerLf)>maxPower){powerLf/=Math.abs(powerLf); powerLf *= maxPower;}
            if(Math.abs(powerLb)>maxPower){powerLb/=Math.abs(powerLb); powerLb *= maxPower;}
            if(Math.abs(powerRf)>maxPower){powerRf/=Math.abs(powerRf); powerRf *= maxPower;}
            if(Math.abs(powerRb)>maxPower){powerRb/=Math.abs(powerRb); powerRb *= maxPower;}

            setPower(powerLf,powerLb,powerRf,powerRb);

            lastErrorLb = errorLb;
            lastErrorLf = errorLf;
            lastErrorRf = errorRf;
            lastErrorRb = errorRb;

            telemetry.addData("Left Front Power",leftFront.getPower());
            telemetry.addData("Left Back Power",leftBack.getPower());
            telemetry.addData("Right Front Power",rightFront.getPower());
            telemetry.addData("Right Back Power",rightBack.getPower());
            telemetry.update();

        }
        setPowerAll(0);
        sleep(100);
    }

    /**
     * Drives to target position using built in run to position.
     * @param lf left front encoder target
     * @param lb left back encoder target
     * @param rf right front encoder target
     * @param rb right back encoder target
     * @param power the power to send to every motor
     */
    final public void driveToPosition(int lf, int lb, int rf,int rb, double power) {

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setTargetPosition(leftFront.getCurrentPosition() + lf);
        leftBack.setTargetPosition(lb + leftBack.getCurrentPosition());
        rightFront.setTargetPosition(rf + rightFront.getCurrentPosition());
        rightBack.setTargetPosition(rb + rightBack.getCurrentPosition());

        setPowerAll(power);

        while (opModeIsActive() &&
                leftBack.isBusy() && leftFront.isBusy()
                && rightFront.isBusy() && rightBack.isBusy())
        {


        }
    }



    public final void detectorEnable()
    {
        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        detector.enable(); // Start the detector!
    }

    //Method initializes the imu sensor.
    public void initImu()
    {

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu.initialize(parameters);
        telemetry.addData("Status","Calibrating...");
        telemetry.update();

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }


        globalAngle = getCurrentAngle();
        telemetry.addData("Status","Done calibrating. Waiting for start.");
        telemetry.update();
    }



    //This method is currently unused. Work in progress turn based on gyro angle.

    public final void setModeAuto() { this.isAuto = true; }

    public final void setModeTeleOp() { this.isAuto = false; }

    //Unused enum for position of mineral in sampling.
    public enum mineralPosition {
        LEFT,
        RIGHT,
        MIDDLE,
        UNKNOWN,
    }
    //This method resets all encoders that are used.
    public void resetEncoders(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lScrew.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public float getCurrentAngle(){
        return (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle*-1)+180;
    }

    public enum turnDirection{
        CLOCKWISE,
        COUNTERCLOCKWISE
    }

    public void unlatch(){
        lScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lScrew.setPower(-1);
        int current = lScrew.getCurrentPosition();
        int target = current -26800;
        while(opModeIsActive()&&lScrew.getCurrentPosition()>target){
            telemetry.addData("Position",lScrew.getCurrentPosition());
            telemetry.update();
        }
        lScrew.setPower(0);
    }

    public void resetLatch(){
        lScrew.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int current = lScrew.getCurrentPosition();
        lScrew.setPower(1);
        int target = current+26800;
        while(opModeIsActive()&&lScrew.getCurrentPosition() < target){
            telemetry.addData("Position",lScrew.getCurrentPosition());
            telemetry.addData("Target",target);
            telemetry.update();
        }
        lScrew.setPower(0);
    }
}
