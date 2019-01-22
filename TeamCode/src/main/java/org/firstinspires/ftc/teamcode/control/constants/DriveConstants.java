package org.firstinspires.ftc.teamcode.control.constants;

/**
 * Class that stores constants for drivetrain control
 * Created by Ronnie on 1/21/2019
 */

public class DriveConstants {

    //Encoder values per revolution and per inch calculated

    public final static int TICKS_PER_REVOLUTION = 1680;
    public final static int WHEEL_DIAMETER = 4;

    public final static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
    public final static double TICKS_PER_INCH = TICKS_PER_REVOLUTION/WHEEL_CIRCUMFERENCE;

}
