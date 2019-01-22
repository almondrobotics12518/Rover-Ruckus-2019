package org.firstinspires.ftc.teamcode.control.motion;

/**
 *
 * Class to calculate PID values
 * Created by Ronnie on 1/21/19
 *
 */

public class PIDCalculate {

    public double calculate(double kp, double ki, double kd, double error, double errorT, double lastError, double integralZone, double zeroZone){
        double control;
        double proportion = kp*error;
        double integral = ki*errorT;
        double derivative = kd*(error-lastError);

        if(Math.abs(integral)>integralZone)
        {
            integral = 0;
        }

        if(Math.abs(error) < zeroZone){
            return 0;
        }

        if(error==0){
            derivative = 0;
            integral = 0;
        }
        control = proportion + integral + derivative;

        return control;
    }
}
