package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsystems.drivetrains.SampleMecanumDriveREV;

@Autonomous(name="Crater Side Road Runner",group="auto")
public class CraterSideRoadRunner extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveBase drive = new SampleMecanumDriveREV(hardwareMap);
        Trajectory craterToDepot = drive.trajectoryBuilder()
                .splineTo(new Pose2d(-36,44,90))
                .splineTo(new Pose2d(-58,44,90))
                .waitFor(1)
                .turnTo(Math.PI/2)
                .waitFor(1)
                .splineTo(new Pose2d(0,44,270))
                .build();
        boolean isRunning = true;
        waitForStart();
        while(opModeIsActive()&&isRunning){
            drive.followTrajectory(craterToDepot);
            while(opModeIsActive()&&drive.isFollowingTrajectory()){
                drive.update();
            }
            isRunning = false;
        }

    }
}
