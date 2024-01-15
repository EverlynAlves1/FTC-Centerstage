package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunnerquickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@Autonomous(name = "BLTrajectory")
public class BLTrajectory extends LinearOpMode {

    private DestemidosBot robot;
    private SampleMecanumDrive drive;

    TrajectorySequence MidSpikeMark;
    TrajectorySequence LeftSpikeMark;
    TrajectorySequence RightSpikeMark;
    TrajectorySequence PixelDeposity;
    TrajectorySequence PixelCollect;




    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        drive = new SampleMecanumDrive(hardwareMap);
        robot = new DestemidosBot(hardwareMap);

        //TRAJETORIAS
        MidSpikeMark = drive.trajectorySequenceBuilder(new Pose2d(16, -60, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-36, -34, Math.toRadians(90)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-52, -34, Math.toRadians(0)))
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(52, -34, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(-52, -34, Math.toRadians(0)))
                .waitSeconds(1)
                .lineToLinearHeading(new Pose2d(52, -34, Math.toRadians(0)))
                .waitSeconds(1)
                .build();

        LeftSpikeMark = drive.trajectorySequenceBuilder(new Pose2d(16, -60, Math.toRadians(90)))
                .turn(Math.toRadians(40))
                .build();

        RightSpikeMark = drive.trajectorySequenceBuilder(new Pose2d(16, -60, Math.toRadians(90)))
                .turn(Math.toRadians(100))
                .back(8)
                .build();

        PixelDeposity = drive.trajectorySequenceBuilder(MidSpikeMark.end())
                .turn(Math.toRadians(-190))
                .back(8)
                .build();

        PixelCollect = drive.trajectorySequenceBuilder(PixelDeposity.end())
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    robot.servo.fistServoRotation(-250);
                })
                .build();


        waitForStart();

        drive.followTrajectorySequence(MidSpikeMark);

        /*switch (drive){
            case "Meio":

                break;
            case "Direita":

                break;
            case "Esquerda":

                break;
        } */




        terminateOpModeNow();


    }
}

