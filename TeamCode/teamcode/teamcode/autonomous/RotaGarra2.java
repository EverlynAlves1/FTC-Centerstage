package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.roadruneerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AutonomoSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;
import org.firstinspires.ftc.teamcode.autonomous.PixelDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "RotaGarra")
public class RotaGarra2 extends OpMode {
    private DestemidosBot robot;
    boolean tagFound = true;
    boolean aliancaVermelha = false;
    int mult = 1;
    boolean curta = false;
    int voltar = 0;

    // configurando o hardware

    // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
   // OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);

    private AutonomoSystem driveAuto;

    TrajectorySequence ajuste;
    TrajectorySequence esque;
    TrajectorySequence spikeMeio2;
    TrajectorySequence spikeMeio;
    TrajectorySequence spikeMeio3;
    TrajectorySequence spikeMeio4;
    TrajectorySequence meio;
    TrajectorySequence meio2;

    ElapsedTime timer;





    @Override
    public void init() {

        robot = new DestemidosBot(hardwareMap);
        driveAuto = new AutonomoSystem(
                robot.drivetrain,
                robot.localizationSystem,
                robot.voltageSensor);
        // configurando camera

        if (aliancaVermelha) {
            mult = -1;
        }

        if (curta) {
            voltar = 50;
        }
        // configurando a pipeline
                /*
                RobotConstants.OPENCV_tagsize,
                RobotConstants.OPENCV_fx,
                RobotConstants.OPENCV_fy,
                RobotConstants.OPENCV_cx,
                RobotConstants.OPENCV_cy; */

        // começa a stream da câmera pro ftc-dashboard

        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().registerSubsystem(robot.armSystem, robot.servo);
        driveAuto.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));


        ajuste = driveAuto.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    robot.servo.closeWrist();
                })
                .forward(32)
                .build();

        meio = driveAuto.trajectorySequenceBuilder(ajuste.end())
                .turn(Math.toRadians(35 * mult))
                .forward(8)
                .build();

        spikeMeio = driveAuto.trajectorySequenceBuilder(meio.end())
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    robot.servo.moveMonheca(400);
                })
                .build();

        spikeMeio2 = driveAuto.trajectorySequenceBuilder(spikeMeio.end())
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    robot.servo.moveMonheca(-200);
                })
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.servo.openWrist();
                })
                .build();

        meio2 = driveAuto.trajectorySequenceBuilder(spikeMeio2.end())
                .back(8)
                .build();

        spikeMeio3 = driveAuto.trajectorySequenceBuilder(meio2.end())
                .turn(Math.toRadians(-35 * mult))
                .build();

        spikeMeio4 = driveAuto.trajectorySequenceBuilder(spikeMeio3.end())
                .back(24)
                .turn(Math.toRadians(-230 * mult))
                .back(110 - voltar)
                .strafeRight(30 * mult)
                .build();

       // FtcDashboard.getInstance().startCameraStream(camera, 60);

        // define o pipeline e inicia a câmera
        /*camera.openCameraDeviceAsync(new AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(
                        RobotConstants.resolutionWidth,
                        RobotConstants.resolutionHeight,
                        OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

                // habilita a telemetria no ftc-dashboard
                telemetry = new MultipleTelemetry(telemetry);
                telemetry.setMsTransmissionInterval(50);
            } */
    }

    @Override
    public void start() {

        driveAuto.followTrajectorySequence(ajuste);
            //while (true)
            // if (sensor color)
            //if (sensor color)
        driveAuto.followTrajectorySequence(meio);

        driveAuto.followTrajectorySequence(spikeMeio);

        driveAuto.followTrajectorySequence(spikeMeio2);

        driveAuto.followTrajectorySequence(meio2);

        driveAuto.followTrajectorySequence(spikeMeio3);

        driveAuto.followTrajectorySequence(spikeMeio4);
        }

    @Override
    public void loop() {

    }
}