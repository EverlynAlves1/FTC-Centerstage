/*package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.roadrunnerquickstart.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.AutonomoSystem;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;
import org.firstinspires.ftc.teamcode.subsystems.SimpleArm;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "Rota Final")
public class RotaFinal extends LinearOpMode {

    //DEFININDO VARIÁVEIS.

    private OpenCvWebcam camera;

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private DestemidosBot robot;

    // configurando o hardware
    private AutonomoSystem driveAuto;
    private CRServoImplEx wristServoA;
    private CRServoImplEx wristServoB;
    public DcMotorEx gripper;
    private ElapsedTime wristTimer;

    int mult = 1;
    boolean esquerda = false;
    int voltar = 0;
    boolean aliancavermelha = false;


    TrajectorySequence ajuste;
    TrajectorySequence meio1;
    TrajectorySequence meio2;
    TrajectorySequence esquerda1;
    TrajectorySequence esquerda2;
    TrajectorySequence direita1;
    TrajectorySequence direita2;
    TrajectorySequence sobePunho;
    TrajectorySequence descePunho;
    TrajectorySequence abreDireita;
    TrajectorySequence fechaDireita;
    TrajectorySequence abreEsquerda;
    TrajectorySequence fechaEsquerda;
    TrajectorySequence backstage;



    //OP MODE

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        // inicializa os sistemas
        robot = new DestemidosBot(hardwareMap);
        SimpleArm simpleArm = new SimpleArm(hardwareMap);
        wristTimer = new ElapsedTime();
        driveAuto = new AutonomoSystem(
                robot.drivetrain,
                robot.localizationSystem,
                robot.voltageSensor);

        leftMotor = hardwareMap.get(DcMotorEx.class, "fore");
        rightMotor = hardwareMap.get(DcMotorEx.class, "forearm");

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wristServoA = (CRServoImplEx) hardwareMap.get(CRServo.class, "servoD");
        wristServoB = (CRServoImplEx) hardwareMap.get(CRServo.class, "servoE");

        wristServoA.setDirection(DcMotorSimple.Direction.FORWARD);
        wristServoB.setDirection(DcMotorSimple.Direction.REVERSE);

        wristServoA.setPwmRange(RobotConstants.MAX_SERVO_RANGE);
        wristServoB.setPwmRange(RobotConstants.MAX_SERVO_RANGE);

        //MULTIPLICADORES DE ROTA (ALIANÇA E LADO)

        if (aliancavermelha = true){
            mult = -1;

        }

        if (esquerda = true){
            voltar = 80;

        }




        //TRAJETORIAS
        ajuste = driveAuto.trajectorySequenceBuilder(new Pose2d())
                .forward(30)
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    robot.servo.openWrist();
                })
                .build();

        meio1 = driveAuto.trajectorySequenceBuilder(ajuste.end())
                .turn(Math.toRadians(40))
                .build();

        esquerda1 = driveAuto.trajectorySequenceBuilder(meio1.end())
                .turn(Math.toRadians(-100))
                .back(8)
                .build();

        direita1 = driveAuto.trajectorySequenceBuilder(esquerda1.end())
                .turn(Math.toRadians(190))
                .back(8)
                .build();

        sobePunho = driveAuto.trajectorySequenceBuilder(direita1.end())
                .waitSeconds(0.5)
                .addDisplacementMarker(() -> {
                    robot.servo.fistServoRotation(-250);
                })
                .build();


        descePunho = driveAuto.trajectorySequenceBuilder(sobePunho.end())
                .waitSeconds(1.5)
                .addDisplacementMarker(() -> {
                    robot.servo.fistServoRotation(160);
                    robot.servo.moveMonheca(0);
                })
                .waitSeconds(1)
                .build();

        abreDireita = driveAuto.trajectorySequenceBuilder(descePunho.end())
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.servo.wristServoA.setPosition(RobotConstants.MIN_SERVO_POSITION);
                })
                .waitSeconds(0.5)
                .build();

        fechaDireita = driveAuto.trajectorySequenceBuilder(abreDireita.end())
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.servo.wristServoA.setPosition(RobotConstants.MAX_SERVO_POSITION);
                })
                .build();

        abreEsquerda = driveAuto.trajectorySequenceBuilder(fechaDireita.end())
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.servo.wristServoB.setPosition(RobotConstants.MIN_SERVO_POSITION);
                })
                .build();

        fechaEsquerda = driveAuto.trajectorySequenceBuilder(abreEsquerda.end())
                .waitSeconds(1)
                .addDisplacementMarker(() -> {
                    robot.servo.wristServoB.setPosition(RobotConstants.MAX_SERVO_POSITION);
                })
                .build();

        backstage = driveAuto.trajectorySequenceBuilder(fechaEsquerda.end())
                .back(26)
                .turn(Math.toRadians(-225))
                .back(120)
                .strafeRight(-30)
                .build();

        meio2 = driveAuto.trajectorySequenceBuilder(backstage.end())
                .turn(Math.toRadians(-40))
                .build();

        esquerda2 = driveAuto.trajectorySequenceBuilder(meio2.end())
                .turn(Math.toRadians(100))
                .forward(8)
                .build();

        direita2 = driveAuto.trajectorySequenceBuilder(esquerda2.end())
                .forward(8)
                .turn(Math.toRadians(-200))
                .build();








        switch (ava.linha){
            case "Meio":
                driveAuto.followTrajectorySequence(ajuste);
                driveAuto.followTrajectorySequence(meio1);
                driveAuto.followTrajectorySequence(descePunho);
                driveAuto.followTrajectorySequence(abreEsquerda);
                driveAuto.followTrajectorySequence(fechaEsquerda);
                driveAuto.followTrajectorySequence(sobePunho);
                driveAuto.followTrajectorySequence(meio2);
                driveAuto.followTrajectorySequence(backstage);
                driveAuto.followTrajectorySequence(descePunho);
                driveAuto.followTrajectorySequence(abreDireita);
                driveAuto.followTrajectorySequence(fechaDireita);
                driveAuto.followTrajectorySequence(sobePunho);
                break;
            case "Direita":
                driveAuto.followTrajectorySequence(ajuste);
                driveAuto.followTrajectorySequence(direita1);
                driveAuto.followTrajectorySequence(descePunho);
                driveAuto.followTrajectorySequence(abreEsquerda);
                driveAuto.followTrajectorySequence(fechaEsquerda);
                driveAuto.followTrajectorySequence(sobePunho);
                driveAuto.followTrajectorySequence(direita2);
                driveAuto.followTrajectorySequence(backstage);
                driveAuto.followTrajectorySequence(descePunho);
                driveAuto.followTrajectorySequence(abreDireita);
                driveAuto.followTrajectorySequence(fechaDireita);
                driveAuto.followTrajectorySequence(sobePunho);
                break;
            case "Esquerda":
                driveAuto.followTrajectorySequence(ajuste);
                driveAuto.followTrajectorySequence(esquerda1);
                driveAuto.followTrajectorySequence(descePunho);
                driveAuto.followTrajectorySequence(abreEsquerda);
                driveAuto.followTrajectorySequence(fechaEsquerda);
                driveAuto.followTrajectorySequence(sobePunho);
                driveAuto.followTrajectorySequence(esquerda2);
                driveAuto.followTrajectorySequence(backstage);
                driveAuto.followTrajectorySequence(descePunho);
                driveAuto.followTrajectorySequence(abreDireita);
                driveAuto.followTrajectorySequence(fechaDireita);
                driveAuto.followTrajectorySequence(sobePunho);
                break;
        }




        terminateOpModeNow();


    }
}
*/
