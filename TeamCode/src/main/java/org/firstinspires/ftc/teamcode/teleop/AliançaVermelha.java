package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ArmSuspend;
import org.firstinspires.ftc.teamcode.subsystems.DestemidosBot;

@TeleOp
public class AliançaVermelha extends CommandOpMode {

    private DestemidosBot robot;
    private GamepadEx player1;
    private GamepadEx player2;



    @Override
    public void initialize() {
        robot = new DestemidosBot(hardwareMap);
        robot.setBulkReadToAuto();

        player1 = new GamepadEx(gamepad1);
        player2 = new GamepadEx(gamepad2);

        CommandScheduler.getInstance().reset();

        player1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> robot.servo.wristServoPlane.setPosition(0.2)));

        player2.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenActive(new InstantCommand(() -> robot.servo.moveMonheca(1)))
                .whenInactive(new InstantCommand(() -> robot.servo.moveMonheca(0)));

        player2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenActive(new InstantCommand(() -> robot.servo.moveMonheca(-1)))
                .whenInactive(new InstantCommand(() -> robot.servo.moveMonheca(0)));

        player2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> robot.servo.openWrist()));

        player2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> robot.servo.closeWrist()));



        player2.getGamepadButton(GamepadKeys.Button.A)
                .whenActive(new InstantCommand(() -> robot.simpleArm.suspendBot(1)))
                .whenInactive(new InstantCommand(() -> robot.simpleArm.clayMotor.setPower(0)));

                //.whenPressed(new InstantCommand(() -> robot.simpleArm.suspendBot(1)));

        player2.getGamepadButton(GamepadKeys.Button.X)
                .whenActive(new InstantCommand(() -> robot.simpleArm.suspendBot(-1)))
                .whenInactive(new InstantCommand(() -> robot.simpleArm.clayMotor.setPower(0)));

        player2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ArmSuspend(robot));

       /* player2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ArmToHighJunction(robot));*/

    }


        @Override
        public void run() {
            CommandScheduler.getInstance().run();
            player2.readButtons();
            player1.readButtons();

        /*if(player2.gamepad.right_trigger > 0.0) {
            robot.gripper.gripper.setPower(player2.gamepad.right_trigger * 0.60);
        }

        if(player2.gamepad.left_trigger > 0.0) {
            robot.gripper.gripper.setPower(-player2.gamepad.left_trigger * 0.60);
        }*/

            robot.drivetrain.standardMecanumController(gamepad1);
            robot.simpleArm.forceArm(player2.gamepad.right_stick_y);

        }


    }

