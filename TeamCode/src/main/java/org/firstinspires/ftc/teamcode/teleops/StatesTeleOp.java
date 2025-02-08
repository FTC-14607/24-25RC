package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.JamalThree;
import org.firstinspires.ftc.teamcode.robots.JamalTwo;

@Config
@TeleOp(name = "JamalThree Main TeleOp", group = "Main")
public class StatesTeleOp extends LinearOpMode {

    public static double UPPER_SLIDE_MAX_SPEED = 0.9; // power
    public static double UPPER_ARM_MAX_SPEED = 400; // servo position / sec
    public static double UPPER_CLAW_PITCH_MAX_SPEED = 0.4;
    public static double LOWER_SLIDE_MAX_SPEED = 0.6; // servo position / sec

    JamalThree robot;
    boolean showTelemetry = true;
    ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot = new JamalThree(this);
        robot.maxDrivePower = 0.9;

        initPositions();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            loopTimer.reset();
            robot.update();

            // TODO: instead of passing a gamepad, make variables like upperClawInput and read

            // semi-automated control
            if (gamepad1.start || gamepad2.start) {
                robot.cancelAllMacros();
            }
            else if (gamepad1.left_bumper && gamepad1.right_bumper) {
                robot.startTransfer();
            }
            else if (gamepad1.x) {
                robot.startPrepareSamplePickup();
            }
            else if (gamepad2.b && gamepad2.left_stick_button) {
                holdingUpperSlides = false;
                robot.startPrepareSpecimenPickup();
            }
            else if (gamepad2.y) {
                holdingUpperSlides = false;
                robot.startPrepareSampleDeposit();
            }
            else if (gamepad2.x) {
                holdingUpperSlides = false;
                robot.prepareSpecimenDeposit();
            }

            // manual control of each individual part
            controlDriveTrain(gamepad1);

            if (robot.allMacrosInactive()) {
                controlLowerSlides(gamepad1);
                controlLowerClaw(gamepad1);
                controlLowerClawPitch(gamepad1);
                controlLowerClawYaw(gamepad1);

                controlUpperSlides(gamepad2);
                controlUpperArm(gamepad2);
                controlUpperClaw(gamepad2);
                controlUpperClawPitch(gamepad2);
            }

            if (showTelemetry) {
                telemetry.addData("Upper Slide Hold Position", holdUpperSlidesPos);
                telemetry.addData("Upper Arm Position", robot.getUpperArmPos());
                telemetry.addData("Upper Claw Pitch", robot.getUpperClawPitchPos());

                telemetry.addData("Lower Slide Position", robot.getLowerSlidesPos());
                telemetry.addData("Lower Claw Pitch", robot.getLowerClawPitchPos());

                telemetry.addData("Max Drive Power", robot.maxDrivePower);
                telemetry.addData("Loop Speed", "%5.2f ms", loopTimer.time() * 1000);
                telemetry.update();
            }
        }
    }

    public void initPositions() {
        robot.upperSlideRight.brake();
        robot.upperSlideLeft.brake();
        robot.setUpperArmPos(JamalThree.UPPER_ARM_REST);
    }

    boolean holdingUpperSlides = false;
    int holdUpperSlidesPos = 0;
    public void controlUpperSlides(Gamepad gamepad) {
        double input = -gamepad.left_stick_y;

        if (input != 0) {
            double slidePower = input * UPPER_SLIDE_MAX_SPEED;

            robot.upperSlideRight.setPowerEx(slidePower);
            robot.upperSlideLeft.setPowerEx(slidePower);

            holdingUpperSlides = false;
        }
        else if (!holdingUpperSlides){
            holdUpperSlidesPos = robot.getUpperSlidesPos();
            robot.setUpperSlidesPos(holdUpperSlidesPos);

            holdingUpperSlides = true;
        }
    }

    public void controlUpperArm(Gamepad gamepad) {
        double input = gamepad.right_trigger - gamepad.left_trigger;

        if (input != 0) {
            double armVelo = input * UPPER_ARM_MAX_SPEED;
            robot.setUpperArmVelocity(armVelo);
        }
    }

    boolean upperClawFirstInput = true;
    boolean upperClawIsOpen = false;
    public void controlUpperClaw(Gamepad gamepad) {
        boolean input = gamepad.a;

        if (input) {
            if (upperClawFirstInput) {
                upperClawFirstInput = false;

                if (upperClawIsOpen) { robot.closeUpperClaw(); upperClawIsOpen = false; }
                else                 { robot.openUpperClaw();  upperClawIsOpen = true;  }
            }
        } else {
            upperClawFirstInput = true;
        }
    }

    public void controlUpperClawPitch(Gamepad gamepad) {
        double input = gamepad.right_stick_y;

        if (input != 0) {
            double deltaTime = loopTimer.time();

            double nextPitchPos = robot.getUpperClawPitchPos() + input * deltaTime * UPPER_CLAW_PITCH_MAX_SPEED;
            robot.setUpperClawPitchPos(nextPitchPos);
        }
    }

    public void controlLowerSlides(Gamepad gamepad) {
        double input = gamepad.right_trigger - gamepad.left_trigger;

        if (input != 0) {
            double deltaTime = loopTimer.time();

            double nextSlidePos = robot.getLowerSlidesPos() + input * deltaTime * LOWER_SLIDE_MAX_SPEED;
            robot.setLowerSlidesPos(nextSlidePos);
        }
    }

    boolean lowerClawFirstInput = true;
    boolean lowerClawIsOpen = false;
    public void controlLowerClaw(Gamepad gamepad) {
        boolean input = gamepad.a;

        if (input) {
            if (lowerClawFirstInput) {
                lowerClawFirstInput = false;

                if (lowerClawIsOpen) { robot.closeLowerClaw(); lowerClawIsOpen = false; }
                else                 { robot.openLowerClaw();  lowerClawIsOpen = true;  }
            }
        } else {
            lowerClawFirstInput = true;
        }
    }

    boolean lowerClawPitchFirstInput = true;
    boolean lowerClawIsDown = true;
    public void controlLowerClawPitch(Gamepad gamepad) {
        boolean input = gamepad.y;

        if (input) {
            if (lowerClawPitchFirstInput) {
                lowerClawPitchFirstInput = false;

                if (lowerClawIsDown) { robot.raiseLowerClaw(); lowerClawIsDown = false;  }
                else                 { robot.lowerLowerClaw(); lowerClawIsDown = true; }
            }
        } else {
            lowerClawPitchFirstInput = true;
        }

    }

    boolean lowerClawYawFirstInput = true;
    int lowerClawYawPos = 0;
    public void controlLowerClawYaw(Gamepad gamepad) {
        boolean input = gamepad.b;

        if (input) {
            if (lowerClawYawFirstInput) {
                lowerClawYawFirstInput = false;

                lowerClawYawPos = (lowerClawYawPos + 1) % 4;
                switch(lowerClawYawPos) {
                    case 0:
                        robot.setLowerClawYawPos(JamalTwo.LOWER_CLAW_YAW_HORIZONTAL_FLIPPED); break;
                    case 1:
                        robot.setLowerClawYawPos(JamalTwo.LOWER_CLAW_YAW_135DEGREES); break;
                    case 2:
                        robot.setLowerClawYawPos(JamalTwo.LOWER_CLAW_YAW_VERTICAL); break;
                    case 3:
                        robot.setLowerClawYawPos(JamalTwo.LOWER_CLAW_YAW_45DEGREES); break;
                }
            }
        } else {
            lowerClawYawFirstInput = true;
        }

    }

    public void controlDriveTrain(Gamepad gamepad) {

        // TODO: field centric

        // change max drive power
        if      (gamepad.dpad_down) robot.maxDrivePower = 0.3;
        else if (gamepad.dpad_left) robot.maxDrivePower = 0.5;
        else if (gamepad.dpad_up) robot.maxDrivePower = 0.7;
        else if (gamepad.dpad_right) robot.maxDrivePower = 0.9;

        // set drivetrain power (flipped cuz i thoguht thw wrong side was forward 3050 2075
        double throttle = gamepad.left_stick_y * -1;
        double strafe   = -gamepad.left_stick_x * -1;
        double rotate   = gamepad.right_stick_x;

//        // dpad for easy orthogonal movement
//        if      (gamepad.dpad_up)    throttle =  1;
//        else if (gamepad.dpad_down)  throttle = -1;
//        else if (gamepad.dpad_right) strafe   =  1;
//        else if (gamepad.dpad_left)  strafe   = -1;

        // TODO: setup rotation corrections

        robot.drive(throttle, strafe, rotate);
    }
}
