package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class jaydenTeleOP extends LinearOpMode {
    private DcMotor back_right;
    private DcMotor back_left;
    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor armMotor;
    private Servo clawArm;
    private Servo autoGrab;
    private Servo claw;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        back_right = hardwareMap.dcMotor.get("FrontRight");
        back_left = hardwareMap.dcMotor.get("FrontLeft");
        front_left = hardwareMap.dcMotor.get("BackLeft");
        front_right = hardwareMap.dcMotor.get("BackRight");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        clawArm = hardwareMap.servo.get("clawArm");
        clawArm.setDirection(Servo.Direction.REVERSE);
        autoGrab = hardwareMap.servo.get("autoGrab");
        autoGrab.setDirection(Servo.Direction.REVERSE);
        claw = hardwareMap.servo.get("claw");



        // Reverse one of the drive motors.
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        //back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        //front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // speed that dpad controls go at; joy stick speed devided by this
                double dpadSpeed = 0.5;
                if (gamepad1.dpad_up == true || gamepad2.dpad_up == true) {
                    //if (gamepad1.dpad_up == true) {
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    back_left.setPower(dpadSpeed);
                    back_right.setPower(dpadSpeed);
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    front_left.setPower(dpadSpeed);
                    front_right.setPower(-dpadSpeed);
                }
                if (gamepad1.dpad_down == true || gamepad2.dpad_down == true) {
                    //if (gamepad1.dpad_down == true) {
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    back_left.setPower(-dpadSpeed);
                    back_right.setPower(-dpadSpeed);
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    front_left.setPower(-dpadSpeed);
                    front_right.setPower(dpadSpeed);
                }
                if (gamepad1.dpad_right == true || gamepad2.dpad_right == true) {
                    //if (gamepad1.dpad_right == true) {
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    back_left.setPower(dpadSpeed);
                    back_right.setPower(-dpadSpeed);
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    front_left.setPower(-dpadSpeed);
                    front_right.setPower(-dpadSpeed);
                }
                if (gamepad1.dpad_left == true || gamepad2.dpad_left == true) {
                    // if (gamepad1.dpad_left == true) {
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    back_left.setPower(-dpadSpeed);
                    back_right.setPower(dpadSpeed);
                    // The Y axis of a joystick ranges from -1 in its topmost position
                    // to +1 in its bottommost position. We negate this value so that
                    // the topmost position corresponds to maximum forward power.
                    front_left.setPower(dpadSpeed);
                    front_right.setPower(dpadSpeed);
                }
                else {

                    //  set motor power to that of the controller sticks
                    // left side
                    back_left.setPower(-gamepad1.left_stick_y);
                    front_left.setPower(-gamepad1.left_stick_y);

                    // right side
                    back_right.setPower(-gamepad1.right_stick_y);
                    front_right.setPower(gamepad1.right_stick_y);

                }

                // put platformClaw down
                // put platform claw up
                if (gamepad1.y) {
                    autoClawUp();
                }
                if (gamepad1.a) {
                    autoClawDown();
                }

                // control arm with left stick on gamepad 2
                armMotor.setPower(-gamepad2.left_stick_y / 1);

                // flip claw out
                if (gamepad2.x) {
                    armOut();
                }
                // flip claw in
                if (gamepad2.b) {
                    armIn();
                }
                // grab stone
                if (gamepad2.a) {
                    closeClaw();
                }

                // release stone
                if (gamepad2.y) {
                    openClaw();
                }

                // platform grab down
                if (gamepad2.left_stick_button) {
                    autoClawDown();
                }
                if (gamepad2.right_stick_button){
                    autoClawUp();
                }

                telemetry.addData("Left Pow", back_left.getPower());
                telemetry.addData("Left Pow", back_right.getPower());
                telemetry.addData("Left Pow", front_left.getPower());
                telemetry.addData("Right Pow", front_right.getPower());
                telemetry.update();
            }
        }
    }
    public void armOut() {
        clawArm.setPosition(0.72);
    }

    public void armIn() {
        clawArm.setPosition(0.04);
    }

    public void autoClawDown() {
        autoGrab.setPosition(1);

    }
    public void autoClawUp() {
        autoGrab.setPosition(0.35);
    }

    private void closeClaw() {
        claw.setPosition(1);
    }




    private void openClaw() {
        claw.setPosition(0.5);
    }
}
