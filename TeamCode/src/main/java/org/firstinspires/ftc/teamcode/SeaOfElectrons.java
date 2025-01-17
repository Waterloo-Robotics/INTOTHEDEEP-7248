/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode executes a Tank Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left and right joysticks control the left and right motors respectively.
 * Pushing a joystick forward will make the attached motor drive forward.
 * It raises and lowers the claw using the Gamepad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="7248 Tele", group="Robot")
public class SeaOfElectrons extends OpMode{

    /* Declare OpMode members. */
    public DcMotor  leftFrontDrive   = null;
    public DcMotor  rightFrontDrive  = null;
    public DcMotor  leftBackDrive  = null;
    public DcMotor  rightBackDrive  = null;
    public DcMotor  rightSlide  = null;
    public DcMotor  leftSlide  = null;


    public Servo intake_claw = null;
    public Servo intake_claw_orientation = null;
    public Servo intake_claw_rotation = null;
    public Servo intake_arm_rotation_right = null;
    public Servo intake_arm_rotation_left = null;
    public Servo intake_slider = null;
    public Servo scoring_arm_right = null;
    public Servo scoring_arm_left = null;
    public Servo scoring_claw = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
//        // Define and Initialize Motors
//        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LD");
//        rightFrontDrive = hardwareMap.get(DcMotor.class, "RD");
//        leftBackDrive = hardwareMap.get(DcMotor.class, "bldr");
//        rightBackDrive = hardwareMap.get(DcMotor.class, "brdr");
//
//        rightSlide = hardwareMap.get(DcMotor.class, "R_slide");
//        leftSlide = hardwareMap.get(DcMotor.class, "L_slide");

        intake_claw = hardwareMap.get(Servo.class, "intake_claw");
        intake_claw_orientation = hardwareMap.get(Servo.class, "intake_claw_orientation");
        intake_claw_rotation = hardwareMap.get(Servo.class, "intake_claw_rotation");
        intake_arm_rotation_right = hardwareMap.get(Servo.class, "intake_arm_rotation_r");
        intake_arm_rotation_left = hardwareMap.get(Servo.class, "intake_arm_rotation_l");
        intake_slider = hardwareMap.get(Servo.class, "intake_slider");
        scoring_arm_right = hardwareMap.get(Servo.class, "scoring_arm_right");
        scoring_arm_left = hardwareMap.get(Servo.class, "scoring_arm_left");
        scoring_claw = hardwareMap.get(Servo.class, "scoring_claw");

        this.close_intake_claw();
        this.close_scoring_claw();
        this.home_claw_orientation();
        this.straight_claw_rotation();
        this.straight_intake_arm_rotation();
        this.home_intake_slider();
        this.home_scoring_arm();


//        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
//        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
//        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press START.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        double forward;
        double strafe;
        double rotation;
        double extend;

        if (gamepad1.a){
            open_scoring_claw();
        } else {
            close_scoring_claw();
        }

        if (gamepad1.b){
            this.intake_claw_rotation_intake();
            this.sub_intake_arm_rotation();
        }
        // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
//        forward = gamepad1.left_stick_y;
//        strafe = -gamepad1.left_stick_x;
//        rotation = -gamepad1.right_stick_x;
//
//        leftFrontDrive.setPower(forward + strafe + rotation);
//        leftBackDrive.setPower(forward - strafe + rotation);
//        rightFrontDrive.setPower(forward - strafe - rotation);
//        rightBackDrive.setPower(forward + strafe - rotation);

        telemetry.addData(">", "Robot Ready.  Press START.");
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void open_intake_claw() {
        intake_claw.setPosition(Constants.INTAKE_OPEN);
    }


    public void close_intake_claw() {
        intake_claw.setPosition(Constants.INTAKE_CLOSE);
    }


    public void open_scoring_claw() {
        scoring_claw.setPosition(Constants.SCORING_OPEN);
    }


    public void close_scoring_claw() {
        scoring_claw.setPosition(Constants.SCORING_CLOSE);
    }

    public void home_claw_orientation() {
        intake_claw_orientation.setPosition(Constants.ORIENTATION_HOME);
    }


    public void straight_claw_rotation() {
        intake_claw_rotation.setPosition(Constants.ROTATION_STRAIGHT);
    }

    public void straight_intake_arm_rotation() {
        double position = Constants.INTAKE_ARM_STRAIGHT;
        intake_arm_rotation_right.setPosition(position);
        intake_arm_rotation_left.setPosition(1-position);
    }
    public void home_intake_slider() {
        intake_slider.setPosition(Constants.SLIDER_HOME);
    }

    public void home_scoring_arm() {
        double position = Constants.ARM_HOME;
        scoring_arm_left.setPosition(position);
        scoring_arm_right.setPosition(1 - position);
    }

    public void transfer_scoring_arm() {
        double position = Constants.SCORING_ARM_TRANSFER;
        scoring_arm_left.setPosition(position);
        scoring_arm_right.setPosition(1 - position);
    }

    public void score_scoring_arm() {
        double position = Constants.SCORING_SCORE;
        scoring_arm_left.setPosition(position);
        scoring_arm_right.setPosition(1 - position);
    }

    public void sub_intake_arm_rotation() {
        double position = Constants.INTAKE_ROTATION_SUB;
        intake_arm_rotation_right.setPosition(position);
        intake_arm_rotation_left.setPosition(1-position);
    }

    public void intake_claw_rotation_intake() {
        intake_claw_rotation.setPosition(Constants.INTAKE_ROTATION_INTAKE);
    }

}
