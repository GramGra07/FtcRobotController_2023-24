functionCode = {
    "useAutoClose": "if (!touchPressed && myOpMode.gamepad2.placeHolder && useAutoClose) {\n    useAutoClose = false;\n} else if (!touchPressed && myOpMode.gamepad2.placeHolder && !useAutoClose) {\n    useAutoClose = true;\n}\ntouchPressed = myOpMode.gamepad2.placeHolder;\n",
    "closeClaw1": "if (myOpMode.gamepad2.placeHolder) {\n   closeClaw(claw1);\n}\n",
    "openClaw1": "if (myOpMode.gamepad2.placeHolder) {\n    openClaw(claw1);\n}\n",
    "closeClaw2": "if (myOpMode.gamepad2.placeHolder) {\n    closeClaw(claw2);\n}\n",
    "openClaw2": "if (myOpMode.gamepad2.placeHolder) {\n    openClaw(claw2);\n}\n",
    "extensionPower": "extensionPower = Range.clip(-myOpMode.gamepad2.placeHolder, slideMin, slideMax);",
    "rotationPower": "rotationPower = Range.clip(myOpMode.gamepad2.placeHolder, flipperMin, flipperMax);",
    "flipPose30": "if (myOpMode.gamepad2.placeHolder) {\n   calculateFlipPose(30, flipServo);\n} ",
    "flipPose45": "else if (myOpMode.gamepad2.placeHolder) {\n   calculateFlipPose(45, flipServo);\n} ",
    "flipPose70": "else if (myOpMode.gamepad2.placeHolder) {\n   calculateFlipPose(70, flipServo);\n} ",
    "flipPose0": "else if (myOpMode.gamepad2.placeHolder) {\n   calculateFlipPose(0, flipServo);\n}\n",
}
