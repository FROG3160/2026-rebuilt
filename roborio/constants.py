from phoenix6.signals.spn_enums import InvertedValue, NeutralModeValue

drive_motor_output_params = {
    # set neutral mode for drive motor
    "neutral_mode": NeutralModeValue.BRAKE,
    # with the non-inverted SwerveDriveSpecialties swerve modules, and
    # the bevel gears facing left, the drive motors need to be inverted
    # in order to move the drivetrain forward with a positive value.
    # the default inverted setting is CCW positive.
    "inverted": InvertedValue.CLOCKWISE_POSITIVE,
}
steer_motor_output_params = {
    "neutral_mode": NeutralModeValue.BRAKE,
    "inverted": InvertedValue.COUNTER_CLOCKWISE_POSITIVE,
}
