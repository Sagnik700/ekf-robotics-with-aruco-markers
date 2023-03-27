import ev3_dc as ev3

with ev3.EV3(protocol=ev3.USB) as my_ev3:

    motor_a = ev3.Motor(ev3.PORT_A, ev3_obj=my_ev3)
    motor_d = ev3.Motor(ev3.PORT_D, ev3_obj=my_ev3)

    motor_a.start_move_by(360*2, speed=80, brake=True)
    motor_d.start_move_by(360*2, speed=80, brake=True)
