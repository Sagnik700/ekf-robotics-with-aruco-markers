import ev3_dc as ev3
with ev3.EV3(protocol=ev3.USB) as my_ev3:
    my_touch = ev3.Touch(ev3.PORT_1, ev3_obj=my_ev3)
    print('touched' if my_touch.touched else 'not touched')