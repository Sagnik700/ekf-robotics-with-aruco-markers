import ev3_dc as ev3
with ev3.EV3(protocol=ev3.USB) as my_ev3:
    print(my_ev3.sensors)
    my_color = ev3.Color(ev3.PORT_4, ev3_obj=my_ev3)
    print('The color is', my_color.color)
    print('The reflected intensity is ', my_color.reflected, '%')
    print("The ambient colour is", my_color.ambient)