import serial
from evdev import uinput, ecodes as e
ser = serial.Serial('/dev/ttyUSB0')
while 1:
    line = ser.readline()#.decode('utf-8')[:-4]
    line = line[0]
    if line == 'a':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_A, 1)
            ui.syn()
    elif line == 'b':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_B, 1)
            ui.syn()
    elif line == 'c':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_C, 1)
            ui.syn()
    elif line == 'd':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_D, 1)
            ui.syn()
    elif line == 'e':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_E, 1)
            ui.syn()
    elif line == 'f':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_F, 1)
            ui.syn()
    elif line == 'g':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_G, 1)
            ui.syn()
    elif line == 'h':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_H, 1)
            ui.syn()
    elif line == 'i':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_I, 1)
            ui.syn()
    elif line == 'j':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_J, 1)
            ui.syn()
    elif line == 'k':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_K, 1)
            ui.syn()
    elif line == 'l':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_L, 1)
            ui.syn()
    elif line == 'm':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_M, 1)
            ui.syn()
    elif line == 'n':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_N, 1)
            ui.syn()
    elif line == 'o':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_O, 1)
            ui.syn()
    elif line == 'p':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_P, 1)
            ui.syn()
    elif line == 'q':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_Q, 1)
            ui.syn()
    elif line == 'r':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_R, 1)
            ui.syn()
    elif line == 's':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_S, 1)
            ui.syn()
    elif line == 't':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_T, 1)
            ui.syn()
    elif line == 'u':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_U, 1)
            ui.syn()
    elif line == 'v':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_V, 1)
            ui.syn()
    elif line == 'w':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_W, 1)
            ui.syn()
    elif line == 'x':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_X, 1)
            ui.syn()
    elif line == 'y':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_Y, 1)
            ui.syn()
    elif line == 'z':
        with uinput.UInput() as ui:
            ui.write(e.EV_KEY, e.KEY_Z, 1)
            ui.syn()
