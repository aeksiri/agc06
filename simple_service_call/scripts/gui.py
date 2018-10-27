# imports
import Tkinter as tk
import ttk

from subprocess import call

def head_lamp_on():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/light_srv", "input: '1'"])
    # print s

def head_lamp_off():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/light_srv", "input: '0'"])
    # print s

def cart_lock_up():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/bodylock_srv", "input: '1'"])
    # print s

def cart_lock_down():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/bodylock_srv", "input: '0'"])
    # print s

def led_red_on():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/led_srv", "input: '1'"])
    # print s

def led_blue_on():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/led_srv", "input: '2'"])
    # print s

def led_green_on():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/led_srv", "input: '3'"])
    # print s

def led_blue_blink():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/led_srv", "input: '4'"])
    # print s

def led_red_blink():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/led_srv", "input: '5'"])
    # print s

def leds_off():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/led_srv", "input: '0'"])
    # print s    

def buzzer_1():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/voice_srv", "input: '1'"])
    # print s

def buzzer_2():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/voice_srv", "input: '2'"])
    # print s

def buzzer_3():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/voice_srv", "input: '3'"])
    # print s 

def buzzer_4():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/voice_srv", "input: '4'"])
    # print s

def buzzer_off():
    ### $ rosservice call /agc04/test_srv "input: '1111'" ###

    # s = subprocess.call(["rosservice", "call", "/agc04/iocon_srv", "input: '1'"])
    s = call(["rosservice", "call", "/voice_srv", "input: '0'"])
    # print s

win = tk.Tk()
win.title("AGC05 Control Panel")
#win.resizable(0, 0) # disable resizing the GUI

# Adding a Label
ledOnLabel = ttk.Label(win, text="HEAD LAMP CONTROL")
ledOnLabel.grid(column=0, row=0)
# Adding a button
action = ttk.Button(win, text="ON", command=head_lamp_on)
action.grid(column=1, row=0)
action = ttk.Button(win, text="OFF", command=head_lamp_off)
action.grid(column=1, row=1)

# Adding a Label
ledOnLabel = ttk.Label(win, text="CART LOCK CONTROL")
ledOnLabel.grid(column=0, row=3)
# Adding a button
action = ttk.Button(win, text="UP", command=cart_lock_up)
action.grid(column=1, row=3)
action = ttk.Button(win, text="DOWN", command=cart_lock_down)
action.grid(column=1, row=4)

# Adding a Label
ledOnLabel = ttk.Label(win, text="LEDs CONTROL")
ledOnLabel.grid(column=2, row=0)
# Adding a button
action = ttk.Button(win, text="RED", command=led_red_on)
action.grid(column=3, row=0)
action = ttk.Button(win, text="BLUE", command=led_blue_on)
action.grid(column=3, row=1)
action = ttk.Button(win, text="GREEN", command=led_green_on)
action.grid(column=3, row=2)
action = ttk.Button(win, text="BLUE BLINK", command=led_blue_blink)
action.grid(column=3, row=3)
action = ttk.Button(win, text="RED BLINK", command=led_red_blink)
action.grid(column=3, row=4)
action = ttk.Button(win, text="OFF", command=leds_off)
action.grid(column=3, row=5)

# Adding a Label
ledOnLabel = ttk.Label(win, text="BUZZER CONTROL")
ledOnLabel.grid(column=0, row=6)
# Adding a button
action = ttk.Button(win, text="Sound 1", command=buzzer_1)
action.grid(column=1, row=6)
action = ttk.Button(win, text="Sound 2", command=buzzer_2)
action.grid(column=1, row=7)
action = ttk.Button(win, text="Sound 3", command=buzzer_3)
action.grid(column=1, row=8)
action = ttk.Button(win, text="Sound 4", command=buzzer_4)
action.grid(column=1, row=9)
action = ttk.Button(win, text="OFF", command=buzzer_off)
action.grid(column=1, row=10)

win.mainloop()