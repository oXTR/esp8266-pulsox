import pyboard
import time
import csv
import matplotlib
matplotlib.use("tkAgg")
import matplotlib.pyplot as plt
import numpy as np

pyb = pyboard.Pyboard('/dev/ttyUSB0')
pyb.timeout = 5

plot_window = 60
###y_var_red = np.array(np.zeros([plot_window]))
y_var_red = np.zeros([plot_window])
#y_var_ir = np.array(np.zeros([plot_window]))

# counter = [1]
log = np.zeros((0,2))
f = []
diff = []
sten = []

plt.ion()
fig, ax = plt.subplots()
line_red, = ax.plot(y_var_red)
#line_ir = ax.plot(y_var_ir)

pyb.enter_raw_repl()
pyb.exec('import m02; m=m02.M02(samples_per_avg=2)') #samples_per_avg=1)')

try:
    while True:
        fifo_bytes = pyb.exec('print(m.read_fifo())').decode()[:-2]
        try:
            fifo_data = eval(fifo_bytes)
            print(fifo_data)
        except:
            print("Exception")
            continue
#        with open("logger_data.csv", "a") as f:
#            writer = csv.writer(f, delimiter=",")
#            writer.writerow(fifo_data)
        log = np.append(log, [fifo_data], axis=0)
            
        time.sleep(.01)
        
        y_var_red = np.append(y_var_red, 200000-float(fifo_data[0]))
    #        y_var_ir = np.append(y_var, fifo_data[1])
        y_var_red = y_var_red[1:plot_window+1]
    #        y_var_ir = y_var_ir[1:plot_window+1]
        line_red.set_ydata(y_var_red)
    #        line_ir.set_ydata(y_var_ir)
        ax.relim()
        ax.autoscale_view()
        fig.canvas.draw()
        fig.canvas.flush_events()

        f = np.append(f,y_var_red)
        try:
            diff = (3*f[-5]-16*f[-4]+36*f[-3]-48*f[-2]+25*f[-1])/(12*1.0*h**1)
            sten = np.append(sten, diff)
            print("It's working")
        except:
            continue
            
except:
    print("\n\nProcess interruped...")

print("\nTrying to m.reset()...")
print("(Hit 'Ctrl-c' if process takes more than 10s)")

f = open("logger_data.csv", "a")

writer = csv.writer(f, delimiter=",")
writer.writerows(log)
try:
    f.close()
    print("Successfully written to logger_data.csv")
except:
    print("Could not write to logger_data.csv")
    
g = open("stencil.csv", "a")

writer = csv.writer(g, delimiter=",")
writer.writerows(sten)
g.close()
    
try:
    pyb.exec('m.reset()')
    print("\nOk... successfully m.reset() !")
except:
    print("\nm02 could not reset. Pyboard disconnected?")
    
'''
question = input("\nDo you want to log off? Y/n\n")

if (question == "n") or (question == "N"):
    print("\nOk. Pyboard object still open and in raw_repl....")
    try:
        pyb.exec('m.reset()')
    except:
        print("\nFailed to m.reset()....")
    print("\n... The following pyb object remains open: ", str(pyb), "\n")
else:    
    try:
        pyb.exec('m.reset')
        pyb.exit_raw_repl()
        pyb.close()
        print("\nLogging off... Thank you.\n")
    except:
        print("\nPyboard object already out of raw_repl or closed\n")
'''
