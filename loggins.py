import pyboard
import time
import numpy as np
import matplotlib.pyplot as plt
## import matplotlib as mpl

plot_window = 120
Y = np.zeros(plot_window)
X = range(plot_window)

## mpl.rcParams['path.simplify_threshold'] = 1
plt.ion()
fig, axlist = plt.subplots()
line, = axlist.plot(X,Y)

pyb = pyboard.Pyboard('/dev/ttyUSB0')
pyb.enter_raw_repl()
pyb.exec('import m02; m=m02.M02()')

t0 = time.time()

counter = 0
try:
    while True:
        fifo_bytes = pyb.exec('print(m.read_fifo())').decode()[:-2]
        fifo_data = eval(fifo_bytes)
        # print(fifo_data)
        t1 = time.time()
        print(t1-t0)
        t0 = t1
        time.sleep(.01)
        
        Y[counter%plot_window] = fifo_data[0]    
        line.set_ydata(Y)
        counter = counter+1
        
        axlist.relim()
        axlist.autoscale_view()
        fig.canvas.draw()
#        axlist.plot(X,Y)
        fig.canvas.flush_events()
except:
    pass

pyb.exec('m.reset()')    
pyb.exit_raw_repl()
pyb.close()
