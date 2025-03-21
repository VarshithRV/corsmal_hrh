import time
import threading

P = 0.50
I = 0.001
D = 0.01
K = 1.0
error = 0.0
setpoint = 10.0
current = 0.0
error_last = 0.0
error_sum = 0.0

def pid():
    global error 
    global setpoint
    global current
    global error_last
    global error_sum

    signal = 0.0
    dt = 0.1
    error_last = error
    error = setpoint - current
    error_sum += error
    signal = K*(P*error + D*(error - error_last)/dt + I*(error_sum*dt))
    time.sleep(dt)

    return signal

if __name__ == "__main__":
    
    time.sleep(2)

    while True : 
        signal = pid()
        current += signal
        print(f"Current: {current:.4f}, Error: {error:.4f}, Signal: {signal:.6f}")