# File used to test and understand how to poll the LSS efficiently
import time
from threading import Thread
from typing import List

from lss_const import LSS_QueryPosition
import lss_const as lssc
def test_sensor(value=1):
    # Mock sensor that takes a few seconds to respond
    time.sleep(0.5)
    return value

global_values = [0, 0, 0]

def main():
    Thread(target=update_value_i, args=(0,), daemon=True).start()
    Thread(target=update_value_i, args=(1,), daemon=True).start()
    Thread(target=update_value_i, args=(2,), daemon=True).start()
    while True:
        input()
        print("FINAL VALUES:", global_values)

def update_value_i(i=0):
    global global_values
    while True:
        
        start = time.time()
        global_values[i] = test_sensor()
        end = time.time()
        print(f"{i}: {global_values[i]} ({end-start:.2f}s)")

def test():
    s = modifiedWrite([1,2,3], LSS_QueryPosition)
    print(s)
    
def modifiedWrite(ids: List[int], cmd):
    combinedCommand = [ lssc.LSS_CommandStart + str(ids[i]) + cmd + lssc.LSS_CommandEnd for i in range(len(ids))]
    combinedCommand = "".join(combinedCommand)
    return combinedCommand


if __name__ == "__main__":
    # main()
    test()