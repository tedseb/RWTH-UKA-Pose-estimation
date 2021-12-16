from threading import Timer
from gymy_tools import ResetTimer, time_out
from gymy_tools import ThreadingTimeout, SignalTimeout
import time


def test_reset_timer():
    test_time = 0.2
    reset_time = 0.1
    test_arg1 = "test"
    test_arg2 = 42
    start_time = 0
    flag = False

    def test_func(arg1, arg2, arg3=False, is_reset=True):
        nonlocal test_arg1
        nonlocal test_arg2
        nonlocal flag
        nonlocal start_time
        nonlocal test_time
        nonlocal reset_time
        assert test_arg1 == arg1
        assert test_arg2 == arg2
        assert arg3 == False
        if is_reset:
            if time.time() - start_time >= test_time + reset_time:
                flag = True
        else:
            if time.time() - start_time >= test_time:
                flag = True

    # Test timer
    timer = ResetTimer(test_time, test_func, [test_arg1, test_arg2], {"is_reset" : False})
    start_time = time.time()
    timer.start()
    time.sleep(test_time + 0.002)
    assert flag == True
    flag = False

    # Restart same Timer after finishing
    timer.reset()
    time.sleep(test_time + 0.002)
    assert flag == True
    timer.stop()

    # Test timer reset
    flag = False
    timer = ResetTimer(test_time, test_func, [test_arg1, test_arg2], {"is_reset" : True})
    start_time = time.time()
    timer.start()
    time.sleep(reset_time + 0.002)
    timer.reset()
    time.sleep(test_time + 1)
    assert flag == True
    timer.stop()

    # Test timer cancel
    flag = False
    timer = ResetTimer(test_time, test_func, [test_arg1, test_arg2], {"is_reset" : False})
    start_time = time.time()
    timer.start()
    timer.cancel()
    time.sleep(test_time + 0.002)
    assert flag == False
    timer.stop()

def test_timeouts():
    with ThreadingTimeout(0.1) as timeout:
        time.sleep(0.2)
    assert not timeout.not_interupted()
    assert not timeout

    with ThreadingTimeout(0.1) as timeout:
        pass
    assert timeout.not_interupted()
    assert timeout

    with ThreadingTimeout(0.1) as timeout:
        timeout.cancel()
        time.sleep(0.2)
    assert timeout.not_interupted()
    assert timeout

    with SignalTimeout(0.1) as timeout:
        time.sleep(0.2)
    assert not timeout.not_interupted()
    assert not timeout

    with SignalTimeout(0.1) as timeout:
        pass
    assert timeout.not_interupted()
    assert timeout

    with SignalTimeout(0.1) as timeout:
        timeout.cancel()
        time.sleep(0.2)
    assert timeout.not_interupted()
    assert timeout