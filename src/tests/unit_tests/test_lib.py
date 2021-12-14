from gymy_tools import ResetTimer
import time


def test_reset_timer():
    test_time = 0.5
    reset_time = 0.25
    test_arg1 = "test"
    test_arg2 = 42
    flag = False

    def test_func(arg1, arg2, arg3 = False, is_reset = False):
        nonlocal test_arg1
        nonlocal test_arg2
        nonlocal flag
        assert test_arg1 == arg1
        assert test_arg1 == arg2
        assert arg3 == False
        flag = True

    timer = ResetTimer(test_time, test_func, [test_arg1, test_arg2], {"is_reset" : False})
    time.sleep(test_time + 0.002)
    assert flag == True
    timer = ResetTimer(test_time, test_func, [test_arg1, test_arg2], {"is_reset" : True})
    time.sleep(reset_time)
    timer.reset()
    time.sleep(test_time + 0.002)
    assert flag == True
