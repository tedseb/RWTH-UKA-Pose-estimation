import logy
import time
import threading

def _test_unbounded_messages():
    '''Normale Log Funktionen, die verwendet werden können. Der Default Logger ist auf Debug level eingestellet.
    Dieser kann aber auch umkonfiguriert werden. Die konfiguration hat globale Auswirkungen auf alle Module und
    threads von dem aktuellen Prozess. Zu Beginn jedes Prozesses sollte der Root Logger einmal konfiguriert werden'''

    logy.debug("DEBUG - You should not see this Message")
    logy.info("INFO  - You should not see this Message ")
    logy.warn("WARNING - Hello")
    logy.error("ERROR - Hello")
    logy.critical("CRITICAL - Hello")
    logy.fatal("FATAL - Hello")

    logy.Logy().set_root_debug_level(logy.DEBUG)
    logy.debug("DEBUG - Now you can see this Message")
    logy.info("INFO  - Now you can see this Message")

    logy.Logy().set_root_module("TEST")
    logy.info("Set logger Module Information")

def _test_custom_handle_messages():
    '''Es können auch custom Logger erstellt werden, welche konfiguriert werden können. Zur Zeit lässt
    sich nur das Debug level und den Modulnamen einstellen einstellen.'''

    custom_logger = logy.Logy().get_or_create_logger("cutom_logger", logy.DEBUG, module_name="CUSTOM")
    custom_logger.debug("DEBUG custom handler")
    custom_logger.info("INFO custom handler")
    custom_logger.warn("WARNING custom handler")
    custom_logger.error("ERROR custom handler")
    custom_logger.critical("CRITICAL custom handler")
    custom_logger.fatal("FATAL custom handler")

def _test_new_function_same_handler():
    '''Wenn ein Logger schon mal vom gleichen Prozess angefordert wurde, bleiben die Einstellungen erhalten.
    Beispiel: custom_logger wurde schon in einer anderen Funktion erstellt. Hier wird dieser nicht mehr neu
    erzeugt sondern wieder verwendet. Aber auch der Root logger is noch wie in der letzten Funktion konfiguriert'''

    custom_logger = logy.Logy().get_or_create_logger("cutom_logger")
    custom_logger.debug("DEBUG custom handler other function")
    custom_logger.info("INFO custom handler other function")
    custom_logger.warn("WARNING custom handler other function")
    custom_logger.error("ERROR custom handler other function")
    custom_logger.critical("CRITICAL custom handler other function")
    custom_logger.fatal("FATAL custom handler other function")

    logy.info("Modulname is TEST and Debug Level is INFO")

def configfure_logy_from_thread():
    '''Alle Einstellungen werden über den Prozess geteilt'''
    logy.Logy().set_root_debug_level(logy.WARNING)
    logy.Logy().set_root_module("THREAD")

def _thread_test():
    '''Logger wird von Thread umkonfiguriert'''
    thread = threading.Thread(target = configfure_logy_from_thread)
    thread.start()
    thread.join()
    logy.info("you should not see this message")
    logy.warn("But you can se this message")

def _loop_test(loop_range = 10, sleep = 0.1):
    for _ in range(loop_range):
        logy.warn(f"LOOP TEST, Range: {loop_range}, Sleep: {sleep}")
        time.sleep(sleep)

def test_all_message():
    _test_unbounded_messages()
    _test_custom_handle_messages()
    _test_new_function_same_handler()
    _thread_test()

def time_test():
    for _ in range(10):
        time_ns = time.time_ns()
        logy.warn("WARNING")
        time_ns = time.time_ns() - time_ns
        print(f"need {time_ns}ns")

def multithread_test():
    threads = []
    threads.append(threading.Thread(target = _loop_test, args = (80, 0.1, )))
    threads.append(threading.Thread(target = _loop_test, args = (160, 0.05, )))
    threads.append(threading.Thread(target = _loop_test, args = (800, 0.01, )))
    threads.append(threading.Thread(target = _loop_test, args = (40, 0.2, )))
    threads.append(threading.Thread(target = _loop_test, args = (400, 0.02, )))

    for thread in threads:
        thread.start()

    for thread in threads:
        thread.join()

def fail_function():
    fail = 42
    fail.no_valid_function()

if __name__ == '__main__':
    logy.Logy().basic_config(module_name="ROOT", debug_level=logy.WARNING)
    test_all_message()
    # logy.Logy().set_root_debug_level(logy.DEBUG)
    # logy.debug("DEBUG - Now you can see this Message")
    # logy.error("INFO  - Now you can see this Message")