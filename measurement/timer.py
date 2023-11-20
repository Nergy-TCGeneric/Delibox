import time

def measure_time_in_ns(target):
    start_time = time.perf_counter_ns()
    def wrapper_func():
        return target
    end_time = time.perf_counter_ns()
    print(f"Took {end_time - start_time} ns to finish")