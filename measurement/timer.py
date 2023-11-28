import time

def measure_time_in_ns(target) -> callable:
    def wrapper_func(*args, **kwargs):
        start_time = time.perf_counter_ns()
        result = target(*args, **kwargs)
        end_time = time.perf_counter_ns()
        print(f"{target.__name__} : Took {end_time - start_time} ns to finish")
        return result
    return wrapper_func