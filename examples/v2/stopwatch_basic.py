from evo.tools import StopWatch, wait


watch = StopWatch()

wait(250)
print("Elapsed:", watch.time())

watch.reset()
wait(100)
print("Elapsed after reset:", watch.time())
