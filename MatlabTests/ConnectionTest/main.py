import matlab.engine
eng = matlab.engine.start_matlab()
tf = eng.test(5.0)
print(tf)