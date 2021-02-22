from multiprocessing import Process
import os

def info(title):
    print(title)
    print('module name:', __name__)
    print('parent process:', os.getppid())
    print('process id:', os.getpid())

def f(name):
    info('function f')
    print('hello', name)

if __name__ == '__main__':
    info('main line')
    
    for i in range(0,4):
        p = Process(target=f, args=(f'bob {i}',))
        p.start()
        p.join()