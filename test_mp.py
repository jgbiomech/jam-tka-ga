import multiprocessing

def f(name):
    print(name,flush=True)

if __name__ == '__main__':
    pool = multiprocessing.Pool(processes=4) #use all available cores, otherwise specify the number you want as an argument
    for i in range(0, 512):
        pool.apply_async(f, args=(i,))        
    pool.close()
    pool.join()