import numpy as np
import main
import sys
import time

init = np.reshape([int(x) for x in sys.argv[1:]], (3,3))
goal = np.array([[0,1,2], [3,4,5], [6,7,8]])

start = time.process_time()
result = main.dfs(init, goal)
elapsed = time.process_time() - start

main.print_trace(result)

print(f'Time taken: {elapsed}s')