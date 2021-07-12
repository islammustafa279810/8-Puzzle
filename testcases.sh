#!/usr/bin/env sh

# DFS
python3 dfs_test.py 1 2 5 3 4 0 6 7 8 > dfs1.txt
python3 dfs_test.py 3 1 2 0 4 5 6 7 8 > dfs2.txt
python3 dfs_test.py 0 8 7 6 5 4 3 2 1 > dfs3.txt

# A* Manhattan
python3 astar_manhattan_test.py 1 2 5 3 4 0 6 7 8 > manhattan1.txt
python3 astar_manhattan_test.py 3 1 2 0 4 5 6 7 8 > manhattan2.txt
python3 astar_manhattan_test.py 0 8 7 6 5 4 3 2 1 > manhattan3.txt

# A* Euclid
python3 astar_euclid_test.py 1 2 5 3 4 0 6 7 8 > euclid1.txt
python3 astar_euclid_test.py 3 1 2 0 4 5 6 7 8 > euclid2.txt
python3 astar_euclid_test.py 0 8 7 6 5 4 3 2 1 > euclid3.txt