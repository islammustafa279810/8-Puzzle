import numpy as np
from collections import deque
import heapq
import textwrap

class State:
	def __init__(self, array, step):
		self.array = np.array(array)
		self.step = step

	def _copy_swap(self, index1, index2, step):
		result = self.array.copy()
		tmp = result[index1]
		result[index1] = result[index2]
		result[index2] = tmp
		return State(result, step)
	
	def neighbours(self):
		pos_x, pos_y = np.argwhere(self.array == 0)[0]
		result = []
		
		# Return neighbors in reverse-UDLR order for DFS
		if pos_y == 0:
			result.append(self._copy_swap((pos_x, 0), (pos_x, 1), 'RIGHT'))
		elif pos_y == 2:
			result.append(self._copy_swap((pos_x, 2), (pos_x, 1), 'LEFT'))
		else:
			result.append(self._copy_swap((pos_x, 1), (pos_x, 2), 'RIGHT'))
			result.append(self._copy_swap((pos_x, 1), (pos_x, 0), 'LEFT'))
		
		if pos_x == 0:
			result.append(self._copy_swap((0, pos_y), (1, pos_y), 'DOWN'))
		elif pos_x == 2:
			result.append(self._copy_swap((2, pos_y), (1, pos_y), 'UP'))
		else:
			result.append(self._copy_swap((1, pos_y), (2, pos_y), 'DOWN'))
			result.append(self._copy_swap((1, pos_y), (0, pos_y), 'UP'))
		
		return result
	
	def __hash__(self):
		return hash(tuple(map(tuple, self.array)))
	
	def __eq__(self, other):
		if isinstance(other, np.ndarray):
			return np.all(self.array == other)
		elif isinstance(other, State):
			return np.all(self.array == other.array)
		else:
			return False

class DFSItem:
	def __init__(self, state, prev_item, cost):
		self.state = state
		self.prev_item = prev_item
		self.cost = cost
	
	def __str__(self):
		path = [self]
		prev_item = self.prev_item
		while prev_item != None:
			path.append(prev_item)
			prev_item = prev_item.prev_item
		
		result = []
		route = []
		for node in reversed(path):
			route.append(node.state.step)
			result.append(f'{node.state.step}\n{node.state.array}\n')
		return '----------\n'.join(result) + f'\nPath cost: {self.cost}\nPath:\n{textwrap.fill(", ".join(route[1:]))}\n'
	

def dfs(init, goal):
	init = State(init, 'START')
	frontier = [DFSItem(init, None, 0)]
	explored = set([init])
	expanded = 1
	max_depth = 0

	while len(frontier) > 0:
		expanded += 1
		item = frontier.pop()
		if item.state == goal:
			return (item, expanded, max_depth)
		
		for neighbour in item.state.neighbours():
			if neighbour not in explored:
				frontier.append(DFSItem(neighbour, item, item.cost+1))
				explored.add(neighbour)
				max_depth = max(max_depth, item.cost+1)

class AStarItem:
	step_order = {'UP': 0, 'DOWN': 1, 'LEFT': 2, 'RIGHT': 3}

	def __init__(self, priority_fn, state, cost, prev_item):
		self.priority = priority_fn(state) + cost
		self.state = state
		self.cost = cost
		self.prev_item = prev_item
	
	def __lt__(self, other):
		return (
			AStarItem.step_order[self.state.step] < AStarItem.step_order[other.state.step]
			if self.priority == other.priority
			else self.priority < other.priority
		)
	
	def __str__(self):
		path = [self]
		prev_item = self.prev_item
		while prev_item != None:
			path.append(prev_item)
			prev_item = prev_item.prev_item
		
		result = []
		admissable_count = 0
		route = []
		for actual_cost, node in zip(range(len(path)-1, -1, -1), reversed(path)):
			heuristic = node.priority-node.cost
			admissable = heuristic <= actual_cost
			route.append(node.state.step)
			result.append(
				f'{node.state.step}\n{node.state.array}\nHeuristic: {heuristic}\nActual cost: {actual_cost} ({"" if admissable else "NOT "}ADMISSABLE)\n'
			)
			if admissable:
				admissable_count += 1
		
		return "----------\n".join(result)+f'\nAdmissable rate: {admissable_count/len(path)}' + f'\nPath cost: {self.cost}\nPath:\n{textwrap.fill(", ".join(route[1:]))}\n'

def astar(init, goal, distance_fn):
	heuristic = lambda state: distance_fn(state.array, goal)
	prioritized = lambda state, cost, prev_item: AStarItem(heuristic, state, cost, prev_item)
	frontier = []
	init = State(init, 'START')
	heapq.heappush(frontier, prioritized(init, 0, None))
	explored = set([init])
	expanded = 1
	max_depth = 0

	while len(frontier) > 0:
		expanded += 1
		item = heapq.heappop(frontier)
		if item.state == goal:
			return (item, expanded, max_depth)
		
		for neighbour in item.state.neighbours():
			if neighbour not in explored:
				heapq.heappush(frontier, prioritized(neighbour, item.cost+1, item))
				explored.add(neighbour)
				max_depth = max(max_depth, item.cost+1)

def loc_dict(array):
	result = {}
	for x in range(array.shape[0]):
		for y in range(array.shape[1]):
			result[array[(x,y)]] = np.array([x,y])
	return result

def euclid_distance(array1, array2):
	array1_locs = loc_dict(array1)
	array2_locs = loc_dict(array2)
	return np.sum(
		np.sqrt(
			np.square(list(array1_locs[i] - array2_locs[i] for i in range(9))).sum(axis=1)
		)
	)

def manhattan_distance(array1, array2):
	array1_locs = loc_dict(array1)
	array2_locs = loc_dict(array2)
	return np.sum(np.abs(list(array1_locs[i] - array2_locs[i] for i in range(9))))

def print_trace(result):
	print(f'{result[0]}\nNodes expanded: {result[1]}\nSearch depth: {result[2]}')