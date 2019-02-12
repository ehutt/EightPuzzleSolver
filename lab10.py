from eightpuzzle import eightpuzzle
import time

start_time = time.time()


class node:
    def __init__(self,s,parent,cost,action,heuristic):
        self.s = s
        self.parent = parent
        self.cost = cost
        self.heuristic = heuristic
        self.action = action


#cost is number of moves taken, heuristic is sum of misplaced tiles
def heuristic(state):
    heuristic = 0
    for i in range(len(state)):
        if state[i] != i:
            heuristic += 1
    return heuristic


def expand(parent,children):
    if parent.s == puzzle.goal:
        children.remove(parent)
        solutions.append(parent)
        return children
    else:
        actions = puzzle.actions(s=parent.s)
        for action in actions:
            new_state = puzzle.step(s=parent.s, a = action)
            h = heuristic(new_state)
            child = node(state=new_state, parent = parent, cost = parent.cost + 1, action=action, heuristic = h)
            children.append(child)
        children.remove(parent)
    return children

#initializations
puzzle = eightpuzzle(mode='easy')
init_state = puzzle.reset()
heuristic = heuristic(init_state)
root = node(s=init_state,parent=None,cost=0,action=None,heuristic=heuristic)
children = list()
children.append(root)
children = expand(root, children)
solutions = list()


while len(children) != 0:
        min_cost = 1000
        min_cost_node = None
        for child in children:
            if child.cost + child.heuristic <= min_cost:
                min_cost = child.cost + child.heuristic
                min_cost_node = child
        children = expand(min_cost_node, children)

min_cost_path = 1000
solution = None
for node in solutions:
    if node.cost <= min_cost_path:
        min_cost_path = node.cost
        solution = node

print('Minimum number of turns to solve: ', solution.cost)
elapsed_time = time.time() - start_time
print('Elapsed Time: %.1f seconds' % elapsed_time)

puzzle.show(solution.s)
