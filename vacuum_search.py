"""
To install pip, you need to first choose python 3.9. Then install future package. Details in this youtube
video:  https://youtu.be/2iLKvq6z7lI?si=a4hfRqs5oKD-ISqe
To install numpy use terminal at the bottom left panel and call : >pip intall numpy
"""

import os.path
from tkinter import *
from agents import *
from search import *
import sys
import math

"""
1- BFS: Breadth first search. Using tree or graph version, whichever makes more sense for the problem
2- DFS: Depth-First search. Again using tree or graph version.
3- UCS: Uniform-Cost-Search. Using the following cost function to optimise the path, from initial to current state.
4- Greedy: Uses Manhattan distance to the next closest dirty room as heuristic for greedy algorithm. To find the next closest dirty room, use Manhattan distance.
5- A*:  Using A star search.
"""
searchTypes = ['None', 'BFS', 'DFS', 'UCS', 'Greedy', 'A*']
"""
Cost function used for UCS and A* search.
-'Step' counts the numbers of steps from start
-'StepTurn' adds number of turns to Step cost
-'StayLeft' favors staying on the left side of the map
-'StayUp' favors staying on the top side of the map
"""
costFunctions = ['Step', 'StepTurn', 'StayLeft', 'StayUp']

args = dict()

class VacuumPlanning(Problem):
    """ The problem of find the next room to clean in a grid of m x n rooms.
    A state is represented by state of the grid cells locations. Each room is specified by index set
    (i, j), i in range(m) and j in range (n). Final goal is to clean all dirty rooms. We go by performing sub-goals, each being cleaning the "next" dirty room.
    """

    def __init__(self, env, searchtype):
        """ Define goal state and initialize a problem
            initial is a pair (i, j) of where the agent is
            goal is next pair(k, l) where map[k][l] is dirty
        """
        self.solution = None
        self.env = env
        self.state = env.agent.location
        super().__init__(self.state)
        self.map = env.things
        self.searchType = searchtype
        env.agent.direction = 'UP'  #initial direction of the agent.
        self.agent = env.agent


    def generateSolution(self):
        """ generate full path to the next goal based on type of the search chosen by user"""
        if self.searchType == 'None':
            print("generateSolution: searchType not set!")
            return

        self.env.read_env()
        self.state = env.agent.location
        super().__init__(self.state)
        path = None
        explored = None
        if self.searchType == 'BFS':
            path, explored = breadth_first_graph_search(self)
        elif self.searchType == 'DFS':
            path, explored = depth_first_graph_search(self)
        elif self.searchType == 'UCS':
            path, explored = best_first_graph_search(self, lambda node: node.path_cost)
        elif self.searchType == 'Greedy':
            path, explored = best_first_graph_search(self, None)
        elif self.searchType == 'A*':
            path, explored = astar_search(self, None)
        else:
            raise 'NameError'

        if ( path != None):
            self.env.set_solution(path)
        else:
            print("There is no solution!\n")

        if (explored != None):
            self.env.display_explored(explored)
            self.env.exploredCount += len(explored)
            self.env.pathCount += len(self.env.path)
            ExploredCount_label.config(text=str(self.env.exploredCount))
            PathCount_label.config(text=str(self.env.pathCount))
        else:
            print("There is not explored list!\n")

    def generateNextSolution(self):
        self.generateSolution()


    def actions(self, state):
        """ Return the actions that can be executed in the given state.
        The result would be a list, since there are only four possible actions
        in any given state of the environment """

        possible_neighbors = self.env.things_near(state)
        possible_actions = ['UP', 'DOWN', 'LEFT', 'RIGHT']
        for slot in possible_neighbors:
            if isinstance(slot[0], Wall):
                x, y = slot[0].location
                if x == state[0] and y == state[1] + 1:
                    possible_actions.remove('UP')
                if x == state[0] and y == state[1] - 1:
                    possible_actions.remove('DOWN')
                if x == state[0] + 1 and y == state[1]:
                    possible_actions.remove('RIGHT')
                if x == state[0] - 1 and y == state[1]:
                    possible_actions.remove('LEFT')

        return possible_actions

    def result(self, state, action):
        """ Given state and action, return a new state that is the result of the action.
        Action is assumed to be a valid action for the state """
        self.agent.direction = action
        new_state = list(state)
        if action == 'RIGHT':
            new_state[0] += 1
        elif action == 'LEFT':
            new_state[0] -= 1
        elif action == 'UP':
            new_state[1] += 1
        elif action == 'DOWN':
            new_state[1] -= 1

        return new_state

    def goal_test(self, state):
        """ Given a state, return True if state is a goal state or False, otherwise """
        return self.env.some_things_at(state, Dirt)

    # def path_cost(self, curNode, state1, action, state2):
    #     """To be used for UCS and A* search. Returns the cost of a solution path that arrives at state2 from
    #     state2 via action, assuming it costs c to get up to state1. For our problem state is (x, y) coordinate pair.
    #     Rotation of the Vacuum machine costs equivalent of 0.5 unit for each 90' rotation. """
    #     print("path_cost: to be done by students")
    #     cost = curNode.path_cost
    #     print("Your code goes here.\n")
    #
    #     return cost
    def path_cost(self, curNode, state1, action, state2):
        """Calculate the cost to reach state2 from state1."""
        cost = curNode.path_cost + 1  # Default step cost is 1
        cost += self.computeTurnCost(curNode.action, action)
        if self.costFunc == 'StayLeft':
            cost += state2[0] / 10
        elif self.costFunc == 'StayUp':
            cost += state2[1] / 10
        return cost

    # def computeTurnCost(self, action1, action):
    #     print("computeTurnCost: to be done by students")
    #     print(" Your code goes here\n")
    #     return 0
    def computeTurnCost(self, action1, action2):
        """Calculate the cost of turning from action1 to action2."""
        turns = {
            ('UP', 'LEFT'): 1, ('UP', 'RIGHT'): 1, ('DOWN', 'LEFT'): 1, ('DOWN', 'RIGHT'): 1,
            ('LEFT', 'UP'): 1, ('LEFT', 'DOWN'): 1, ('RIGHT', 'UP'): 1, ('RIGHT', 'DOWN'): 1,
            ('UP', 'DOWN'): 2, ('DOWN', 'UP'): 2, ('LEFT', 'RIGHT'): 2, ('RIGHT', 'LEFT'): 2,
        }
        return turns.get((action1, action2), 0) * 0.5

    # def findMinManhattanDist(self, pos):
    #     """use distance_manhattan() function to find the min distance between position pos and any of the dirty rooms.
    #     Dirty rooms which are maintained in env.dirtyRooms.
    #     """
    #     print("findMinManhattanDist: to be done by students.")
    #     minDist = math.inf
    #     print(" Your code goes here\n")
    #
    #     return 0
    def findMinManhattanDist(self, pos):
        """Find the minimum Manhattan distance from pos to any dirty room."""
        minDist = math.inf
        for dirt in self.env.dirtyRooms:
            dist = distance_manhattan(pos, dirt)
            if dist < minDist:
                minDist = dist
        return minDist

    # def findMinEuclidDist(self, pos):
    #     """find min Euclidean dist to any of the dirty rooms
    #     hint: Use distance_euclid() in utils.py"""
    #     print("findMinEuclidDist: to be done by students.")
    #     minDist = math.inf
    #     print(" Your code goes here\n")
    #
    #     return 0
    # def h(self, node):
    #     """ Return the heuristic value for a given state. For this problem use minimum Manhattan
    #     distance to a dirty room, among all the dirty rooms.
    #     hint:
    #     """
    #     print("h(heuristic): to be defined and implemented by students.")
    #     heur = 0
    #
    #     print(" Your code goes here\n")
    #     return heur
    def findMinEuclidDist(self, pos):
        """Find the minimum Euclidean distance from pos to any dirty room."""
        minDist = math.inf
        for dirt in self.env.dirtyRooms:
            dist = distance_euclid(pos, dirt)
            if dist < minDist:
                minDist = dist
        return minDist

    def h(self, node):
        """Heuristic function for A* search - minimum Manhattan distance to a dirty room."""
        return self.findMinManhattanDist(node.state)




def agent_label(agt):
    """creates a label based on direction"""
    dir = agt.direction
    lbl = '^'
    if dir == Direction.D:
        lbl = 'v'
    elif dir == Direction.L:
        lbl = '<'
    elif dir == Direction.R:
        lbl = '>'

    return lbl

def is_agent_label(lbl):
    """determines if the label is one of the labels tht agents have: ^ v < or >"""
    return lbl == '^' or lbl == 'v' or lbl == '<' or lbl == '>'

class Gui(VacuumEnvironment):
    """This is a two-dimensional GUI environment. Each location may be
    dirty, clean or can have a wall. The user can change these at each step.
    """
    xi, yi = (0, 0)

    #perceptible_distance = 1

    def __init__(self, root, width, height):
        self.dirtCount = 0
        self.frames = None
        self.path = None
        self.searchType = searchTypes[0]
        self.searchEngineSet = False
        self.costFunc = costFunctions[0]
        self.explored = None
        self.done = True
        self.solution = None
        self.searchAgent = None
        self.exploredCount = 0
        self.pathCount = 0
        print("creating xv with width ={} and height={}".format(width, height))
        super().__init__(width, height)

        self.agent = None
        self.root = root
        self.create_frames(height)
        self.create_buttons(width)
        self.create_walls()
        self.setupTestEnvironment()

    def setupTestEnvironment(self):
        """ sets up the environment"""

        xi = self.width // 2
        yi = self.height // 2
        if self.agent is None:
            theAgent = XYSearchAgent(program=XYSearchAgentProgram, loc=(yi, xi))
            xi, yi = theAgent.location
            self.add_agent(theAgent, (yi, xi))
        else:
            self.agent.location = [xi, yi]
            xi, yi = self.agent.location
            self.buttons[yi][xi].config(text='')
            self.agent.direction = 'UP'
            if len(self.agents) > 0:
                self.delete_thing(self.agents[0])
            self.add_thing(Agent(), (xi, yi))
            self.buttons[yi][xi].config(bg='white', text=agent_label(self.agent), state='normal')


        self.searchType = searchTypes[0]
        self.agent.performance = 0
        self.direction = Direction("up")

        """create internal wall cells"""
        self.createFixedBlockingCells()

        self.create_dirts()

        self.searchType = args['searchType']
        self.solution = []
        self.explored = set()
        self.read_env()

    def createFixedBlockingCells(self):
        """create a fixed pattern internal wall blocks. Bellow blks are collection of L-shape walls"""
        x1 = wid // 10
        y1 = x1+1
        x2 = x1 + 1
        y2 = wid//2
        #lower left block
        blk_ll = [(x1, y1), (x1, y1+1), (x1, y1+2), (x1, y1+3), (x1+1, y1+3), (x1+2, y1+3), (x1+3, y1+3), (x1+4, y1+3), (x1+5, y1+3), (x1+6, y1+3)]
        #upper right block
        blk_ul = [(x2+3, y2), (x2+2, y2), (x2+1, y2), (x2, y2), (x2, y2+1), (x2, y2+2), (x2, y2+3), (x2, y2+4), (x2, y2+5), (x2, y2+6)]
        #upper middle block
        blk_um = [(x2+2, y2+3), (x2+3, y2+3), (x2+4, y2+3), (x2+5, y2+3)]
        x3=wid//4 + 1
        y3 = y1-1
        #lower right block
        blk_lr = [(x3, y3+2), (x3, y3+1), (x3, y3), (x3+1, y3), (x3+2, y3), (x3+3, y3), (x3+4, y3), (x3+5, y3), (x3+6, y3), (x3+7, y3)]
        x4 = wid//2 + 1
        y4 = wid - wid//4
        #upper right block
        blk_ur = [(x4, y4+2), (x4, y4+1), (x4, y4), (x4+1, y4), (x4+2, y4), (x4+3, y4), (x4+4, y4), (x4+5, y4), (x4+6, y4), (x4+6, y4-1), (4+6, y4-2)]
        x5 = x4
        y5 = wid //2
        #middle right block
        blk_mr = [(x5, y5 + 3), (x5, y5+2), (x5, y5+1), (x5, y5), (x5, y5-1), (x5, y5-2), (x5+1, y5-2), (x5+2, y5-2)]
        x6 = wid // 2
        y6 = y3 + 2
        blk_ml = [(x6, y6), (x6+1, y6), (x6+2, y6), (x6 +3, y6), (x6+4, y6), (x6+5, y6)]
        #right middle column
        x7 = wid - 2
        y7 = hig // 4 + 2
        blk_mrb = [(x7, y7), (x7-1, y7), (x7-2, y7), (x7-2, y7+1), (x7-2, y7+2), (x7-2, y7+3)]
        blk = blk_ll + blk_ul + blk_lr + blk_ur + blk_mr + blk_ml + blk_um + blk_mrb

        if (args['corner'] == True):
            blk = blk + [(1, hig//2), (1, hig//2 - 1), (wid//3, hig - 2), (wid//3, hig-3)]

        for pnt in blk:
            self.buttons[pnt[1]][pnt[0]].config(bg='red', text='W', disabledforeground='black')


    def createRandomBlockingCells(self):
        """next create a random number of block walls inside the grid as well"""
        xi, yi = self.agent.location
        roomCount = (self.width - 1) * (self.height - 1)
        blockCount = random.choice(range(roomCount//7, roomCount//3))
        for _ in range(blockCount):
            rownum = random.choice(range(1, self.height - 1))
            colnum = random.choice(range(1, self.width - 1))
            while(rownum ==yi and colnum==xi):
                rownum = random.choice(range(1, self.height - 1))
                colnum = random.choice(range(1, self.width - 1))
            self.buttons[rownum][colnum].config(bg='red', text='W', disabledforeground='black')


    def create_frames(self, h):
        """Adds h row frames to the GUI environment."""
        self.frames = []
        for _ in range(h):
            frame = Frame(self.root, bg='blue')
            frame.pack(side='bottom')
            self.frames.append(frame)

    def create_buttons(self, w):
        """Adds w buttons to the respective row frames in the GUI."""
        self.buttons = []
        for frame in self.frames:
            button_row = []
            for _ in range(w):
                button = Button(frame, bg='white', state='normal', height=1, width=1, padx=1, pady=1)
                button.config(command=lambda btn=button: self.toggle_element(btn))
                button.pack(side='left')
                button_row.append(button)
            self.buttons.append(button_row)


    def create_walls(self):
        """Creates the outer boundary walls which do not move. Also create a random number of
        internal blocks of walls."""
        for row, button_row in enumerate(self.buttons):
            if row == 0 or row == len(self.buttons) - 1:
                for button in button_row:
                    button.config(bg='red', text='W', state='disabled', disabledforeground='black')
            else:
                button_row[0].config(bg='red', text='W', state='disabled', disabledforeground='black')
                button_row[len(button_row) - 1].config(bg='red', text='W', state='disabled', disabledforeground='black')

    def create_dirts(self):
        """ set a small random number of rooms to be dirty at random location on the grid
        This function should be called after all other objects created"""
        self.read_env()   # this is needed to make sure wall objects are created

        if ( args['corner'] == False):
            self.dirtCount = 6 #random.choice(range(lowDirtyCount, 2*lowDirtyCount))
            self.dirtyRooms = {(2, 14), (15, 11), (16, 16), (10, 8), (8, 1), (7, 16)}
        else:
            self.dirtCount = 4  # 4 dirty rooms in 4 corners of the map
            self.dirtyRooms = {(1, 1), (1, self.height-2), (self.width-2, 1), (self.width-2, self.height-2)}

        for rm in self.dirtyRooms:
            self.buttons[rm[1]][rm[0]].config(bg="grey") # or use #fff for different shades of grey

        #self.createRandomDirtyRooms()


    def createRandomDirtyRooms(self):
        # bellow is for the case you want to have random dirty room locations. In this case comment out about 4 lines
        numRooms = (self.width-1) * (self.height -1)
        self.dirtCount = 10
        dirtCreated = 0
        self.dirtyRooms = set()
        #print ("lowdirtCount = ",lowDirtyCount, ", width=", self.width, ", height=", self.height)
        while dirtCreated != self.dirtCount:
            rownum = random.choice(range(1, self.height-1))
            colnum = random.choice(range(1, self.width-1))
            if self.some_things_at((colnum, rownum)):
                continue
            self.buttons[rownum][colnum].config(bg='grey')
            dirtCreated += 1
            self.dirtyRooms.add((colnum, rownum))

        print(self.dirtyRooms)


    def setSearchEngine(self, choice):
        """sets the chosen search engine for solving this problem"""
        self.searchType = choice
        self.searchAgent = VacuumPlanning(self, self.searchType)
        self.searchAgent.generateSolution()
        self.searchEngineSet = True
        self.done = False

    def set_solution(self, path):
        sol = path.solution()
        self.solution = list(reversed(sol))
        self.path = []
        if(self.agent == None):
            return
        while(path.state != self.agent.location):
            self.path.append(path.state)
            path = path.parent
        if(len(self.path)>0):
            self.path.pop(0)


    def display_explored(self, explored):
        """display explored slots in a light pink color"""
        if len(self.explored) > 0:     # means we have explored list from previous search. So need to clear their visual fist
            for (x, y) in self.explored:
                self.buttons[y][x].config(bg='white')

        # now pink color the new explored list
        self.explored = explored
        for (x, y) in explored:
            self.buttons[y][x].config(bg='pink')

        # finally color orange the found path
        for (x, y) in self.path:
            self.buttons[y][x].config(bg='orange')

    def add_agent(self, agt, loc):
        """add an agent to the GUI"""
        self.add_thing(Agent(), loc)
        assert(len(self.agents) == 1)
        # Place the agent at the provided location.
        lbl = agent_label(agt)
        self.buttons[loc[1]][loc[0]].config(bg='white', text=lbl, state='normal')
        self.agent = agt

    def toggle_element(self, button):
        """toggle the element type on the GUI when a room is clicked"""
        bgcolor = button['bg']
        txt = button['text']
        if is_agent_label(txt):
            return
        else:
            if bgcolor == 'red':
                button.config(bg='grey', text='')
            elif bgcolor == 'grey':
                button.config(bg='white', text='', state='normal')
            elif bgcolor == 'white':
                button.config(bg='red', text='W')

    def removeDirtyRoom(self, loc):
        for room in self.dirtyRooms:
            if(room[0] == loc[0] and room[1]==loc[1]):
                self.dirtyRooms.discard(room)
                return
        print("removeDirtyRoom: error! dirty room ({}, {}) not found".format(room[0], room[1]))


    def execute_action(self, agent, action):
        """Determines the action the agent performs."""
        if(agent == None):
            return
        xi, yi = agent.location
        #print("agent at location (", xi, yi, ") and action ", action)
        if action == 'Suck':
            dirt_list = self.list_things_at(agent.location, Dirt)
            if dirt_list:
                dirt = dirt_list[0]
                if self.buttons[yi][xi]['bg'] != 'grey':
                    print("Error!: execute_action: mismatch with dirty room color")
                agent.performance += 10

                self.delete_thing(dirt)
                self.removeDirtyRoom(agent.location)
                self.buttons[yi][xi].config(bg='white', state='normal')
        else:   # means action == 'Move'
            agent.location = self.searchAgent.result(agent.location, action)
            self.buttons[yi][xi].config(text='')
            xf, yf = agent.location
            self.buttons[yf][xf].config(text=agent_label(agent))
            self.move_to(self.agent, agent.location)


    def read_env(self):
        """read_env: This sets proper wall or Dirt status based on bg color"""
        """Reads the current state of the GUI environment."""
        self.dirtCount = 0
        for j, btn_row in enumerate(self.buttons):
            for i, btn in enumerate(btn_row):
                if (j != 0 and j != len(self.buttons) - 1) and (i != 0 and i != len(btn_row) - 1):
                    if self.some_things_at((i, j)):  # and (i, j) != agt_loc:
                        for thing in self.list_things_at((i, j)):
                            if not isinstance(thing, Agent):
                                self.delete_thing(thing)
                    if btn['bg'] == 'grey':  # adding dirt
                        self.add_thing(Dirt(), (i, j))
                        self.dirtCount += 1
                    elif btn['bg'] == 'red':  # adding wall
                        self.add_thing(Wall(), (i, j))

    def update_env(self):
        """Updates the GUI environment according to the current state."""
        self.read_env()
        self.done = False
        self.step()

    def step(self):
        """updates the environment one step. Currently it is associated with one click of 'Step' button.
        """
        if env.dirtCount == 0:
            print("Everything is clean. DONE!")
            self.done = True
            return

        if( self.searchEngineSet == False):
            self.setSearchEngine(args['searchType'])

        if len(self.solution) == 0: # agent has reached a dirty room. So the proper action is 'suck'
            self.execute_action(self.agent, 'Suck')
            self.read_env()
            if env.dirtCount > 0 and self.searchAgent is not None:
                self.searchAgent.generateNextSolution()
                #self.running = False
        else:   # agent is moving towards the next goal. So the proper action is 'move'
            move = self.solution.pop()
            self.execute_action(self.agent, move)


    def run(self, delay=0.3):
        """Run the Environment for given number of time steps,"""
        if (self.searchEngineSet == False):
            self.setSearchEngine(args['searchType'])
            delay = 0.5
        else:
            delay = 0.2

        self.running = True
        while self.done is not True:
            if self.is_done() or self.running is False:
                break
            self.update_env()
            sleep(delay)
            Tk.update(self.root)

        if (args['auto'] == True and self.dirtCount > 0):
            #self.searchEngineSet = False
            self.run()

    def reset_env(self):
        """Resets the GUI and agents environment to the initial clear state."""
        self.running = False
        ExploredCount_label.config(text=str(0))
        PathCount_label.config(text=str(0))
        self.exploredCount = 0
        self.pathCount = 0
        searchTypeStr.set(args['searchType'])

        for j, btn_row in enumerate(self.buttons):
            for i, btn in enumerate(btn_row):
                if (j != 0 and j != len(self.buttons) - 1) and (i != 0 and i != len(btn_row) - 1):
                    if self.some_things_at((i, j)):
                        for thing in self.list_things_at((i, j)):
                            self.delete_thing(thing)
                    btn.config(bg='white', text='', state='normal')

        self.setupTestEnvironment()

    """def switchTurnCost(self):
        #enables / disables turn cost of 1 for each 90' turn of the agent
        if self.turnCostOn == False:
            self.turnCostOn = True
            turn_button.config(bg = "white")
        else:
            self.turnCostOn = False
            turn_button.config(bg = "grey")
        #self.reset_env()

        self.searchAgent = VacuumPlanning(self, self.searchType)
        self.searchAgent.generateSolution()
        self.done = False
       """

    def setCostFunction(self, choice):
        """sets the chosen path cost function for solving this problem"""
        self.costFunc = choice
        self.done = False

"""
Our search Agents ignore environment percepts for planning. The planning is done based on static
 data from environment at the beginning. The environment is fully observable
 """
def XYSearchAgentProgram(percept):
    pass


class XYSearchAgent(Agent):
    """The modified SimpleRuleAgent for the GUI environment."""

    def __init__(self, program, loc):
        super().__init__(program)
        self.location = loc
        self.direction = Direction("up")
        self.searchType = searchTypes[0]
        self.stepCount = 0

def readCommand( argv ):
    """
    Processes the command used to run vacuumSearch from the command line.
    """
    from optparse import OptionParser
    usageStr = """
    USAGE:      python vacuum_search.py <options>
    EXAMPLES:   (1) python vacuum_search.py
                    - starts an interactive game
                (2) python vacuum_search.py -s A*
                    -perform A* search to clean all rooms
                (3)  python vacuum_search.py -s Greedy -r Manhattan
                    - perform greedy algorithm search with Manhattan cost function
                (4) python vacuum_search.py -s UCS -c StayTop
    """
    parser = OptionParser(usageStr)

    parser.add_option('-s', '--searchType', dest='searchType',
                      help='the algorithm to be used for search: options are BFS, DFS, UCS, Greedy, A*',
                      choices=['BFS', 'DFS', 'UCS', 'Greedy', 'A*'],
                      default='BFS')
    parser.add_option('-c', '--cost', dest='costFunc',
                      help='cost function to be used with Greedy and A* algorithms. choices are: Step, StepTurn, StayLeft, StayTop',
                      choices=['Step', 'StepTurn', 'StayLeft', 'StayTop'],
                      default='Step')
    parser.add_option('-r', '--heuristic', dest='heuristic',
                      help='heuristic function to be used with Greedy and A* algorithms. Options are: Manhattan, Euclid',
                      choices=['Manhattan', 'Euclid'],
                      default='Manhattan')
    parser.add_option('-n', '--corner', action='store_true', dest='cornerSearch',
                      help='corner search is for having dirt is 4 corners only, to be used by Greedy and A* algorithms. Options are: False, True',
                      default=False)

    parser.add_option('-a', '--automatic', action='store_true', dest='auto',
                      help='Runs the search through all the goals to the end', default=False)

    options, otherjunk = parser.parse_args(argv)
    if len(otherjunk) != 0:
        raise Exception('Command line input not understood: ' + str(otherjunk))
    args = dict()

    # Fix the random seed
    #if options.fixRandomSeed: random.seed('cmpt310')

    args['searchType'] = options.searchType
    args['costFunc'] = options.costFunc
    args['corner'] = options.cornerSearch
    args['heuristic'] = options.heuristic
    args['auto'] = options.auto

    return args


if __name__ == "__main__":
    """
    The main function called when run from command line: >python vacuum_search.py
    """
    args = readCommand(sys.argv[1:])  # Get game components based on input

    win = Tk()
    win.title("Searching Cleaning Robot")
    win.geometry("710x710+50+0")
    win.resizable(True, True)
    frame = Frame(win, bg='black')
    frame.pack(side='bottom')
    topframe = Frame(win, bg='black')
    topframe.pack(side='top')

    wid = 20
    hig = wid

    env = Gui(win, wid, hig)

    ExploredCount_label = Label(topframe, text='ExploredCount: 0', bg='green', fg='white', bd=2, padx=2, pady=2)
    ExploredCount_label.pack(side='left')
    PathCount_label = Label(topframe, text='PathCount: 0', bg='blue', fg='white', padx=2, pady=2)
    PathCount_label.pack(side='right')
    reset_button = Button(frame, text='Reset', height=2, width=5, padx=2, pady=2)
    reset_button.pack(side='left')
    next_button = Button(frame, text='Next', height=2, width=5, padx=2, pady=2)
    next_button.pack(side='left')
    run_button = Button(frame, text='Run', height=2, width=5, padx=2, pady=2)
    run_button.pack(side='left')

    costFuncStr = StringVar(win)
    costFuncStr.set(args['costFunc'])
    costFuncStr_dropdown = OptionMenu(frame, costFuncStr, *costFunctions, command=env.setCostFunction)
    costFuncStr_dropdown.pack(side='left')

    next_button.config(command=env.update_env)
    reset_button.config(command=env.reset_env)
    run_button.config(command=env.run)

    searchTypeStr = StringVar(win)
    searchTypeStr.set(args['searchType'])
    searchTypeStr_dropdown = OptionMenu(frame, searchTypeStr, *searchTypes, command=env.setSearchEngine)
    searchTypeStr_dropdown.pack(side='left')

    env.costFunc = args['costFunc']

    win.mainloop()
