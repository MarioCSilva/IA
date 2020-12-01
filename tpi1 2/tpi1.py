from tree_search import *



class MyNode(SearchNode):
    def __init__(self, state, parent, depth=0, cost=0, evalfunc=0):
        super().__init__(state, parent)
        self.depth = depth
        self.cost = cost
        self.evalfunc = evalfunc
        self.children = None

class MyTree(SearchTree):

    def __init__(self,problem, strategy='breadth',max_nodes=None):
        self.root = MyNode(problem.initial, None, 0, 0, problem.domain.heuristic(problem.initial, problem.goal))
        self.open_nodes = [self.root]
        self.problem = problem
        self.strategy = strategy
        self.max_nodes = max_nodes
        self.solution = None
        self.total_nodes = 1
        self.non_terminal_nodes = 0
        self.terminal_nodes = 0

    def astar_add_to_open(self,lnewnodes):
        self.open_nodes.extend(lnewnodes)
        self.open_nodes.sort(key=lambda  node: node.evalfunc + node.cost)

    def effective_branching_factor(self):
        # n = self.total_nodes
        # d = self.solution_length 
        # tol = 0.000001 

        # n_linha = 0
        # i = 0

        # while abs(n - n_linha)> tol:
        #     i+=1
        #     n_linha += (n**(1/d))**i
        pass

    def update_ancestors(self,node):
        if node.children:
            node.evalfunc = sorted([node.evalfunc for node in node.children])[0]
            if node.parent:
                self.update_ancestors(node.parent)

        
    def discard_worse(self):
        pass


    @property
    def solution_cost(self):
        return self.solution.cost

    @property
    def solution_length(self):
        return self.solution.depth
    
    def search2(self):
        while self.open_nodes != []:
            node = self.open_nodes.pop(0)
            if self.problem.goal_test(node.state):
                self.solution = node
                return self.get_path(node)

            self.non_terminal_nodes += 1 

            lnewnodes = []
            for a in self.problem.domain.actions(node.state):
                newstate = self.problem.domain.result(node.state,a)
                if newstate not in self.get_path(node):
                    newnode = MyNode(
                        newstate,
                        node,
                        node.depth + 1, 
                        node.cost + self.problem.domain.cost(node.state, a), 
                        self.problem.domain.heuristic(newstate, self.problem.goal)
                    )                    
                    newnode.evalfunc += newnode.cost
                    lnewnodes.append(newnode)
                    self.total_nodes += 1
                
            node.children = lnewnodes
            self.update_ancestors(node)
            

            self.add_to_open(lnewnodes)
            self.terminal_nodes = len(self.open_nodes)  

            # if self.max_nodes:
            #     while self.non_terminal_nodes + self.terminal_nodes> self.max_nodes:
            #         self.discard_worse()
            
        return None

    # shows the search tree in the form of a listing
    def show(self,heuristic=False,node=None,indent=''):
        if node==None:
            self.show(heuristic,self.root)
            print('\n')
        else:
            line = indent+node.state
            if heuristic:
                line += (' [' + str(node.evalfunc) + ']')
            print(line)
            if node.children==None:
                return
            for n in node.children:
                self.show(heuristic,n,indent+'  ')



