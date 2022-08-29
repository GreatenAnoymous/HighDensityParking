from MCP import *

        
if __name__=="__main__":
    problem=OneShotInstance("./demo/large_parking_retrieval.json")
    solver=BCPRSolver(problem)
    solver.solve()
    paths=[]
    for a in  solver.agents:
        paths.append(a.path)
    mpc=MPC(paths)
    mpc.mpcSolve()