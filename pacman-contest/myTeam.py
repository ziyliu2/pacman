# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).

import sys
sys.path.append('teams/nugget/')
from captureAgents import CaptureAgent
import random, time, util
from game import Directions
import game
from distanceCalculator import Distancer
from util import nearestPoint
import numpy as np
DEPTH = 2
#################
 # Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'heuristicSearchAgent', second = 'MinmaxAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.

  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """

  # The following line is an example only; feel free to change it.
  return [eval(first)(firstIndex), eval(second)(secondIndex)]

##########
# Agents #
##########
      
class MinmaxAgent(CaptureAgent):
    def registerInitialState(self, gameState):
        CaptureAgent.registerInitialState(self, gameState)
        gameMap =gameState.getWalls()
#
        height,width = gameMap.height,gameMap.width
        dead_end = np.zeros([height,width])
        for a in gameMap.asList():
            x=a[0]
            y=a[1]
            dead_end[y,x] = 1
        self.dead_end = findDeadEnd(dead_end) 
               
        self.foodCounter = util.Counter({'food':0})
        self.initPos =gameState.getInitialAgentPosition(self.index)
        
    def enemyStep(self,index,gameState):
        enemyActions = gameState.getLegalActions(index)
        pos = gameState.getAgentPosition(self.index)
        score = float('Inf')
        chooseAction = []
        for action in enemyActions:
            temp = gameState.generateSuccessor(index,action)
            enemyPos = temp.getAgentState(index).getPosition()
            disScore = self.distancer.getDistance(enemyPos, pos)
            if disScore<score:
                chooseAction = [action]
                score = disScore
            elif disScore == score:
                chooseAction.append(action)
                score = disScore
        action = random.choice(chooseAction)
        return action,gameState.generateSuccessor(index,action)
    
    def myStep(self,gameState):
        myActions = gameState.getLegalActions(self.index)
        score = float('-Inf')
        chooseAction = []
        for action in myActions:
            temp = gameState.generateSuccessor(self.index,action)
#            nextPos = temp.getAgentState(self.index).getPosition()
            myScore = self.evaluationFunction(self,temp)
            if myScore>score:
                chooseAction = [action]
                score = myScore
            elif myScore == score:
                chooseAction.append(action)
                score = myScore
        action = random.choice(chooseAction)
        return action,gameState.generateSuccessor(self.index,action)
    
    def max_value(self,state, currentDepth):
        #depth plus one
        currentDepth=currentDepth+1
        # if depth now equals what we set,finish it
        if state.isOver() or currentDepth == DEPTH:
            return self.evaluationFunction(state)
        # init v
        v= float('-Inf')
        # find the max for every branch
        
        for action in state.getLegalActions(self.index):
            v=max(v, self.min_value(state.generateSuccessor(self.index, action), currentDepth))
        return v
    def min_value(self,state,currentDepth):
#        print()
        # if over
        if state.isOver():
            return self.EvaluationFunction(state)
        # init v
        v=float('Inf')
        # find min for every max 
        actions = state.getLegalActions(self.enemiesIndex[0])       

        for action in actions:
            v=min(v, self.max_value(state.generateSuccessor(self.enemiesIndex[0], action), currentDepth))
        return v
    def chooseAction(self, gameState):

        self.foodNum = gameState.data.agentStates[self.index].numCarrying
        actions = gameState.getLegalActions(self.index)
        self.position = gameState.getAgentPosition(self.index)
        maximum = float('-Inf')
        result = None
        
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        self.invaders = [a for a in enemies if (not a.isPacman) and a.getPosition() != None]
        
        self.enemiesIndex = [i for i in self.getOpponents(gameState) if gameState.getAgentState(i).getPosition()!=None]
#        print([invader.getPosition() for invader in invaders],self.enemiesIndex)

        
        if self.invaders==[] :#no visible enemies
            #in this case,we use different rule to save time
            maxScore = float('-Inf')
            for action in actions:
                if action != Directions.STOP:
                    #step once to find food
                    successor = self.getSuccessor(gameState, action)
                    foodList = self.getFood(successor).asList()
#                    features = util.Counter()
#                    features['successorScore'] = -len(foodList)
                    rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
                    if action == rev: reverse = 1
                    else: reverse = 0
                    
                    if self.foodNum >= 3:
                        pos = successor.getAgentState(self.index).getPosition()
                        foodWeight = self.getMazeDistance(pos,self.initPos)
                    else:
                        foodWeight = 0
                    # Compute distance to the nearest food
                    if len(foodList) > 0: 
                      myPos = successor.getAgentState(self.index).getPosition()
                      minDistance = min([self.getMazeDistance(myPos, food) for food in foodList])
#                      features['distanceToFood'] = minDistance
                      score = -len(foodList)*100 - minDistance - foodWeight*100 - reverse*1
                      if score>maxScore:
                          maxScore = score
                          bestAction = action
            return bestAction
        else:          
            #if there is enemy around
            maxScore = float('-Inf')
            chooseAction = []
            for action in actions :
                if action!=Directions.STOP:
                    print(action)
                    score = self.evaluationFunction2(gameState.generateSuccessor(self.index, action))
                    if score>maxScore:
                        chooseAction = [action]
                        maxScore = score
                    elif maxScore == score:
                        chooseAction.append(action)
                        maxScore = score
            action = random.choice(chooseAction)            
#            for action in actions:
#                if(action != Directions.STOP):
#                    currentDepth = 0
#                    # step once and find minstate for enemy
#                    currentMax = self.min_value(gameState.generateSuccessor(self.index, action), currentDepth)
#                    if currentMax > maximum:
#                        maximum=currentMax
#                        result =action
            print("finalchoose:",action)
            print("-------------------------------------------------")
            
            
            return action
    
    def getSuccessor(self, gameState, action):
        """
        Finds the next successor which is a grid position (location tuple).
        """
        successor = gameState.generateSuccessor(self.index, action)
        pos = successor.getAgentState(self.index).getPosition()
        if pos != nearestPoint(pos):
          # Only half a grid position was covered
          return successor.generateSuccessor(self.index, action)
        else:
          return successor
    def evaluationFunction2(self, currentGameState):
        
        A=10
        B=10
        C=5
        D=10000
        
        pos = currentGameState.getAgentState(self.index).getPosition()
        enemyPos = [currentGameState.getAgentState(self.enemiesIndex[i]).getPosition() for i in range(len(self.invaders))]
        initPos = currentGameState.getInitialAgentPosition(self.index)
        
        for enemyIndex in self.enemiesIndex:
            _,currentGameState = self.enemyStep(enemyIndex,currentGameState)
            
        pos = currentGameState.getAgentState(self.index).getPosition()
        
        if(pos == initPos):
            death = 1
        else:
            death = 0
        
#        distancer = Distancer(currentGameState.data.layout)
        if self.foodNum>=2:
            foodWeight = (self.distancer.getDistance(pos,initPos)+1)*self.foodNum**2
        else:
            foodWeight = 0
            
        ghostDist = 0
        enemyPos = [currentGameState.getAgentState(self.enemiesIndex[i]).getPosition() for i in range(len(self.invaders))]
        for p in enemyPos:
            ghostDist += self.distancer.getDistance(pos,p)
            print(p,ghostDist)
        ghostDist = ghostDist/len(enemyPos)
        
        deadEnd = self.dead_end[int(pos[1]),int(pos[0])]
        
        score = A*ghostDist-B*deadEnd-C*foodWeight-D*death
        print(ghostDist,deadEnd,foodWeight,death,score)
        return score
        
    def evaluationFunction(self, currentGameState):
        #parameters
        A=100
        B=10
        C=5
        D=10000
        
        #get enemy and its position
        pos = currentGameState.getAgentState(self.index).getPosition()
        enemyPos = currentGameState.getAgentState(self.enemiesIndex[0]).getPosition()
        initPos = currentGameState.getInitialAgentPosition(self.index)
        if(pos == initPos):
            death = 1
        else:
            death = 0
#        print(initPos)
        distancer = Distancer(currentGameState.data.layout)
        #if there is some food carrying,think about go back
        if self.foodNum>=2:
            foodWeight = 50/(distancer.getDistance(pos,initPos)+1)*self.foodNum**2
        else:
            foodWeight = 0
        #get food info
        if currentGameState.isOnRedTeam(self.index):
            food = currentGameState.getBlueFood()
        else:
            food = currentGameState.getRedFood()
        foodList = food.asList()            
        
        foodDist = [100000]
        for foodPos in foodList:
            foodDist.append(distancer.getDistance(pos,foodPos))
        #inv it because closer to food is better
        invFoodDist = 1.0 / (min(foodDist)+1)   
                
        ghostDist = distancer.getDistance(pos,enemyPos)
#        print(invFoodDist,ghostDist,foodWeight)

        dead_end = self.dead_end[int(pos[1]),int(pos[0])]
            
        score = A*ghostDist-B*dead_end+C*foodWeight-D*death
        
        return score
    

class heuristicSearchAgent(CaptureAgent):
    def registerInitialState(self, gameState):
        CaptureAgent.registerInitialState(self, gameState)
        self.initPos =gameState.getInitialAgentPosition(self.index)
        self.foodList = self.getFoodYouAreDefending(gameState).asList()
        self.foodLost = []
        gameMap =gameState.getWalls()
        self.width = gameMap.width
        self.mid = self.width/2
        self.lastEnemyPos = None
        
    def chooseAction(self,gameState):
        self.enemies = self.getOpponents(gameState)
        scaredTimes = [gameState.getAgentState(enemy).scaredTimer for enemy in self.enemies]        
        newFoodList = self.getFoodYouAreDefending(gameState).asList()
        temp = self.foodLost
        self.foodLost = [a for a in self.foodList if a not in newFoodList]
        if self.foodLost ==[]:
            self.foodLost = temp
        #get position
        self.pos = gameState.getAgentPosition(self.index)
        #get enemy info
        enemies = [gameState.getAgentState(i) for i in self.getOpponents(gameState)]
        self.invaders = [a for a in enemies if  a.isPacman and a.getPosition() != None]
        
        
        self.numInvaders = len(self.invaders)
        
        capsules = self.getCapsulesYouAreDefending(gameState)
        self.capsulesDistances = [self.getMazeDistance(self.pos, capsule) for capsule in capsules]
        
        if self.numInvaders == 0:
            self.isDefending = False
        else:
            self.isDefending = True
        
        actions = gameState.getLegalActions(self.index)
        
        values = [self.evaluate(gameState, a) for a in actions]
        maxValue = max(values)
        bestActions = [a for a, v in zip(actions, values) if v == maxValue]
        
        self.foodList = newFoodList
        return random.choice(bestActions)
        
    def evaluate(self,gameState,action):

        #step once
        successor = gameState.generateSuccessor(self.index, action)
        #get next state
        myState = successor.getAgentState(self.index)
        pos = myState.getPosition()
#        print(type(pos),pos)
        #enemy dist which we will minimize it
        
        if len(self.invaders) > 0:
            dists = [self.distancer.getDistance(pos, a.getPosition()) for a in self.invaders]
        elif len(self.foodLost) > 0:
#            print(self.foodLost)
            dists = [self.distancer.getDistance(pos,a) for a in self.foodLost]
        else:
            dists = [10000]
        minEnemyDist = min(dists)
        #make the ghost always defending 
        onDefense = 1
        if myState.isPacman: onDefense = 0
        #food info
        foodDefending = self.getFoodYouAreDefending(successor).asList()
        foodDistances = [self.distancer.getDistance(self.pos, food) for food in foodDefending]
        minFoodDistance = min(foodDistances) if len(foodDistances) else 0
        #capsule info
        minCapsuleDistance = min(self.capsulesDistances) if len(self.capsulesDistances) else 0
        #better not rev or stop
        rev = Directions.REVERSE[gameState.getAgentState(self.index).configuration.direction]
        if action == rev: reverse = 1
        else: reverse = 0
        if action == Directions.STOP: stop = 1
        else: stop = 0
        
        if gameState.isOnRedTeam(self.index):
            toMidDis = self.pos[0]
        else:
            toMidDis = self.width - self.pos[0]
#        toMidDis=0
#        print(toMidDis)
#        print(toMidDis)
#        toMidDis = 0
        if self.isDefending:
            score = 100*self.numInvaders - 10 * minEnemyDist - minCapsuleDistance -2*reverse - 100*stop +100*onDefense - toMidDis
        else:
            score = 100*self.numInvaders - 10 * minEnemyDist - minCapsuleDistance -2*reverse - 100*stop +100*onDefense - toMidDis
        return score
    
def findDeadEnd(mapList):
#    print(mapList.astype(np.int8))
    height,width = mapList.shape
    endflag = 1
    while endflag:
        endflag = 0
        for i in range(1,height-1):
            for j in range(1,width-1):
                if mapList[i,j] == 0:
                    count = 0
                    if mapList[i-1,j]: count+=1
                    if mapList[i+1,j]: count+=1
                    if mapList[i,j-1]: count+=1
                    if mapList[i,j+1]: count+=1
                    if count == 3:
                        mapList[i,j] = 1
                        endflag = 1
#    mapList = np.flipud(mapList)
#    print(mapList.astype(np.int8))
    return mapList.astype(np.int8)
                    