from typing import Iterable, Dict, Callable, Tuple


class RobotLoop:
    stateMergeStrategy: Callable
    actionMergeStrategy: Callable
    actuationHandler: Callable

    def defaultStateMergeStrategy(self, prevRobotState: Dict, prevWorldState: Dict, state_updates: Iterable) -> Tuple:
        """Merge robot and world state with a simple last-one-wins strategy"""
        state = ({**prevRobotState}, {**prevWorldState}) # Copy
        for i, update in enumerate(state_updates):
            state = ({**state[0], **update[0]}, {**state[1], **update[1]})
        return state

    def defaultActionMergeStrategy(self, actions: Iterable):
        """Merge actions with a simple last-one-wins strategy"""
        merged_actions = {}
        for i, action in enumerate(actions):
            merged_actions = {**merged_actions, **action}
        return merged_actions

    def defaultActuationHandler(self, actions: Iterable):
        """Take actions - default does nothing."""
        pass

    def __init__(self) -> None:
        self.stateMergeStrategy = self.defaultStateMergeStrategy
        self.actionMergeStrategy = self.defaultActionMergeStrategy
        self.actuationHandler = self.defaultActuationHandler

    def loop_step(self, prevRobotState: Dict, prevWorldState: Dict, context: Dict, perceptFuncs: Iterable, processingFuncs: Iterable) -> None:
        """
        One step of the robot loop.
        Constists of gathering data from sensors to update the robot/world state,
        doing some computation over that state to generate actions,
        and finally commanding the robot to take those actions.
        Synchronous, blocking.defaultActionMergeStrategy
        Pretty abstract, but that's on purpose!
        """
        #execute all sensing functions, which return updated states/state portions
        #(As opposed to raw measurements) 
        state_updates = [perceptFunc(prevRobotState, prevWorldState, context) for perceptFunc in perceptFuncs]
        #Merge state updates together using a pluggable strategy
        currentRobotState, currentWorldState = self.stateMergeStrategy(prevRobotState, prevWorldState, state_updates)
        #Execute processing functions... Intent is a little murky here
        #Could these be mulitprocessing, for example?
        #They might modify world state, they might not?
        #I think ideally, we are no longer modifying state here.
        #That should ALL BE DONE as part of the sensing functions above.
        #Roll with that for now.
        #Merge actions again with a pluggable strategy.
        #Could be a priority system, or whatever. Robot specific. Default to a simple last-wins.
        candidate_actions = [procFunc(currentRobotState, currentWorldState, context) for procFunc in processingFuncs]
        actions = self.actionMergeStrategy(candidate_actions)
        #Take the desired actions, in any way the outside code wants to.
        self.actuationHandler(actions, context)
        #TODO call back in to update the robot state to reflect the actions...
        #Return the updated states and actions
        return (currentRobotState, currentWorldState, actions)
        #Possibly some logging or telemetry or whatever refinement along the way as well,
        #But this should work.  Cheers!
