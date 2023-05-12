#!/usr/bin/env python

from unified_planning.shortcuts import *
from unified_planning.engines import CompilationKind


def quantifiers_removing_compiler(problem):
  with Compiler(
        problem_kind = problem.kind, 
        compilation_kind = CompilationKind.QUANTIFIERS_REMOVING
    ) as quantifiers_remover:
    # After we have the compiler, we get the compilation result
    qr_result = quantifiers_remover.compile(
        problem, 
        CompilationKind.QUANTIFIERS_REMOVING
    )
    qr_problem = qr_result.problem
    qr_kind = qr_problem.kind
    
    # Check the result of the compilation
    #assert problem_kind.has_existential_conditions() #and problem_kind.has_universal_conditions()
    assert not qr_kind.has_existential_conditions() and not qr_kind.has_universal_conditions()
  return qr_problem

def conditionalEffects_removing_compiler(qr_problem):
  with Compiler(
        problem_kind = qr_problem.kind, 
        compilation_kind = CompilationKind.CONDITIONAL_EFFECTS_REMOVING
    ) as conditional_effects_remover:
    # After we have the compiler, we get the compilation result
    cer_result = conditional_effects_remover.compile(
        qr_problem, 
        CompilationKind.CONDITIONAL_EFFECTS_REMOVING
    )
    cer_problem = cer_result.problem
    cer_kind = cer_problem.kind
    
    # Check the result of the compilation
    #assert original_problem_kind.has_conditional_effects()
    #assert qr_kind.has_conditional_effects()
    assert not cer_kind.has_conditional_effects()
  return cer_problem

def disjunctiveConditions_removing_compiler(cer_problem):
  with Compiler(
        problem_kind = cer_problem.kind,
        compilation_kind = CompilationKind.DISJUNCTIVE_CONDITIONS_REMOVING
    ) as disjunctive_conditions_remover:
    # After we have the compiler, we get the compilation result
    dcr_result = disjunctive_conditions_remover.compile(
        cer_problem, 
        CompilationKind.DISJUNCTIVE_CONDITIONS_REMOVING
    )
    dcr_problem = dcr_result.problem
    dcr_kind = dcr_problem.kind
    
    # Check the result of the compilation
    #assert qr_kind.has_disjunctive_conditions()
    #assert cer_kind.has_disjunctive_conditions()
    assert not dcr_kind.has_disjunctive_conditions()
  return dcr_problem 










class POPGenerator:

    def __init__(self):

        # -------------------------------------------------
        #                    DOMAIN
        # -------------------------------------------------

        # Types
        LocationG = UserType("LocationG")  # Locations where the robot can go
        LocationP = UserType("LocationP")  # locations where a pouch can be found
        StateG = UserType("StateG")        # State of the gripper (available or not)
        PouchObj = UserType("PouchObj")    # Pouch object

        # Fluents
        self.at = Fluent("at", BoolType(), loc=LocationG)
        self.inStateG = Fluent("inStateG", BoolType(), state=StateG)
        self.pouchAt = Fluent("pouchAt", BoolType(), pouch=PouchObj, fromL=LocationP)
        self.detected = Fluent("detected", BoolType(), pouch=PouchObj)
        self.grasped = Fluent("grasped", BoolType(), pouch=PouchObj)
        self.measuredAt = Fluent("measuredAt", BoolType(), pouch=PouchObj, device=LocationP)
        self.relativeLoc = Fluent("relativeLoc", BoolType(), locP=LocationP, locG=LocationG)
        self.adj = Fluent("adj", BoolType(), fromL=LocationG, to=LocationG)
        self.reset = Fluent("reset", BoolType(), device=LocationP)
        self.free = Fluent("free", BoolType(), device=LocationP)
        self.opened = Fluent("opened", BoolType(), device=LocationP)
        self.isDraw = Fluent("isDraw", BoolType(), loc=LocationP)

        self.fluents = [self.at, self.inStateG, self.pouchAt, self.detected, self.grasped, self.measuredAt, self.relativeLoc, self.adj, self.reset, self.free, self.opened, self.isDraw]

        # Objects
        self.available = Object("available", StateG)
        self.vision2 = Object("vision2", LocationG)
        self.drawersLoc = Object("drawersLoc", LocationG)
        self.scaleLoc = Object("scaleLoc", LocationG)
        self.mark10SLoc = Object("mark10SLoc", LocationG)
        self.binLoc = Object("binLoc", LocationG)
        self.drawer = Object("drawer", LocationP)
        self.scale = Object("scale", LocationP)
        self.mark10S = Object("mark10S", LocationP)
        self.bin = Object("bin", LocationP)
        self.pouchWM = Object("pouchWM", PouchObj)       # Washing Machine Pouch
        self.pouchDW = Object("pouchDW", PouchObj)       # Dish Washer Pouch

        self.pouchA = Object("pouchA", PouchObj)       # Washing Machine Pouch


        self.objects = [self.available, self.vision2, self.drawersLoc, self.scaleLoc, self.mark10SLoc, self.binLoc, self.drawer, self.scale, self.mark10S, self.bin, self.pouchA]       

        # -------------------------------------------------
        #                    ACTIONS
        # -------------------------------------------------
        self.openDrawer = InstantaneousAction("openDrawer", draw=LocationP)
        draw = self.openDrawer.parameter("draw")
        self.openDrawer.add_precondition(self.inStateG(self.available))
        self.openDrawer.add_precondition(self.at(self.drawersLoc))
        self.openDrawer.add_precondition(self.isDraw(draw))
        self.openDrawer.add_precondition(Not(self.opened(draw)))
        self.openDrawer.add_effect(self.opened(draw), True)

        self.goto = InstantaneousAction("goto", fromL=LocationG, to=LocationG)
        fromL = self.goto.parameter("fromL")
        to = self.goto.parameter("to")
        self.goto.add_precondition(self.at(fromL))
        self.goto.add_precondition(Not(Equals(fromL, to)))
        self.goto.add_precondition(self.adj(fromL, to))
        self.goto.add_effect(self.at(to), True)
        self.goto.add_effect(self.at(fromL), False)

        self.senseImaging = InstantaneousAction("senseImaging", pouch=PouchObj)
        pouch = self.senseImaging.parameter("pouch")
        self.senseImaging.add_precondition(self.at(self.vision2))
        self.senseImaging.add_precondition(self.opened(self.drawer))
        self.senseImaging.add_precondition(Not(self.detected(pouch)))
        self.senseImaging.add_precondition(self.inStateG(self.available))
        self.p_var = Variable("p_var", PouchObj)
        self.senseImaging.add_precondition(Forall((Or( Equals(self.p_var, pouch),   Or(Not(self.detected(self.p_var)), Not(self.pouchAt(self.p_var, self.drawer)))   )), self.p_var))
        self.senseImaging.add_effect(self.detected(pouch), True)
        self.senseImaging.add_effect(self.pouchAt(pouch, self.drawer), True)

        self.pickPouch = InstantaneousAction("pickPouch", pouch=PouchObj, whereP=LocationP, gripPos=LocationG)
        pouch = self.pickPouch.parameter("pouch")
        whereP = self.pickPouch.parameter("whereP")
        gripPos = self.pickPouch.parameter("gripPos")
        self.pickPouch.add_precondition(self.inStateG(self.available))
        self.pickPouch.add_precondition(self.detected(pouch))
        self.pickPouch.add_precondition(self.pouchAt(pouch, whereP))
        self.pickPouch.add_precondition(Not(self.free(whereP)))
        self.pickPouch.add_precondition(self.at(gripPos))
        self.pickPouch.add_precondition(self.relativeLoc(whereP, gripPos))
        self.pickPouch.add_precondition(Not(Equals(whereP, self.bin)))
        self.pickPouch.add_precondition(Not(Equals(whereP, self.mark10S)))
        self.pickPouch.add_precondition(Or( Not(self.isDraw(whereP)), self.opened(whereP) ))
        self.pickPouch.add_effect(self.grasped(pouch), True)
        self.pickPouch.add_effect(self.inStateG(self.available), False)
        self.pickPouch.add_effect(self.pouchAt(pouch, whereP), False)
        self.pickPouch.add_effect(self.free(whereP), True, Not(self.isDraw(whereP)))

        self.putPouchOn = InstantaneousAction("putPouchOn", pouch=PouchObj, whereP=LocationP, gripPos=LocationG)
        pouch = self.putPouchOn.parameter("pouch")
        whereP = self.putPouchOn.parameter("whereP")
        gripPos = self.putPouchOn.parameter("gripPos")
        self.putPouchOn.add_precondition(self.grasped(pouch))
        self.putPouchOn.add_precondition(Not(self.inStateG(self.available)))
        self.putPouchOn.add_precondition(Not(Equals(whereP, self.drawer)))
        self.putPouchOn.add_precondition(Not(Equals(whereP, self.bin)))
        self.putPouchOn.add_precondition(self.relativeLoc(whereP, gripPos))
        self.putPouchOn.add_precondition(self.at(gripPos))
        self.putPouchOn.add_precondition(self.free(whereP))
        self.putPouchOn.add_effect(self.pouchAt(pouch, whereP), True)
        self.putPouchOn.add_effect(self.grasped(pouch), False)
        self.putPouchOn.add_effect(self.inStateG(self.available), True)
        self.putPouchOn.add_effect(self.free(whereP), False)

        self.dropPouch = InstantaneousAction("dropPouch", pouch=PouchObj, gripPos=LocationG)
        pouch = self.dropPouch.parameter("pouch")
        gripPos = self.dropPouch.parameter("gripPos")
        self.dropPouch.add_precondition(self.at(gripPos))
        #dropPouch.add_precondition(Or(Equals(gripPos, binLoc), And(Equals(gripPos, mark10SLoc), pouchAt(pouch, mark10S)) ))     ################################
        self.dropPouch.add_precondition(Or( And(self.grasped(pouch), Not(self.inStateG(self.available)), Equals(gripPos, self.binLoc)), And(self.measuredAt(pouch, self.mark10S), self.pouchAt(pouch, self.mark10S), Equals(gripPos, self.mark10SLoc)) ))
        self.dropPouch.add_effect(self.pouchAt(pouch, self.bin), True)
        self.dropPouch.add_effect(self.inStateG(self.available), True)
        self.dropPouch.add_effect(self.grasped(pouch), False)
        self.dropPouch.add_effect(self.free(self.mark10S), True, Equals(gripPos, self.mark10SLoc))
        self.dropPouch.add_effect(self.pouchAt(pouch, self.mark10S), False, Equals(gripPos, self.mark10SLoc))

        self.measure = InstantaneousAction("measure", pouch=PouchObj, dev=LocationP) #
        pouch = self.measure.parameter("pouch")
        dev = self.measure.parameter("dev")
        self.measure.add_precondition(self.pouchAt(pouch, dev))
        self.measure.add_precondition(self.reset(dev))
        self.measure.add_precondition(Not(self.isDraw(dev)))
        self.measure.add_precondition(Not(Equals(dev, self.bin)))  
        self.measure.add_effect(self.measuredAt(pouch, dev), True)
        self.measure.add_effect(self.reset(dev), False, Equals(dev, self.scale))  #when

        self.device_reset = InstantaneousAction("device_reset", dev=LocationP)
        dev = self.device_reset.parameter("dev")
        self.device_reset.add_precondition(Not(self.isDraw(dev)))
        self.device_reset.add_precondition(Not(Equals(dev, self.bin)))
        self.device_reset.add_precondition(Not(self.reset(dev)))
        self.device_reset.add_precondition(self.free(dev))
        self.device_reset.add_effect(self.reset(dev), True)

        self.actions = [self.openDrawer, self.goto, self.senseImaging, self.pickPouch, self.putPouchOn, self.dropPouch, self.measure, self.device_reset]

        # -------------------------------------------------
        #             INITIALIZING THE PROBLEM
        # -------------------------------------------------
    
        self.problem = None
        self.initialize()           # Initializing Problem

    def initialize(self):

        self.problem = Problem("PouchProblem")

        for obj in self.objects:
           self.problem.add_object(obj)
        
        for fluent in self.fluents:
            self.problem.add_fluent(fluent, default_initial_value=False)
        
        for action in self.actions:
           self.problem.add_action(action)
        
        self.setInitialValues()

    def setInitialValues(self): 
        self.problem.set_initial_value(self.inStateG(self.available), True)
        self.problem.set_initial_value(self.at(self.drawersLoc), True)
        self.problem.set_initial_value(self.isDraw(self.drawer), True)
        self.problem.set_initial_value(self.opened(self.drawer), True)   
        self.problem.set_initial_value(self.relativeLoc(self.drawer, self.vision2), True)
        self.problem.set_initial_value(self.relativeLoc(self.scale, self.scaleLoc), True)
        self.problem.set_initial_value(self.relativeLoc(self.mark10S, self.mark10SLoc), True)
        self.problem.set_initial_value(self.relativeLoc(self.bin, self.binLoc), True)   
        self.problem.set_initial_value(self.adj(self.binLoc, self.mark10SLoc), True)
        self.problem.set_initial_value(self.adj(self.mark10SLoc, self.binLoc), True)
        self.problem.set_initial_value(self.adj(self.mark10SLoc, self.scaleLoc), True)
        self.problem.set_initial_value(self.adj(self.scaleLoc, self.mark10SLoc), True)
        self.problem.set_initial_value(self.adj(self.mark10SLoc, self.drawersLoc), True)
        self.problem.set_initial_value(self.adj(self.drawersLoc, self.mark10SLoc), True)
        self.problem.set_initial_value(self.adj(self.scaleLoc, self.drawersLoc), True)
        self.problem.set_initial_value(self.adj(self.drawersLoc, self.scaleLoc), True)
        self.problem.set_initial_value(self.adj(self.drawersLoc, self.vision2), True)
        self.problem.set_initial_value(self.adj(self.vision2, self.drawersLoc), True)   
        self.problem.set_initial_value(self.reset(self.scale), True)
        self.problem.set_initial_value(self.reset(self.mark10S), True)
        self.problem.set_initial_value(self.free(self.scale), True)
        self.problem.set_initial_value(self.free(self.mark10S), True)
    
    def solve_problem(self):

        compiled_prob = disjunctiveConditions_removing_compiler(conditionalEffects_removing_compiler(quantifiers_removing_compiler(self.problem)))
            
        with OneshotPlanner(name = "fast-downward") as planner:
            assert planner.supports(compiled_prob.kind)       # Make sure the planner supports the compiled problem
            print("SUPPORTS!!")
            self.plan = planner.solve(compiled_prob).plan

            if self.plan:
                for i, cmd in enumerate(self.plan.actions):
                    print("{}:\t{}".format(i, cmd))

    def reset_problem(self):
        self.problem = None
        self.initialize()
        

    def setGoals(self, pouches):
        """
        This function sets the goals for the problem. Args:
            - pouches (class.Pouch or list): In case of problem for one pouch use the Pouch Type Object. In case of multiple pouches, 
                                            creates a list of Pouch Type Objects. Pouch object must contain the following parameters:
                                                - Type (str): WM (Washing Machine) or DW (Dish Washer)
                                                - weight (bool): True if scale test is required, else False
                                                - tight (bool): True if tight test is required, else False
                                                - strength (bool): True if strength test is required, else False
        """


        self.problem.add_goal(And(  self.measuredAt(self.pouchA, self.scale), self.measuredAt(self.pouchA, self.mark10S), (self.pouchAt(self.pouchA, self.bin))  ))
        """
        if "B" in problem.name:
            problem.add_goal(And(  measuredAt(pouchB, scale), Not(measuredAt(pouchB, mark10S)), (pouchAt(pouchB, bin))  ))
        if "C" in problem.name:
            problem.add_goal(And(  measuredAt(pouchC, scale), (pouchAt(pouchC, bin))  ))
        """
        self.problem.add_goal(self.at(self.drawersLoc))  
        self.problem.add_goal(self.reset(self.scale))
    









class Pouch:
    """
    This class helps in setting goals for the POP Creator
    Args:
        - Type (str): WM (Washing Machine) or DW (Dish Washer)
        - weight (bool): True if scale test is required, else False
        - tight (bool): True if tight test is required, else False
        - strength (bool): True if strength test is required, else False
    """

    def __init__(self, type, weight = True, tight = True, strength = True):
        
        if (type == 'WM' or
            type == 'DW'):
            self.type = type
        else:
            raise ValueError('Pouch can have WM or DW type but {} was received'.format(type))
        
        self.weight = weight
        self.tight = tight
        self.strength = strength




if __name__  == '__main__':
    print("b'Undeclared predicate: instateg\\n'\n")

    pop = POPGenerator()

    p = Pouch('WM')

    pop.setGoals(p)
    pop.solve_problem()
    