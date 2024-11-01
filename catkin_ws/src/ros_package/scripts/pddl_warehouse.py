from unified_planning.shortcuts import *
from unified_planning.io import PDDLWriter
from unified_planning.engines import PlanGenerationResultStatus
import rospy

def create_plan(costs: dict, packages: list, targets: list, rob_pos: tuple):
    """Create an optimal plan for delivering all the packaeges to the required targets according to the caculated costs.
    Return the plan in the form of location to go.
    """
    Location = UserType("Location")
    # Package = Object("Package", Location)
    Delivered = Fluent("Delivered", BoolType(), t = Location)
    At = Fluent("At", BoolType(), l = Location)
    warehouse_problem = Problem("Warehouse_Problem")

    Start_Location = Object("Start_Location", Location)
    
    t_objects = {}
    for i, t in enumerate(targets):
        t_objects[(packages[i], t)] = Object(f"Target{i}", Location)
        
    warehouse_problem.add_objects(list(t_objects.values()))
    warehouse_problem.add_object(Start_Location)
    warehouse_problem.add_fluent(At , default_initial_value=False)
    warehouse_problem.add_fluent(Delivered, default_initial_value=False)
    
    warehouse_problem.set_initial_value(At(Start_Location), True)

    
    mac = {}
    ac = {}
    for i, t1 in enumerate(targets):
        t1_obj = t_objects[(packages[i], t1)]
        start_cost = costs[(rob_pos, packages[i])] + costs[(packages[i], t1)]

        temp_a = InstantaneousAction(f"deliver_{i}")
        temp_a.add_precondition(At(Start_Location))
        temp_a.add_effect(Delivered(t1_obj), True)
        temp_a.add_effect(At(Start_Location), False)
        temp_a.add_effect(At(t1_obj), True)
        mac[temp_a] = Int(int(start_cost))
        ac[temp_a] = [((rob_pos, packages[i])), (packages[i], t1)]
        warehouse_problem.add_action(temp_a)

        for j, t2 in enumerate(targets):
            t2_obj = t_objects[(packages[j], t2)]
            if t1_obj == t2_obj:
                continue
            cost = costs[(t2, packages[i])] + costs[(packages[i], t1)]
            temp_a = InstantaneousAction(f"deliver_{j}_{i}")
            temp_a.add_precondition(At(t2_obj))
            temp_a.add_effect(Delivered(t1_obj), True)
            temp_a.add_effect(At(t2_obj), False)
            temp_a.add_effect(At(t1_obj), True)
            mac[temp_a] = Int(int(cost))
            ac[temp_a] = [((t2, packages[i])), (packages[i], t1)]
            warehouse_problem.add_action(temp_a)
        warehouse_problem.add_goal(Delivered(t1_obj))
    
    warehouse_problem.add_quality_metric(up.model.metrics.MinimizeActionCosts(mac))

    with OneshotPlanner(
    problem_kind=warehouse_problem.kind,
    optimality_guarantee=PlanGenerationResultStatus.SOLVED_OPTIMALLY) as planner:
        result = planner.solve(warehouse_problem)
        plan = result.plan
        rospy.loginfo("*"*50)
        rospy.loginfo(f"Total Cost ofr the plan: {Get_Total_Cost(plan,mac)}" )
        if plan is not None:
            plan = get_actions(plan, ac)
    return plan

def Get_Total_Cost(plan, mac):
    "Calculate the total cost of the plan"
    total_cost = 0
    for a in plan.actions:
        total_cost += mac[a.action]._content.payload
    return total_cost

def get_actions(plan, ac):
    "Convert plan to package and targets locations."
    plan_locations = []
    for a in plan.actions:
        plan_locations += ac[a.action] 
    return plan_locations
        



if __name__ == '__main__':
    Locations = [(130, 26), (135, 45), (76, 95), (92, 25), (38, 8)]
    Targets = [(151, 92), (151, 92), (152, 24), (70, 70), (151, 92)]
    Rob_pos =  (25, 18)
    costs = {((25, 18), (135, 45)): 4901.666666666665, ((135, 45), (151, 92)): 3668.999999999998, ((25, 18), (130, 26)): 4023.666666666665, ((130, 26), (151, 92)): 3908.333333333331, ((25, 18), (76, 95)): 4806.999999999998, ((76, 95), (152, 24)): 5164.0, ((25, 18), (38, 8)): 1700.0000000000002, ((38, 8), (151, 92)): 6318.000000000003, ((25, 18), (92, 25)): 2971.3333333333335, ((92, 25), (70, 70)): 3879.3333333333326, ((151, 92), (130, 26)): 3963.6666666666647, ((151, 92), (76, 95)): 2544.9999999999995, ((151, 92), (38, 8)): 6318.000000000003, ((151, 92), (92, 25)): 4786.666666666665, ((151, 92), (135, 45)): 3453.0000000000005, ((152, 24), (135, 45)): 2063.3333333333335, ((152, 24), (130, 26)): 1455.6666666666665, ((152, 24), (38, 8)): 4395.666666666664, ((152, 24), (92, 25)): 2203.0, ((70, 70), (135, 45)): 4294.666666666665, ((70, 70), (130, 26)): 4285.333333333331, ((70, 70), (76, 95)): 2260.333333333334, ((70, 70), (38, 8)): 3773.666666666665}
    create_plan(costs=costs, packages=Locations, targets=Targets, rob_pos=Rob_pos)

    





        

        
            