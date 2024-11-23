from unified_planning.shortcuts import *
from unified_planning.io import PDDLWriter
from unified_planning.engines import PlanGenerationResultStatus
import rospy

def create_plan(costs: dict, packages: list, targets: list, rob_pos: tuple):
    """Create an optimal plan for delivering all the packaeges to the required targets according to the caculated costs.
    Return the plan in the form of location to go.
    """
    Location = UserType("Location")
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
    pass

    





        

        
            