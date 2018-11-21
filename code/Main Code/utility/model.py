import pyomo.environ as pe
import json

def generate_model(num_nodes,tw,trans,filename):
    tw_name = 'yes' if tw==True else 'no'
    folder_name = './processed_data_{}_nodes_{}_time_window/'.format(num_nodes,tw_name)

    with open((folder_name+filename).replace('.dat','.json'), 'r') as fp:
        location_data = json.load(fp)

    model = pe.ConcreteModel(name='815Project')
    data = pe.DataPortal()
    data.load(filename=folder_name+filename)

    # Total nodes
    model.N = pe.Set(initialize=data['N'])
    # total vehicles
    model.K = pe.Set(initialize=data['K'])
    # total requests
    model.R = pe.Set(initialize=data['R'])
    # transshipment node
    if trans:
        print('Adding Transshipment')
        model.T = pe.Set(within=model.N,initialize=data['T'])
    else:
        model.T = pe.Set(within=model.N)

    # load carrying capacity
    model.u = pe.Param(model.K,within=pe.NonNegativeReals,initialize=data['u'])
    # initial depot
    model.o = pe.Param(model.K,within=model.N,initialize=data['o'])
    # final depot
    model.o_ = pe.Param(model.K,within=model.N,initialize=data['o_'])
    # request quantity
    model.q = pe.Param(model.R,within=pe.NonNegativeReals,initialize=data['q'])
    # request pick up
    model.p = pe.Param(model.R,within=model.N,initialize=data['p'])
    # request drop pff
    model.d = pe.Param(model.R,within=model.N,initialize=data['d'])
    # transport cost
    model.c = pe.Param(model.N,model.N,model.K,within=pe.NonNegativeReals,initialize=0,mutable=True)

    if tw:
        print('Adding Time Window')
        # transport time
        model.tau = pe.Param(model.N,model.N,model.K,within=pe.NonNegativeReals,initialize=0,mutable=True)
        # earliest time
        model.t_e = pe.Param(model.N,initialize=data['t_e'])
        # latest time
        model.t_l = pe.Param(model.N,initialize=data['t_l'])

    # put values to cost and time
    data = pe.DataPortal()
    data.load(filename=folder_name+filename,model=model)
    for index,value in data['c'].items():
        model.c[index] = value
    if tw:
        for index,value in data['tau'].items():
            model.tau[index] = value
    # total arcs
    def arcs_rule(model,i,j):
        return model.c[i,j,1].value > 0
    model.A = pe.Set(initialize=model.N*model.N, filter = arcs_rule)


    # if a car K travels in arc A
    model.x = pe.Var(model.A,model.K,within=pe.Binary)
    # if a car K travels in arc A carries order R
    model.y = pe.Var(model.A,model.K,model.R,within=pe.Binary)
    # if node i precedes (not necessarily immediately) node j in the route of the vehicle k
    model.z = pe.Var(model.A,model.K,within=pe.Binary)

    if tw:
        # actual arrival time
        model.t_a = pe.Var(model.N,model.K)
        # actual departure time
        model.t_d = pe.Var(model.N,model.K)
        # if transshipment happens
        model.s = pe.Var(model.T,model.R,model.K,model.K,within=pe.Binary)

    # (1) enforce that each vehicle may initiate at most one route from its origin depot; constraints
    def one_car_start_rule(model,i,k):
        if i == model.o[k]:
            return sum(model.x[i,j,k] for j in model.N if (i,j) in model.A) <= 1
        else:
            return pe.Constraint.Skip
    model.one_car_start_con = pe.Constraint(model.N,model.K,rule=one_car_start_rule)

    # **Special Note: Addition (correction) from paper formulation**
    def no_car_enter_rule(model,i,k):
        if i == model.o[k]:
            return sum(model.x[j,i,k] for j in model.N if (j,i) in model.A) == 0
        else:
            return pe.Constraint.Skip
    model.no_car_enter_con = pe.Constraint(model.N,model.K,rule=no_car_enter_rule)


    # (2) enforce that the same vehicle must end the route at its ﬁnal depot.
    def end_at_final_rule(model,i,l,k):
        if i == model.o[k] and l == model.o_[k]:
            return sum(model.x[i,j,k] for j in model.N if (i,j) in model.A)\
            == sum(model.x[j,l,k] for j in model.N if (j,l) in model.A)
        else:
            return pe.Constraint.Skip
    model.end_at_final_con = pe.Constraint(model.N,model.N,model.K,rule=end_at_final_rule)


    # **Special Note: Addition (correction) from paper formulation**
    def no_car_leave_rule(model,i,k):
        if i == model.o_[k]:
            return sum(model.x[i,j,k] for j in model.N if (i,j) in model.A) == 0
        else:
            return pe.Constraint.Skip
    model.no_car_leave_con = pe.Constraint(model.N,model.K,rule=no_car_leave_rule)


    # (3) maintain ﬂow conservation of the vehicles through the nodes in the network.
    def flow_conservation_rule(model,i,k):
        if i != model.o[k] and i != model.o_[k]:
            return sum(model.x[i,j,k] for j in model.N if (i,j) in model.A)\
            == sum(model.x[j,i,k] for j in model.N if (j,i) in model.A)
        else:
            return pe.Constraint.Skip
    model.flow_conservation_con = pe.Constraint(model.N,model.K,rule=flow_conservation_rule)


    # (4) & (5) enforce all pickups and deliveries of the customer requests.
    def pickup_request_rule(model,i,r):
        if i == model.p[r]:
            return sum(model.y[i,j,k,r] for k in model.K for j in model.N if (i,j) in model.A) == 1
        else:
            return pe.Constraint.Skip
    model.pickup_request_con = pe.Constraint(model.N,model.R,rule=pickup_request_rule)

    def deliver_request_rule(model,i,r):
        if i == model.d[r]:
            return sum(model.y[j,i,k,r] for k in model.K for j in model.N if (j,i) in model.A) == 1
        else:
            return pe.Constraint.Skip
    model.deliver_request_con = pe.Constraint(model.N,model.R,rule=deliver_request_rule)


    # (6) maintain the request ﬂow conservation at the transshipment nodes allowing requests to switch from one vehicle to another while constraints
    # **Special Note: Deviation (correction) from paper formulation**
    def trans_conservation_rule(model,i,r):

        how_many_package_to_be_dropped = 1 if i == model.d[r] else 0
        how_many_package_to_be_picked = 1 if i == model.p[r] else 0

        return sum(model.y[i,j,k,r] for k in model.K for j in model.N if (i,j) in model.A)\
        + how_many_package_to_be_dropped == sum(model.y[j,i,k,r] for k in model.K for j in model.N if (j,i) in model.A)\
        + how_many_package_to_be_picked
    model.trans_conservation_con = pe.Constraint(model.T,model.R,rule=trans_conservation_rule)


    # (7) maintain the request ﬂow conservation at the non-transshipment nodes requiring that any vehicle bringing a request must also leave carrying the same request.
    def request_conservation_rule(model,i,k,r):
        if i not in model.T and i != model.p[r] and i != model.d[r]:
            return sum(model.y[i,j,k,r] for j in model.N if (i,j) in model.A)\
            == sum(model.y[j,i,k,r] for j in model.N if (j,i) in model.A)
        else:
            return pe.Constraint.Skip
    model.request_conservation_con = pe.Constraint(model.N,model.K,model.R,rule=request_conservation_rule)


    # (8) enforce a vehicle ﬂow on an arc if there is some request ﬂow in the same vehicle on the same arc.
    def request_needs_car_rule(model,i,j,k,r):
        return model.y[i,j,k,r] <= model.x[i,j,k]
    model.request_needs_car_con = pe.Constraint(model.A,model.K,model.R,rule=request_needs_car_rule)


    # (9) ensure capacity of each vehicle on each arc of the network
    def capacity_rule(model,i,j,k):
        return sum(model.q[r]*model.y[i,j,k,r] for r in model.R) <= model.u[k] * model.x[i,j,k]
    model.capacity_con = pe.Constraint(model.A,model.K,rule=capacity_rule)


    # (12,13,14) subtour elimination
    def immediate_order_rule(model,i,j,k):
        if i == model.o[k] or j == model.o_[k]:
            return pe.Constraint.Skip
        else:
            return model.x[i,j,k] <= model.z[i,j,k]
    model.immediate_order_con = pe.Constraint(model.A,model.K,rule=immediate_order_rule)

    def one_direction_ahead_rule(model,i,j,k):
        if i == model.o[k] or j == model.o_[k]:
            return pe.Constraint.Skip
        else:
            return model.z[i,j,k] + model.z[j,i,k] == 1
    model.one_direction_ahead_con = pe.Constraint(model.A,model.K,rule=one_direction_ahead_rule)

    def no_triangle_rule(model,i,j,l,k):
        if (i,j) in model.A and (j,l) in model.A and (l,i) in model.A:
            return model.z[i,j,k] + model.z[j,l,k] + model.z[l,i,k] <= 2
        else:
            return pe.Constraint.Skip
    model.no_triangle_con = pe.Constraint(model.N,model.N,model.N,model.K,rule=no_triangle_rule)


    if tw:
        # (15,16) timing calculation
        def travel_time_rule(model,i,j,k):
            return model.t_d[i,k] + model.tau[i,j,k] - model.t_a[j,k] <= 3000*(1-model.x[i,j,k])
        model.travel_time_con = pe.Constraint(model.A,model.K,rule=travel_time_rule)

        def arrive_first_rule(model,i,k):
            return model.t_a[i,k] <= model.t_d[i,k]
        model.arrive_first_con = pe.Constraint(model.N,model.K,rule=arrive_first_rule)


        # (17,18) pick up and delivery time window
        def pick_up_arrival_rule(model,r,k):
            return model.t_e[model.p[r]] <= model.t_a[model.p[r],k]
        model.pick_up_arrival_con = pe.Constraint(model.R,model.K,rule=pick_up_arrival_rule)

        def pick_up_depart_rule(model,r,k):
            return model.t_l[model.p[r]] >= model.t_d[model.p[r],k]
        model.pick_up_depart_con = pe.Constraint(model.R,model.K,rule=pick_up_depart_rule)

        def delivery_arrival_rule(model,r,k):
            return model.t_e[model.d[r]] <= model.t_a[model.d[r],k]
        model.delivery_arrival_con = pe.Constraint(model.R,model.K,rule=delivery_arrival_rule)

        def delivery_depart_rule(model,r,k):
            return model.t_l[model.d[r]] >= model.t_d[model.d[r],k]
        model.delivery_depart_con = pe.Constraint(model.R,model.K,rule=delivery_depart_rule)


        # (19,20) transshipment time window
        def transshipment_detector_rule(model,r,i,k,l):
            if k == l:
                return pe.Constraint.Skip
            else:
                return sum(model.y[j,i,k,r] for j in model.N if (j,i) in model.A)\
                + sum(model.y[i,j,l,r] for j in model.N if (i,j) in model.A) <= model.s[i,r,k,l] + 1
        model.transshipment_detector_con = pe.Constraint(model.R,model.T,model.K,model.K,rule=transshipment_detector_rule)

        def transshipment_time_window_rule(model,r,i,k,l):
            if k == l:
                return pe.Constraint.Skip
            else:
                return model.t_a[i,k] - model.t_d[i,l] <= 3000*(1-model.s[i,r,k,l])
        model.transshipment_time_window_con = pe.Constraint(model.R,model.T,model.K,model.K,rule=transshipment_time_window_rule)

    model.obj = pe.Objective(expr=sum(model.c[i,j,k]*model.x[i,j,k] for i,j in model.A for k in model.K))

    return model, location_data

def get_statistics(results):
    ''' return time, obj, gap
    '''
    time = float(results.Solver[0]['Time'])
    obj = float(results.Solution[0]['Objective']['obj']['Value'])
    lb = float(results.Problem[0]['Lower bound'])
    ub = float(results.Problem[0]['Upper bound'])
    gap = (ub-lb) / ub
    return time, obj, gap
