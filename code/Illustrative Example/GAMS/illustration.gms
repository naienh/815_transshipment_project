option limrow = 10

Sets
    N "all nodes" /A, B, C, D, E, F, G, H, I, J/
    T(N) "transfer nodes" /E, F, I/
    K "vehicles" /1, 2/
    R "orders" /1, 2, 3/;

alias(N,N_,N_p);

Sets
    o(K,N)      "starting depot"
    /   1.A
        2.A   /
    o_(K,N)     "end depot"
    /   1.G
        2.G   /
    p(R,N)      "pick up location"
    /   1.B
        2.D
        3.E   /
    d(R,N)      "drop off location"
    /   1.F
        2.J
        3.C   /;

Parameters
    u(K)        "vehicle carring capacity"
    /   1   6
        2   3   /
    q(R)        "order amount"
    /   1   3
        2   2
        3   4   /;

Table
    c(N, N_, K) "travelling cost"
                1   2
        A.  B   1   3
        A.  E   6   3
        A.  D   2   1
        B.  C   8   4
        B.  F   16  3
        B.  E   1   1
        C.  G   1   2
        D.  E   2   1
        D.  H   2   1
        E.  F   10  5
        E.  I   2   1
        F.  C   1   2
        F.  G   6   3
        F.  J   2   1
        F.  E   2   1
        H.  E   2   1
        H.  I   12  10
        I.  F   2   1
        I.  J   6   3
        J.  G   4   2
;

Set A(N,N_)     "available arcs";
A(N,N_) = yes$( c(N,N_,'1') <> 0 );

Binary Variables
    x(N,N_,K)       "if a car K travels in arc A"
    y(N,N_,K,R)     "if a car K travels in arc A carries order R"
;

variables
    obj "total cost";

Equations
    cost            "total cost"
    eq1(N,K)        "each K initiate once from original depot"
    eq2(N,N_,K)     "each K must end at its end depot"
    eq3(N,K)        "flow conservation through the nodes"
    eq4(R,N)     "requests are picked up"
    eq5(R,N)     "requests are delivered"
    eq6(R,T)     "transshipment node switch"
    eq7(K,R,N)   "same request flow conservation"
    eq8(N,N_,K,R)     "x>y"
    eq9(N,N_,K)     "vehicle capacity is respected"
;

cost..                  obj =e= sum(K,sum((N,N_)$(A(N,N_)),c(N,N_,K)*x(N,N_,K)));
eq1(N,K)$(o(K,N))..     sum(N_$(A(N,N_)),x(N,N_,K)) =l= 1;
eq2(N,N_,K)$(o(K,N) and o_(K,N_))..         sum(N_p$(A(N,N_p)),x(N,N_p,K)) =e= sum(N_p$(A(N_p,N_)),x(N_p,N_,K));
eq3(N,K)$((not o(K,N)) and (not o_(K,N))).. sum(N_$(A(N,N_)),x(N,N_,K)) =e= sum(N_$(A(N_,N)),x(N_,N,K));
eq4(R,N)$(p(R,N))..     sum((K,N_)$(A(N,N_)),y(N,N_,K,R)) =e= 1;
eq5(R,N)$(d(R,N))..     sum((K,N_)$(A(N_,N)),y(N_,N,K,R)) =e= 1;
eq6(R,T).. sum((K,N_)$(A(T,N_)),y(T,N_,K,R)) + 1$(d(R,T)) =e= sum((K,N_)$(A(N_,T)),y(N_,T,K,R)) + 1$(p(R,T));
eq7(K,R,N)$((not p(R,N)) and (not d(R,N)) and (not T(N))).. sum(N_$(A(N,N_)),y(N,N_,K,R)) =e= sum(N_$(A(N_,N)),y(N_,N,K,R));
eq8(N,N_,K,R)$(A(N,N_)).. y(N,N_,K,R) =l= x(N,N_,K);
eq9(N,N_,K)$(A(N,N_)).. sum(R,q(R)*y(N,N_,K,R)) =l= u(K)*x(N,N_,K);

Model transport /cost eq1 eq2 eq3 eq4 eq5 eq6 eq7 eq8 eq9/ ;

Option MIP=CPLEX;
Option optca=0;
Option optcr=0;

Solve transport using MIP minimizing obj ;



