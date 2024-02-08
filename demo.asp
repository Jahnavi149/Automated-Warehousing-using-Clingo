#include "parsing.asp".
#include "convert_input.asp".

#const n=100.

%%%%%%%%%%% ADDING ACTIONS

move(0,1;0,-1;-1,0;1,0).

{robotMove(R,move(DX,DY),T):move(DX,DY)}1:- R=1..NR, numRobots(NR), T=0..TN,TN=n-1.
{pickUpShelf(R,SI,T):shelf(SI)}1:- R=1..NR, numRobots(NR), T=0..TN,TN=n-1.
{shelfDrop(R,SI,T):shelf(SI)}1:- R=1..NR, numRobots(NR), T=0..TN,TN=n-1.
{deliver(R,OI,with(SI,PR,DQ),T):orderAt(OI,object(node,ND),contains(PR,OQ),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T), DQ=1..PQ}1:- R=1..NR, numRobots(NR), T=0..TN,TN=n-1.

action(object(robot,R),move(DX,DY),T):-robotMove(R,move(DX,DY),T).
action(object(robot,R),pickup,T):-pickUpShelf(R,_,T).
action(object(robot,R),putdown,T):-shelfDrop(R,_,T).
action(object(robot,R),deliver(OI,PRI,DQ),T):-deliver(R,OI,with(SI,PRI,DQ),T).



%%%%%%%%%%% ADDING ACTION CONSTRAINTS
:- action(object(robot,R),A1,T), action(object(robot,R),A2,T), A1!=A2.

%%%%%%%%%%% ADDING ROBO MOVES
:- robotAt(RI,object(node,ND),T), robotMove(R,move(DX,DY),T), nodeAt(ND,pair(X,Y)), X+DX<1.
:- robotAt(RI,object(node,ND),T), robotMove(R,move(DX,DY),T), nodeAt(ND,pair(X,Y)), Y+DY<1.
:- robotAt(RI,object(node,ND),T), robotMove(R,move(DX,DY),T), nodeAt(ND,pair(X,Y)), X+DX>NC, numColumns(NC).
:- robotAt(RI,object(node,ND),T), robotMove(R,move(DX,DY),T), nodeAt(ND,pair(X,Y)), Y+DY>NR, numRows(NR).


%%%%%%%%%%% ADDING SHELF PICKUP
:- 2{pickUpShelf(R,S,T): robot(R)}, shelf(S).

:- pickUpShelf(RI,S1,T), shelfOn(S2,object(robot,RI),T).

:- pickUpShelf(R1,S,T), shelfOn(S,object(robot,R2),T).

:- pickUpShelf(RI,S,T), shelfOn(S,object(node,ND),T), not robotAt(RI,object(node,ND),T). 


%%%%%%%%%%% ADDING SHELF PUTDOWN
:- 2{shelfDrop(R,S,T): robot(R)}, shelf(S).

:- shelfDrop(RI,S,T), not shelfOn(S,object(robot,RI),T).

:- shelfDrop(RI,S,T), robotAt(RI,object(node,ND),T), highway(ND). 


%%%%%%%%%%% ADDING DELIVERY
:- deliver(R,OI,with(_,PR,_),T), orderAt(OI,object(node,ND),contains(PR,_),T), not robotAt(R,object(node, ND),T).

:- deliver(R,OI,with(SI,PR,_),T), productOn(PR,object(shelf,SI),with(quantity,_),T), not shelfOn(SI,object(robot,R),T).

:- deliver(R,OI,with(SI,PR,DQ),T), orderAt(OI,object(node,ND),contains(PR,OQ),T), DQ>OQ.

:- deliver(R,OI,with(SI,PR,DQ),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T), DQ>PQ.

%%%%%%%%%%% ADDING CONSTRAINTS
:- pickingStationAt(_,NDI), highway(NDI).

:- shelfOn(S,object(node,NDI),_), highway(NDI).

:- 2{robotAt(R,object(node,ND),T):node(ND)}, robot(R), T=0..n.

:- 2{robotAt(R,object(node,ND),T):robot(R)}, node(ND), T=0..n.

:- robotAt(R1,object(node,ND1),T), robotAt(R1,object(node,ND2),T+1), robotAt(R2,object(node,ND2),T), robotAt(R2,object(node,ND1),T+1), R1!=R2.

:- 2{shelfOn(S,object(robot,NR),T): robot(NR)}, shelf(S), T=0..n.

:- 2{shelfOn(S,object(robot,NR),T): shelf(S)}, robot(NR), T=0..n.

:- 2{shelfOn(S,object(node,ND),T): node(ND)}, shelf(S), T=0..n.

:- 2{shelfOn(S,object(node,ND),T): shelf(S)}, node(ND), T=0..n.

:- shelfOn(S,object(node,_),T), shelfOn(S,object(robot,_),T).

%%%%%%%%%%% ADDING ACTION STATES

robotAt(R,object(node,NEW_ND),T+1):- robotAt(R,object(node,ND),T), nodeAt(ND,pair(X,Y)), nodeAt(NEW_ND, pair(X+DX,Y+DY)), robotMove(R,move(DX,DY),T).

shelfOn(S,object(robot,RI),T+1):- pickUpShelf(RI,S,T), shelfOn(S,object(node,ND),T), robotAt(RI,object(node,ND),T).

shelfOn(S,object(node,ND),T+1):- shelfDrop(RI,S,T), shelfOn(S,object(robot,RI),T), robotAt(RI,object(node,ND),T).

orderAt(OI,object(node,ND),contains(PR,OU-DQ),T+1):- deliver(R,OI,with(SI,PR,DQ),T), orderAt(OI,object(node,ND),contains(PR,OU),T).
productOn(PR,object(shelf,SI),with(quantity,PQ-DQ),T+1):- deliver(R,OI,with(SI,PR,DQ),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T).

%%%%%%%%%%% ADDING COMMON-SENSE LAW OF INERTIA

robotAt(R,object(node,ND),T+1):- robotAt(R,object(node,ND),T), not robotMove(R,move(_,_),T), T<n.
shelfOn(S,object(node,ND),T+1):-shelfOn(S,object(node,ND),T), not pickUpShelf(_,S,T), T<n.
shelfOn(S,object(robot,RI),T+1):-shelfOn(S,object(robot,RI),T), not shelfDrop(RI,S,T), T<n.
orderAt(OI,object(node,ND),contains(PR,OU),T+1):- orderAt(OI,object(node,ND),contains(PR,OU),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T), not deliver(_,OI,with(SI,PR,_),T), T<n.
productOn(PR,object(shelf,SI),with(quantity,PQ),T+1):- productOn(PR,object(shelf,SI),with(quantity,PQ),T), not deliver(_,_,with(SI,PR,_),T), T<n.

% Goal state
:- not orderAt(OI,object(node,_),contains(PR,0),n), orderAt(OI,object(node,_),contains(PR,_),0).

numActions(N):-N=#sum{1,O,A,T:action(O,A,T)}.
timeTaken(N-1):-N=#count{T:action(O,A,T)}.
#minimize{1,O,A,T:action(O,A,T)}.
#minimize{T:action(O,A,T)}.

#show action/3.
#show numActions/1.