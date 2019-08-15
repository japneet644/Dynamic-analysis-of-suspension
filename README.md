# Dynamic-analysis-of-suspension
## DesignReport.pdf contains overall report of suspesnion design for B19
An approximation to the trajectory of suspension.

## Suspension Forces:

1) **Front_Suspension.m**: Used for calculation of Forces in Double Wishbone Suspesion
2) **Rear_Suspesion.m**: Used for calculating forces in multi link rear suspesion with damper mounted on knucle.
3) **Pullrood.m** : Used for calculating forces in Pull rod type suspension.

## Suspension Analysis:

1) ***Steeringcamber.m***: Plots the camber angle at each tire with steer. Positive steer angle means outer tire.
2) ***NumericalOptimzation.m***: This file can be usewd for numerical optimzation of suspesion points constrained within cuboids (determined from packaging constraints). The algorithmn determines the optimized the X,Y,Z co-ordinates of each suspesipon points by minimizing a cost function through a pattern search algorithmn.
3) **Analysis.m**: Code does Dynamic analysis (for bump) of suspension given the initial chasis and knucle points for Double Wishbone suspension
 
 ### Steering Analysis
 1) **Ackermann.m**: Can be used to find the optimum ackermann angle for 100% ackermann geometry.
