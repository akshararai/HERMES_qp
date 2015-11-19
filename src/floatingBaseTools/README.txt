This package does not build by itself, but provides code for other <robot>User packages. This code needs to be build inside of <robot>User, because it depends on code in <robot>. E.g. hermesLowerUser depends on floatingBaseTools. 

tests/solve_lexmin.cpp:
Reads a hierarchical QP from files in ${LAB_ROOT}/logs/lexmin/ and solves them. 
When finished, it writes back the solutions of each levels into the same 
folder. It is built e.q. by hermesLowerUser as an optional target. In order to 
build the code, run catkin_make solve_lexmin.
Hierarchical QPs can be generated/solved with python/lexmin.py.